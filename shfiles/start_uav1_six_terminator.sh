#!/usr/bin/env bash

# UAV1 传感器、规划器和 AutoTrans 的 Terminator 六分屏一键入口。
# 不自动解锁、不切换 OFFBOARD、不发布目标点。

set -o pipefail

# 允许用户误用 sh 调用；ROS setup.bash 和本脚本均需要 Bash。
if [[ -z "${BASH_VERSION:-}" ]]; then
  exec /bin/bash "$0" "$@"
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_PATH="${SCRIPT_DIR}/$(basename -- "${BASH_SOURCE[0]}")"
MATCH_WS="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
NETWORK_SETUP="${SCRIPT_DIR}/configure_dual_uav_ros_network.sh"
ROS_SETUP="${UAV1_SIX_ROS_SETUP:-/opt/ros/noetic/setup.bash}"
LAYOUT_CONFIG="${UAV1_SIX_LAYOUT_CONFIG:-${SCRIPT_DIR}/terminator_uav1_six.conf}"
RUN_ID="${UID:-$(id -u)}"
TERMINATOR_LOG="${UAV1_SIX_TERMINATOR_LOG:-/tmp/uav1_six_terminator_${RUN_ID}.log}"
TERMINATOR_PID_FILE="${UAV1_SIX_TERMINATOR_PID_FILE:-/tmp/uav1_six_terminator_${RUN_ID}.pid}"
START_LOCK_FILE="${UAV1_SIX_START_LOCK_FILE:-/tmp/uav1_six_start.lock}"
RUNTIME_CONFIG="${UAV1_SIX_RUNTIME_CONFIG:-/tmp/uav1_six_terminator_${RUN_ID}.conf}"
WAIT_TIMEOUT="${UAV1_SIX_WAIT_TIMEOUT:-180}"

MAVROS_STATE_TOPIC="/UAV1/mavros/state"
LIDAR_TOPIC="/UAV1/livox/lidar"
LIDAR_IMU_TOPIC="/UAV1/livox/imu"
HIGH_FREQ_ODOM_TOPIC="/UAV1/fast_lio/Odom_high_freq"
ODOM_TOPIC="${HIGH_FREQ_ODOM_TOPIC}"
CLOUD_TOPIC="/UAV1/fast_lio/cloud_registered"
PLANNER_HEARTBEAT_TOPIC="/drone_1_traj_server/heartbeat"
VISION_POSE_TOPIC="/UAV1/mavros/vision_pose/pose"
VISION_STABILIZE_SECONDS="${UAV1_SIX_VISION_STABILIZE_SECONDS:-8}"

PANE=""
SHOW_HELP=false
STOP_REQUEST=false

log() {
  printf '[uav1_six] %s\n' "$*"
}

usage() {
  cat <<'EOF'
用法：
  bash shfiles/start_uav1_six_terminator.sh
  bash shfiles/start_uav1_six_terminator.sh stop

作用：
  打开一个 Terminator 六分屏窗口并依次等待、启动：
  1 MAVROS、2 MID360、3 FAST-LIO、4 高频视觉位姿回传、
  5 Diff-Planner/RViz、6 AutoTrans 控制器/桥接/日志/自动 rosbag。

说明：
  - 不自动解锁、不切换 OFFBOARD、不发送目标点。
  - 第 6 屏的 uav1_diff_autotrans.launch 默认 enable_planner=false，
    因此不会与第 5 屏重复启动 Diff-Planner。
  - 可用环境变量 UAV1_SIX_WAIT_TIMEOUT 修改前级等待超时，默认 180 秒。
  - 可用 UAV1_SIX_VISION_STABILIZE_SECONDS 修改视觉融合等待，默认 8 秒。
EOF
}

keep_pane_open() {
  printf '\n[uav1_six] 本分屏保持打开；运行中的节点请按 Ctrl+C 停止。\n'
  if [[ -t 0 ]]; then
    exec "${SHELL:-/bin/bash}" -i
  fi
  exec "${SHELL:-/bin/bash}"
}

source_ros_environment() {
  if [[ ! -f "${ROS_SETUP}" ]]; then
    log "找不到 ROS 环境：${ROS_SETUP}"
    return 1
  fi
  if [[ ! -f "${MATCH_WS}/devel/setup.bash" ]]; then
    log "match_ws 尚未编译或缺少：${MATCH_WS}/devel/setup.bash"
    return 1
  fi

  set +u
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
  # shellcheck disable=SC1091
  source "${MATCH_WS}/devel/setup.bash"
  set -u

  if [[ ! -r "${NETWORK_SETUP}" ]]; then
    log "找不到双机 ROS 网络配置：${NETWORK_SETUP}"
    return 1
  fi
  # shellcheck disable=SC1090
  source "${NETWORK_SETUP}"
  configure_dual_uav_ros_network uav1 || return 1
  unset ROS_LOG_DIR
}

prepare_graphical_terminal() {
  local desktop_pid desktop_var x_socket
  desktop_pid="$(pgrep -u "$(id -u)" -x gnome-shell | head -n 1 || true)"
  if [[ -n "${desktop_pid}" && -r "/proc/${desktop_pid}/environ" ]]; then
    while IFS= read -r -d '' desktop_var; do
      case "${desktop_var}" in
        DISPLAY=*) export DISPLAY="${desktop_var#DISPLAY=}" ;;
        XAUTHORITY=*) export XAUTHORITY="${desktop_var#XAUTHORITY=}" ;;
        DBUS_SESSION_BUS_ADDRESS=*) export DBUS_SESSION_BUS_ADDRESS="${desktop_var#DBUS_SESSION_BUS_ADDRESS=}" ;;
      esac
    done < "/proc/${desktop_pid}/environ"
  fi

  if [[ -z "${DISPLAY:-}" ]]; then
    for x_socket in /tmp/.X11-unix/X*; do
      if [[ -S "${x_socket}" ]]; then
        export DISPLAY=":${x_socket##*X}"
        break
      fi
    done
  fi
  if [[ -z "${DISPLAY:-}" ]]; then
    log '没有找到 DISPLAY，请在 Ubuntu 图形桌面终端中运行'
    return 1
  fi
  if [[ -z "${XAUTHORITY:-}" && -f "${HOME}/.Xauthority" ]]; then
    export XAUTHORITY="${HOME}/.Xauthority"
  fi
  if [[ -z "${DBUS_SESSION_BUS_ADDRESS:-}" && -S "/run/user/$(id -u)/bus" ]]; then
    export DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$(id -u)/bus"
  fi
}

terminator_window_running() {
  local pid
  if [[ -r "${TERMINATOR_PID_FILE}" ]]; then
    pid="$(tr -d '[:space:]' < "${TERMINATOR_PID_FILE}")"
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      if [[ -r "/proc/${pid}/cmdline" ]] &&
          tr '\0' ' ' < "/proc/${pid}/cmdline" | grep -Fq 'uav1_six'; then
        return 0
      fi
    fi
  fi
  pgrep -af '[t]erminator.*uav1_six' >/dev/null 2>&1
}

stop_terminator_window() {
  local pid
  if [[ ! -r "${TERMINATOR_PID_FILE}" ]]; then
    log '没有找到本脚本记录的 Terminator 窗口；如窗口仍在，请手动关闭'
    return 0
  fi
  pid="$(tr -d '[:space:]' < "${TERMINATOR_PID_FILE}")"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill "${pid}" 2>/dev/null || true
    log "已请求关闭 Terminator 窗口（PID ${pid}）；ROS 节点不会被此命令停止"
  else
    log '记录的 Terminator 进程已经退出'
  fi
  rm -f -- "${TERMINATOR_PID_FILE}"
}

wait_for_ros_master() {
  local elapsed=0
  while (( elapsed < WAIT_TIMEOUT )); do
    if rosnode list >/dev/null 2>&1; then
      return 0
    fi
    if (( elapsed == 0 || elapsed % 10 == 0 )); then
      printf '[等待] ROS master 尚未就绪（%ss/%ss）\n' "${elapsed}" "${WAIT_TIMEOUT}"
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  printf '[错误] 等待 ROS master 超时：%ss\n' "${WAIT_TIMEOUT}"
  return 1
}

wait_for_topic_message() {
  local topic="$1" elapsed=0
  while (( elapsed < WAIT_TIMEOUT )); do
    if timeout 2 rostopic echo -n 1 "${topic}" >/dev/null 2>&1; then
      printf '[就绪] 已收到话题数据：%s\n' "${topic}"
      return 0
    fi
    if (( elapsed == 0 || elapsed % 10 == 0 )); then
      printf '[等待] 尚未收到 %s（%ss/%ss）\n' "${topic}" "${elapsed}" "${WAIT_TIMEOUT}"
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  printf '[错误] 等待话题数据超时：%s\n' "${topic}"
  return 1
}

wait_for_mavros_connected() {
  local elapsed=0
  while (( elapsed < WAIT_TIMEOUT )); do
    if timeout 2 rostopic echo -n 1 "${MAVROS_STATE_TOPIC}" 2>/dev/null |
        grep -F 'connected: True' >/dev/null 2>&1; then
      printf '[就绪] UAV1 MAVROS 已连接飞控\n'
      return 0
    fi
    if (( elapsed == 0 || elapsed % 10 == 0 )); then
      printf '[等待] UAV1 MAVROS 尚未连接（%ss/%ss）\n' "${elapsed}" "${WAIT_TIMEOUT}"
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  printf '[错误] 等待 UAV1 MAVROS 连接超时\n'
  return 1
}

pane_init() {
  local pane_number="$1" pane_name="$2"
  printf '\n========== UAV1 第 %s 屏：%s ==========\n' "${pane_number}" "${pane_name}"
  if ! source_ros_environment; then
    keep_pane_open
  fi
}

run_mavros_pane() {
  pane_init 1 MAVROS
  wait_for_ros_master || keep_pane_open
  printf '[启动] UAV1 MAVROS，并设置 IMU/姿态/里程计/ESC 频率\n'
  sh "${MATCH_WS}/shfiles/run_uav1_sensor_stack.sh" mavros
  printf '[退出] MAVROS 分屏，返回码=%s\n' "$?"
  keep_pane_open
}

run_mid360_pane() {
  pane_init 2 MID360
  wait_for_ros_master || keep_pane_open
  printf '[启动] UAV1 MID360\n'
  sh "${MATCH_WS}/shfiles/run_uav1_sensor_stack.sh" mid360
  printf '[退出] MID360 分屏，返回码=%s\n' "$?"
  keep_pane_open
}

run_fastlio_pane() {
  pane_init 3 FAST-LIO
  wait_for_topic_message "${LIDAR_TOPIC}" || keep_pane_open
  wait_for_topic_message "${LIDAR_IMU_TOPIC}" || keep_pane_open
  printf '[启动] UAV1 FAST-LIO\n'
  sh "${MATCH_WS}/shfiles/run_uav1_sensor_stack.sh" fastlio
  printf '[退出] FAST-LIO 分屏，返回码=%s\n' "$?"
  keep_pane_open
}

run_pose_pane() {
  pane_init 4 高频视觉位姿回传
  wait_for_topic_message "${HIGH_FREQ_ODOM_TOPIC}" || keep_pane_open
  printf '[启动] %s -> /UAV1/mavros/vision_pose/pose\n' "${HIGH_FREQ_ODOM_TOPIC}"
  rosrun cxr_ego_ctrl laser_mid360_high_freq.py
  printf '[退出] 视觉位姿回传分屏，返回码=%s\n' "$?"
  keep_pane_open
}

run_planner_pane() {
  pane_init 5 Diff-Planner
  wait_for_topic_message "${ODOM_TOPIC}" || keep_pane_open
  wait_for_topic_message "${CLOUD_TOPIC}" || keep_pane_open
  printf '[启动] UAV1 Diff-Planner 和 RViz\n'
  roslaunch diff_planner run_swarm.launch
  printf '[退出] Diff-Planner 分屏，返回码=%s\n' "$?"
  keep_pane_open
}

run_controller_pane() {
  pane_init 6 AutoTrans控制器
  wait_for_mavros_connected || keep_pane_open
  wait_for_topic_message "${HIGH_FREQ_ODOM_TOPIC}" || keep_pane_open
  # 等周期心跳而不是 planning/status；后者要等目标触发并成功出轨迹才会发布。
  wait_for_topic_message "${PLANNER_HEARTBEAT_TOPIC}" || keep_pane_open
  wait_for_topic_message "${VISION_POSE_TOPIC}" || keep_pane_open
  printf '[等待] 已收到外部视觉位姿，等待 PX4 EKF 稳定融合 %ss\n' \
    "${VISION_STABILIZE_SECONDS}"
  sleep "${VISION_STABILIZE_SECONDS}"
  # 等待结束后再收一帧，避免视觉桥接仅短暂出现后已经退出。
  wait_for_topic_message "${VISION_POSE_TOPIC}" || keep_pane_open
  printf '[启动] UAV1 轨迹桥接、AutoTrans、日志和自动 rosbag\n'
  printf '[说明] uav1_diff_autotrans.launch 默认不重复启动 Diff-Planner\n'
  roslaunch autotrans_reference_bridge uav1_diff_autotrans.launch
  printf '[退出] AutoTrans 分屏，返回码=%s\n' "$?"
  keep_pane_open
}

run_pane() {
  case "${PANE}" in
    mavros) run_mavros_pane ;;
    mid360) run_mid360_pane ;;
    fastlio) run_fastlio_pane ;;
    pose) run_pose_pane ;;
    planner) run_planner_pane ;;
    controller) run_controller_pane ;;
    *) log "未知分屏：${PANE}"; keep_pane_open ;;
  esac
}

while (($# > 0)); do
  case "$1" in
    --pane)
      if [[ $# -lt 2 ]]; then
        log '--pane 需要指定 mavros/mid360/fastlio/pose/planner/controller'
        exit 2
      fi
      PANE="$2"
      shift 2
      ;;
    stop)
      STOP_REQUEST=true
      shift
      ;;
    -h|--help)
      SHOW_HELP=true
      shift
      ;;
    *)
      log "未知参数：$1"
      SHOW_HELP=true
      shift
      ;;
  esac
done

if [[ "${SHOW_HELP}" == true ]]; then
  usage
  exit 0
fi
if [[ "${STOP_REQUEST}" == true ]]; then
  stop_terminator_window
  exit 0
fi
if [[ -n "${PANE}" ]]; then
  run_pane
  exit 0
fi

if ! command -v terminator >/dev/null 2>&1; then
  log '未安装 terminator；请先执行 sudo apt install terminator'
  exit 1
fi
if ! command -v timeout >/dev/null 2>&1; then
  log '缺少 timeout 命令，无法进行可靠的话题等待'
  exit 1
fi
if [[ ! -f "${LAYOUT_CONFIG}" ]]; then
  log "找不到 Terminator 布局：${LAYOUT_CONFIG}"
  exit 1
fi
if [[ ! -f "${MATCH_WS}/devel/setup.bash" ]]; then
  log "match_ws 尚未编译或缺少：${MATCH_WS}/devel/setup.bash"
  exit 1
fi
if terminator_window_running; then
  log 'UAV1 六分屏窗口已经运行，跳过重复启动'
  exit 0
fi
if ! prepare_graphical_terminal; then
  exit 1
fi

if command -v flock >/dev/null 2>&1; then
  exec 9>"${START_LOCK_FILE}"
  if ! flock -n 9; then
    log '另一个 UAV1 六分屏启动正在进行，跳过本次重复启动'
    exit 0
  fi
fi
if terminator_window_running; then
  log '检测到 UAV1 六分屏窗口正在启动，跳过重复启动'
  exit 0
fi

sed "s|__UAV1_SIX_SCRIPT__|${SCRIPT_PATH}|g" \
  "${LAYOUT_CONFIG}" > "${RUNTIME_CONFIG}"

log '打开 UAV1 Terminator 六分屏'
log '上排：1 MAVROS | 2 MID360 | 3 FAST-LIO'
log '下排：4 视觉位姿 | 5 Diff/RViz | 6 AutoTrans/日志/rosbag'
log "前级等待超时：${WAIT_TIMEOUT}s"
log "PX4 外部视觉稳定等待：${VISION_STABILIZE_SECONDS}s"
log "Terminator 启动日志：${TERMINATOR_LOG}"

nohup terminator --no-dbus --maximise \
  --config="${RUNTIME_CONFIG}" \
  --layout=uav1_six \
  >"${TERMINATOR_LOG}" 2>&1 &
terminator_pid=$!
printf '%s\n' "${terminator_pid}" > "${TERMINATOR_PID_FILE}"
log "Terminator 已启动，PID=${terminator_pid}"
