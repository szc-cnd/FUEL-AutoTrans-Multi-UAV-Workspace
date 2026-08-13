#!/usr/bin/env bash

# UAV0 比赛流程前七步的 Terminator 七分屏入口。
# 该脚本只负责启动机载端七个步骤，不自动解锁，也不启动 Windows 接收端。

set -o pipefail

# 允许从 Terminator、bash 或误用 sh 调用；ROS 的 setup.bash 需要 Bash。
if [[ -z "${BASH_VERSION:-}" ]]; then
  exec /bin/bash "$0" "$@"
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_PATH="${SCRIPT_DIR}/$(basename -- "${BASH_SOURCE[0]}")"
MATCH_WS="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="${UAV0_FIRST_SEVEN_ROS_SETUP:-/opt/ros/noetic/setup.bash}"
LAYOUT_CONFIG="${UAV0_FIRST_SEVEN_LAYOUT_CONFIG:-${SCRIPT_DIR}/terminator_uav0_first_seven.conf}"
RUN_ID="${UID:-$(id -u)}"
TERMINATOR_LOG="${UAV0_FIRST_SEVEN_TERMINATOR_LOG:-/tmp/uav0_first_seven_terminator_${RUN_ID}.log}"
TERMINATOR_PID_FILE="${UAV0_FIRST_SEVEN_TERMINATOR_PID_FILE:-/tmp/uav0_first_seven_terminator_${RUN_ID}.pid}"
START_LOCK_FILE="${UAV0_FIRST_SEVEN_START_LOCK_FILE:-/tmp/uav0_first_seven_start.lock}"
RUNTIME_CONFIG="${UAV0_FIRST_SEVEN_RUNTIME_CONFIG:-/tmp/uav0_first_seven_terminator_${RUN_ID}.conf}"

THERMAL="${UAV0_FIRST_SEVEN_THERMAL:-true}"
ODOM_TOPIC="${UAV0_FIRST_SEVEN_ODOM_TOPIC:-/UAV0/fast_lio/Odometry}"
PANE=""
SHOW_HELP=false
STOP_REQUEST=false

log() {
  printf '[uav0_first_seven] %s\n' "$*"
}

usage() {
  cat <<'EOF'
用法：
  bash shfiles/start_uav0_first_seven_terminator.sh [选项]

选项：
  --thermal                 第 5 屏同时启用热成像（默认）
  --no-thermal              第 5 屏关闭热成像
  --odom-topic TOPIC        FAST-LIO 里程计话题，默认 /UAV0/fast_lio/Odometry
  stop                      只关闭本脚本打开的 Terminator 窗口，不停止 ROS 节点
  -h, --help                显示帮助

示例：
  bash shfiles/start_uav0_first_seven_terminator.sh
  bash shfiles/start_uav0_first_seven_terminator.sh --no-thermal
  bash shfiles/start_uav0_first_seven_terminator.sh --odom-topic /Odometry
EOF
}

keep_pane_open() {
  printf '\n[uav0_first_seven] 本分屏保持打开；运行中的节点请按 Ctrl+C 停止。\n'
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

  # ROS setup 中可能访问未定义变量，不能在 source 时启用 nounset。
  set +u
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
  # shellcheck disable=SC1091
  source "${MATCH_WS}/devel/setup.bash"
  set -u

  # 与规划器保持一致，使用 ROS 默认的 ~/.ros/log/。
  unset ROS_LOG_DIR
}

prepare_graphical_terminal() {
  # 通过 SSH 启动时，尝试从当前用户的桌面会话补齐图形环境。
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
        log "检测到图形显示：${DISPLAY}"
        break
      fi
    done
  fi

  if [[ -z "${DISPLAY:-}" ]]; then
    log "没有找到 DISPLAY，请在 Ubuntu 图形桌面终端中运行此脚本"
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
      if [[ -r "/proc/${pid}/cmdline" ]] && tr '\0' ' ' < "/proc/${pid}/cmdline" | grep -Fq 'uav0_first_seven'; then
        return 0
      fi
    fi
  fi
  pgrep -af '[t]erminator.*uav0_first_seven' >/dev/null 2>&1
}

stop_terminator_window() {
  local pid
  if [[ ! -r "${TERMINATOR_PID_FILE}" ]]; then
    log "没有找到本脚本记录的 Terminator 窗口；如窗口仍在，请手动关闭"
    return 0
  fi
  pid="$(tr -d '[:space:]' < "${TERMINATOR_PID_FILE}")"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill "${pid}" 2>/dev/null || true
    log "已请求关闭 Terminator 窗口（PID ${pid}）；ROS 节点不会被此命令停止"
  else
    log "记录的 Terminator 进程已经退出"
  fi
  rm -f -- "${TERMINATOR_PID_FILE}"
}

wait_for_ros_master() {
  local timeout_s="${1:-90}" elapsed=0
  while (( elapsed < timeout_s )); do
    if rosnode list >/dev/null 2>&1; then
      return 0
    fi
    if (( elapsed == 0 || elapsed % 5 == 0 )); then
      printf '[等待] ROS master 尚未就绪（%ss/%ss）\n' "${elapsed}" "${timeout_s}"
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  printf '[错误] 等待 ROS master 超时：%ss\n' "${timeout_s}"
  return 1
}

wait_for_topic() {
  local topic="$1" timeout_s="${2:-120}" elapsed=0
  while (( elapsed < timeout_s )); do
    # 不使用 grep -q，避免 pipefail 下 grep 提前退出导致 rostopic 被 SIGPIPE。
    if rostopic list 2>/dev/null | grep -Fx "${topic}" >/dev/null 2>&1; then
      printf '[就绪] 已发现话题 %s\n' "${topic}"
      return 0
    fi
    if (( elapsed == 0 || elapsed % 10 == 0 )); then
      printf '[等待] 话题 %s 尚未出现（%ss/%ss）\n' "${topic}" "${elapsed}" "${timeout_s}"
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  printf '[错误] 等待话题超时：%s\n' "${topic}"
  return 1
}

find_down_camera() {
  local device
  device="$(find /dev/v4l/by-id -maxdepth 1 -type l \
    -name 'usb-Generic_USB_Camera_*-video-index0' 2>/dev/null | sort | head -n 1)"
  if [[ -z "${device}" ]]; then
    printf '[错误] 未发现下视相机：/dev/v4l/by-id/usb-Generic_USB_Camera_*-video-index0\n' >&2
    return 1
  fi
  printf '%s\n' "${device}"
}

pane_init() {
  local pane_name="$1"
  printf '\n========== UAV0 第 %s 步 =========\n' "${pane_name}"
  if ! source_ros_environment; then
    keep_pane_open
  fi
}

run_mavros_pane() {
  pane_init 1
  printf '[启动] MAVROS：shfiles/run.sh\n'
  sh "${MATCH_WS}/shfiles/run.sh"
  printf '[退出] MAVROS 分屏，返回码=%s\n' "$?"
  keep_pane_open
}

run_mid360_pane() {
  pane_init 2
  wait_for_ros_master 90 || keep_pane_open
  printf '[启动] UAV0 MID360\n'
  roslaunch livox_ros_driver2 msg_MID360.launch \
    vehicle_ns:=UAV0 \
    msg_frame_id:=UAV0/livox_frame \
    publish_freq:=30.0
  printf '[退出] MID360 分屏，返回码=%s\n' "$?"
  keep_pane_open
}

run_fastlio_pane() {
  pane_init 3
  wait_for_topic /UAV0/livox/lidar 180 || keep_pane_open
  wait_for_topic /UAV0/livox/imu 180 || keep_pane_open
  printf '[启动] UAV0 FAST-LIO\n'
  roslaunch fast_lio mapping_mid360.launch \
    vehicle_ns:=UAV0 \
    odom_topic:="${ODOM_TOPIC}" \
    rviz:=false
  printf '[退出] FAST-LIO 分屏，返回码=%s\n' "$?"
  keep_pane_open
}

run_pose_pane() {
  pane_init 4
  wait_for_topic "${ODOM_TOPIC}" 180 || keep_pane_open
  printf '[启动] UAV0 视觉位姿回传，输入=%s\n' "${ODOM_TOPIC}"
  cd "${MATCH_WS}/src/cxr_ego_ctrl/src" || {
    log "无法进入视觉位姿脚本目录"
    keep_pane_open
  }
  python3 laser_mid360.py iris 0 fastlio off \
    _odom_topic:="${ODOM_TOPIC}"
  printf '[退出] 视觉位姿回传分屏，返回码=%s\n' "$?"
  keep_pane_open
}

run_detection_pane() {
  pane_init 5
  wait_for_topic "${ODOM_TOPIC}" 180 || keep_pane_open
  local mission_id="onboard_test_$(date +%Y%m%d)" down_camera
  down_camera="$(find_down_camera)" || keep_pane_open
  printf '[启动] UAV0 D435/检测/TF/远程上报/精确降落\n'
  printf '[参数] thermal=%s, odom=%s, mission_id=%s, down_camera=%s\n' \
    "${THERMAL}" "${ODOM_TOPIC}" "${mission_id}" "${down_camera}"
  printf '[安全] 降落节点只监听 /UAV0/need_to_land，不会在启动时自动触发。\n'
  rosrun uav0_competition_bringup start_uav0_detection_landing_stack.sh \
    enable_realsense:=true \
    enable_thermal:="${THERMAL}" \
    enable_thermal_d435_fusion:="${THERMAL}" \
    realsense_color_width:=1280 \
    realsense_color_height:=720 \
    realsense_depth_width:=1280 \
    realsense_depth_height:=720 \
    realsense_enable_pointcloud:=true \
    enable_camera_body_tf:=true \
    enable_camera_body_odom_tf:=false \
    enable_target_reporting:=true \
    target_reporting_mission_id:="${mission_id}" \
    enable_down_camera:=true \
    enable_precision_landing:=true \
    landing_vehicle_ns:=UAV0 \
    down_camera_device:="${down_camera}"
  printf '[退出] 检测与上报分屏，返回码=%s\n' "$?"
  keep_pane_open
}

run_planner_pane() {
  pane_init 6
  wait_for_topic "${ODOM_TOPIC}" 180 || keep_pane_open
  wait_for_topic /UAV0/fast_lio/cloud_registered 180 || keep_pane_open
  local planner_launch
  planner_launch="$(rospack find diff_planner 2>/dev/null)/launch/exp/run_swarm_indoor1_fuel_exploration.launch"
  if [[ ! -f "${planner_launch}" ]]; then
    log "找不到 FUEL 规划器 launch：${planner_launch}"
    keep_pane_open
  fi
  printf '[启动] UAV0 FUEL 规划器和 RViz\n'
  roslaunch "${planner_launch}" \
    odom_topic:="${ODOM_TOPIC}"
  printf '[退出] FUEL 规划器分屏，返回码=%s\n' "$?"
  keep_pane_open
}

run_controller_pane() {
  pane_init 7
  wait_for_topic /UAV0/planning/pos_cmd 240 || keep_pane_open
  printf '[启动] UAV0 简单控制器\n'
  printf '[安全] 控制器不会自动解锁、切换 OFFBOARD 或起飞。\n'
  roslaunch exploration_control simple_controller.launch \
    vehicle_ns:=UAV0 \
    node_name:=UAV0_controller
  printf '[退出] 简单控制器分屏，返回码=%s\n' "$?"
  keep_pane_open
}

run_pane() {
  case "${PANE}" in
    mavros) run_mavros_pane ;;
    mid360) run_mid360_pane ;;
    fastlio) run_fastlio_pane ;;
    pose) run_pose_pane ;;
    detection) run_detection_pane ;;
    planner) run_planner_pane ;;
    controller) run_controller_pane ;;
    *)
      log "未知分屏：${PANE}"
      keep_pane_open
      ;;
  esac
}

while (($# > 0)); do
  case "$1" in
    --pane)
      if [[ $# -lt 2 ]]; then
        log '--pane 需要指定 mavros/mid360/fastlio/pose/detection/planner/controller'
        exit 2
      fi
      PANE="$2"
      shift 2
      ;;
    --thermal)
      THERMAL=true
      shift
      ;;
    --no-thermal)
      THERMAL=false
      shift
      ;;
    --thermal=*)
      THERMAL="${1#*=}"
      shift
      ;;
    --odom-topic)
      if [[ $# -lt 2 ]]; then
        log '--odom-topic 需要一个话题名'
        exit 2
      fi
      ODOM_TOPIC="$2"
      shift 2
      ;;
    --odom-topic=*)
      ODOM_TOPIC="${1#*=}"
      shift
      ;;
    -h|--help)
      SHOW_HELP=true
      shift
      ;;
    stop)
      STOP_REQUEST=true
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

case "${THERMAL,,}" in
  true|false) THERMAL="${THERMAL,,}" ;;
  *) log "--thermal 只能是 true 或 false，当前值：${THERMAL}"; exit 2 ;;
esac

if [[ "${STOP_REQUEST}" == true ]]; then
  stop_terminator_window
  exit 0
fi

if [[ -n "${PANE}" ]]; then
  run_pane
  exit 0
fi

if ! command -v terminator >/dev/null 2>&1; then
  log '未安装 terminator；请先安装后再使用七分屏入口'
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
  log 'UAV0 前七步 Terminator 窗口已经运行，跳过重复启动'
  exit 0
fi
if ! prepare_graphical_terminal; then
  exit 1
fi

# 避免两个终端同时执行入口时打开两个七分屏窗口。
if command -v flock >/dev/null 2>&1; then
  exec 9>"${START_LOCK_FILE}"
  if ! flock -n 9; then
    log '另一个七分屏启动正在进行，跳过本次重复启动'
    exit 0
  fi
fi
if terminator_window_running; then
  log '检测到七分屏窗口已经在启动，跳过重复启动'
  exit 0
fi

# 通过环境变量把主入口参数传给七个 Terminator 子分屏，避免在布局文件中拼接 shell 参数。
export UAV0_FIRST_SEVEN_THERMAL="${THERMAL}"
export UAV0_FIRST_SEVEN_ODOM_TOPIC="${ODOM_TOPIC}"

sed "s|__UAV0_FIRST_SEVEN_SCRIPT__|${SCRIPT_PATH}|g" \
  "${LAYOUT_CONFIG}" > "${RUNTIME_CONFIG}"

log '打开 UAV0 前七步 Terminator 七分屏'
log '上排：1 MAVROS | 2 MID360 | 3 FAST-LIO'
log '下排：4 视觉位姿 | 5 检测/上报 | 6 FUEL 规划器/RViz | 7 简单控制器'
log "检测参数：thermal=${THERMAL}, odom=${ODOM_TOPIC}"
log "Terminator 启动日志：${TERMINATOR_LOG}"

nohup terminator --no-dbus --maximise \
  --config="${RUNTIME_CONFIG}" \
  --layout=uav0_first_seven \
  >"${TERMINATOR_LOG}" 2>&1 &
terminator_pid=$!
printf '%s\n' "${terminator_pid}" > "${TERMINATOR_PID_FILE}"
log "Terminator 已启动，PID=${terminator_pid}"
