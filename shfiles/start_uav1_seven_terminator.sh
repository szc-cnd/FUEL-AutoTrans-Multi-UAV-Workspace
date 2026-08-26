#!/usr/bin/env bash

# UAV1 本机 ROS master、传感器、单机 Diff 和 AutoTrans 七分屏入口。
# 不自动解锁、不切换 OFFBOARD、不发布额外目标点。

set -o pipefail

if [[ -z "${BASH_VERSION:-}" ]]; then
  exec /bin/bash "$0" "$@"
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_PATH="${SCRIPT_DIR}/$(basename -- "${BASH_SOURCE[0]}")"
MATCH_WS="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="${UAV1_SINGLE_ROS_SETUP:-/opt/ros/noetic/setup.bash}"
LAYOUT_CONFIG="${UAV1_SINGLE_LAYOUT_CONFIG:-${SCRIPT_DIR}/terminator_uav1_six.conf}"
LOCAL_MASTER_IP="${UAV1_SINGLE_ROS_MASTER_IP:-10.54.87.232}"
LOCAL_ROS_IP="${UAV1_SINGLE_ROS_IP:-10.54.87.232}"
RUN_ID="${UID:-$(id -u)}"
TERMINATOR_LOG="${UAV1_SINGLE_TERMINATOR_LOG:-/tmp/uav1_seven_terminator_${RUN_ID}.log}"
TERMINATOR_PID_FILE="${UAV1_SINGLE_TERMINATOR_PID_FILE:-/tmp/uav1_seven_terminator_${RUN_ID}.pid}"
START_LOCK_FILE="${UAV1_SINGLE_START_LOCK_FILE:-/tmp/uav1_seven_start.lock}"
RUNTIME_CONFIG="${UAV1_SINGLE_RUNTIME_CONFIG:-/tmp/uav1_seven_terminator_${RUN_ID}.conf}"
ROSCORE_LOG="${UAV1_SINGLE_ROSCORE_LOG:-/tmp/uav1_single_roscore_${RUN_ID}.log}"
WAIT_TIMEOUT="${UAV1_SINGLE_WAIT_TIMEOUT:-180}"
THERMAL="${UAV1_SINGLE_THERMAL:-true}"

MAVROS_STATE_TOPIC="/UAV1/mavros/state"
LIDAR_TOPIC="/UAV1/livox/lidar"
LIDAR_IMU_TOPIC="/UAV1/livox/imu"
HIGH_FREQ_ODOM_TOPIC="/UAV1/fast_lio/Odom_high_freq"
ODOM_TOPIC="${HIGH_FREQ_ODOM_TOPIC}"
CLOUD_TOPIC="/UAV1/fast_lio/cloud_registered"
DOWN_CAMERA_TOPIC="/UAV1/down_camera/image_raw"
PLANNER_HEARTBEAT_TOPIC="/drone_1_traj_server/heartbeat"
VISION_POSE_TOPIC="/UAV1/mavros/vision_pose/pose"
VISION_STABILIZE_SECONDS="${UAV1_SINGLE_VISION_STABILIZE_SECONDS:-4}"

PANE=""
SHOW_HELP=false
STOP_REQUEST=false
OWNED_MASTER_PID=""

log() {
  printf '[uav1_seven] %s\n' "$*"
}

usage() {
  cat <<'EOF'
用法：
  bash shfiles/start_uav1_seven_terminator.sh
  bash shfiles/start_uav1_seven_terminator.sh stop

作用：
  打开一个 UAV1 单机 Terminator 七分屏窗口并依次等待、启动：
  1 本机 ROS master/MAVROS、2 MID360、3 FAST-LIO、4 高频视觉位姿回传、
  5 D435/颜色/二维码/前视ArUco/目标上报、6 run_swarm Diff/RViz、
  7 AutoTrans 控制器/桥接/日志/自动 rosbag。

说明：
  - ROS master 固定使用 UAV1 本机 10.54.87.232:11311，不依赖 UAV0。
  - 第 6 屏执行 run_swarm.launch 中当前配置的单机预设航点。
  - 第 6 屏启动下视相机，但不启动双机接力、坐标对齐、搜索精降或降落仲裁节点。
  - AutoTrans 直接向 /UAV1/mavros/setpoint_raw/attitude 发布控制量。
  - 不自动解锁、不切换 OFFBOARD、不额外发送目标点。

选项：
  --thermal                 第 5 屏同时启动热成像检测和 D435 融合
  --no-thermal              关闭热成像
EOF
}

keep_pane_open() {
  printf '\n[uav1_seven] 本分屏保持打开；运行中的节点请按 Ctrl+C 停止。\n'
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

  export ROS_MASTER_URI="http://${LOCAL_MASTER_IP}:11311"
  export ROS_IP="${LOCAL_ROS_IP}"
  unset ROS_HOSTNAME ROS_LOG_DIR
  printf '[uav1_single_ros] master=%s local_ip=%s\n' "${ROS_MASTER_URI}" "${ROS_IP}"
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
          tr '\0' ' ' < "/proc/${pid}/cmdline" | grep -Fq 'uav1_seven'; then
        return 0
      fi
    fi
  fi
  pgrep -af '[t]erminator.*uav1_seven' >/dev/null 2>&1
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
    log "已请求关闭单机七分屏窗口（PID ${pid}）"
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
      printf '[等待] UAV1 本机 ROS master 尚未就绪（%ss/%ss）\n' "${elapsed}" "${WAIT_TIMEOUT}"
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  printf '[错误] 等待 UAV1 本机 ROS master 超时：%ss\n' "${WAIT_TIMEOUT}"
  return 1
}

start_or_reuse_local_master() {
  if rosnode list >/dev/null 2>&1; then
    printf '[复用] UAV1 本机 ROS master 已经运行：%s\n' "${ROS_MASTER_URI}"
    return 0
  fi

  printf '[启动] UAV1 本机 roscore：%s\n' "${ROS_MASTER_URI}"
  roscore >"${ROSCORE_LOG}" 2>&1 &
  OWNED_MASTER_PID=$!
  if wait_for_ros_master; then
    printf '[就绪] UAV1 本机 roscore PID=%s，日志=%s\n' \
      "${OWNED_MASTER_PID}" "${ROSCORE_LOG}"
    return 0
  fi
  return 1
}

stop_owned_local_master() {
  if [[ -n "${OWNED_MASTER_PID}" ]] && kill -0 "${OWNED_MASTER_PID}" 2>/dev/null; then
    printf '[停止] 关闭本分屏启动的 UAV1 本机 roscore（PID %s）\n' "${OWNED_MASTER_PID}"
    kill -INT "${OWNED_MASTER_PID}" 2>/dev/null || true
    wait "${OWNED_MASTER_PID}" 2>/dev/null || true
  fi
  OWNED_MASTER_PID=""
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
  printf '\n========== UAV1 单机第 %s 屏：%s ==========\n' "${pane_number}" "${pane_name}"
  if ! source_ros_environment; then
    keep_pane_open
  fi
}

run_mavros_pane() {
  local pane_status
  pane_init 1 本机ROS-master与MAVROS
  trap stop_owned_local_master EXIT INT TERM
  if ! start_or_reuse_local_master; then
    stop_owned_local_master
    trap - EXIT INT TERM
    keep_pane_open
  fi
  printf '[启动] UAV1 MAVROS，并设置 IMU/姿态/里程计/ESC 频率\n'
  sh "${MATCH_WS}/shfiles/run_uav1_sensor_stack.sh" mavros
  pane_status=$?
  stop_owned_local_master
  trap - EXIT INT TERM
  printf '[退出] MAVROS 分屏，返回码=%s\n' "${pane_status}"
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

run_detection_pane() {
  local mission_id="uav1_single_$(date +%Y%m%d)"
  pane_init 5 D435与目标检测
  wait_for_topic_message "${ODOM_TOPIC}" || keep_pane_open
  printf '[启动] UAV1 D435、颜色、二维码、前视ArUco、目标上报和RViz标记\n'
  printf '[参数] thermal=%s, odom=%s, mission_id=%s\n' \
    "${THERMAL}" "${ODOM_TOPIC}" "${mission_id}"
  roslaunch uav0_competition_bringup uav1_detection_stack.launch \
    enable_thermal:="${THERMAL}" \
    odom_topic:="${ODOM_TOPIC}" \
    target_reporting_mission_id:="${mission_id}"
  printf '[退出] 目标检测分屏，返回码=%s\n' "$?"
  keep_pane_open
}

run_planner_pane() {
  local down_camera_pid="" planner_pid="" rviz_status

  stop_owned_down_camera() {
    if [[ -n "${down_camera_pid}" ]] && kill -0 "${down_camera_pid}" 2>/dev/null; then
      printf '[停止] 关闭本分屏启动的 UAV1 下视相机（PID %s）\n' "${down_camera_pid}"
      kill -INT "${down_camera_pid}" 2>/dev/null || true
      wait "${down_camera_pid}" 2>/dev/null || true
    fi
  }

  stop_owned_planner() {
    if [[ -n "${planner_pid}" ]] && kill -0 "${planner_pid}" 2>/dev/null; then
      kill -INT "${planner_pid}" 2>/dev/null || true
      wait "${planner_pid}" 2>/dev/null || true
    fi
  }

  stop_owned_planner_and_camera() {
    stop_owned_planner
    stop_owned_down_camera
  }

  pane_init 6 单机Diff与RViz
  wait_for_topic_message "${ODOM_TOPIC}" || keep_pane_open
  wait_for_topic_message "${CLOUD_TOPIC}" || keep_pane_open
  if timeout 2 rostopic echo -n 1 "${DOWN_CAMERA_TOPIC}" >/dev/null 2>&1; then
    printf '[复用] UAV1 下视相机已经发布：%s\n' "${DOWN_CAMERA_TOPIC}"
  else
    printf '[启动] UAV1 下视相机；合并标注画面将在综合 RViz 中显示\n'
    sh "${MATCH_WS}/shfiles/run_uav1_sensor_stack.sh" landing &
    down_camera_pid=$!
  fi
  trap stop_owned_planner_and_camera EXIT INT TERM
  if ! wait_for_topic_message "${DOWN_CAMERA_TOPIC}"; then
    stop_owned_planner_and_camera
    trap - EXIT INT TERM
    keep_pane_open
  fi
  printf '[启动] diff_planner run_swarm.launch 和 UAV1 综合检测 RViz\n'
  printf '[隔离] 不启动前机接力、双机坐标对齐或搜索精降\n'
  roslaunch diff_planner run_swarm.launch enable_rviz:=false &
  planner_pid=$!
  rosrun rviz rviz \
    -d "${MATCH_WS}/src/Diff-Planner/src/diff_planner/plan_manage/launch/include/uav1_lite.rviz" \
    /move_base_simple/goal:=/UAV1/planning/goal \
    /drone__diff_planner_node/goal_point:=/drone_1_diff_planner_node/goal_point \
    /diff_planner_node/global_list:=/drone_1_diff_planner_node/global_list \
    /diff_planner_node/a_star_list:=/drone_1_diff_planner_node/a_star_list
  rviz_status=$?
  stop_owned_planner_and_camera
  trap - EXIT INT TERM
  printf '[退出] UAV1 综合检测 RViz，返回码=%s\n' "${rviz_status}"
  keep_pane_open
}

run_controller_pane() {
  pane_init 7 AutoTrans控制器
  wait_for_mavros_connected || keep_pane_open
  wait_for_topic_message "${HIGH_FREQ_ODOM_TOPIC}" || keep_pane_open
  wait_for_topic_message "${PLANNER_HEARTBEAT_TOPIC}" || keep_pane_open
  wait_for_topic_message "${VISION_POSE_TOPIC}" || keep_pane_open
  printf '[等待] 已收到外部视觉位姿，等待 PX4 EKF 稳定融合 %ss\n' \
    "${VISION_STABILIZE_SECONDS}"
  sleep "${VISION_STABILIZE_SECONDS}"
  wait_for_topic_message "${VISION_POSE_TOPIC}" || keep_pane_open
  printf '[启动] UAV1 轨迹桥接、AutoTrans、日志和自动 rosbag\n'
  printf '[控制] AutoTrans 直接发布 /UAV1/mavros/setpoint_raw/attitude\n'
  roslaunch autotrans_reference_bridge uav1_diff_autotrans.launch \
    enable_planner:=false \
    setpoint_topic:=/UAV1/mavros/setpoint_raw/attitude
  printf '[退出] AutoTrans 分屏，返回码=%s\n' "$?"
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
    *) log "未知分屏：${PANE}"; keep_pane_open ;;
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
    --thermal) THERMAL=true; shift ;;
    --no-thermal) THERMAL=false; shift ;;
    --thermal=*) THERMAL="${1#*=}"; shift ;;
    stop) STOP_REQUEST=true; shift ;;
    -h|--help) SHOW_HELP=true; shift ;;
    *) log "未知参数：$1"; SHOW_HELP=true; shift ;;
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
  log 'UAV1 单机七分屏窗口已经运行，跳过重复启动'
  exit 0
fi
if ! prepare_graphical_terminal; then
  exit 1
fi

if command -v flock >/dev/null 2>&1; then
  exec 9>"${START_LOCK_FILE}"
  if ! flock -n 9; then
    log '另一个 UAV1 单机七分屏启动正在进行，跳过本次重复启动'
    exit 0
  fi
fi
if terminator_window_running; then
  log '检测到 UAV1 单机七分屏窗口正在启动，跳过重复启动'
  exit 0
fi

export UAV1_SINGLE_THERMAL="${THERMAL}"

sed "s|__UAV1_SIX_SCRIPT__|${SCRIPT_PATH}|g" \
  "${LAYOUT_CONFIG}" > "${RUNTIME_CONFIG}"

log '打开 UAV1 单机 Terminator 七分屏'
log '上排：1 本机Master/MAVROS | 2 MID360 | 3 FAST-LIO'
log '下排：4 视觉位姿 | 5 检测/上报 | 6 run_swarm Diff/RViz | 7 AutoTrans'
log "本机 ROS master：http://${LOCAL_MASTER_IP}:11311，ROS_IP=${LOCAL_ROS_IP}"
log "检测参数：thermal=${THERMAL}, odom=${ODOM_TOPIC}"
log "前级等待超时：${WAIT_TIMEOUT}s"
log "Terminator 启动日志：${TERMINATOR_LOG}"

nohup terminator --no-dbus --maximise \
  --config="${RUNTIME_CONFIG}" \
  --layout=uav1_six \
  >"${TERMINATOR_LOG}" 2>&1 &
terminator_pid=$!
printf '%s\n' "${terminator_pid}" > "${TERMINATOR_PID_FILE}"
log "Terminator 已启动，PID=${terminator_pid}"
