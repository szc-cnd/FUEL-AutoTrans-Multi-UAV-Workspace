#!/usr/bin/env bash

# UAV0 搜索降落独立测试的一键 Terminator 入口。
# 默认隔离飞控输出；--flight 才启动 0.60 m 起飞悬停控制器。
# 无论哪种模式，都不会自动解锁、切换 OFFBOARD 或发布“已出通道”信号。

set -o pipefail

if [[ -z "${BASH_VERSION:-}" ]]; then
  exec /bin/bash "$0" "$@"
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_PATH="${SCRIPT_DIR}/$(basename -- "${BASH_SOURCE[0]}")"
MATCH_WS="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="${UAV0_SEARCH_TEST_ROS_SETUP:-/opt/ros/noetic/setup.bash}"
LAYOUT_CONFIG="${UAV0_SEARCH_TEST_LAYOUT_CONFIG:-${SCRIPT_DIR}/terminator_uav0_search_landing_test.conf}"
RUN_ID="${UID:-$(id -u)}"
RUNTIME_CONFIG="${UAV0_SEARCH_TEST_RUNTIME_CONFIG:-/tmp/uav0_search_landing_test_${RUN_ID}.conf}"
TERMINATOR_LOG="${UAV0_SEARCH_TEST_TERMINATOR_LOG:-/tmp/uav0_search_landing_test_${RUN_ID}.log}"
TERMINATOR_PID_FILE="${UAV0_SEARCH_TEST_PID_FILE:-/tmp/uav0_search_landing_test_${RUN_ID}.pid}"
START_LOCK_FILE="${UAV0_SEARCH_TEST_LOCK_FILE:-/tmp/uav0_search_landing_test.lock}"
RUN_TIMESTAMP="${UAV0_SEARCH_TEST_RUN_TIMESTAMP:-$(date +%Y%m%d_%H%M%S)}"
TEST_LOG_DIR="${UAV0_SEARCH_TEST_LOG_DIR:-${HOME}/search_landing_logs/${RUN_TIMESTAMP}}"

FLIGHT_CONTROL="${UAV0_SEARCH_TEST_FLIGHT_CONTROL:-false}"
ENABLE_RVIZ="${UAV0_SEARCH_TEST_ENABLE_RVIZ:-true}"
HOVER_HEIGHT="${UAV0_SEARCH_TEST_HOVER_HEIGHT:-0.60}"
ODOM_TOPIC="${UAV0_SEARCH_TEST_ODOM_TOPIC:-/UAV0/fast_lio/Odom_high_freq}"
PANE=""
STOP_REQUEST=false

log() {
  printf '[uav0_search_test] %s\n' "$*"
}

usage() {
  cat <<'EOF'
用法：
  bash shfiles/start_uav0_search_landing_test.sh [选项]

选项：
  --safe                    隔离飞控输出（默认）
  --flight                  启用实飞控制，先在 0.60 m 悬停
  --hover-height HEIGHT     实飞等待高度，默认 0.60 m
  --odom-topic TOPIC        FAST-LIO 里程计话题
  --no-rviz                 不打开 RViz
  stop                      关闭本脚本打开的 Terminator 窗口
  -h, --help                显示帮助

脚本不会自动解锁、切换 OFFBOARD 或发布“已出通道”信号。
EOF
}

keep_pane_open() {
  printf '\n[uav0_search_test] 本分屏保持打开；运行中的节点请按 Ctrl+C 停止。\n'
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
  # shellcheck disable=SC1090
  source "${MATCH_WS}/devel/setup.bash"
  set -u
  unset ROS_LOG_DIR
}

prepare_graphical_terminal() {
  local desktop_pid desktop_var x_socket user_bus
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
  user_bus="/run/user/$(id -u)/bus"
  if [[ -z "${DBUS_SESSION_BUS_ADDRESS:-}" && -S "${user_bus}" ]]; then
    export DBUS_SESSION_BUS_ADDRESS="unix:path=${user_bus}"
  fi
}

wait_for_ros_master() {
  local timeout_s="${1:-90}" elapsed=0
  while (( elapsed < timeout_s )); do
    if rosnode list >/dev/null 2>&1; then
      return 0
    fi
    if (( elapsed == 0 || elapsed % 5 == 0 )); then
      printf '[等待] ROS master（%ss/%ss）\n' "${elapsed}" "${timeout_s}"
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  printf '[错误] 等待 ROS master 超时\n'
  return 1
}

wait_for_topic() {
  local topic="$1" timeout_s="${2:-180}" elapsed=0
  while (( elapsed < timeout_s )); do
    if rostopic list 2>/dev/null | grep -Fx "${topic}" >/dev/null 2>&1; then
      printf '[就绪] %s\n' "${topic}"
      return 0
    fi
    if (( elapsed == 0 || elapsed % 10 == 0 )); then
      printf '[等待] %s（%ss/%ss）\n' "${topic}" "${elapsed}" "${timeout_s}"
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  printf '[错误] 等待话题超时：%s\n' "${topic}"
  return 1
}

pane_init() {
  mkdir -p "${TEST_LOG_DIR}" || {
    log "无法创建日志目录：${TEST_LOG_DIR}"
    keep_pane_open
  }
  exec > >(tee -a "${TEST_LOG_DIR}/${PANE}.log") 2>&1
  printf '\n========== UAV0 搜索降落独立测试：%s ==========\n' "$1"
  printf '[日志] %s\n' "${TEST_LOG_DIR}/${PANE}.log"
  source_ros_environment || keep_pane_open
}

run_mavros_pane() {
  pane_init '1 MAVROS'
  sh "${MATCH_WS}/shfiles/run.sh"
  keep_pane_open
}

run_mid360_pane() {
  pane_init '2 MID360'
  wait_for_ros_master 90 || keep_pane_open
  roslaunch livox_ros_driver2 msg_MID360.launch \
    vehicle_ns:=UAV0 \
    msg_frame_id:=UAV0/livox_frame \
    publish_freq:=10.0
  keep_pane_open
}

run_fastlio_pane() {
  pane_init '3 FAST-LIO'
  wait_for_topic /UAV0/livox/lidar 180 || keep_pane_open
  wait_for_topic /UAV0/livox/imu 180 || keep_pane_open
  roslaunch fast_lio mapping_mid360.launch \
    vehicle_ns:=UAV0 \
    odom_topic:="${ODOM_TOPIC}" \
    rviz:=false
  keep_pane_open
}

run_pose_pane() {
  pane_init '4 位姿回传'
  wait_for_topic "${ODOM_TOPIC}" 180 || keep_pane_open
  cd "${MATCH_WS}/src/cxr_ego_ctrl/src" || keep_pane_open
  python3 laser_mid360.py iris 0 fastlio off \
    _odom_topic:="${ODOM_TOPIC}"
  keep_pane_open
}

run_search_pane() {
  pane_init '5 搜索降落与 RViz'
  wait_for_topic "${ODOM_TOPIC}" 180 || keep_pane_open
  wait_for_topic /UAV0/fast_lio/cloud_registered 180 || keep_pane_open
  printf '[模式] enable_flight_control=%s, hover_height=%s, rviz=%s\n' \
    "${FLIGHT_CONTROL}" "${HOVER_HEIGHT}" "${ENABLE_RVIZ}"
  if [[ "${FLIGHT_CONTROL}" == true ]]; then
    printf '[安全] 控制器将发布 %s m 起飞悬停设定点，但不会自动解锁或切换 OFFBOARD。\n' \
      "${HOVER_HEIGHT}"
  else
    printf '[安全] 飞控输出已隔离，不启动简单控制器。\n'
  fi
  roslaunch uav0_competition_bringup uav0_search_landing_test.launch \
    enable_flight_control:="${FLIGHT_CONTROL}" \
    enable_rviz:="${ENABLE_RVIZ}" \
    hover_height:="${HOVER_HEIGHT}" \
    odom_topic:="${ODOM_TOPIC}"
  keep_pane_open
}

run_monitor_pane() {
  pane_init '6 状态与人工触发'
  wait_for_topic /landing_diff_search_manager/state 240 || keep_pane_open
  printf '\n[就绪] 搜索管理器已启动，当前状态：\n'
  timeout 5 rostopic echo -n 1 /landing_diff_search_manager/state || true
  printf '\n[重要] 本屏不会自动触发。确认 0.60 m 悬停稳定后，手动执行：\n\n'
  printf '%s\n' "rostopic pub -1 /UAV0/mission/task_status std_msgs/String \"data: 'SEARCH_OUTSIDE_LANDING'\""
  printf '\n常用查询：\n'
  printf '%s\n' 'rostopic echo /landing_diff_search_manager/state'
  printf '%s\n' 'rostopic echo /planner_command_arbiter/owner'
  printf '%s\n' 'rostopic echo /UAV0/landing/search/status'
  keep_pane_open
}

run_pane() {
  case "${PANE}" in
    mavros) run_mavros_pane ;;
    mid360) run_mid360_pane ;;
    fastlio) run_fastlio_pane ;;
    pose) run_pose_pane ;;
    search) run_search_pane ;;
    monitor) run_monitor_pane ;;
    *) log "未知分屏：${PANE}"; exit 2 ;;
  esac
}

terminator_window_running() {
  local pid
  if [[ -r "${TERMINATOR_PID_FILE}" ]]; then
    pid="$(tr -d '[:space:]' < "${TERMINATOR_PID_FILE}")"
    [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null && return 0
  fi
  pgrep -af '[t]erminator.*uav0_search_landing_test' >/dev/null 2>&1
}

stop_terminator_window() {
  local pid
  if [[ ! -r "${TERMINATOR_PID_FILE}" ]]; then
    log '没有找到本脚本记录的 Terminator 窗口'
    return 0
  fi
  pid="$(tr -d '[:space:]' < "${TERMINATOR_PID_FILE}")"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill "${pid}" 2>/dev/null || true
    log "已请求关闭 Terminator 窗口（PID ${pid}）"
  fi
  rm -f -- "${TERMINATOR_PID_FILE}"
}

while (($# > 0)); do
  case "$1" in
    --pane)
      [[ $# -ge 2 && -n "$2" ]] || { log '--pane 缺少分屏名称'; exit 2; }
      PANE="$2"
      shift 2
      ;;
    --safe) FLIGHT_CONTROL=false; shift ;;
    --flight) FLIGHT_CONTROL=true; shift ;;
    --no-rviz) ENABLE_RVIZ=false; shift ;;
    --hover-height)
      [[ $# -ge 2 && -n "$2" ]] || { log '--hover-height 缺少数值'; exit 2; }
      HOVER_HEIGHT="$2"
      shift 2
      ;;
    --hover-height=*) HOVER_HEIGHT="${1#*=}"; shift ;;
    --odom-topic)
      [[ $# -ge 2 && -n "$2" ]] || { log '--odom-topic 缺少话题名'; exit 2; }
      ODOM_TOPIC="$2"
      shift 2
      ;;
    --odom-topic=*) ODOM_TOPIC="${1#*=}"; shift ;;
    stop) STOP_REQUEST=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) log "未知参数：$1"; usage; exit 2 ;;
  esac
done

case "${FLIGHT_CONTROL,,}" in true|false) FLIGHT_CONTROL="${FLIGHT_CONTROL,,}" ;; *) log '飞控模式参数无效'; exit 2 ;; esac
case "${ENABLE_RVIZ,,}" in true|false) ENABLE_RVIZ="${ENABLE_RVIZ,,}" ;; *) log 'RViz 参数无效'; exit 2 ;; esac
if ! [[ "${HOVER_HEIGHT}" =~ ^[0-9]+([.][0-9]+)?$ ]] || ! awk -v height="${HOVER_HEIGHT}" 'BEGIN { exit !(height >= 0.50 && height <= 1.00) }'; then
  log "悬停高度必须在 0.50～1.00 m，当前值：${HOVER_HEIGHT}"
  exit 2
fi

if [[ "${STOP_REQUEST}" == true ]]; then
  stop_terminator_window
  exit 0
fi
if [[ -n "${PANE}" ]]; then
  run_pane
  exit 0
fi

command -v terminator >/dev/null 2>&1 || { log '未安装 terminator'; exit 1; }
[[ -f "${LAYOUT_CONFIG}" ]] || { log "找不到布局：${LAYOUT_CONFIG}"; exit 1; }
[[ -f "${MATCH_WS}/devel/setup.bash" ]] || { log 'match_ws 尚未编译'; exit 1; }
terminator_window_running && { log '搜索降落测试窗口已经运行'; exit 0; }
prepare_graphical_terminal || exit 1

if command -v flock >/dev/null 2>&1; then
  exec 9>"${START_LOCK_FILE}"
  flock -n 9 || { log '另一个启动过程正在运行'; exit 0; }
fi

export UAV0_SEARCH_TEST_FLIGHT_CONTROL="${FLIGHT_CONTROL}"
export UAV0_SEARCH_TEST_ENABLE_RVIZ="${ENABLE_RVIZ}"
export UAV0_SEARCH_TEST_HOVER_HEIGHT="${HOVER_HEIGHT}"
export UAV0_SEARCH_TEST_ODOM_TOPIC="${ODOM_TOPIC}"
export UAV0_SEARCH_TEST_RUN_TIMESTAMP="${RUN_TIMESTAMP}"
export UAV0_SEARCH_TEST_LOG_DIR="${TEST_LOG_DIR}"

mkdir -p "${TEST_LOG_DIR}" || {
  log "无法创建日志目录：${TEST_LOG_DIR}"
  exit 1
}

sed "s|__UAV0_SEARCH_TEST_SCRIPT__|${SCRIPT_PATH}|g" \
  "${LAYOUT_CONFIG}" > "${RUNTIME_CONFIG}"

log "打开六分屏：flight=${FLIGHT_CONTROL}, hover=${HOVER_HEIGHT}m, rviz=${ENABLE_RVIZ}"
log '脚本不会自动发布出通道信号'
log "本次六分屏日志：${TEST_LOG_DIR}"
nohup terminator --no-dbus --maximise \
  --config="${RUNTIME_CONFIG}" \
  --layout=uav0_search_landing_test \
  >"${TERMINATOR_LOG}" 2>&1 &
terminator_pid=$!
printf '%s\n' "${terminator_pid}" > "${TERMINATOR_PID_FILE}"
log "Terminator 已启动，PID=${terminator_pid}，日志=${TERMINATOR_LOG}"
