#!/usr/bin/env bash

# UAV0 传感器、FUEL-DIFF 混合规划和 AutoTrans 的 Terminator 六分屏入口。
# 不自动解锁、不切换 OFFBOARD、不发布目标点。

set -o pipefail

if [[ -z "${BASH_VERSION:-}" ]]; then
  exec /bin/bash "$0" "$@"
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_PATH="${SCRIPT_DIR}/$(basename -- "${BASH_SOURCE[0]}")"
MATCH_WS="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
LEGACY_PANE_SCRIPT="${SCRIPT_DIR}/start_uav0_first_seven_terminator.sh"
ROS_SETUP="${UAV0_SIX_ROS_SETUP:-/opt/ros/noetic/setup.bash}"
LAYOUT_CONFIG="${UAV0_SIX_LAYOUT_CONFIG:-${SCRIPT_DIR}/terminator_uav0_six.conf}"
RUN_ID="${UID:-$(id -u)}"
RUNTIME_CONFIG="${UAV0_SIX_RUNTIME_CONFIG:-/tmp/uav0_six_terminator_${RUN_ID}.conf}"
TERMINATOR_LOG="${UAV0_SIX_TERMINATOR_LOG:-/tmp/uav0_six_terminator_${RUN_ID}.log}"
PID_FILE="${UAV0_SIX_PID_FILE:-/tmp/uav0_six_terminator_${RUN_ID}.pid}"
WAIT_TIMEOUT="${UAV0_SIX_WAIT_TIMEOUT:-180}"
VISION_STABILIZE_SECONDS="${UAV0_SIX_VISION_STABILIZE_SECONDS:-8}"
ODOM_TOPIC="/UAV0/fast_lio/Odom_high_freq"
PLANNER_HEARTBEAT_TOPIC="/drone_0_traj_server/heartbeat"
PANE=""

log() { printf '[uav0_six] %s\n' "$*"; }

usage() {
  cat <<'EOF'
用法：
  bash shfiles/start_uav0_six_terminator.sh
  bash shfiles/start_uav0_six_terminator.sh stop

六屏依次为：MAVROS、MID360、FAST-LIO、高频视觉位姿、FUEL-DIFF/RViz、
FUEL-DIFF bridge + AutoTrans NMPC + logger + rosbag。

脚本不自动解锁、不切换 OFFBOARD、不发送目标点。
EOF
}

source_ros() {
  [[ -r "${ROS_SETUP}" ]] || { log "找不到 ${ROS_SETUP}"; return 1; }
  [[ -r "${MATCH_WS}/devel/setup.bash" ]] || { log 'match_ws 尚未编译'; return 1; }
  set +u
  # shellcheck disable=SC1090,SC1091
  source "${ROS_SETUP}"
  source "${MATCH_WS}/devel/setup.bash"
  set -u
  unset ROS_LOG_DIR
}

keep_open() {
  printf '\n[uav0_six] 本分屏保持打开；按 Ctrl+C 停止运行中的节点。\n'
  exec "${SHELL:-/bin/bash}" -i
}

wait_for_topic() {
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

wait_for_mavros() {
  local elapsed=0
  while (( elapsed < WAIT_TIMEOUT )); do
    if timeout 2 rostopic echo -n 1 /UAV0/mavros/state 2>/dev/null |
        grep -F 'connected: True' >/dev/null 2>&1; then
      printf '[就绪] UAV0 MAVROS 已连接飞控\n'
      return 0
    fi
    if (( elapsed == 0 || elapsed % 10 == 0 )); then
      printf '[等待] UAV0 MAVROS 尚未连接（%ss/%ss）\n' "${elapsed}" "${WAIT_TIMEOUT}"
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  printf '[错误] 等待 UAV0 MAVROS 连接超时\n'
  return 1
}

run_existing_pane() {
  local pane_name="$1"
  export UAV0_FIRST_SEVEN_ODOM_TOPIC="${ODOM_TOPIC}"
  exec "${LEGACY_PANE_SCRIPT}" --pane "${pane_name}"
}

run_controller_pane() {
  printf '\n========== UAV0 第 6 屏：AutoTrans 控制器与日志 ==========\n'
  source_ros || keep_open
  wait_for_mavros || keep_open
  wait_for_topic "${ODOM_TOPIC}" || keep_open
  wait_for_topic /UAV0/mavros/local_position/odom || keep_open
  wait_for_topic "${PLANNER_HEARTBEAT_TOPIC}" || keep_open
  wait_for_topic /UAV0/mavros/vision_pose/pose || keep_open
  printf '[等待] PX4 外部视觉稳定融合 %ss\n' "${VISION_STABILIZE_SECONDS}"
  sleep "${VISION_STABILIZE_SECONDS}"
  wait_for_topic /UAV0/mavros/vision_pose/pose || keep_open

  printf '[启动] UAV0 FUEL-DIFF bridge、AutoTrans NMPC、logger 与同目录 rosbag\n'
  printf '[安全] 本屏不重复启动 FUEL，不自动解锁或切换 OFFBOARD。\n'
  roslaunch autotrans_reference_bridge uav0_autotrans_controller.launch
  printf '[退出] AutoTrans 分屏，返回码=%s\n' "$?"
  keep_open
}

case "${1:-}" in
  --pane)
    PANE="${2:-}"
    case "${PANE}" in
      mavros|mid360|fastlio|pose|planner) run_existing_pane "${PANE}" ;;
      controller) run_controller_pane ;;
      *) log "未知分屏：${PANE}"; exit 2 ;;
    esac
    ;;
  stop)
    if [[ -r "${PID_FILE}" ]]; then
      terminator_pid="$(tr -d '[:space:]' < "${PID_FILE}")"
      [[ -n "${terminator_pid}" ]] && kill "${terminator_pid}" 2>/dev/null || true
      rm -f -- "${PID_FILE}"
    fi
    exit 0
    ;;
  -h|--help) usage; exit 0 ;;
  '') ;;
  *) log "未知参数：$1"; usage; exit 2 ;;
esac

command -v terminator >/dev/null 2>&1 || { log '未安装 terminator'; exit 1; }
command -v timeout >/dev/null 2>&1 || { log '缺少 timeout 命令'; exit 1; }
[[ -x "${LEGACY_PANE_SCRIPT}" ]] || { log "找不到 ${LEGACY_PANE_SCRIPT}"; exit 1; }
[[ -r "${LAYOUT_CONFIG}" ]] || { log "找不到 ${LAYOUT_CONFIG}"; exit 1; }
[[ -n "${DISPLAY:-}" ]] || { log '没有 DISPLAY，请在 Ubuntu 图形桌面终端中运行'; exit 1; }

sed "s|__UAV0_SIX_SCRIPT__|${SCRIPT_PATH}|g" "${LAYOUT_CONFIG}" > "${RUNTIME_CONFIG}"
log '打开 UAV0 Terminator 六分屏'
log '上排：1 MAVROS | 2 MID360 | 3 FAST-LIO'
log '下排：4 视觉位姿 | 5 FUEL-DIFF/RViz | 6 AutoTrans/logger/rosbag'
nohup terminator --no-dbus --maximise --config="${RUNTIME_CONFIG}" --layout=uav0_six \
  >"${TERMINATOR_LOG}" 2>&1 &
terminator_pid=$!
printf '%s\n' "${terminator_pid}" > "${PID_FILE}"
log "Terminator 已启动，PID=${terminator_pid}"
