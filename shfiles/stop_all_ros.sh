#!/usr/bin/env bash

# 强制停止当前用户启动的 ROS1 节点、启动器和 UAV0 六分屏。
#
# 这个脚本故意不把 rosnode kill -a 作为唯一手段：ROS master 卡住或已经退出
# 时，rosnode 命令可能长时间等待，导致脚本看起来“没有反应”。先用一个很短
# 的超时尝试优雅通知，然后直接按进程特征 SIGKILL，行为与 /home/oem/scripts/
# ros_stop_all.sh 保持一致。

set -u

CURRENT_USER="$(id -un)"
DRY_RUN=false
KEEP_TERMINATOR=false
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MATCH_WS="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PROTECTED_PIDS=()

usage() {
  cat <<'EOF'
用法：
  bash shfiles/stop_all_ros.sh [选项]

作用：
  立即强制停止当前用户的 ROS 主节点、ROS 节点、RViz、视觉位姿脚本和
  UAV0 前六步 Terminator。不会自动降落、上锁或切换飞行模式。

选项：
  --dry-run           只显示将清理的进程，不发送信号
  --keep-terminator   保留 UAV0 前六步 Terminator 窗口
  -h, --help          显示帮助
EOF
}

log() {
  printf '[stop_all_ros] %s\n' "$*"
}

close_uav0_terminator() {
  [[ "${KEEP_TERMINATOR}" == true ]] && return 0
  local launcher="${MATCH_WS}/shfiles/start_uav0_first_six_terminator.sh"
  if [[ -x "${launcher}" ]]; then
    log '关闭 UAV0 前六步 Terminator 窗口'
    # stop 分支只处理窗口 PID 文件，不依赖 ROS master，也不会等待节点退出。
    "${launcher}" stop >/dev/null 2>&1 || true
  fi
}

add_protected_pid() {
  local pid="$1" existing
  [[ "${pid}" =~ ^[0-9]+$ ]] || return 0
  for existing in "${PROTECTED_PIDS[@]}"; do
    [[ "${existing}" == "${pid}" ]] && return 0
  done
  PROTECTED_PIDS+=("${pid}")
}

collect_protected_pids() {
  local pid parent
  # 不允许脚本误杀当前终端及其祖先进程。直接 pkill -f 在调用命令本身
  # 包含 roslaunch 字样时会把父 shell 一并杀掉，这是停止脚本最容易出现的假死原因。
  add_protected_pid "$$"
  parent="${PPID}"
  while [[ "${parent}" =~ ^[0-9]+$ ]] && [[ "${parent}" -gt 1 ]]; do
    add_protected_pid "${parent}"
    parent="$(ps -o ppid= -p "${parent}" 2>/dev/null | tr -d '[:space:]')"
  done
}

is_protected_pid() {
  local pid="$1" protected
  for protected in "${PROTECTED_PIDS[@]}"; do
    [[ "${protected}" == "${pid}" ]] && return 0
  done
  return 1
}

matching_pids() {
  local pattern="$1"
  pgrep -u "${CURRENT_USER}" -f "${pattern}" 2>/dev/null || true
}

kill_pattern() {
  local description="$1"
  local pattern="$2"
  local pid command found=false

  while IFS= read -r pid; do
    [[ "${pid}" =~ ^[0-9]+$ ]] || continue
    if is_protected_pid "${pid}"; then
      continue
    fi
    found=true
    command="$(ps -p "${pid}" -o args= 2>/dev/null | sed 's/^[[:space:]]*//')"
    log "${description}：${pid} ${command}"
    if [[ "${DRY_RUN}" == false ]]; then
      kill -KILL "${pid}" 2>/dev/null || true
    fi
  done < <(matching_pids "${pattern}")

  [[ "${found}" == true ]] || return 0
}

while (($# > 0)); do
  case "$1" in
    --dry-run)
      DRY_RUN=true
      shift
      ;;
    --keep-terminator)
      KEEP_TERMINATOR=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      log "未知参数：$1"
      usage
      exit 2
      ;;
  esac
done

collect_protected_pids

if [[ "${DRY_RUN}" == false ]]; then
  close_uav0_terminator

  # ROS master 正常时先通知节点退出；最多等待 2 秒，绝不让停止脚本卡住。
  if command -v rosnode >/dev/null 2>&1 && command -v timeout >/dev/null 2>&1; then
    log '尝试通知 ROS 节点退出（最多等待 2 秒）'
    timeout 2s rosnode kill -a >/dev/null 2>&1 || true
  fi
fi

# 参考 /home/oem/scripts/ros_stop_all.sh 的直接强制清理方式。第一组覆盖
# ROS 启动器和 master；第二组覆盖 ROS 二进制、RViz 以及本工程的 Python 节点。
kill_pattern 'ROS 启动器和 master' \
  '(^|[[:space:]\/])(roslaunch|rosrun|roscore|rosmaster)([[:space:]]|$)|roslaunch\.parent|rosmaster\.master'
kill_pattern '系统 ROS 节点与 RViz' \
  '/opt/ros/noetic/(lib|bin)/'
kill_pattern '视觉位姿回传' \
  '(^|[[:space:]\/])laser_mid360\.py([[:space:]]|$)'
kill_pattern 'match_ws ROS 节点' \
  '/home/oem/match_ws/(devel|build)/lib/|/home/oem/match_ws/src/[^[:space:]]+\.py([[:space:]]|$)'
kill_pattern 'db_ws ROS 节点' \
  '/home/oem/db_ws/(devel|build)/lib/|/home/oem/db_ws/src/[^[:space:]]+\.py([[:space:]]|$)'
kill_pattern 'rh_ws ROS 节点' \
  '/home/oem/rh_ws/(devel|build)/lib/|/home/oem/rh_ws/src/[^[:space:]]+\.py([[:space:]]|$)'

if [[ "${DRY_RUN}" == true ]]; then
  log 'dry-run 完成，未发送任何信号'
else
  log '所有可识别的 ROS 进程已强制终止'
fi
