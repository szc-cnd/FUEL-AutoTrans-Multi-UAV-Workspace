#!/usr/bin/env bash

# 停止当前用户启动的全部 ROS1 节点及 ROS 启动器。
# 使用前请确认无人机已经落地、退出 OFFBOARD、上锁；本脚本不会执行降落。

set -o pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MATCH_WS="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="${STOP_ALL_ROS_SETUP:-/opt/ros/noetic/setup.bash}"
FORCE=false
DRY_RUN=false
KEEP_TERMINATOR=false
TARGET_PIDS=()

usage() {
  cat <<'EOF'
用法：
  bash shfiles/stop_all_ros.sh [选项]

作用：
  先通过 rosnode kill -a 优雅关闭当前 ROS master 中的全部节点，
  再清理当前用户残留的 roslaunch、roscore、rosmaster、rosrun、RViz
  等 ROS 启动进程。默认也会关闭本仓库六分屏入口打开的 Terminator 窗口。

选项：
  --force             等待后仍未退出的目标进程发送 SIGKILL
  --dry-run           只显示将要处理的进程，不发送任何信号
  --keep-terminator   不关闭 start_uav0_first_six_terminator.sh 打开的窗口
  -h, --help          显示帮助

安全提示：
  本脚本不会自动降落、上锁或切换飞行模式。执行前必须确认飞行器处于安全状态。
  默认只处理当前用户的进程；使用 sudo 启动的其他用户进程不会被本脚本清理。
EOF
}

log() {
  printf '[stop_all_ros] %s\n' "$*"
}

source_ros_environment() {
  if [[ ! -f "${ROS_SETUP}" ]]; then
    log "找不到 ROS 环境文件：${ROS_SETUP}，跳过 rosnode kill"
    return 1
  fi

  set +u
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
  if [[ -f "${MATCH_WS}/devel/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${MATCH_WS}/devel/setup.bash"
  fi
  set -u
  return 0
}

add_pid() {
  local pid="$1" existing
  [[ "${pid}" =~ ^[0-9]+$ ]] || return 0
  [[ "${pid}" -gt 1 && "${pid}" -ne "$$" ]] || return 0
  for existing in "${TARGET_PIDS[@]}"; do
    [[ "${existing}" == "${pid}" ]] && return 0
  done
  TARGET_PIDS+=("${pid}")
}

collect_node_pids() {
  local node pid
  if ! command -v rosnode >/dev/null 2>&1; then
    log '找不到 rosnode，无法读取 ROS 节点 PID'
    return 0
  fi
  if ! rosnode list >/dev/null 2>&1; then
    log 'ROS master 当前不可访问，跳过节点 PID 查询'
    return 0
  fi

  while IFS= read -r node; do
    [[ -n "${node}" ]] || continue
    pid="$(timeout 5s rosnode info "${node}" 2>/dev/null | awk '/^[[:space:]]*Pid:[[:space:]]*[0-9]+/{print $2; exit}')"
    add_pid "${pid}"
  done < <(rosnode list 2>/dev/null)
}

collect_ros_launcher_pids() {
  local pid
  # 节点已经由 rosnode kill -a 请求退出；这里清理可能仍在等待子进程的启动器。
  while IFS= read -r pid; do
    add_pid "${pid}"
  done < <(
    ps -eo pid=,user=,args= | awk -v current_user="$(id -un)" -v self_pid="$$" '
      $1 != self_pid && $2 == current_user {
        command = $0
        sub(/^[[:space:]]*[0-9]+[[:space:]]+[^[:space:]]+[[:space:]]+/, "", command)
        if (command ~ /(^|[[:space:]\/])ros(launch|core|run|node|topic|service|param|bag)([[:space:]]|$)/ ||
            command ~ /\/opt\/ros\/noetic\/bin\/ros(launch|core|run|node|topic|service|param|bag)([[:space:]]|$)/ ||
            command ~ /\/opt\/ros\/noetic\/(lib|share)\/(rosmaster|roslaunch|rosout)(\/|[[:space:]]|$)/ ||
            command ~ /\/opt\/ros\/noetic\/lib\/(rviz|rqt|gazebo_ros)(\/|[[:space:]]|$)/ ||
            command ~ /(^|[[:space:]\/])(rosmaster\.master|roslaunch\.parent)([[:space:]]|$)/)
          print $1
      }
    '
  )
}

describe_pid() {
  local pid="$1"
  ps -p "${pid}" -o pid=,user=,args= 2>/dev/null | sed 's/^[[:space:]]*//'
}

remove_exited_pids() {
  local kept=() pid
  for pid in "${TARGET_PIDS[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kept+=("${pid}")
    fi
  done
  TARGET_PIDS=("${kept[@]}")
}

signal_targets() {
  local signal="$1" pid
  remove_exited_pids
  [[ "${#TARGET_PIDS[@]}" -gt 0 ]] || return 0
  log "发送 ${signal} 到 ${#TARGET_PIDS[@]} 个残留进程"
  for pid in "${TARGET_PIDS[@]}"; do
    if kill "-${signal}" "${pid}" 2>/dev/null; then
      log "${signal}: $(describe_pid "${pid}")"
    fi
  done
}

close_uav0_terminator() {
  if [[ "${KEEP_TERMINATOR}" == true ]]; then
    return 0
  fi
  local launcher="${MATCH_WS}/shfiles/start_uav0_first_six_terminator.sh"
  if [[ -x "${launcher}" ]]; then
    log '关闭 UAV0 前六步 Terminator 窗口（不代替 ROS 节点清理）'
    "${launcher}" stop >/dev/null 2>&1 || true
  fi
}

while (($# > 0)); do
  case "$1" in
    --force)
      FORCE=true
      shift
      ;;
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

if ! source_ros_environment; then
  # 没有 ROS setup 时仍继续清理可识别的启动器进程。
  log '继续执行残留进程扫描'
fi

collect_node_pids

if [[ "${DRY_RUN}" == true ]]; then
  collect_ros_launcher_pids
  remove_exited_pids
  if [[ "${#TARGET_PIDS[@]}" -eq 0 ]]; then
    log '未发现可清理的 ROS 进程'
  else
    log "dry-run：发现 ${#TARGET_PIDS[@]} 个 ROS 相关进程（不会发送信号）"
    for pid in "${TARGET_PIDS[@]}"; do
      log "候选：$(describe_pid "${pid}")"
    done
  fi
  exit 0
fi

if command -v rosnode >/dev/null 2>&1 && rosnode list >/dev/null 2>&1; then
  log '请求 ROS master 关闭全部节点'
  rosnode kill -a >/dev/null 2>&1 || log 'rosnode kill -a 返回非零，继续清理残留进程'
else
  log '未检测到可访问的 ROS master，直接清理启动器和已发现进程'
fi

close_uav0_terminator
sleep 2
collect_ros_launcher_pids
signal_targets INT
sleep 3
signal_targets TERM

if [[ "${FORCE}" == true ]]; then
  sleep 2
  signal_targets KILL
else
  remove_exited_pids
  if [[ "${#TARGET_PIDS[@]}" -gt 0 ]]; then
    log '仍有进程未退出；如确认安全，可重新执行并加 --force'
    for pid in "${TARGET_PIDS[@]}"; do
      log "未退出：$(describe_pid "${pid}")"
    done
  fi
fi

remove_exited_pids
if [[ "${#TARGET_PIDS[@]}" -eq 0 ]]; then
  log 'ROS 节点和可识别的 ROS 启动进程已停止'
else
  log "清理结束，仍存活进程数：${#TARGET_PIDS[@]}"
fi
