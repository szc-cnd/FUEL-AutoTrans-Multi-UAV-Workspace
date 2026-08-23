#!/usr/bin/env bash

# 强制停止当前用户启动的 ROS1 节点、启动器和 UAV0 六分屏。
#
# 这个脚本故意不把 rosnode kill -a 作为唯一手段：ROS master 卡住或已经退出
# 时，rosnode 命令可能长时间等待，导致脚本看起来“没有反应”。AutoTrans
# logger 会先单独收到退出信号、关闭日志并启动独立 EVO 后处理；其余节点再按
# 原有策略清理。EVO 不属于强杀范围，脚本会等待其生成 summary.md。

set -u

CURRENT_USER="$(id -un)"
DRY_RUN=false
KEEP_TERMINATOR=false
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MATCH_WS="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PROTECTED_PIDS=()
EVO_REPORT_PIDS=()
EVO_REPORT_DIRS=()
LOGGER_STOP_TIMEOUT="${STOP_ALL_LOGGER_TIMEOUT:-12}"
EVO_WAIT_TIMEOUT="${STOP_ALL_EVO_TIMEOUT:-90}"

if [[ ! "${LOGGER_STOP_TIMEOUT}" =~ ^[0-9]+$ ]] || ((LOGGER_STOP_TIMEOUT < 1)); then
  printf '[stop_all_ros] STOP_ALL_LOGGER_TIMEOUT 必须是正整数秒\n' >&2
  exit 2
fi
if [[ ! "${EVO_WAIT_TIMEOUT}" =~ ^[0-9]+$ ]] || ((EVO_WAIT_TIMEOUT < 1)); then
  printf '[stop_all_ros] STOP_ALL_EVO_TIMEOUT 必须是正整数秒\n' >&2
  exit 2
fi

usage() {
  cat <<'EOF'
用法：
  bash shfiles/stop_all_ros.sh [选项]

作用：
  立即强制停止当前用户的 ROS 主节点、ROS 节点、RViz、视觉位姿脚本和
  UAV0 前六步 Terminator。AutoTrans logger 会先正常关闭日志，并自动等待
  独立 EVO 后处理生成 summary.md。不会自动降落、上锁或切换飞行模式。

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

add_unique_value() {
  local array_name="$1"
  local value="$2"
  local existing
  local -n target_array="${array_name}"
  for existing in "${target_array[@]}"; do
    [[ "${existing}" == "${value}" ]] && return 0
  done
  target_array+=("${value}")
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

is_evo_report_command() {
  local command="$1"
  [[ "${command}" =~ (^|[[:space:]/])generate_evo_report\.py([[:space:]]|$) ]]
}

track_evo_report() {
  local pid="$1"
  local arg previous=""
  local -a argv=()
  add_protected_pid "${pid}"
  add_unique_value EVO_REPORT_PIDS "${pid}"

  if [[ -r "/proc/${pid}/cmdline" ]]; then
    mapfile -d '' -t argv < "/proc/${pid}/cmdline" || true
    for arg in "${argv[@]}"; do
      if [[ "${previous}" == "--run-dir" ]] && [[ -n "${arg}" ]]; then
        add_unique_value EVO_REPORT_DIRS "${arg}"
        break
      fi
      previous="${arg}"
    done
  fi
}

collect_evo_reports() {
  local pid command
  while IFS= read -r pid; do
    [[ "${pid}" =~ ^[0-9]+$ ]] || continue
    command="$(ps -p "${pid}" -o args= 2>/dev/null | sed 's/^[[:space:]]*//')"
    is_evo_report_command "${command}" || continue
    track_evo_report "${pid}"
  done < <(matching_pids '(^|[[:space:]/])generate_evo_report\.py([[:space:]]|$)')
}

shutdown_autotrans_loggers() {
  local pid command
  local -a logger_pids=()
  while IFS= read -r pid; do
    [[ "${pid}" =~ ^[0-9]+$ ]] || continue
    is_protected_pid "${pid}" && continue
    logger_pids+=("${pid}")
    command="$(ps -p "${pid}" -o args= 2>/dev/null | sed 's/^[[:space:]]*//')"
    log "先正常关闭 AutoTrans logger：${pid} ${command}"
    if [[ "${DRY_RUN}" == false ]]; then
      kill -INT "${pid}" 2>/dev/null || true
    fi
  done < <(matching_pids '(^|[[:space:]/])autotrans_mpc_logger\.py([[:space:]]|$)')

  [[ "${DRY_RUN}" == false ]] || return 0
  ((${#logger_pids[@]} > 0)) || return 0

  local deadline=$((SECONDS + LOGGER_STOP_TIMEOUT))
  local running
  while ((SECONDS < deadline)); do
    running=false
    for pid in "${logger_pids[@]}"; do
      if kill -0 "${pid}" 2>/dev/null; then
        running=true
        break
      fi
    done
    [[ "${running}" == false ]] && break
    sleep 0.2
  done

  for pid in "${logger_pids[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      log "警告：logger ${pid} 在 ${LOGGER_STOP_TIMEOUT}s 内未完成收尾，稍后进入常规清理"
    fi
  done
  collect_evo_reports
}

wait_for_evo_reports() {
  ((${#EVO_REPORT_PIDS[@]} > 0)) || return 0
  local deadline=$((SECONDS + EVO_WAIT_TIMEOUT))
  local pid command running
  log "等待 EVO 后处理完成（最多 ${EVO_WAIT_TIMEOUT}s）"
  while ((SECONDS < deadline)); do
    running=false
    for pid in "${EVO_REPORT_PIDS[@]}"; do
      command="$(ps -p "${pid}" -o args= 2>/dev/null | sed 's/^[[:space:]]*//')"
      if [[ -n "${command}" ]] && is_evo_report_command "${command}"; then
        running=true
        break
      fi
    done
    [[ "${running}" == false ]] && break
    sleep 0.5
  done

  for pid in "${EVO_REPORT_PIDS[@]}"; do
    command="$(ps -p "${pid}" -o args= 2>/dev/null | sed 's/^[[:space:]]*//')"
    if [[ -n "${command}" ]] && is_evo_report_command "${command}"; then
      log "警告：EVO ${pid} 超过等待时间，已保留进程继续运行，不会强杀"
    fi
  done

  local run_dir summary
  for run_dir in "${EVO_REPORT_DIRS[@]}"; do
    summary="${run_dir}/evo_report/summary.md"
    if [[ -s "${summary}" ]]; then
      log "EVO 分析已生成：${summary}"
    else
      log "警告：尚未生成 ${summary}，请检查该目录下的 postprocess.log/report.log"
    fi
  done
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
    command="$(ps -p "${pid}" -o args= 2>/dev/null | sed 's/^[[:space:]]*//')"
    if is_evo_report_command "${command}"; then
      track_evo_report "${pid}"
      log "保留 EVO 后处理：${pid} ${command}"
      continue
    fi
    found=true
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
  # Terminator 或 rosnode 全量退出都可能提前终止 logger，必须先单独让它完成
  # rosbag/CSV 收尾并启动脱离 ROS 进程组的 EVO。
  shutdown_autotrans_loggers
  close_uav0_terminator

  # ROS master 正常时先通知节点退出；最多等待 2 秒，绝不让停止脚本卡住。
  if command -v rosnode >/dev/null 2>&1 && command -v timeout >/dev/null 2>&1; then
    log '尝试通知 ROS 节点退出（最多等待 2 秒）'
    timeout 2s rosnode kill -a >/dev/null 2>&1 || true
  fi
  collect_evo_reports
else
  shutdown_autotrans_loggers
  collect_evo_reports
fi

# 第一组覆盖 ROS 启动器和 master；第二组覆盖 ROS 二进制、RViz 以及本工程的
# Python 节点。工作空间路径由当前脚本位置和当前用户 HOME 自动确定。
kill_pattern 'ROS 启动器和 master' \
  '(^|[[:space:]\/])(roslaunch|rosrun|roscore|rosmaster)([[:space:]]|$)|roslaunch\.parent|rosmaster\.master'
kill_pattern '系统 ROS 节点与 RViz' \
  '/opt/ros/noetic/(lib|bin)/'
kill_pattern '视觉位姿回传' \
  '(^|[[:space:]\/])laser_mid360\.py([[:space:]]|$)'
kill_pattern 'match_ws ROS 节点' \
  "${MATCH_WS}/(devel|build)/lib/|${MATCH_WS}/src/[^[:space:]]+\\.py([[:space:]]|$)"
kill_pattern 'db_ws ROS 节点' \
  "${HOME}/db_ws/(devel|build)/lib/|${HOME}/db_ws/src/[^[:space:]]+\\.py([[:space:]]|$)"
kill_pattern 'rh_ws ROS 节点' \
  "${HOME}/rh_ws/(devel|build)/lib/|${HOME}/rh_ws/src/[^[:space:]]+\\.py([[:space:]]|$)"

if [[ "${DRY_RUN}" == true ]]; then
  log 'dry-run 完成，未发送任何信号'
else
  collect_evo_reports
  wait_for_evo_reports
  log '所有可识别的 ROS 进程已强制终止'
fi
