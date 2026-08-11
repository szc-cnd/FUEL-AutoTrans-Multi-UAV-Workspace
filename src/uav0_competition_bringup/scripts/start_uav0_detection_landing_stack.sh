#!/usr/bin/env bash

set -e

# 兼容从源码目录直接执行和通过 rosrun 执行（devel/lib 下的安装副本）。
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="$(cd "$script_dir/../../.." && pwd)"
log_root="${UAV0_LOG_ROOT:-$workspace_root/logs/uav0}"
run_id="$(date +%Y%m%d_%H%M%S_%N)"
run_log_dir="$log_root/$run_id"

mkdir -p "$run_log_dir"

if [ -f /opt/ros/noetic/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
fi
if [ -f "$workspace_root/devel/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$workspace_root/devel/setup.bash"
fi

export ROS_LOG_DIR="$run_log_dir"
echo "UAV0 本次运行日志：$ROS_LOG_DIR"

exec roslaunch uav0_competition_bringup \
  uav0_detection_landing_stack.launch "$@"
