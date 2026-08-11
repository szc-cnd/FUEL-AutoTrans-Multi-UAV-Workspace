#!/usr/bin/env bash

set -e

# 兼容从源码目录直接执行和通过 rosrun 执行（devel/lib 下的安装副本）。
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="$(cd "$script_dir/../../.." && pwd)"

if [ -f /opt/ros/noetic/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
fi
if [ -f "$workspace_root/devel/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$workspace_root/devel/setup.bash"
fi

# 检测统一入口与规划器保持一致，使用 ROS 默认的 ~/.ros/log/。
# 清除上层 shell 可能遗留的自定义目录设置，避免日志位置不一致。
unset ROS_LOG_DIR

exec roslaunch uav0_competition_bringup \
  uav0_detection_landing_stack.launch "$@"
