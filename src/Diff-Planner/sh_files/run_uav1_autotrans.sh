#!/usr/bin/env bash

set -Eeuo pipefail

# 传感器/MAVROS 已由基础启动脚本运行后，启动 Diff-Planner、bridge 和 AutoTrans。
# 本脚本不自动解锁、不切换 OFFBOARD；CH6/QGC/PX4 负责飞控模式，CH8/CH10 负责 AutoTrans 状态。
source /opt/ros/noetic/setup.bash
source "${MATCH_WS:-$HOME/match_ws}/devel/setup.bash"

exec roslaunch autotrans_reference_bridge uav1_diff_autotrans.launch "$@"
