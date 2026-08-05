#!/usr/bin/env bash

set -Eeuo pipefail

# 传感器/MAVROS 已由基础启动脚本运行后，启动 Diff-Planner、bridge 和 AutoTrans。
# 本脚本不自动解锁、不切换 OFFBOARD；CH6/QGC/PX4 负责飞控模式，CH8/CH10 负责 AutoTrans 状态。
source /opt/ros/noetic/setup.bash
# 先加载 AutoTrans，再加载 Match；最后加载的 traj_utils 才是 PolyTraj 规划消息版本。
source "${AUTOTRANS_WS:-$HOME/autotrans_quad_wind_ws}/devel/setup.bash"
source "${MATCH_WS:-$HOME/match_ws_autotrans_dev}/devel/setup.bash" --extend

exec roslaunch autotrans_reference_bridge uav1_diff_autotrans.launch "$@"
