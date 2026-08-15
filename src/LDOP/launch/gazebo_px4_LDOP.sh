#!/bin/sh

launch_pids=""

start_launch() {
  # 每个 roslaunch 单独开一个 session，退出时才能按进程组清理其子进程。
  setsid "$@" &
  pid=$!
  launch_pids="$launch_pids $pid"
}

cleanup() {
  status=$?
  trap - INT TERM EXIT

  echo "Stopping launched roslaunch processes..."
  for pid in $launch_pids; do
    # 先发 SIGINT，让 roslaunch 按 Ctrl-C 路径优雅关闭 Gazebo/PX4/MAVROS。
    kill -INT -"$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
  done

  sleep 2

  for pid in $launch_pids; do
    # 兜底清理仍未退出的进程组，避免仿真相关子进程残留。
    kill -TERM -"$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  done

  wait 2>/dev/null || true
  exit "$status"
}

trap cleanup INT TERM EXIT

# PX4 的 interactive shell 不能放到后台运行；后台启动时用 daemon 模式避免 sitl 读到 EOF 后退出。
start_launch roslaunch ldop px4.launch interactive:=false
sleep 20

start_launch roslaunch external_pos_fusion fusion_mapping_mid360.launch
sleep 5

# Gazebo ground truth 桥接：将 p3d 插件的 Odometry 转为 PoseStamped
start_launch roslaunch ground_truth_bridge gt_bridge.launch
sleep 1

start_launch roslaunch ldop run_ldop.launch
wait
