#!/bin/bash

# 确保脚本在出错时退出
set -e

# 定义工作空间路径
ZYC_FUEL_WS="$HOME/zyc_fuel_ws"
SHIYAN_WS="$HOME/shiyan_catkin_ws_target"

# 打开第一个终端并执行命令
gnome-terminal -- bash -c "cd $ZYC_FUEL_WS && pwd && source devel/setup.bash && roslaunch exploration_manager exploration.launch; exec bash"

# 等待30秒
sleep 5

# 打开第二个终端并执行命令（带重试机制）
gnome-terminal -- bash -c "cd $SHIYAN_WS && pwd && source devel/setup.bash && until roslaunch yolo_detector yolo_ros.launch; do echo 'Command roslaunch yolo_detector yolo_ros.launch failed, retrying...'; sleep 2; done; exec bash"

# 等待30秒
sleep 3

# 打开第三个终端并执行命令
gnome-terminal -- bash -c "cd $SHIYAN_WS && pwd && source devel/setup.bash && roslaunch sort_ros sort_ros.launch; exec bash"

# 等待30秒
sleep 3

# 打开第四个终端并执行命令
gnome-terminal -- bash -c "cd $ZYC_FUEL_WS/scripts && pwd && source $ZYC_FUEL_WS/devel/setup.bash && python3 marker_wenzi_jisuan.py; exec bash"

# # 等待30秒
# sleep 5

# # 打开第五个终端并执行命令
# gnome-terminal -- bash -c "cd $ZYC_FUEL_WS && pwd && source devel/setup.bash && rosrun exploration_manager fuel_nav; exec bash"