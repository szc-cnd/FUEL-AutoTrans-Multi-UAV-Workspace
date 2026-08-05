#!/bin/bash

# 确保脚本在出错时退出
set -e

# 定义工作空间路径
FAST_DRONE_WS="$HOME/Fast-Drone-250"
ZYC_FUEL_WS="$HOME/zyc_fuel_ws"

# 打开第一个终端并执行命令
gnome-terminal -- bash -c "cd $FAST_DRONE_WS && pwd && source devel/setup.bash && sh shfiles/run.sh; exec bash"

# 等待30秒
sleep 30

# 打开第二个终端并执行命令
gnome-terminal -- bash -c "cd $ZYC_FUEL_WS && pwd && source devel/setup.bash && rosrun vins_to_mavros vins_to_mavros_node; exec bash"

# 等待30秒
sleep 3

# 打开第三个终端并执行命令
gnome-terminal -- bash -c "cd $ZYC_FUEL_WS && pwd && source devel/setup.bash && rosrun pose_to_odom_converter pose_to_odom_converter_node; exec bash"

sleep 2 
gnome-terminal -- bash -c "rostopic echo /converted_odom"




