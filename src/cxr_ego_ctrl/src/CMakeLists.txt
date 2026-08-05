cmake_minimum_required(VERSION 3.0.2)
project(cxr_ego_ctrl)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  rospy
  std_msgs
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS geometry_msgs mavros_msgs rospy std_msgs
)

## Install Python scripts
catkin_install_python(PROGRAMS
  # 2026-07-22: 安装双机统一起飞脚本，日常比赛不再分别运行两个 ego_start.py。
  dual_ego_start.py
  ego_start.py
  laser_mid360.py
  # 2026-07-22: 去掉错误转义，使 catkin 展开真实包目录并生成可用的 rosrun 包装器。
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
