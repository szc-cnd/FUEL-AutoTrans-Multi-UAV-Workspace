#!/bin/bash
# ROS bag 录制脚本 - FUEL 探索实验
#
# 话题分类说明:
# ========== 规划可视化相关 ==========
#   /planning_vis/frontier
#   /planning_vis/viewpoints
#   /planning_vis/trajectory
#   /planning/travel_traj
#   /planning/position_cmd_vis
#   /planning/pos_cmd
#   /visualization_marker
#
# ========== 地图相关 ==========
#   /sdf_map/occupancy_local
#   /sdf_map/occupancy_all
#
# ========== 定位与路径 ==========
#   /vins_fusion/path
#   /converted_odom
#
# ========== 飞控状态 ==========
#   /mavros/battery
#   /mavros/state
#   /mavros/local_position/velocity_local
#   /mavros/imu/data_raw
#
# ========== 相机图像 ==========
#   /image_converter/output_video
#   /camera/depth/image_rect_raw
#   /camera/color/image_raw
#   /camera/infra1/image_rect_raw
#   /camera/infra2/image_rect_raw

rosbag record \
  /planning_vis/frontier \
  /planning_vis/viewpoints \
  /planning_vis/trajectory \
  /planning/travel_traj \
  /planning/position_cmd_vis \
  /planning/pos_cmd \
  /visualization_marker \
  /sdf_map/occupancy_local \
  /sdf_map/occupancy_all \
  /vins_fusion/path \
  /converted_odom \
  /mavros/battery \
  /mavros/state \
  /mavros/local_position/velocity_local \
  /mavros/imu/data_raw \
  /image_converter/output_video \
  /camera/depth/image_rect_raw \
  /camera/color/image_raw \
  /camera/infra1/image_rect_raw \
  /camera/infra2/image_rect_raw