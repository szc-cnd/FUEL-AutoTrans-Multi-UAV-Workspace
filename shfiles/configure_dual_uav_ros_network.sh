#!/usr/bin/env bash

# 双机 ROS1 网络环境。由启动脚本 source，不单独启动 ROS 节点。

configure_dual_uav_ros_network() {
  local role="${1:-}"
  local master_ip="${DUAL_UAV_ROS_MASTER_IP:-10.32.24.212}"
  local local_ip

  case "${role}" in
    uav0) local_ip="${UAV0_ROS_IP:-10.32.24.212}" ;;
    uav1) local_ip="${UAV1_ROS_IP:-10.32.24.232}" ;;
    *)
      printf '[dual_uav_ros][错误] 角色必须是 uav0 或 uav1\n' >&2
      return 2
      ;;
  esac

  export ROS_MASTER_URI="http://${master_ip}:11311"
  export ROS_IP="${local_ip}"
  unset ROS_HOSTNAME

  printf '[dual_uav_ros] role=%s master=%s local_ip=%s\n' \
    "${role}" "${ROS_MASTER_URI}" "${ROS_IP}"
}
