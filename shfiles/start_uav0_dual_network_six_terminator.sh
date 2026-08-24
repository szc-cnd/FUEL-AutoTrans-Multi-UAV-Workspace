#!/usr/bin/env bash

# 固定双机 ROS 网络后，调用当前 UAV0 七分屏入口。

set -e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
NETWORK_SETUP="${SCRIPT_DIR}/configure_dual_uav_ros_network.sh"
UAV0_LAUNCHER="${SCRIPT_DIR}/start_uav0_six_terminator.sh"

[[ -r "${NETWORK_SETUP}" ]] || { printf '找不到 %s\n' "${NETWORK_SETUP}" >&2; exit 1; }
[[ -x "${UAV0_LAUNCHER}" ]] || { printf '找不到 %s\n' "${UAV0_LAUNCHER}" >&2; exit 1; }

# shellcheck disable=SC1090
source "${NETWORK_SETUP}"
configure_dual_uav_ros_network uav0

# 双机检测栈会启动降落控制仲裁器。正常阶段由它透传 AutoTrans 姿态指令；
# 精降触发后停止透传姿态指令，只向 PX4 转发精降 PositionTarget。
export UAV0_AUTOTRANS_SETPOINT_TOPIC="/UAV0/control/attitude_setpoint"

exec "${UAV0_LAUNCHER}" "$@"
