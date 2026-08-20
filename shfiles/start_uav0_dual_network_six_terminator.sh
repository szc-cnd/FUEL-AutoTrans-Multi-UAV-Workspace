#!/usr/bin/env bash

# 固定双机 ROS 网络后，调用当前 UAV0 六分屏入口。

set -e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
NETWORK_SETUP="${SCRIPT_DIR}/configure_dual_uav_ros_network.sh"
UAV0_LAUNCHER="${SCRIPT_DIR}/start_uav0_six_terminator.sh"

[[ -r "${NETWORK_SETUP}" ]] || { printf '找不到 %s\n' "${NETWORK_SETUP}" >&2; exit 1; }
[[ -x "${UAV0_LAUNCHER}" ]] || { printf '找不到 %s\n' "${UAV0_LAUNCHER}" >&2; exit 1; }

# shellcheck disable=SC1090
source "${NETWORK_SETUP}"
configure_dual_uav_ros_network uav0

# 双机搜索降落模式必须经 landing_setpoint_arbiter 转发控制指令；
# 原始单机六分屏不设置该变量，仍保持直接连接 MAVROS。
export UAV0_AUTOTRANS_SETPOINT_TOPIC="/UAV0/control/attitude_setpoint"

exec "${UAV0_LAUNCHER}" "$@"
