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

# 当前六分屏运行纯FUEL且不启动 landing_setpoint_arbiter，控制器必须直连MAVROS。
# 显式覆盖该变量，避免父终端遗留的搜索降落配置截断控制指令。
export UAV0_AUTOTRANS_SETPOINT_TOPIC="/UAV0/mavros/setpoint_raw/attitude"

exec "${UAV0_LAUNCHER}" "$@"
