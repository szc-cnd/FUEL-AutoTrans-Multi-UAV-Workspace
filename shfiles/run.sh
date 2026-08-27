#!/bin/sh

# Resolve match_ws from this script's location so the script works regardless
# of the caller's current directory, and expose workspace launch files to ROS.
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
MATCH_WS=$(dirname -- "$SCRIPT_DIR")
. "$MATCH_WS/devel/setup.sh"

FCU_DEVICE=/dev/ttyACM0
if [ ! -e "$FCU_DEVICE" ]; then
    echo "[UAV0 MAVROS][错误] 未发现飞控串口：$FCU_DEVICE"
    exit 1
fi
if [ ! -r "$FCU_DEVICE" ] || [ ! -w "$FCU_DEVICE" ]; then
    echo "[UAV0 MAVROS][错误] 当前用户无权读写 $FCU_DEVICE；请确认用户属于 dialout 组。"
    exit 1
fi

roslaunch cxr_ego_ctrl mavros_uav.launch vehicle_ns:=UAV0 &

# 等待 UAV0 MAVROS 的 command service 和 FCU 连接可用，避免串口尚未连接时频率请求失败。
wait_for_mavros_command() {
    i=1
    while [ "$i" -le 60 ]; do
        if rosservice list 2>/dev/null | grep -qx "/UAV0/mavros/cmd/command" && \
           timeout 2 rostopic echo -n 1 /UAV0/mavros/state 2>/dev/null | grep -q "connected: True"; then
            return 0
        fi
        sleep 1
        i=$((i + 1))
    done
    return 1
}

# 通过 MAV_CMD_SET_MESSAGE_INTERVAL 请求 PX4 发布指定 MAVLink 消息的周期。
# period_us 是消息周期，单位 us；对应频率为 1e6 / period_us。
set_message_interval() {
    message_name="$1"
    message_id="$2"
    period_us="$3"
    attempt=1

    while [ "$attempt" -le 3 ]; do
        echo "[UAV0 MAVROS] 设置 ${message_name} 目标频率: ${period_us} us（第 ${attempt} 次）"
        if rosrun mavros mavcmd -n /UAV0/mavros long 511 "$message_id" "$period_us" 0 0 0 0 0; then
            return 0
        fi
        sleep 1
        attempt=$((attempt + 1))
    done
    return 1
}

if wait_for_mavros_command; then
    set_message_interval "HIGHRES_IMU" 105 5000 || echo "[UAV0 MAVROS][错误] HIGHRES_IMU 频率请求失败。"
    set_message_interval "ATTITUDE_QUATERNION" 31 5000 || echo "[UAV0 MAVROS][错误] ATTITUDE_QUATERNION 频率请求失败。"
    set_message_interval "LOCAL_POSITION_NED" 32 10000 || echo "[UAV0 MAVROS][错误] LOCAL_POSITION_NED 频率请求失败。"
    set_message_interval "ESC_STATUS" 291 5000 || echo "[UAV0 MAVROS][错误] ESC_STATUS 频率请求失败。"
else
    echo "[UAV0 MAVROS][错误] 等待 /UAV0/mavros/cmd/command 或 connected: True 超时，未发送频率请求。"
fi

wait
