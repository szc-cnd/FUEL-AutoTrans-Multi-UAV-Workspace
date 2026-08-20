#!/bin/sh

# UAV1 传感器链路启动器：启动 MAVROS、MID360、FAST-LIO 或下视相机。
# 不启动控制器、不自动解锁、不切换 OFFBOARD，也不发布目标点。

set -eu

# 用户可能用 sh 调用；ROS setup.bash 需要 Bash 语法，自动切换到 Bash 执行本脚本。
if [ -z "${BASH_VERSION:-}" ]; then
    exec bash "$0" "$@"
fi

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
MATCH_WS=$(dirname -- "$SCRIPT_DIR")
WAIT_TIMEOUT=$(printenv WAIT_TIMEOUT 2>/dev/null || echo 45)

usage()
{
    cat <<'EOF'
用法：
  sh shfiles/run_uav1_sensor_stack.sh mavros
  sh shfiles/run_uav1_sensor_stack.sh mid360
  sh shfiles/run_uav1_sensor_stack.sh fastlio
  sh shfiles/run_uav1_sensor_stack.sh landing

说明：
  mavros  启动 /UAV1/mavros，并设置 IMU、姿态、里程计和 ESC 频率。
  mid360  启动 /UAV1/livox/lidar 和 /UAV1/livox/imu。
  fastlio 启动 /UAV1/fast_lio 下的 FAST-LIO 话题。
  landing 只启动 /UAV1/down_camera；搜索和精降节点由
          leader_safe_path_follower.launch 统一启动，避免重复节点。
EOF
}

fail()
{
    echo "[UAV1 sensor stack][错误] $*" >&2
    exit 1
}

source_ros()
{
    [ -r /opt/ros/noetic/setup.bash ] || fail "未找到 /opt/ros/noetic/setup.bash。"
    [ -r "$MATCH_WS/devel/setup.bash" ] || fail "未找到 $MATCH_WS/devel/setup.bash。"

    # 用 POSIX '.' 兼容用户现有的 sh 调用方式。
    # ROS setup 脚本可能访问尚未定义的变量，加载期间暂时关闭 nounset。
    set +u
    . /opt/ros/noetic/setup.bash
    . "$MATCH_WS/devel/setup.bash"
    set -u
}

wait_for_mavros()
{
    now=$(date +%s)
    deadline=$((now + WAIT_TIMEOUT))
    echo "[UAV1 MAVROS] 等待 /UAV1/mavros/state connected: True ..."

    while ! timeout 2 rostopic echo -n 1 /UAV1/mavros/state 2>/dev/null |
        grep -q 'connected: True'; do
        now=$(date +%s)
        [ "$now" -lt "$deadline" ] ||
            fail "等待 UAV1 MAVROS 连接超时。"
        sleep 1
    done
}

set_interval()
{
    message_id=$1
    interval_us=$2
    description=$3
    attempt=1

    while [ "$attempt" -le 3 ]; do
        echo "[UAV1 MAVROS] 设置 $description: $interval_us us（第 $attempt 次）"
        if rosrun mavros mavcmd -n /UAV1/mavros long 511 "$message_id" "$interval_us" 0 0 0 0 0; then
            return 0
        fi
        attempt=$((attempt + 1))
        sleep 1
    done

    fail "设置 $description 频率失败。"
}

run_mavros()
{
    mavros_pid=
    cleanup()
    {
        if [ -n "$mavros_pid" ] && kill -0 "$mavros_pid" 2>/dev/null; then
            echo "[UAV1 MAVROS] 正在关闭 MAVROS ..."
            kill -INT "$mavros_pid" 2>/dev/null || true
        fi
    }
    trap cleanup INT TERM EXIT

    roslaunch cxr_ego_ctrl mavros_uav.launch vehicle_ns:=UAV1 &
    mavros_pid=$!
    wait_for_mavros

    # MAV_CMD_SET_MESSAGE_INTERVAL(511) 的间隔单位为微秒。
    set_interval 105 5000 "HIGHRES_IMU 目标 200 Hz"
    set_interval 31 5000 "ATTITUDE_QUATERNION 目标 200 Hz"
    set_interval 32 10000 "LOCAL_POSITION_NED 目标 100 Hz"
    set_interval 291 5000 "ESC_STATUS 目标 200 Hz"

    echo "[UAV1 MAVROS] 频率请求已发送，保持当前终端显示 MAVROS 日志。"
    wait "$mavros_pid"
}

run_mid360()
{
    exec roslaunch livox_ros_driver2 msg_MID360.launch
}

run_fastlio()
{
    exec roslaunch fast_lio mapping_mid360.launch \
        rviz_goal_topic:=/UAV1/planning/goal
}

find_down_camera()
{
    device=$(find /dev/v4l/by-id -maxdepth 1 -type l \
        -name 'usb-Generic_USB_Camera_*-video-index0' 2>/dev/null |
        sort | head -n 1)
    [ -n "$device" ] ||
        fail "未发现下视相机：/dev/v4l/by-id/usb-Generic_USB_Camera_*-video-index0。"
    printf '%s\n' "$device"
}

run_landing()
{
    camera_device=$(find_down_camera)
    camera_info="$HOME/.ros/camera_info/down_camera.yaml"
    [ -r "$camera_info" ] || fail "未找到下视相机标定文件：$camera_info。"

    echo "[UAV1 Landing] 下视相机：$camera_device"
    echo "[UAV1 Landing] 仅启动下视相机；搜索和精降由后机接力入口启动。"
    exec roslaunch precision_landing landing_stack.launch \
        vehicle_ns:=UAV1 \
        video_device:="$camera_device" \
        camera_info_url:="file://$camera_info" \
        enable_precision_landing:=false
}

[ "$#" -eq 1 ] || {
    usage
    exit 1
}

source_ros

case "$1" in
    mavros)
        run_mavros
        ;;
    mid360)
        run_mid360
        ;;
    fastlio)
        run_fastlio
        ;;
    landing)
        run_landing
        ;;
    -h|--help)
        usage
        ;;
    *)
        usage >&2
        fail "未知子命令：$1"
        ;;
esac
