#!/usr/bin/env bash
set -eo pipefail

ROLE="${1:-}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_WORKSPACE="${ROS_WORKSPACE:-$HOME/rh_ws}"
ACTIVE_PID=""

load_ros_environment() {
    if [[ -f "${ROS_WORKSPACE}/devel/setup.bash" ]]; then
        # shellcheck disable=SC1091
        source "${ROS_WORKSPACE}/devel/setup.bash"
    elif [[ -f "${HOME}/ldop_ws/ldop_env.sh" ]]; then
        # shellcheck disable=SC1091
        source "${HOME}/ldop_ws/ldop_env.sh"
    else
        return 1
    fi
}

node_is_running() {
    local node="$1"
    rosnode list 2>/dev/null | grep -Eq "^/${node}($|/)"
}

topic_has_message() {
    local topic="$1"
    timeout 3s rostopic echo -n 1 "${topic}" >/dev/null 2>&1
}

wait_for_topics_or_fail() {
    local topic
    local -a pending
    while true; do
        pending=()
        for topic in "$@"; do
            if ! topic_has_message "${topic}"; then
                pending+=("${topic}")
            fi
        done

        if [[ ${#pending[@]} -eq 0 ]]; then
            return 0
        fi

        if [[ -n "${ACTIVE_PID}" ]] && ! kill -0 "${ACTIVE_PID}" 2>/dev/null; then
            local status=1
            if wait "${ACTIVE_PID}"; then
                status=1
            else
                status=$?
            fi
            return 1
        fi

        sleep 1
    done
}

start_or_skip_node() {
    local node="$1"
    shift
    if node_is_running "${node}"; then
        ACTIVE_PID=""
        return 0
    fi

    "$@" &
    ACTIVE_PID=$!
}

finish_node_role() {
    if ! wait_for_topics_or_fail "$@"; then
        ACTIVE_PID=""
        exec bash
    fi

    if [[ -n "${ACTIVE_PID}" ]]; then
        local status=0
        if wait "${ACTIVE_PID}"; then
            status=0
        else
            status=$?
        fi
        ACTIVE_PID=""
    fi
    exec bash
}

cleanup_active_process() {
    if [[ -n "${ACTIVE_PID}" ]] && kill -0 "${ACTIVE_PID}" 2>/dev/null; then
        kill -TERM "${ACTIVE_PID}" 2>/dev/null || true
        wait "${ACTIVE_PID}" 2>/dev/null || true
    fi
}

logger_pid_file() {
    echo "${LDOP_LOG_ROOT:-$HOME/ldop_logs}/.flight_logger.pid"
}

logger_is_running() {
    local pid_file
    local pid
    pid_file="$(logger_pid_file)"
    [[ -f "${pid_file}" ]] || return 1
    pid="$(cat "${pid_file}" 2>/dev/null || true)"
    if [[ "${pid}" =~ ^[0-9]+$ ]] && kill -0 "${pid}" 2>/dev/null; then
        return 0
    fi
    rosnode list 2>/dev/null | grep -Eq '^/ldop_flight_logger(_|$)'
}

wait_for_logger_or_fail() {
    while true; do
        if logger_is_running; then
            return 0
        fi
        if [[ -n "${ACTIVE_PID}" ]] && ! kill -0 "${ACTIVE_PID}" 2>/dev/null; then
            return 1
        fi
        sleep 1
    done
}

start_livox() {
    load_ros_environment
    cd "${ROS_WORKSPACE}"
    start_or_skip_node livox_lidar_publisher2 roslaunch livox_ros_driver2 msg_MID360.launch
    finish_node_role /livox/lidar /livox/imu
}

start_mavros() {
    load_ros_environment
    cd "${ROS_WORKSPACE}"
    wait_for_topics_or_fail /livox/lidar /livox/imu
    local gcs_url="${GCS_URL:-udp://@192.168.31.212:14550}"
    start_or_skip_node mavros roslaunch mavros px4.launch \
        fcu_url:=/dev/ttyACM0:921600 \
        "gcs_url:=${gcs_url}"
    finish_node_role /mavros/state
}

start_fastlio() {
    load_ros_environment
    cd "${ROS_WORKSPACE}"
    wait_for_topics_or_fail /livox/lidar /livox/imu /mavros/state
    start_or_skip_node laserMapping roslaunch fast_lio mapping_mid360.launch rviz:=false
    finish_node_role /cloud_registered /Odometry
}

start_fusion() {
    load_ros_environment
    cd "${ROS_WORKSPACE}"
    wait_for_topics_or_fail /cloud_registered /Odometry /mavros/state
    start_or_skip_node slam_to_mavros rosrun external_pos_fusion fastlio_fusion
    finish_node_role /mavros/vision_pose/pose
}

start_ldop() {
    load_ros_environment
    wait_for_topics_or_fail /cloud_registered /Odometry /mavros/vision_pose/pose
    start_or_skip_node ldop_node roslaunch ldop run_ldop.launch
    finish_node_role /ldop/dynamic_objects
}

start_logger() {
    load_ros_environment
    wait_for_topics_or_fail /Odometry /ldop/dynamic_objects
    if logger_is_running; then
        ACTIVE_PID=""
    else
        "${SCRIPT_DIR}/ros_start_flight_logger.sh" &
        ACTIVE_PID=$!
    fi

    if ! wait_for_logger_or_fail; then
        ACTIVE_PID=""
        exec bash
    fi

    if [[ -n "${ACTIVE_PID}" ]]; then
        local status=0
        if wait "${ACTIVE_PID}"; then
            status=0
        else
            status=$?
        fi
        ACTIVE_PID=""
    fi
    exec bash
}

trap cleanup_active_process INT TERM EXIT

case "${ROLE}" in
    lidar)
        start_livox
        ;;
    mavros)
        start_mavros
        ;;
    fastlio)
        start_fastlio
        ;;
    fusion)
        start_fusion
        ;;
    ldop)
        start_ldop
        ;;
    logger)
        start_logger
        ;;
    *)
        exit 2
        ;;
esac
