#!/bin/sh

# Stop the UAV0 processes started by the commands in the operating notes.
# For flight safety, run this only after UAV0 has landed and disarmed.

stop_processes() {
    label=$1
    pattern=$2

    pids=$(pgrep -f -- "$pattern" 2>/dev/null || true)
    if [ -z "$pids" ]; then
        echo "[skip] $label is not running"
        return
    fi

    echo "[stop] $label (PID: $(echo "$pids" | tr '\n' ' '))"
    kill -INT $pids 2>/dev/null || true

    count=0
    while [ "$count" -lt 5 ]; do
        sleep 1
        remaining=$(pgrep -f -- "$pattern" 2>/dev/null || true)
        [ -z "$remaining" ] && return
        count=$((count + 1))
    done

    echo "[term] $label did not exit after 5 seconds"
    kill -TERM $remaining 2>/dev/null || true
}

echo "Stopping UAV0 controller and navigation stack..."

# Stop consumers before stopping localization, lidar, and the flight-control link.
stop_processes "UAV0 controller" "cxr_egoctrl_v1.*__name:=UAV0_controller"
stop_processes "swarm planner" "roslaunch[[:space:]]+diff_planner[[:space:]]+run_swarm_indoor1_fuel_exploration.launch"
stop_processes "FAST-LIO pose bridge" "python3.*laser_mid360.py[[:space:]]+iris[[:space:]]+0[[:space:]]+fastlio[[:space:]]+off"
stop_processes "FAST-LIO" "roslaunch[[:space:]]+fast_lio[[:space:]]+mapping_mid360.launch"
stop_processes "UAV0 MID360 driver" "roslaunch[[:space:]]+livox_ros_driver2[[:space:]]+msg_MID360.launch.*vehicle_ns:=UAV0"
stop_processes "UAV0 MAVROS" "roslaunch[[:space:]]+cxr_ego_ctrl[[:space:]]+mavros_uav.launch.*vehicle_ns:=UAV0"

echo "UAV0 processes have been stopped."
