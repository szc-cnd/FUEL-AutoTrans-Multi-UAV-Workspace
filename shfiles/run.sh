#!/bin/sh

# Resolve match_ws from this script's location so the script works regardless
# of the caller's current directory, and expose workspace launch files to ROS.
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
MATCH_WS=$(dirname -- "$SCRIPT_DIR")
. "$MATCH_WS/devel/setup.sh"

sudo chmod 777 /dev/ttyACM0
sleep 2
roslaunch cxr_ego_ctrl mavros_uav.launch vehicle_ns:=UAV0 &
sleep 10
wait
