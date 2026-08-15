#!/bin/sh

# Start UAV1 LDOP from its isolated GCC 10 catkin workspace.

set -eu

if [ -z "${BASH_VERSION:-}" ]; then
    exec bash "$0" "$@"
fi

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
MATCH_WS=$(dirname -- "$SCRIPT_DIR")
WORKSPACE_PARENT=$(dirname -- "$MATCH_WS")
LDOP_WS="$MATCH_WS/.ldop_only_ws_gcc10"
LDOP_ENV="$WORKSPACE_PARENT/ldop_ws/ldop_env.sh"
LDOP_NODE="$LDOP_WS/devel/lib/ldop/ldop_node"

fail()
{
    echo "[UAV1 LDOP][error] $*" >&2
    exit 1
}

[ -r /opt/ros/noetic/setup.bash ] || fail "Missing ROS Noetic setup.bash."
[ -r "$LDOP_ENV" ] || fail "Missing dependency environment: $LDOP_ENV"
[ -r "$LDOP_WS/devel/setup.bash" ] || fail "LDOP workspace is not built: $LDOP_WS"
[ -x "$LDOP_NODE" ] || fail "LDOP executable is missing: $LDOP_NODE"

set +u
. /opt/ros/noetic/setup.bash
. "$LDOP_ENV"
. "$LDOP_WS/devel/setup.bash"
set -u

if [ "$(rospack find ldop 2>/dev/null || true)" != "$LDOP_WS/src/LDOP" ]; then
    fail "ROS did not resolve the isolated LDOP package."
fi

case "${1:-}" in
    "")
        exec roslaunch ldop run_ldop.launch
        ;;
    --no-rviz)
        exec roslaunch ldop run_ldop.launch rviz:=false
        ;;
    *)
        echo "Usage: bash shfiles/run_uav1_ldop.sh [--no-rviz]" >&2
        exit 2
        ;;
esac
