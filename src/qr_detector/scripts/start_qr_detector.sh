#!/usr/bin/env bash

# Start the QR detection stack in one Terminator window with three foreground
# panes: ROS Master, RealSense, and the QR detector.

set -u
set -o pipefail

DB_WS="${QR_DETECTOR_DB_WS:-/home/asus/db_ws}"
ROS_SETUP="/opt/ros/noetic/setup.bash"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MASTER_WAIT_SEC="${QR_MASTER_WAIT_SEC:-20}"
CAMERA_WAIT_SEC="${QR_CAMERA_WAIT_SEC:-30}"
DETECTOR_WAIT_SEC="${QR_DETECTOR_WAIT_SEC:-20}"
LOCK_FILE="${QR_DETECTOR_LOCK_FILE:-/tmp/qr_detector_start.lock}"
TERMINATOR_CONFIG_DEFAULT="${QR_DETECTOR_TERMINATOR_CONFIG:-/home/asus/scripts/terminator_qr_detector.conf}"

log() {
  printf '[qr_detector_start] %s\n' "$*"
}

wait_for() {
  local timeout_sec="$1"
  shift
  local second

  for ((second = 0; second < timeout_sec; second++)); do
    if "$@"; then
      return 0
    fi
    sleep 1
  done

  "$@"
}

ros_master_ready() {
  rosnode list >/dev/null 2>&1
}

topic_exists() {
  local topic="$1"
  rostopic list 2>/dev/null | grep -Fx "$topic" >/dev/null
}

node_exists() {
  local node="$1"
  rosnode list 2>/dev/null | grep -Fx "$node" >/dev/null
}

camera_node_exists() {
  rosnode list 2>/dev/null | grep -E '^/camera/realsense2_camera($|_[0-9]+$)|^/camera/realsense2_camera_manager$' >/dev/null
}

camera_topics_ready() {
  topic_exists '/camera/color/image_raw' &&
    topic_exists '/camera/aligned_depth_to_color/image_raw'
}

camera_ready() {
  camera_node_exists || camera_topics_ready
}

detector_ready() {
  node_exists '/qr_detector_node' || topic_exists '/UAV0/vision/qr_detected'
}

print_view_commands() {
  printf '\n'
  printf '%s\n' '[qr_detector_start] View commands:'
  printf '%s\n' 'rqt_image_view /UAV0/vision/qr_debug_image'
  printf '%s\n' 'rostopic echo /UAV0/vision/qr_detected'
  printf '%s\n' 'rostopic echo /UAV0/vision/qr_pose_camera'
  printf '%s\n' 'rostopic hz /UAV0/vision/qr_detected'
  printf '%s\n' 'rostopic hz /camera/color/image_raw'
  printf '\n'
}

print_all_running() {
  log 'ROS Master already running; skip roscore'
  log 'RealSense camera already running; skip camera launch'
  log 'QR detector already running; skip detector launch'
  log 'all required components were already running; no Terminator window opened'
  print_view_commands
}

prepare_graphical_terminal() {
  if ! command -v terminator >/dev/null 2>&1; then
    log 'terminator is not installed'
    exit 1
  fi

  # SSH shells often do not inherit the graphical session environment. Read
  # the values from the logged-in GNOME Shell process when it is available.
  local desktop_pid
  local desktop_var
  desktop_pid="$(pgrep -u "$(id -u)" -x gnome-shell 2>/dev/null | head -n 1 || true)"
  if [[ -n "$desktop_pid" && -r "/proc/$desktop_pid/environ" ]]; then
    while IFS= read -r desktop_var; do
      case "$desktop_var" in
        DISPLAY=*)
          export DISPLAY="${desktop_var#DISPLAY=}"
          ;;
        XAUTHORITY=*)
          export XAUTHORITY="${desktop_var#XAUTHORITY=}"
          ;;
        DBUS_SESSION_BUS_ADDRESS=*)
          export DBUS_SESSION_BUS_ADDRESS="${desktop_var#DBUS_SESSION_BUS_ADDRESS=}"
          ;;
      esac
    done < <(tr '\0' '\n' < "/proc/$desktop_pid/environ")
  fi

  if [[ -z "${DISPLAY:-}" || "${DISPLAY:-}" == ':' ]]; then
    local x_socket
    for x_socket in /tmp/.X11-unix/X*; do
      if [[ -S "$x_socket" ]]; then
        DISPLAY=":${x_socket##*X}"
        export DISPLAY
        log "detected X display: $DISPLAY"
        break
      fi
    done
  fi

  if [[ -z "${DISPLAY:-}" || "${DISPLAY:-}" == ':' ]]; then
    log 'no graphical DISPLAY found; run from the Ubuntu desktop or configure X11 forwarding'
    exit 1
  fi

  if [[ -z "${XAUTHORITY:-}" && -f "$HOME/.Xauthority" ]]; then
    export XAUTHORITY="$HOME/.Xauthority"
  fi

  if [[ -z "${DBUS_SESSION_BUS_ADDRESS:-}" && -S "/run/user/$(id -u)/bus" ]]; then
    export DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$(id -u)/bus"
  fi
}

find_terminator_config() {
  local candidate

  for candidate in \
    "$TERMINATOR_CONFIG_DEFAULT" \
    "$SCRIPT_DIR/terminator_qr_detector.conf" \
    "$DB_WS/src/qr_detector/config/terminator_qr_detector.conf"; do
    if [[ -f "$candidate" ]]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done

  return 1
}

run_roscore_component() {
  if ros_master_ready; then
    printf '%s\n' '[ROS Master] already running; skip roscore'
  else
    printf '%s\n' '[ROS Master] running in foreground: roscore'
    roscore
    local result=$?
    printf '[ROS Master] roscore exited with code: %s\n' "$result"
  fi

  printf '%s\n' '[ROS Master] pane is kept open; press Ctrl+C to stop a running command'
  exec bash
}

run_camera_component() {
  if camera_ready; then
    printf '%s\n' '[RealSense D435/D455] already running; skip camera launch'
  elif ! wait_for "$MASTER_WAIT_SEC" ros_master_ready; then
    printf '[RealSense D435/D455] ROS Master did not become ready within %ss\n' "$MASTER_WAIT_SEC"
  else
    printf '%s\n' '[RealSense D435/D455] running in foreground: roslaunch realsense2_camera rs_camera.launch'
    roslaunch realsense2_camera rs_camera.launch \
      align_depth:=true \
      color_width:=640 color_height:=480 color_fps:=30 \
      depth_width:=640 depth_height:=480 depth_fps:=30
    local result=$?
    printf '[RealSense D435/D455] roslaunch exited with code: %s\n' "$result"
  fi

  printf '%s\n' '[RealSense D435/D455] pane is kept open; press Ctrl+C to stop a running command'
  exec bash
}

run_detector_component() {
  if detector_ready; then
    printf '%s\n' '[QR Detector] already running; skip detector launch'
  elif ! wait_for "$MASTER_WAIT_SEC" ros_master_ready; then
    printf '[QR Detector] ROS Master did not become ready within %ss\n' "$MASTER_WAIT_SEC"
  elif ! wait_for "$CAMERA_WAIT_SEC" camera_topics_ready; then
    printf '[QR Detector] camera topics did not become ready within %ss\n' "$CAMERA_WAIT_SEC"
  else
    printf '%s\n' '[QR Detector] running in foreground: roslaunch qr_detector qr_detector.launch'
    roslaunch qr_detector qr_detector.launch
    local result=$?
    printf '[QR Detector] roslaunch exited with code: %s\n' "$result"
  fi

  printf '%s\n' '[QR Detector] pane is kept open; press Ctrl+C to stop a running command'
  exec bash
}

if [[ ! -f "$ROS_SETUP" ]]; then
  log "ROS setup file not found: $ROS_SETUP"
  exit 1
fi

if [[ ! -d "$DB_WS" ]]; then
  log "workspace not found: $DB_WS"
  exit 1
fi

# ROS Noetic setup scripts may reference unset variables. Disable nounset only
# while sourcing ROS and the workspace, then restore it for this launcher.
set +u
# shellcheck disable=SC1091
source "$ROS_SETUP"
if [[ -f "$DB_WS/devel/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$DB_WS/devel/setup.bash"
else
  log "workspace is not built: $DB_WS/devel/setup.bash"
  set -u
  exit 1
fi
set -u

case "${1:-}" in
  --roscore)
    run_roscore_component
    exit 0
    ;;
  --camera)
    run_camera_component
    exit 0
    ;;
  --detector)
    run_detector_component
    exit 0
    ;;
  '')
    ;;
  *)
    log "unknown option: $1"
    log "usage: $0 [--roscore|--camera|--detector]"
    exit 2
    ;;
esac

if ros_master_ready && camera_ready && detector_ready; then
  print_all_running
  exit 0
fi

if ! command -v flock >/dev/null 2>&1; then
  log 'flock is required but was not found'
  exit 1
fi

# Hold the lock until the controller confirms the stack is ready. A second SSH
# invocation during startup therefore cannot open another Terminator window.
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
  log 'another startup is already in progress; skip opening another Terminator window'
  print_view_commands
  exit 0
fi

# Recheck after taking the lock in case another invocation finished startup.
if ros_master_ready && camera_ready && detector_ready; then
  print_all_running
  exit 0
fi

TERMINATOR_CONFIG="$(find_terminator_config || true)"
if [[ -z "$TERMINATOR_CONFIG" ]]; then
  log "Terminator layout not found; expected $TERMINATOR_CONFIG_DEFAULT"
  exit 1
fi

prepare_graphical_terminal

TERMINATOR_LOG="/tmp/qr_detector_terminator.log"
log 'opening one Terminator window with three parallel panes'
log "layout: $TERMINATOR_CONFIG"
log "Terminator output: $TERMINATOR_LOG"

# Only Terminator is detached. Each ROS command is the foreground command of
# its own pane and remains visible in the same window.
nohup terminator --no-dbus --maximise \
  --config="$TERMINATOR_CONFIG" \
  --layout=qr_detector \
  >"$TERMINATOR_LOG" 2>&1 9>&- &
TERMINATOR_PID=$!

sleep 1
if ! kill -0 "$TERMINATOR_PID" 2>/dev/null; then
  log "Terminator exited immediately; inspect $TERMINATOR_LOG"
  if [[ -s "$TERMINATOR_LOG" ]]; then
    sed -n '1,80p' "$TERMINATOR_LOG"
  fi
  exit 1
fi

if ! wait_for "$MASTER_WAIT_SEC" ros_master_ready; then
  log "ROS Master did not become ready within ${MASTER_WAIT_SEC}s"
  log 'inspect the ROS Master pane'
  exit 1
fi
log 'ROS Master is ready'

if ! wait_for "$CAMERA_WAIT_SEC" camera_topics_ready; then
  log "required camera topics did not become ready within ${CAMERA_WAIT_SEC}s"
  log 'inspect the RealSense pane'
  exit 1
fi
log 'camera topics are ready'

if wait_for "$DETECTOR_WAIT_SEC" detector_ready; then
  log 'QR detector is ready'
  print_view_commands
else
  log "QR detector did not become ready within ${DETECTOR_WAIT_SEC}s"
  log 'inspect the QR Detector pane'
  exit 1
fi
