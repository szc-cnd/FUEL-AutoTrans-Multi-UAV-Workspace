#!/usr/bin/env bash

# Start the ROS stack in one Terminator window with three parallel panes.
# The panes run ROS Master, RealSense, and the color-tag detector in the
# foreground, so their output remains visible for troubleshooting.

set -u

USER_HOME="${HOME:-/home/$(id -un)}"
DB_WS="${COLOR_TAG_DB_WS:-$USER_HOME/db_ws}"
ROS_SETUP="/opt/ros/noetic/setup.bash"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MASTER_WAIT_SEC=15
CAMERA_WAIT_SEC=30
DETECTOR_WAIT_SEC=15

log() {
  printf '[color_tag_start] %s\n' "$*"
}

wait_for() {
  local timeout_sec="$1"
  shift
  local second

  for ((second=0; second<timeout_sec; second++)); do
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
  rostopic list 2>/dev/null | grep -Fxq "$topic"
}

camera_node_exists() {
  rosnode list 2>/dev/null | grep -Eq '^/camera/realsense2_camera($|_[0-9]+$)'
}

camera_topics_ready() {
  topic_exists "/camera/color/image_raw" &&
    topic_exists "/camera/aligned_depth_to_color/image_raw"
}

camera_ready() {
  camera_node_exists || camera_topics_ready
}

detector_ready() {
  rosnode list 2>/dev/null | grep -Eq '^/color_tag_detector$' ||
    topic_exists "/UAV0/color_tag_detector/debug_image"
}

print_view_commands() {
  printf '\n'
  printf '%s\n' '[color_tag_start] View commands:'
  printf '%s\n' 'rqt_image_view /UAV0/color_tag_detector/debug_image'
  printf '%s\n' 'rostopic echo /UAV0/color_tag_detector/result_text'
  printf '%s\n' 'rostopic echo /UAV0/color_tag_detector/target_point_camera'
  printf '%s\n' 'rostopic hz /UAV0/color_tag_detector/debug_image'
  printf '\n'
}

print_all_running() {
  log "ROS Master already running; skip roscore"
  log "RealSense camera already running; skip camera launch"
  log "color-tag detector already running; skip detector launch"
  log "all required components were already running; nothing to start"
  print_view_commands
}

prepare_graphical_terminal() {
  if ! command -v terminator >/dev/null 2>&1; then
    log "terminator is not installed"
    exit 1
  fi

  # An SSH shell often lacks the desktop DISPLAY and D-Bus variables. Reuse
  # the environment of the logged-in GNOME Shell session when available.
  local desktop_pid
  local desktop_var
  desktop_pid="$(pgrep -u "$(id -u)" -x gnome-shell | head -n 1 || true)"
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

  if [[ -z "${DISPLAY:-}" ]]; then
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

  if [[ -z "${DISPLAY:-}" ]]; then
    log "no graphical DISPLAY found; run this from the Ubuntu desktop or configure X11 forwarding"
    exit 1
  fi

  if [[ -z "${XAUTHORITY:-}" && -f "$HOME/.Xauthority" ]]; then
    export XAUTHORITY="$HOME/.Xauthority"
  fi

  # Terminator is started with --no-dbus, but retaining the desktop D-Bus
  # address helps GTK applications launched from the panes behave normally.
  if [[ -z "${DBUS_SESSION_BUS_ADDRESS:-}" && -S "/run/user/$(id -u)/bus" ]]; then
    export DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$(id -u)/bus"
  fi
}

find_terminator_config() {
  local configured_path="${COLOR_TAG_TERMINATOR_CONFIG:-$USER_HOME/scripts/terminator_color_tag.conf}"
  local candidate

  for candidate in \
    "$configured_path" \
    "$SCRIPT_DIR/../config/terminator_color_tag.conf" \
    "$DB_WS/src/color_tag_detector/config/terminator_color_tag.conf"; do
    if [[ -f "$candidate" ]]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done

  return 1
}

run_roscore_component() {
  if ros_master_ready; then
    echo "[ROS Master] already running; skip roscore"
  else
    echo "[ROS Master] running in foreground: roscore"
    roscore
    local result=$?
    echo "[ROS Master] exited with code: $result"
  fi

  echo "[ROS Master] pane is kept open. Press Ctrl+C to stop a running command."
  exec bash
}

run_camera_component() {
  if camera_ready; then
    echo "[RealSense D435] already running; skip camera launch"
  elif ! wait_for "$MASTER_WAIT_SEC" ros_master_ready; then
    echo "[RealSense D435] ROS Master did not become ready within ${MASTER_WAIT_SEC}s"
  else
    echo "[RealSense D435] running in foreground: roslaunch realsense2_camera rs_camera.launch"
    roslaunch realsense2_camera rs_camera.launch \
      align_depth:=true \
      color_width:=640 color_height:=480 color_fps:=30 \
      depth_width:=640 depth_height:=480 depth_fps:=30
    local result=$?
    echo "[RealSense D435] roslaunch exited with code: $result"
  fi

  echo "[RealSense D435] pane is kept open. Press Ctrl+C to stop a running command."
  exec bash
}

run_detector_component() {
  if detector_ready; then
    echo "[Color Tag Detector] already running; skip detector launch"
  elif ! wait_for "$MASTER_WAIT_SEC" ros_master_ready; then
    echo "[Color Tag Detector] ROS Master did not become ready within ${MASTER_WAIT_SEC}s"
  elif ! wait_for "$CAMERA_WAIT_SEC" camera_topics_ready; then
    echo "[Color Tag Detector] camera topics did not become ready within ${CAMERA_WAIT_SEC}s"
  else
    echo "[Color Tag Detector] running in foreground: roslaunch color_tag_detector color_tag_detector.launch"
    roslaunch color_tag_detector color_tag_detector.launch
    local result=$?
    echo "[Color Tag Detector] roslaunch exited with code: $result"
  fi

  echo "[Color Tag Detector] pane is kept open. Press Ctrl+C to stop a running command."
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

# ROS Noetic's setup scripts reference variables before defining them. Keep
# nounset for this launcher, but temporarily disable it while sourcing ROS.
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

# These modes are called by the three Terminator panes. Each mode checks the
# live ROS graph again, so an already-running component is never duplicated.
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
  "")
    ;;
  *)
    log "unknown option: $1"
    log "usage: $0 [--roscore|--camera|--detector]"
    exit 2
    ;;
esac

# If everything is already running, do not open a new GUI window.
if ros_master_ready && camera_ready && detector_ready; then
  print_all_running
  exit 0
fi

# Prevent two SSH invocations during startup from opening two Terminator
# windows. The lock is held until this controller confirms the stack is ready.
if command -v flock >/dev/null 2>&1; then
  exec 9>/tmp/color_tag_detector_start.lock
  if ! flock -n 9; then
    log "startup is already in progress; skip opening another Terminator window"
    print_view_commands
    exit 0
  fi
fi

# Recheck after taking the lock because another invocation may have completed
# startup between the first graph check and the lock acquisition.
if ros_master_ready && camera_ready && detector_ready; then
  print_all_running
  exit 0
fi

TERMINATOR_CONFIG="$(find_terminator_config || true)"
if [[ -z "$TERMINATOR_CONFIG" ]]; then
  log "Terminator layout not found"
  log "expected $USER_HOME/scripts/terminator_color_tag.conf or the package config directory"
  exit 1
fi

prepare_graphical_terminal

TERMINATOR_LOG="/tmp/color_tag_terminator.log"
log "opening one Terminator window with three parallel panes"
log "layout: $TERMINATOR_CONFIG"
log "Terminator output: $TERMINATOR_LOG"

# Only the GUI launcher is detached. The ROS commands inside all panes stay
# in the foreground and their output remains visible in that same window.
nohup terminator --no-dbus --maximise \
  --config="$TERMINATOR_CONFIG" \
  --layout=color_tag_detector \
  >"$TERMINATOR_LOG" 2>&1 &
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
  log "inspect the ROS Master pane"
  exit 1
fi
log "ROS Master is ready"

if ! wait_for "$CAMERA_WAIT_SEC" camera_topics_ready; then
  log "required camera topics did not become ready within ${CAMERA_WAIT_SEC}s"
  log "inspect the RealSense D435 pane"
  exit 1
fi
log "camera topics are ready"

if wait_for "$DETECTOR_WAIT_SEC" detector_ready; then
  log "color-tag detector is ready"
  print_view_commands
else
  log "color-tag detector did not become ready within ${DETECTOR_WAIT_SEC}s"
  log "inspect the Color Tag Detector pane"
  exit 1
fi
