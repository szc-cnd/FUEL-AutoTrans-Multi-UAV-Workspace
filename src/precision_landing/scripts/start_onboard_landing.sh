#!/usr/bin/env bash

set -o pipefail

SCRIPT_PATH="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/$(basename -- "${BASH_SOURCE[0]}")"
DB_WS="${PRECISION_LANDING_DB_WS:-${HOME}/db_ws}"
ROS_SETUP="${PRECISION_LANDING_ROS_SETUP:-/opt/ros/noetic/setup.bash}"
CAMERA_LOG="${PRECISION_LANDING_CAMERA_LOG:-/tmp/precision_landing_down_camera.log}"
IMAGE_VIEW_LOG="${PRECISION_LANDING_IMAGE_VIEW_LOG:-/tmp/precision_landing_rqt_image_view.log}"
TERMINATOR_LOG="${PRECISION_LANDING_TERMINATOR_LOG:-/tmp/precision_landing_terminator.log}"
TERMINATOR_PID_FILE="${PRECISION_LANDING_TERMINATOR_PID_FILE:-/tmp/precision_landing_terminator.pid}"
START_LOCK_FILE="${PRECISION_LANDING_START_LOCK_FILE:-/tmp/precision_landing_start.lock}"
RUNTIME_CONFIG="${PRECISION_LANDING_RUNTIME_CONFIG:-/tmp/precision_landing_terminator.conf}"

log() {
  printf '[landing_start] %s\n' "$*"
}

keep_pane_open() {
  printf '\n[landing_start] pane is kept open; press Ctrl+C to stop a running command.\n'
  exec "${SHELL:-/bin/bash}"
}

source_ros_environment() {
  if [[ ! -f "${ROS_SETUP}" ]]; then
    log "ROS setup file not found: ${ROS_SETUP}"
    return 1
  fi
  if [[ ! -d "${DB_WS}" ]]; then
    log "workspace not found: ${DB_WS}"
    return 1
  fi

  # ROS setup scripts may reference unset variables while they are sourced.
  set +u
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
  if [[ -f "${DB_WS}/devel/setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "${DB_WS}/devel/setup.bash"
  else
    log "workspace is not built: ${DB_WS}/devel/setup.bash"
    set -u
    return 1
  fi
  set -u
}

find_camera_device() {
  find /dev/v4l/by-id -maxdepth 1 -type l \
    -name 'usb-Generic_USB_Camera_*-video-index0' \
    -print -quit 2>/dev/null || true
}

find_layout_config() {
  local package_dir
  package_dir="$(rospack find precision_landing 2>/dev/null || true)"
  for candidate in \
    "${PRECISION_LANDING_TERMINATOR_CONFIG:-}" \
    "${SCRIPT_PATH%/*}/../config/terminator_precision_landing.conf" \
    "${DB_WS}/src/precision_landing/config/terminator_precision_landing.conf" \
    "${package_dir}/config/terminator_precision_landing.conf"; do
    if [[ -n "${candidate}" && -f "${candidate}" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done
  return 1
}

ros_nodes() {
  timeout 4s rosnode list 2>/dev/null || true
}

ros_node_running() {
  local node_name="$1"
  ros_nodes | grep -Fxq "${node_name}"
}

image_view_running() {
  pgrep -af '[r]qt_image_view.*landing/debug_image' >/dev/null 2>&1
}

terminator_window_running() {
  local pid
  if [[ -r "${TERMINATOR_PID_FILE}" ]]; then
    pid="$(tr -d '[:space:]' < "${TERMINATOR_PID_FILE}")"
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      if [[ -r "/proc/${pid}/cmdline" ]] && tr '\0' ' ' < "/proc/${pid}/cmdline" | grep -Fq 'precision_landing'; then
        return 0
      fi
    fi
  fi

  pgrep -af '[t]erminator.*precision_landing' >/dev/null 2>&1
}

prepare_graphical_terminal() {
  # An SSH shell may not inherit the desktop DISPLAY and D-Bus variables.
  local desktop_pid desktop_var
  desktop_pid="$(pgrep -u "$(id -u)" -x gnome-shell | head -n 1 || true)"
  if [[ -n "${desktop_pid}" && -r "/proc/${desktop_pid}/environ" ]]; then
    while IFS= read -r -d '' desktop_var; do
      case "${desktop_var}" in
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
    done < "/proc/${desktop_pid}/environ"
  fi

  if [[ -z "${DISPLAY:-}" ]]; then
    local x_socket
    for x_socket in /tmp/.X11-unix/X*; do
      if [[ -S "${x_socket}" ]]; then
        export DISPLAY=":${x_socket##*X}"
        log "detected X display: ${DISPLAY}"
        break
      fi
    done
  fi

  if [[ -z "${DISPLAY:-}" ]]; then
    log "no graphical DISPLAY found; run this from the Ubuntu desktop or configure X11 forwarding"
    return 1
  fi
  if [[ -z "${XAUTHORITY:-}" && -f "${HOME}/.Xauthority" ]]; then
    export XAUTHORITY="${HOME}/.Xauthority"
  fi
  if [[ -z "${DBUS_SESSION_BUS_ADDRESS:-}" && -S "/run/user/$(id -u)/bus" ]]; then
    export DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$(id -u)/bus"
  fi
}

run_camera_pane() {
  local camera_device camera_info_file existing_device
  camera_device="$(find_camera_device)"
  camera_info_file="${HOME}/.ros/camera_info/down_camera.yaml"
  if ros_node_running /usb_cam; then
    existing_device="$(timeout 3s rosparam get /usb_cam/video_device 2>/dev/null || true)"
    if [[ -n "${existing_device}" && "${existing_device}" != "${camera_device}" ]]; then
      echo "[Down Camera] /usb_cam already uses ${existing_device}; expected ${camera_device}"
    else
      echo "[Down Camera] /usb_cam already running; skip duplicate camera launch"
    fi
    keep_pane_open
  fi
  if [[ -z "${camera_device}" ]]; then
    echo "[Down Camera] Generic USB Camera by-id device not found"
    keep_pane_open
  fi
  if [[ ! -f "${camera_info_file}" ]]; then
    echo "[Down Camera] WARNING: calibration file not found: ${camera_info_file}"
  fi

  echo "[Down Camera] starting ${camera_device} at 640x480 MJPG 60 FPS"
  roslaunch precision_landing down_camera.launch \
    video_device:="${camera_device}" \
    camera_info_url:="file://${camera_info_file}" 2>&1 | tee "${CAMERA_LOG}"
  echo "[Down Camera] roslaunch exited with code ${PIPESTATUS[0]}"
  keep_pane_open
}

run_image_view_pane() {
  if image_view_running; then
    echo "[Debug Image] rqt_image_view /landing/debug_image already running; skip duplicate"
    keep_pane_open
  fi
  if [[ -z "${DISPLAY:-}" ]]; then
    echo "[Debug Image] DISPLAY is not set; cannot open rqt_image_view"
    keep_pane_open
  fi

  echo "[Debug Image] starting rqt_image_view /landing/debug_image"
  rqt_image_view /landing/debug_image 2>&1 | tee "${IMAGE_VIEW_LOG}"
  echo "[Debug Image] rqt_image_view exited with code ${PIPESTATUS[0]}"
  keep_pane_open
}

run_mission_pane() {
  if ros_node_running /precision_landing_node || ros_node_running /landing_test; then
    echo "[Landing Mission] landing node already running; skip duplicate mission launch"
    keep_pane_open
  fi

  echo "[Landing Mission] starting real takeoff/hover/landing mission"
  rosrun precision_landing start_landing_test.sh
  echo "[Landing Mission] mission exited with code $?"
  keep_pane_open
}

stop_terminator_window() {
  local pid
  if [[ ! -r "${TERMINATOR_PID_FILE}" ]]; then
    log "no recorded precision_landing Terminator process; close its window manually if it is open"
    return 0
  fi
  pid="$(tr -d '[:space:]' < "${TERMINATOR_PID_FILE}")"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill "${pid}" 2>/dev/null || true
    log "requested close for precision_landing Terminator process ${pid}"
  else
    log "recorded Terminator process is no longer running"
  fi
  rm -f -- "${TERMINATOR_PID_FILE}"
}

if ! source_ros_environment; then
  exit 1
fi

case "${1:-}" in
  --camera)
    run_camera_pane
    exit 0
    ;;
  --image-view)
    run_image_view_pane
    exit 0
    ;;
  --mission)
    run_mission_pane
    exit 0
    ;;
  stop)
    stop_terminator_window
    exit 0
    ;;
  "")
    ;;
  *)
    log "unknown option: $1"
    log "usage: $0 [stop|--camera|--image-view|--mission]"
    exit 2
    ;;
esac

if ! command -v terminator >/dev/null 2>&1; then
  log "terminator is not installed; install it before using the three-pane launcher"
  exit 1
fi

if terminator_window_running; then
  log "precision_landing Terminator window already running; skip duplicate startup"
  exit 0
fi

if ! prepare_graphical_terminal; then
  exit 1
fi

camera_device="$(find_camera_device)"
if [[ -z "${camera_device}" ]]; then
  log "Generic USB Camera by-id device not found"
  exit 1
fi

camera_info_file="${HOME}/.ros/camera_info/down_camera.yaml"
if [[ ! -f "${camera_info_file}" ]]; then
  log "WARNING: calibration file not found: ${camera_info_file}"
fi

layout_config="$(find_layout_config || true)"
if [[ -z "${layout_config}" ]]; then
  log "Terminator layout not found: terminator_precision_landing.conf"
  exit 1
fi

# Avoid opening two windows when two SSH commands arrive at nearly the same time.
if command -v flock >/dev/null 2>&1; then
  exec 9>"${START_LOCK_FILE}"
  if ! flock -n 9; then
    log "another landing startup is already in progress; skip duplicate window"
    exit 0
  fi
fi
if terminator_window_running; then
  log "precision_landing Terminator window appeared during startup; skip duplicate"
  exit 0
fi

sed "s|__PRECISION_LANDING_SCRIPT__|${SCRIPT_PATH}|g" "${layout_config}" > "${RUNTIME_CONFIG}"

log "opening one Terminator window with three panes"
log "top-left: down camera"
log "top-right: ArUco debug image"
log "bottom: takeoff and landing mission"
log "camera log: ${CAMERA_LOG}"
log "mission logs: ${DB_WS}/landing_logs/landing_*.log"

nohup terminator --no-dbus --maximise \
  --config="${RUNTIME_CONFIG}" \
  --layout=precision_landing \
  >"${TERMINATOR_LOG}" 2>&1 &
terminator_pid=$!
printf '%s\n' "${terminator_pid}" > "${TERMINATOR_PID_FILE}"
log "Terminator started with pid ${terminator_pid}"
