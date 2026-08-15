#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_WORKSPACE="${ROS_WORKSPACE:-$HOME/rh_ws}"
LDOP_ENV="${LDOP_ENV:-}"
if [[ -z "${LDOP_ENV}" ]]; then
    if [[ -f "${ROS_WORKSPACE}/devel/setup.bash" ]]; then
        LDOP_ENV="${ROS_WORKSPACE}/devel/setup.bash"
    elif [[ -f "${HOME}/ldop_ws/ldop_env.sh" ]]; then
        LDOP_ENV="${HOME}/ldop_ws/ldop_env.sh"
    fi
fi
LOG_ROOT="${LDOP_LOG_ROOT:-$HOME/ldop_logs}"
PID_FILE="${LOG_ROOT}/.flight_logger.pid"

if [[ ! -f "${LDOP_ENV}" ]]; then
    exit 1
fi
source "${LDOP_ENV}"
set -u

mkdir -p "${LOG_ROOT}"

if [[ -f "${PID_FILE}" ]]; then
    OLD_PID="$(cat "${PID_FILE}" 2>/dev/null || true)"
    if [[ "${OLD_PID}" =~ ^[0-9]+$ ]] && kill -0 "${OLD_PID}" 2>/dev/null; then
        exit 0
    fi
    rm -f "${PID_FILE}"
fi

STAMP="$(date +%Y%m%d_%H%M%S_%N)"
LOG_DIR="${LOG_ROOT}/${STAMP}"
mkdir -p "${LOG_DIR}"

echo "log_dir=${LOG_DIR}" > "${LOG_ROOT}/current_flight.txt"
python3 -u "${SCRIPT_DIR}/ros_flight_logger.py" --log-dir "${LOG_DIR}" \
    > "${LOG_DIR}/logger_console.log" 2>&1 &
LOGGER_PID=$!
echo "${LOGGER_PID}" > "${PID_FILE}"
echo "pid=${LOGGER_PID}" >> "${LOG_ROOT}/current_flight.txt"

cleanup() {
    rm -f "${PID_FILE}"
}
trap cleanup EXIT

wait "${LOGGER_PID}"
