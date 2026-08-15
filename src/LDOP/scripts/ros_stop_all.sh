#!/bin/bash
set -u
LOG_ROOT="${LDOP_LOG_ROOT:-$HOME/ldop_logs}"
PID_FILE="${LOG_ROOT}/.flight_logger.pid"

if [[ -f "${PID_FILE}" ]]; then
    LOGGER_PID="$(cat "${PID_FILE}" 2>/dev/null || true)"
    if [[ "${LOGGER_PID}" =~ ^[0-9]+$ ]] && kill -0 "${LOGGER_PID}" 2>/dev/null; then
        kill -TERM "${LOGGER_PID}" 2>/dev/null || true
        for _ in {1..20}; do
            if ! kill -0 "${LOGGER_PID}" 2>/dev/null; then
                break
            fi
            sleep 0.1
        done
    fi
    rm -f "${PID_FILE}"
fi

pkill -f "roslaunch"
pkill -f "rosrun"
pkill -f "roscore"
pkill -f "rosmaster"

echo "=================================="
echo "=================================="
