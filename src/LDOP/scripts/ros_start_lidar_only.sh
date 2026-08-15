#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LAYOUT_CONFIG="${SCRIPT_DIR}/terminator_ldop.conf"
PANE_SCRIPT="${SCRIPT_DIR}/ros_start_terminator_pane.sh"

if ! command -v terminator >/dev/null 2>&1; then
    exit 1
fi
if [[ ! -f "${LAYOUT_CONFIG}" ]]; then
    exit 1
fi
if [[ ! -x "${PANE_SCRIPT}" ]]; then
    exit 1
fi

export LDOP_SCRIPT_DIR="${SCRIPT_DIR}"

cat <<'EOF'

rostopic echo /mavros/vision_pose/pose
rostopic echo /mavros/local_position/pose
rostopic echo /mavros/state
EOF

TERMINATOR_LOG="${TMPDIR:-/tmp}/ldop_terminator_lidar.log"
nohup terminator --config="${LAYOUT_CONFIG}" --layout=ldop_lidar \
    >"${TERMINATOR_LOG}" 2>&1 </dev/null &
TERMINATOR_PID=$!
disown "${TERMINATOR_PID}" 2>/dev/null || true
