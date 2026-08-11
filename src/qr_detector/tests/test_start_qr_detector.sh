#!/usr/bin/env bash

set -u

TEST_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd -- "${TEST_DIR}/.." && pwd)"
SCRIPT_PATH="${PACKAGE_DIR}/scripts/start_qr_detector.sh"
TMP_DIR="$(mktemp -d)"
trap 'rm -rf "${TMP_DIR}"' EXIT

FAKE_BIN="${TMP_DIR}/bin"
mkdir -p "${FAKE_BIN}" "${TMP_DIR}/workspace/devel"
cat > "${TMP_DIR}/workspace/devel/setup.bash" <<EOF
export PATH="${FAKE_BIN}:\$PATH"
EOF
export HOME="${TMP_DIR}/home"
export QR_DETECTOR_DB_WS="${TMP_DIR}/workspace"
export QR_DETECTOR_LOCK_FILE="${TMP_DIR}/qr_detector.lock"
export QR_DETECTOR_TERMINATOR_CONFIG="${TMP_DIR}/terminator_qr_detector.conf"
export QR_MASTER_WAIT_SEC=2
export QR_CAMERA_WAIT_SEC=2
export QR_DETECTOR_WAIT_SEC=2
export ROS_SKIP_ENV=1
export QR_TEST_STATE="${TMP_DIR}/state"
export QR_TEST_TERMINATOR_LOG="${TMP_DIR}/terminator.log"
export DISPLAY=:0
touch "${QR_DETECTOR_TERMINATOR_CONFIG}"

cat > "${FAKE_BIN}/rosnode" <<'EOF'
#!/usr/bin/env bash
if [[ "$1" == "list" && -f "${QR_TEST_STATE}" ]]; then
  printf '/rosout\n'
  grep -v '^master$' "${QR_TEST_STATE}" || true
  exit 0
fi
exit 1
EOF

cat > "${FAKE_BIN}/rostopic" <<'EOF'
#!/usr/bin/env bash
if [[ "$1" == "list" && -f "${QR_TEST_STATE}" ]]; then
  if grep -Fxq '/camera/realsense2_camera_manager' "${QR_TEST_STATE}"; then
    printf '/camera/color/image_raw\n/camera/aligned_depth_to_color/image_raw\n'
  fi
  if grep -Fxq '/qr_detector_node' "${QR_TEST_STATE}"; then
    printf '/UAV0/vision/qr_detected\n'
  fi
fi
exit 0
EOF

cat > "${FAKE_BIN}/pgrep" <<'EOF'
#!/usr/bin/env bash
exit 1
EOF

cat > "${FAKE_BIN}/flock" <<'EOF'
#!/usr/bin/env bash
exit 0
EOF

cat > "${FAKE_BIN}/terminator" <<'EOF'
#!/usr/bin/env bash
printf '%s\n' "$*" >> "${QR_TEST_TERMINATOR_LOG}"
touch "${QR_TEST_STATE}"
printf 'master\n/camera/realsense2_camera_manager\n/qr_detector_node\n' > "${QR_TEST_STATE}"
sleep 5
EOF

cat > "${FAKE_BIN}/id" <<'EOF'
#!/usr/bin/env bash
if [[ "$1" == "-u" ]]; then
  printf '1000\n'
else
  /usr/bin/id "$@"
fi
EOF

chmod +x "${FAKE_BIN}"/*
export PATH="${FAKE_BIN}:${PATH}"

if [[ ! -x "${SCRIPT_PATH}" ]]; then
  printf 'FAIL: startup script is missing or not executable\n' >&2
  exit 1
fi

first_output="$("${SCRIPT_PATH}")"
if ! grep -Fq 'opening one Terminator window with three parallel panes' <<<"${first_output}"; then
  printf 'FAIL: first run did not open the three-pane window\n' >&2
  exit 1
fi
if ! grep -Fq 'rqt_image_view /UAV0/vision/qr_debug_image' <<<"${first_output}"; then
  printf 'FAIL: startup output did not include the direct debug image command\n' >&2
  exit 1
fi

first_count="$(wc -l < "${QR_TEST_TERMINATOR_LOG}")"
if [[ "${first_count}" -ne 1 ]]; then
  printf 'FAIL: expected one Terminator launch, got %s\n' "${first_count}" >&2
  exit 1
fi

second_output="$("${SCRIPT_PATH}")"
second_count="$(wc -l < "${QR_TEST_TERMINATOR_LOG}")"
if [[ "${second_count}" -ne 1 ]]; then
  printf 'FAIL: second run opened a duplicate Terminator window\n' >&2
  exit 1
fi
if ! grep -Fq 'no Terminator window opened' <<<"${second_output}"; then
  printf 'FAIL: second run did not report all components already running\n' >&2
  exit 1
fi

printf 'PASS: QR startup opens one window and skips duplicate startup\n'
