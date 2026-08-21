#!/usr/bin/env bash

# UAV1 下视画面独立查看器。只订阅图像，不启动相机、搜索、精降或控制节点。

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MATCH_WS="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
NETWORK_SETUP="${SCRIPT_DIR}/configure_dual_uav_ros_network.sh"
WAIT_TIMEOUT="${UAV1_VIEW_WAIT_TIMEOUT:-180}"

usage() {
  cat <<'EOF'
用法：
  bash shfiles/view_uav1_down_camera.sh [combined|raw|search|precision]

画面：
  combined  默认。搜索阶段显示 ArUco 搜索标注，精降接管后自动切换精降标注。
  raw       只显示下视相机原始画面。
  search    只显示下视搜索标注画面。
  precision 只显示精降标注画面。

本脚本只打开 rqt_image_view，不会重复启动任何 ROS 节点。
EOF
}

fail() {
  printf '[UAV1 down view][错误] %s\n' "$*" >&2
  exit 1
}

if [[ $# -gt 1 ]]; then
  usage >&2
  exit 2
fi

case "${1:-combined}" in
  combined) topic="/UAV1/landing/combined_debug_image" ;;
  raw) topic="/UAV1/down_camera/image_raw" ;;
  search) topic="/UAV1/landing/search/debug_image" ;;
  precision) topic="/UAV1/landing/debug_image" ;;
  -h|--help)
    usage
    exit 0
    ;;
  *)
    usage >&2
    fail "未知画面类型：$1"
    ;;
esac

[[ -r /opt/ros/noetic/setup.bash ]] || fail "未找到 /opt/ros/noetic/setup.bash。"
[[ -r "${MATCH_WS}/devel/setup.bash" ]] ||
  fail "未找到 ${MATCH_WS}/devel/setup.bash，请先编译工作区。"
[[ -r "${NETWORK_SETUP}" ]] || fail "未找到 ${NETWORK_SETUP}。"
[[ -n "${DISPLAY:-}" ]] || fail "DISPLAY 未设置，请在 UAV1 图形桌面的终端中运行。"

set +u
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1090
source "${MATCH_WS}/devel/setup.bash"
# shellcheck disable=SC1090
source "${NETWORK_SETUP}"
set -u
configure_dual_uav_ros_network uav1

command -v rqt_image_view >/dev/null 2>&1 ||
  fail "未安装 rqt_image_view，请安装 ros-noetic-rqt-image-view。"

deadline=$((SECONDS + WAIT_TIMEOUT))
printf '[UAV1 down view] 等待图像话题 %s（最长 %ss）...\n' \
  "${topic}" "${WAIT_TIMEOUT}"
while ! rostopic list 2>/dev/null | grep -Fxq "${topic}"; do
  (( SECONDS < deadline )) || fail "等待图像话题超时：${topic}"
  sleep 1
done

printf '[UAV1 down view] 打开 %s\n' "${topic}"
exec rqt_image_view "${topic}"
