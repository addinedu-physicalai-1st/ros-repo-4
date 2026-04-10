#!/usr/bin/env bash
# ROS_DOMAIN_ID=22 로 PinkyGps 데모 발행 (LCD 도메인 23 과 DDS 격리 확인용)
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "먼저 ROS 를 source 하세요. 예: source /opt/ros/jazzy/setup.bash" >&2
  exit 1
fi
# shellcheck source=/dev/null
source "$WS_ROOT/install/setup.bash"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-22}"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID (토픽 /pinky_${ROS_DOMAIN_ID}/gps_pos)"
exec python3 "$SCRIPT_DIR/ros2_pinky_gps_publish_domain22_demo.py" "$@"
