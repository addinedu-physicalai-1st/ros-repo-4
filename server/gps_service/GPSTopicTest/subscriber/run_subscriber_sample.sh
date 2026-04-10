#!/usr/bin/env bash
# Default pinky_id=-1 (no filter). Topic default /pinky_${ROS_DOMAIN_ID}/gps_pos.
# Note: no "set -u" — ROS setup.bash references optional vars (e.g. AMENT_TRACE_SETUP_FILES).
set -eo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
: "${ROS_DISTRO:=jazzy}"
: "${PINKY_ID:=-1}"
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck source=/dev/null
source "${WS}/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"
echo "ROS_DISTRO=${ROS_DISTRO} ROS_DOMAIN_ID=${ROS_DOMAIN_ID} PINKY_ID=${PINKY_ID}"
exec ros2 run gps_field_subscriber field_gps_subscriber --ros-args -p "pinky_id:=${PINKY_ID}" "$@"
