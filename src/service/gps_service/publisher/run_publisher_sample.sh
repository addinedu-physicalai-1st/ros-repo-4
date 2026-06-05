#!/usr/bin/env bash
# Workspace root: GPSTopicTest/
# Requires: colcon build && source install/setup.bash
# Note: no "set -u" — ROS setup.bash references optional vars (e.g. AMENT_TRACE_SETUP_FILES).
set -eo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
: "${ROS_DISTRO:=jazzy}"
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck source=/dev/null
source "${WS}/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"
echo "ROS_DISTRO=${ROS_DISTRO} ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
exec ros2 run gps_field_publisher field_gps_publisher "$@"
