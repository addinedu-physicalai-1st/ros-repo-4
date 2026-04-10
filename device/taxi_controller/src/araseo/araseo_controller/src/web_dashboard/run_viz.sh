#!/bin/bash

# Get the directory of this script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
PROJECT_ROOT="$DIR/../.."

echo "Starting IMU Visualization System..."

# Detect architecture and set build folder
ARCH=$(uname -m)
if [ "$ARCH" == "aarch64" ]; then
    BUILD_DIR="build_raspi"
else
    BUILD_DIR="build_pc"
fi

echo "Detected architecture: $ARCH. Using $BUILD_DIR."

# 1. Start the Data Bridge in the background
# Source ROS 2 and workspace environment for DDS libraries
if [ -f "$PROJECT_ROOT/ros2_ws/install_$BUILD_DIR/setup.bash" ]; then
    source "$PROJECT_ROOT/ros2_ws/install_$BUILD_DIR/setup.bash"
elif [ -f "$PROJECT_ROOT/ros2_ws/install_raspi/setup.bash" ]; then
    source "$PROJECT_ROOT/ros2_ws/install_raspi/setup.bash"
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source "/opt/ros/jazzy/setup.bash"
fi

# Run bridge FROM the dashboard directory so data.json is created in the right place
cd "$DIR"
"$PROJECT_ROOT/$BUILD_DIR/data_bridge" < /dev/zero >> viz.log 2>&1 &
BRIDGE_PID=$!
echo "Data Bridge started (PID: $BRIDGE_PID)"

# 2. Start the Web Server
echo "Starting Web Server on http://localhost:8000"
echo "Press Ctrl+C to stop all."
python3 -m http.server 8000

# Cleanup on exit
kill $BRIDGE_PID
