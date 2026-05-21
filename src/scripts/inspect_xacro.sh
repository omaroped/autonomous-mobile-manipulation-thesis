#!/bin/bash
# Inspect what xacro files exist and what they include

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"

source /opt/ros/humble/setup.bash

if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
    source "${WORKSPACE_DIR}/install/setup.bash"
elif [ -f ~/thesis_ws/install/setup.bash ]; then
    source ~/thesis_ws/install/setup.bash
fi

PKG_SRC="${WORKSPACE_DIR}/third_party/agilex_limo_ros2/limo_car"

echo "=== gazebo/ackermann.xacro ==="
cat $PKG_SRC/gazebo/ackermann.xacro

echo ""
echo "=== gazebo/ackermann_with_sensor.xacro ==="
cat $PKG_SRC/gazebo/ackermann_with_sensor.xacro

echo ""
echo "=== gazebo/sensor.xacro ==="
cat $PKG_SRC/gazebo/sensor.xacro
