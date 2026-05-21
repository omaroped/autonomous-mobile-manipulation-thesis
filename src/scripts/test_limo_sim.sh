#!/bin/bash
# Diagnostic script to test LIMO simulation components
set -e

source /opt/ros/humble/setup.bash
source ~/Desktop/Thesis/install/setup.bash

PKG_DIR=$(ros2 pkg prefix limo_car)/share/limo_car

echo "=== Step 1: Check xacro files ==="
echo "Base xacro:"
ls -la $PKG_DIR/urdf/
echo ""
echo "Gazebo xacro:"
ls -la $PKG_DIR/gazebo/
echo ""
echo "Worlds:"
ls -la $PKG_DIR/worlds/
echo ""
echo "Meshes:"
ls -la $PKG_DIR/meshes/

echo ""
echo "=== Step 2: Process xacro ==="
XACRO_FILE=$PKG_DIR/gazebo/ackermann_with_sensor.xacro
xacro $XACRO_FILE > /tmp/limo_full.urdf 2>&1
LINES=$(wc -l < /tmp/limo_full.urdf)
echo "Xacro produced $LINES lines"

echo ""
echo "=== Step 3: Check for Gazebo plugin in URDF ==="
grep -c "gazebo_ros" /tmp/limo_full.urdf || echo "WARNING: No gazebo_ros plugins found!"
grep "libgazebo_ros" /tmp/limo_full.urdf | head -5

echo ""
echo "=== Step 4: Check cmd_vel topic in URDF ==="
grep "cmd_vel" /tmp/limo_full.urdf || echo "WARNING: No cmd_vel topic found in URDF!"

echo ""
echo "=== Step 5: Test robot_state_publisher ==="
export DISPLAY=:0
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat /tmp/limo_full.urdf)" &
RSP_PID=$!
sleep 3

echo ""
echo "=== Step 6: Check nodes ==="
ros2 node list

echo ""
echo "=== Step 7: Check robot_description topic ==="
timeout 5 ros2 topic echo /robot_description --once 2>&1 | head -3

echo ""
echo "=== Cleanup ==="
kill $RSP_PID 2>/dev/null || true
echo "Done. robot_state_publisher test complete."
