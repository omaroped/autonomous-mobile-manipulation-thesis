#!/bin/bash
# Inspect what xacro files exist and what they include
source /opt/ros/humble/setup.bash
source ~/thesis_ws/install/setup.bash

PKG_SRC=~/thesis_ws/src/limo_ros2/limo_car

echo "=== gazebo/ackermann.xacro ==="
cat $PKG_SRC/gazebo/ackermann.xacro

echo ""
echo "=== gazebo/ackermann_with_sensor.xacro ==="
cat $PKG_SRC/gazebo/ackermann_with_sensor.xacro

echo ""
echo "=== gazebo/sensor.xacro ==="
cat $PKG_SRC/gazebo/sensor.xacro
