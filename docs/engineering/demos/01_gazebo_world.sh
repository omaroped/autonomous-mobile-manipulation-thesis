#!/bin/bash
# ============================================================
# DEMO 1: Gazebo World + Robot Model
# ============================================================
# WHAT IT SHOWS:
#   - The Gazebo simulation environment (warehouse-like world)
#   - The combined LIMO + myCobot robot model
#   - The pickup table with the blue target box
#   - All sensors mounted (LiDAR, depth camera, IMU)
#
# WHAT TO TELL THE PROFESSOR:
#   "This is our complete simulation environment in Gazebo.
#    The robot model combines the LIMO PRO base with the
#    myCobot 280 arm, adaptive gripper, depth camera, and
#    LiDAR. All physical parameters — track width, wheelbase,
#    camera FOV, LiDAR range — are audited against the real
#    manufacturer specs."
#
# TIME: ~2 min (20s to load, rest is explaining)
# ============================================================

# Clean any previous session
pkill -9 -f gzserver 2>/dev/null
pkill -9 -f gzclient 2>/dev/null
pkill -9 -f move_group 2>/dev/null
pkill -9 -f rviz2 2>/dev/null
sleep 2

# Source workspace
cd ~/Desktop/Thesisorg/src/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash

echo "============================================"
echo "  DEMO 1: Launching Gazebo simulation..."
echo "============================================"
echo ""
echo "  Wait ~20 seconds for Gazebo to load."
echo "  You should see:"
echo "    - The LIMO robot with the arm on top"
echo "    - A small table with a blue box"
echo "    - The warehouse-like room"
echo ""
echo "  Useful things to show in Gazebo:"
echo "    - Zoom in on the robot to show arm + gripper"
echo "    - Click the robot to show the link tree"
echo "    - Show the LiDAR scan (green dots)"
echo ""

# Launch the simulation (Ackermann base + sensors + arm + world)
ros2 launch limo_car ackermann_gazebo.launch.py
