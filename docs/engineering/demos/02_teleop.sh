#!/bin/bash
# ============================================================
# DEMO 2: Keyboard Teleop (Base + Arm + Gripper)
# ============================================================
# WHAT IT SHOWS:
#   - Manual keyboard control of the LIMO base (WASD)
#   - Manual control of each arm joint (arrow keys)
#   - Gripper open/close
#   - That all controllers (base, arm, gripper) work together
#
# WHAT TO TELL THE PROFESSOR:
#   "Before any autonomy, I built a teleop interface to verify
#    that all the controllers work — base driving, arm joints,
#    and gripper. This was the first integration milestone:
#    getting the LIMO base controllers and the myCobot arm
#    controllers to run under one ros2_control instance.
#    This required patching the gazebo_ros2_control C++ plugin
#    because of a compatibility bug between Gazebo Classic
#    and ROS 2 Humble's controller_manager."
#
# PREREQ: Demo 1 must be running (Gazebo open)
# TIME: ~3 min
# ============================================================

# Source workspace
cd ~/Desktop/Thesisorg/src/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash

echo "============================================"
echo "  DEMO 2: Keyboard Teleop"
echo "============================================"
echo ""
echo "  Controls:"
echo "    W/S     = drive forward/backward"
echo "    A/D     = steer left/right"
echo "    UP/DOWN = move current arm joint"
echo "    1-6     = select arm joint"
echo "    O       = open gripper"
echo "    C       = close gripper"
echo "    Q       = quit"
echo ""
echo "  Show the professor:"
echo "    1. Drive the robot around (car-like steering!)"
echo "    2. Move the arm to reach over the table"
echo "    3. Open and close the gripper"
echo ""

ros2 run limo_car limo_mycobot_teleop
