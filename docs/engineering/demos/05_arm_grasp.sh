#!/bin/bash
# ============================================================
# DEMO 5: Arm Grasp (MoveIt Pick — Fixed Pose, No Navigation)
# ============================================================
# WHAT IT SHOWS:
#   - MoveIt 2 motion planning with collision avoidance
#   - The table registered as a collision object in the
#     planning scene (arm routes around it, not through it)
#   - Full grasp sequence: ready → open → pre-grasp → grasp
#     → close + weld → lift → retract
#   - The custom collision matrix (253→125 pairs) that makes
#     planning possible on the combined robot
#
# WHAT TO TELL THE PROFESSOR:
#   "This demo shows the arm grasping at a known position —
#    no navigation, the robot starts right in front of the box.
#    MoveIt 2 plans the arm trajectory while avoiding the table
#    (which I register as a collision object). The grasp is
#    top-down because my reachability sweep showed that's
#    the only reliable orientation for this arm length.
#    Getting MoveIt to plan at all required manually fixing
#    the collision matrix — the base's coarse collision box
#    overlapped with the arm's meshes, causing phantom
#    collisions that blocked all planning."
#
# PREREQ: Nothing running — this starts its own Gazebo
# TIME: ~3 min
# ============================================================

# Clean everything
pkill -9 -f gzserver 2>/dev/null; pkill -9 -f gzclient 2>/dev/null
pkill -9 -f move_group 2>/dev/null; pkill -9 -f rviz2 2>/dev/null
sleep 2

cd ~/Desktop/Thesisorg/src/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash

echo "============================================"
echo "  DEMO 5: Arm Grasp Test"
echo "============================================"
echo ""
echo "  This runs in 3 steps. Follow along."
echo ""

# --- STEP 1: Gazebo ---
echo "  STEP 1/3: Launching Gazebo (robot at table)..."
echo "            Wait ~20s for the world to load."
echo ""
ros2 launch limo_car arm_grasp_test.launch.py &
SIM_PID=$!
sleep 25

# --- STEP 2: MoveIt ---
echo ""
echo "  STEP 2/3: Launching MoveIt (move_group)..."
echo "            Wait for 'You can start planning now!'"
echo ""
ros2 launch limo_cobot_moveit_config move_group.launch.py &
MOVEIT_PID=$!
sleep 15

# --- STEP 3: Grasp ---
echo ""
echo "  STEP 3/3: Running the grasp sequence!"
echo ""
echo "  >>> WATCH GAZEBO — the arm will pick up the box! <<<"
echo ""
echo "  Sequence: ready → open gripper → hover above box"
echo "          → descend to box → close gripper + weld"
echo "          → LIFT → hold → place back → release → home"
echo ""

ros2 launch limo_car arm_grasp_run.launch.py

echo ""
echo "  Grasp demo complete!"
echo "  Press Ctrl+C to stop everything."
kill $MOVEIT_PID $SIM_PID 2>/dev/null
wait 2>/dev/null
