#!/bin/bash
# ============================================================
# DEMO 6: Full Pipeline (Navigate → Dock → Perceive → Grasp)
# ============================================================
# WHAT IT SHOWS:
#   - The complete orchestrator running the full sequence:
#     1. AMCL initial pose → Nav2 autonomous navigation
#     2. Visual docking (cmd_vel hand-off from Nav2)
#     3. Base pinning (50Hz Gazebo hold for stable manipulation)
#     4. Perception (box pose from depth camera)
#     5. MoveIt grasp (collision-aware pick)
#     6. Retract and back up
#   - Everything coordinated by a single state machine
#
# WHAT TO TELL THE PROFESSOR:
#   "This is the full autonomous pipeline — no human input.
#    The robot starts at spawn, navigates to the table using
#    Nav2, then hands off cmd_vel control to a visual docking
#    stage (because Nav2's tolerance is too coarse for grasping
#    on an Ackermann base). Then it pins the base, reads the
#    box pose from the camera, and MoveIt plans the grasp.
#    This is where all five challenges I identified come together:
#    the Nav2/docking cmd_vel contention, the non-holonomic
#    approach problem, the depth perception bias, the collision
#    matrix, and the traction/stability tradeoff."
#
# PREREQ: Nothing running — this starts everything fresh
# TIME: ~5 min
# ============================================================

# Clean everything
pkill -9 -f gzserver 2>/dev/null; pkill -9 -f gzclient 2>/dev/null
pkill -9 -f move_group 2>/dev/null; pkill -9 -f rviz2 2>/dev/null
pkill -9 -f nav2 2>/dev/null; pkill -9 -f nav_pick_orchestrator 2>/dev/null
sleep 2

cd ~/Desktop/Thesisorg/src/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash

echo "============================================"
echo "  DEMO 6: Full Autonomous Pipeline"
echo "============================================"
echo ""
echo "  This is the big one. Three terminals needed."
echo ""

# --- TERMINAL 1: Gazebo + Nav2 + RViz ---
echo "  STEP 1/3: Launching Gazebo + Nav2 + RViz..."
echo "            (wait ~30s)"
echo ""
ros2 launch limo_car nav_pick.launch.py &
SIM_PID=$!
sleep 35

# --- TERMINAL 2: MoveIt ---
echo "  STEP 2/3: Launching MoveIt..."
echo "            (wait for 'You can start planning now!')"
echo ""
ros2 launch limo_cobot_moveit_config moveit.launch.py &
MOVEIT_PID=$!
sleep 20

# --- TERMINAL 3: Orchestrator ---
echo ""
echo "  STEP 3/3: Launching the orchestrator!"
echo ""
echo "  >>> WATCH GAZEBO — full autonomous sequence! <<<"
echo ""
echo "  The orchestrator will:"
echo "    1. Publish AMCL initial pose"
echo "    2. Send Nav2 goal → robot drives to table"
echo "    3. Deactivate Nav2 → visual docking takes over"
echo "    4. Pin the base → arm can move stably"
echo "    5. Read /box_pose → get target position"
echo "    6. MoveIt: ready → pre-grasp → grasp → lift"
echo "    7. Back up and finish"
echo ""

ros2 run limo_car nav_pick_orchestrator

echo ""
echo "  ══════════════════════════════════════"
echo "    FULL PIPELINE DEMO COMPLETE"
echo "  ══════════════════════════════════════"
echo ""
echo "  Press Ctrl+C to stop everything."
kill $MOVEIT_PID $SIM_PID 2>/dev/null
wait 2>/dev/null
