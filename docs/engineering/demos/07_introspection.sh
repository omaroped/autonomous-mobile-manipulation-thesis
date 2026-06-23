#!/bin/bash
# ============================================================
# DEMO 7: System Introspection (show the professor the guts)
# ============================================================
# WHAT IT SHOWS:
#   - ROS 2 node graph (all running nodes)
#   - Topic list (what's publishing)
#   - TF tree (coordinate frames)
#   - Controller list (arm + base controllers)
#   - MoveIt planning scene
#   - URDF structure
#
# WHAT TO TELL THE PROFESSOR:
#   "Let me show you what's running under the hood.
#    These are all the ROS 2 nodes, topics, and TF frames
#    that make up the system. You can see the controllers
#    for the base and arm running together, the sensor
#    topics, the Nav2 lifecycle nodes, and the full
#    coordinate transform tree."
#
# PREREQ: Any demo must be running (Gazebo + nodes up)
# TIME: ~2 min (just reading terminal output)
# ============================================================

source /opt/ros/humble/setup.bash
source ~/Desktop/Thesisorg/src/ros2_ws/install/setup.bash

echo "============================================"
echo "  DEMO 7: System Introspection"
echo "============================================"

echo ""
echo "═══ 1. ACTIVE ROS 2 NODES ═══"
ros2 node list 2>/dev/null
echo ""

echo "═══ 2. KEY TOPICS (filtered) ═══"
ros2 topic list 2>/dev/null | grep -E "cmd_vel|box_pose|joint_states|scan|image|camera_info|map|amcl|plan|gripper"
echo ""

echo "═══ 3. ACTIVE CONTROLLERS ═══"
ros2 control list_controllers 2>/dev/null
echo ""

echo "═══ 4. TF FRAMES (base_link → gripper_tcp) ═══"
timeout 3 ros2 run tf2_ros tf2_echo base_link gripper_tcp 2>/dev/null | head -5
echo ""

echo "═══ 5. ROBOT DESCRIPTION (link count) ═══"
LINKS=$(timeout 3 ros2 topic echo /robot_description --once 2>/dev/null | grep -c "<link")
JOINTS=$(timeout 3 ros2 topic echo /robot_description --once 2>/dev/null | grep -c "<joint")
echo "  Links: $LINKS   Joints: $JOINTS"
echo ""

echo "═══ 6. NAV2 LIFECYCLE STATES ═══"
for node in amcl planner_server controller_server bt_navigator velocity_smoother; do
    state=$(ros2 lifecycle get /$node 2>/dev/null)
    echo "  $node: $state"
done
echo ""

echo "═══ 7. SENSOR STATUS ═══"
echo "  LiDAR (/scan):"
timeout 2 ros2 topic hz /scan 2>/dev/null | head -1
echo "  Depth camera (/depth_camera/depth/image_raw):"
timeout 2 ros2 topic hz /depth_camera/depth/image_raw 2>/dev/null | head -1
echo "  RGB camera (/depth_camera/image_raw):"
timeout 2 ros2 topic hz /depth_camera/image_raw 2>/dev/null | head -1
echo ""

echo "  Done! This shows the full system architecture running live."
