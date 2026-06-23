#!/bin/bash
# ============================================================
# DEMO 3: Autonomous Navigation (Nav2)
# ============================================================
# WHAT IT SHOWS:
#   - Nav2 full stack (AMCL localisation, costmap, planner)
#   - SmacPlannerHybrid generating Ackermann-feasible paths
#   - RegulatedPurePursuitController following the path
#   - The robot autonomously driving ~3m to the workstation
#   - RViz showing the map, costmap, and planned path
#
# WHAT TO TELL THE PROFESSOR:
#   "This is the autonomous navigation. I use Nav2 with
#    SmacPlannerHybrid — that's the only planner in Nav2
#    that generates paths a car-like robot can actually
#    follow (no in-place rotations). The robot localises
#    with AMCL on a pre-built SLAM map, plans a path to
#    the workstation, and drives there autonomously.
#    Watch — it never tries to spin in place because it
#    knows it's an Ackermann base with a 0.4m turning radius."
#
# PREREQ: Kill Demo 1 first, this launches its own Gazebo
# TIME: ~3 min
# ============================================================

# Clean
pkill -9 -f gzserver 2>/dev/null; pkill -9 -f gzclient 2>/dev/null
pkill -9 -f rviz2 2>/dev/null; pkill -9 -f nav2 2>/dev/null
sleep 2

cd ~/Desktop/Thesisorg/src/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash

echo "============================================"
echo "  DEMO 3: Autonomous Navigation"
echo "============================================"
echo ""
echo "  STEP 1: Launch simulation + Nav2 + RViz"
echo "          (wait ~30s for everything to load)"
echo ""

# Launch Gazebo + Nav2 + RViz together
ros2 launch limo_car nav_pick.launch.py &
NAV_PID=$!
sleep 30

echo ""
echo "  STEP 2: Setting initial pose for AMCL..."
echo ""

# Tell AMCL where the robot is (spawn pose: -2, 7, yaw=-90deg)
ros2 topic pub -t 5 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: 'map'}, pose: {pose: {position: {x: -2.0, y: 7.0, z: 0.0}, orientation: {z: -0.7071, w: 0.7071}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.06]}}" > /dev/null 2>&1

sleep 2

echo "  STEP 3: Sending navigation goal (workstation at -2, 4.6)..."
echo ""
echo "  >>> WATCH GAZEBO — the robot will drive to the table! <<<"
echo "  >>> WATCH RVIZ  — you can see the planned path!       <<<"
echo ""

# Send nav goal to the workstation
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: 'map'}, pose: {position: {x: -2.0, y: 4.6, z: 0.0}, orientation: {z: -0.7071, w: 0.7071}}}}"

echo ""
echo "  Navigation complete! The robot should be near the table."
echo "  Press Ctrl+C to stop."
wait $NAV_PID
