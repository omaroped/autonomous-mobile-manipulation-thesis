# ══════════════════════════════════════════════════════════════
# DEMO COMMANDS — copy-paste each block into its own terminal
# Every block kills old stuff + sources + runs. Self-contained.
# ══════════════════════════════════════════════════════════════


# ┌────────────────────────────────────────────────────────────┐
# │  TEST A: GAZEBO + TELEOP (drive around + move arm)        │
# └────────────────────────────────────────────────────────────┘

# Terminal 1 — Gazebo
pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f move_group; sleep 2
cd ~/Desktop/Thesisorg/src/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch limo_car ackermann_gazebo.launch.py

# Terminal 2 — Teleop
cd ~/Desktop/Thesisorg/src/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run limo_car limo_mycobot_teleop


# ┌────────────────────────────────────────────────────────────┐
# │  TEST B: ARM GRASP (fixed pose, no navigation)            │
# └────────────────────────────────────────────────────────────┘

# Terminal 1 — Gazebo (robot at table)
pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f move_group; sleep 2
cd ~/Desktop/Thesisorg/src/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch limo_car arm_grasp_test.launch.py

# Terminal 2 — MoveIt
cd ~/Desktop/Thesisorg/src/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch limo_cobot_moveit_config move_group.launch.py

# Terminal 3 — Run the grasp
cd ~/Desktop/Thesisorg/src/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch limo_car arm_grasp_run.launch.py


# ┌────────────────────────────────────────────────────────────┐
# │  TEST C: VISUAL APPROACH + PERCEPTION GRASP (not hardcoded)│
# │  Robot starts far, uses camera to drive to box, then picks │
# └────────────────────────────────────────────────────────────┘

# Terminal 1 — Gazebo + perception + visual servo (all-in-one)
pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f move_group; sleep 2
cd ~/Desktop/Thesisorg/src/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch limo_car approach_pick.launch.py

# Terminal 2 — MoveIt
cd ~/Desktop/Thesisorg/src/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch limo_cobot_moveit_config move_group.launch.py


# ┌────────────────────────────────────────────────────────────┐
# │  TEST D: NAVIGATION ONLY (Nav2 drives to table)           │
# └────────────────────────────────────────────────────────────┘

# Terminal 1 — Gazebo + Nav2 + RViz
pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f move_group; pkill -9 -f rviz2; sleep 2
cd ~/Desktop/Thesisorg/src/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch limo_car nav_pick.launch.py

# Terminal 2 — Set initial pose + send goal
cd ~/Desktop/Thesisorg/src/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 topic pub -t 5 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: 'map'}, pose: {pose: {position: {x: -2.0, y: 7.0, z: 0.0}, orientation: {z: -0.7071, w: 0.7071}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.06]}}"
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: -2.0, y: 4.6, z: 0.0}, orientation: {z: -0.7071, w: 0.7071}}}}"


# ┌────────────────────────────────────────────────────────────┐
# │  TEST E: FULL PIPELINE (navigate → dock → perceive → grasp)│
# └────────────────────────────────────────────────────────────┘

# Terminal 1 — Gazebo + Nav2 + RViz
pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f move_group; pkill -9 -f rviz2; sleep 2
cd ~/Desktop/Thesisorg/src/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch limo_car nav_pick.launch.py

# Terminal 2 — MoveIt
cd ~/Desktop/Thesisorg/src/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch limo_cobot_moveit_config moveit.launch.py

# Terminal 3 — Orchestrator (runs the whole sequence)
cd ~/Desktop/Thesisorg/src/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run limo_car nav_pick_orchestrator


# ┌────────────────────────────────────────────────────────────┐
# │  KILL EVERYTHING (run between tests)                      │
# └────────────────────────────────────────────────────────────┘
pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f move_group; pkill -9 -f rviz2; pkill -9 -f nav_pick_orchestrator; pkill -9 -f box_follower; pkill -9 -f base_pin
