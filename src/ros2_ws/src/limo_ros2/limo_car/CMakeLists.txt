cmake_minimum_required(VERSION 3.8)
project(limo_car)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config launch gazebo log meshes rviz src urdf worlds scripts maps media
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/box_follower.py
  RENAME box_follower DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/box_pose_estimator.py
  RENAME box_pose_estimator DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/base_joint_state_pub.py
  RENAME base_joint_state_pub DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/arm_grasp_test.py
  RENAME arm_grasp_test DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/base_pin.py
  RENAME base_pin DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/grasp_attacher.py
  RENAME grasp_attacher DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/reach_sweep.py
  RENAME reach_sweep DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/nav_pick_orchestrator.py
  RENAME nav_pick_orchestrator DESTINATION lib/${PROJECT_NAME})
# No RENAME: nav_pick_orchestrator imports this by module name ("import reach_lookup"),
# which only resolves at runtime if the file keeps its .py name in the same install dir.
install(PROGRAMS scripts/reach_lookup.py DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/smart_grasp.py
  RENAME smart_grasp DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/calibrate_contact_angle.py
  RENAME calibrate_contact_angle DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/calib_pick.py
  RENAME calib_pick DESTINATION lib/${PROJECT_NAME})

# ── Fiducial place pipeline ────────────────────────────────────────────────
install(PROGRAMS scripts/tag_dock_estimator.py
  RENAME tag_dock_estimator DESTINATION lib/${PROJECT_NAME})
# gen_apriltag.py is a one-shot generator (run once, not a ROS node)
install(PROGRAMS scripts/gen_apriltag.py
  RENAME gen_apriltag DESTINATION lib/${PROJECT_NAME})
# gen_stack_box.py is a one-shot mesh generator (run once, not a ROS node)
install(PROGRAMS scripts/gen_stack_box.py
  RENAME gen_stack_box DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/test_place_arm.py
  RENAME test_place_arm DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/cam_track_pick_test.py
  RENAME cam_track_pick_test DESTINATION lib/${PROJECT_NAME})

# grip_bench.py — isolated grasp test rig (grip_bench.launch.py). Teleports the box
# between the fingers each trial so the grip is measured with zero centring error.
install(PROGRAMS scripts/grip_bench.py
  RENAME grip_bench DESTINATION lib/${PROJECT_NAME})

# wait_for_lifecycle_node.py — blocks nav2_limo.launch.py's lifecycle manager
# start until map_server is actually ready, replacing a fixed 6 s guess that
# intermittently fired too early on a loaded machine.
install(PROGRAMS scripts/wait_for_lifecycle_node.py
  RENAME wait_for_lifecycle_node DESTINATION lib/${PROJECT_NAME})

# (mecanum_bridge.py intentionally NOT installed — it was only for the abandoned
#  mecanum/planar_move drive; the diff-drive base needs no cmd_vel/odom bridge.)

ament_package()
