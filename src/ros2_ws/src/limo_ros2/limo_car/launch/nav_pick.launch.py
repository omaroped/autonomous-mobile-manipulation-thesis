"""
nav_pick.launch.py — ONE COMMAND to run the full autonomous pick-and-place.

Starts (in dependency order):
  1. Gazebo simulation  (ackermann_gazebo.launch.py — sim + robot + controllers)
  2. MoveIt move_group  (moveit.launch.py)
  3. Nav2 stack         (nav2_limo.launch.py — map, AMCL, planner, controller)
  4. box_pose_estimator (perception → /box_pose)
  5. grasp_attacher     (LEGACY kinematic weld — only if use_physics_grasp:=false)
  6. base_pin           (Gazebo pose-hold service)
  7. nav_pick_orchestrator (the mission node — waits for everything above)

RViz is launched with nav_pick.rviz (all required displays pre-configured).

Usage:
    ros2 launch limo_car nav_pick.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetLaunchConfiguration
)
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg      = get_package_share_directory('limo_car')
    moveit   = get_package_share_directory('limo_cobot_moveit_config')

    spawn_y = LaunchConfiguration('spawn_y', default='7.0')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_gzclient = LaunchConfiguration('use_gzclient', default='true')
    drive_mode = LaunchConfiguration('drive_mode', default='diff')
    calib_loops = LaunchConfiguration('calib_loops', default='0')
    place_stack_levels = LaunchConfiguration('place_stack_levels', default='1')
    dock_range = LaunchConfiguration('dock_range', default='0.15')
    metrics_csv = LaunchConfiguration('metrics_csv', default='')
    use_physics_grasp = LaunchConfiguration('use_physics_grasp', default='true')

    # ── Argument declarations ─────────────────────────────────────────────────
    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y', default_value='7.0',
        description='Robot spawn Y. Default 7.0 → 3 m from the table (nav problem).')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Whether to launch RViz')

    use_gzclient_arg = DeclareLaunchArgument(
        'use_gzclient', default_value='true',
        description='Whether to launch Gazebo client (GUI)')

    drive_mode_arg = DeclareLaunchArgument(
        'drive_mode', default_value='diff',
        description='Drive mode (ackermann | diff)')

    calib_loops_arg = DeclareLaunchArgument(
        'calib_loops', default_value='0',
        description='0 = run once (normal demo). N>0 = calibration mode: after docking, '
                    'repeat the grasp N times, auto-resetting the box, reading grasp_off_* '
                    'params live each iteration so the offset can be tuned without relaunching.')

    place_stack_levels_arg = DeclareLaunchArgument(
        'place_stack_levels', default_value='1',
        description='Number of boxes to pick+stack at the place table in one run '
                    '(the world has 3 distinct stack_box_N boxes on the pickup table).')

    dock_range_arg = DeclareLaunchArgument(
        'dock_range', default_value='0.15',
        description='AprilTag dock stop distance (m) — tuned so latched drop-x '
                    '(dock_range + table_side/2) lands within arm reach (STOP_DISTANCE).')

    metrics_csv_arg = DeclareLaunchArgument(
        'metrics_csv', default_value='',
        description='Path for the per-cycle metrics CSV. Empty = auto-name under data/.')

    use_physics_grasp_arg = DeclareLaunchArgument(
        'use_physics_grasp', default_value='true',
        description='true  = grasping is done by libgazebo_grasp_plugin (a REAL ODE fixed '
                    'joint created on finger contact, released when the gripper opens past '
                    'release_position). grasp_attacher and smart_grasp are NOT started.\n'
                    'false = legacy kinematic weld: grasp_attacher teleports the box to '
                    'follow the gripper via /set_entity_state, driven by /grasp_attach.\n'
                    'The two mechanisms MUST NOT run together — the teleport fights the '
                    'joint solver. This one flag switches both the launch graph and the '
                    "orchestrator's release logic, so they cannot disagree.")

    # Preserve the real top-level use_rviz BEFORE the gazebo include can touch it.
    # ackermann_gazebo.launch.py declares its OWN 'use_rviz' argument too, and the
    # 'use_rviz': 'false' passed into it below overwrites the shared launch-context
    # value for that name — LaunchConfiguration substitutions resolve lazily, so
    # moveit_launch's read of 'use_rviz' 8 s later was silently picking up 'false'
    # instead of this top-level flag, and RViz never launched. Confirmed via a
    # minimal reproduction (same TimerAction pattern alone launched RViz fine —
    # only broke once the gazebo include with its own 'use_rviz' arg was added).
    preserve_use_rviz = SetLaunchConfiguration('moveit_use_rviz', use_rviz)

    # ── 1. Gazebo + robot + controllers ──────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'ackermann_gazebo.launch.py')),
        launch_arguments={
            'spawn_y': spawn_y,
            'use_rviz': 'false',
            'use_gzclient': use_gzclient,
            'drive_mode': drive_mode,
        }.items())


    # ── 2. MoveIt (move_group) — delay 8 s so controllers come up first ──────
    moveit_launch = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit, 'launch', 'moveit.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'use_rviz': LaunchConfiguration('moveit_use_rviz'),
                'rviz_config': os.path.join(pkg, 'rviz', 'nav_monitor.rviz'),
            }.items())])

    # ── 3. Nav2 — delay 12 s so the robot is spawned + controllers ready ─────
    nav2_launch = TimerAction(
        period=12.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'nav2_limo.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'drive_mode': drive_mode,
            }.items())])

    # ── 4. box_pose_estimator — delay 15 s ───────────────────────────────────
    box_estimator = TimerAction(
        period=15.0,
        actions=[Node(
            package='limo_car',
            executable='box_pose_estimator',
            name='box_pose_estimator',
            output='screen',
            parameters=[{'use_sim_time': True}])])

    # ── 4b. tag_dock_estimator — delay 15 s (parallel with box_estimator) ────
    # Detects AprilTag 36h11 on the place table, publishes /place_tag_pose and
    # /place_drop_point. Requires the same camera topics as box_pose_estimator.
    tag_estimator = TimerAction(
        period=15.0,
        actions=[Node(
            package='limo_car',
            executable='tag_dock_estimator',
            name='tag_dock_estimator',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'tag_size': 0.08125,    # printed marker size: 416/512 px of a 0.10 m plate
                'table_side': 0.18,     # square table side length in metres
                'table_top_z': 0.10,    # table top height above ground in metres
            }])])

    # ── 5. grasp_attacher — LEGACY, only when use_physics_grasp:=false ───────
    #    The kinematic weld. Superseded by libgazebo_grasp_plugin (declared in
    #    gazebo/mycobot_ros2_control.xacro), which makes a real ODE fixed joint.
    #    Kept as a working fallback, NOT deleted: if the physics joint misbehaves,
    #    `use_physics_grasp:=false` restores the exact pipeline that produced the
    #    two successful runs on 2026-07-02.
    #
    #    Why it cannot run alongside the plugin: it calls /set_entity_state at
    #    ~50 Hz to teleport the box onto the gripper, while ODE solves the fixed
    #    joint at 1000 Hz. Both would write the box pose every step and fight.
    grasp_attacher = TimerAction(
        period=15.0,
        condition=UnlessCondition(use_physics_grasp),
        actions=[Node(
            package='limo_car',
            executable='grasp_attacher',
            name='grasp_attacher',
            output='screen',
            parameters=[{'use_sim_time': True}])])

    # ── 6. smart_grasp — LEGACY, only when use_physics_grasp:=false ──────────
    #    Watches the finger contact sensors and publishes /grasp_attach True when
    #    both fingers touch — i.e. it is the *trigger* for the weld above, and is
    #    meaningless without it. The C++ plugin does contact detection internally
    #    (grasp_count_threshold), so this node is redundant under physics grasp.
    smart_grasp = TimerAction(
        period=15.0,
        condition=UnlessCondition(use_physics_grasp),
        actions=[Node(
            package='limo_car',
            executable='smart_grasp',
            name='smart_grasp',
            output='screen',
            parameters=[{'use_sim_time': True}])])

    # ── 7. nav_pick_orchestrator — delay 25 s (Nav2 fully active) ────────────
    orchestrator = TimerAction(
        period=25.0,
        actions=[Node(
            package='limo_car',
            executable='nav_pick_orchestrator',
            name='nav_pick_orchestrator',
            output='screen',
            parameters=[{'use_sim_time': True,
                          'calib_loops': ParameterValue(calib_loops, value_type=int),
                          'place_stack_levels': ParameterValue(place_stack_levels, value_type=int),
                          'dock_range': ParameterValue(dock_range, value_type=float),
                          'metrics_csv': metrics_csv,
                          # Same flag that gates the two legacy nodes above, so the
                          # orchestrator's release logic can never disagree with which
                          # grasp mechanism is actually running.
                          'use_physics_grasp': ParameterValue(
                              use_physics_grasp, value_type=bool)}])])

    return LaunchDescription([
        spawn_y_arg,
        use_rviz_arg,
        use_gzclient_arg,
        drive_mode_arg,
        calib_loops_arg,
        place_stack_levels_arg,
        dock_range_arg,
        metrics_csv_arg,
        use_physics_grasp_arg,
        preserve_use_rviz,
        gazebo,
        moveit_launch,
        nav2_launch,
        box_estimator,
        tag_estimator,
        grasp_attacher,
        smart_grasp,
        orchestrator,
    ])
