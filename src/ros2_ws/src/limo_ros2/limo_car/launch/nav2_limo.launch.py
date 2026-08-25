"""
nav2_limo.launch.py — bring up Nav2 for the LIMO simulation.

Localization is switchable via the `localization` arg:
  * ground_truth (default, sim): a static map->odom identity transform. Because the
    diff_drive plugin runs with odometry_source=WORLD, odom == world == map, so a
    static identity map->odom makes map->base equal the robot's TRUE pose. The robot
    therefore always knows exactly where it is, wherever you place it — no AMCL, no
    initial-pose seeding, no "identical places" confusion. (Sim convenience.)
  * amcl (sim-to-real faithful): real LiDAR-vs-map localization (drifting odom on HW).

Drive mode (ackermann | diff) selects the matching nav2_limo_<mode>.yaml.
Uses thesis_map.yaml (pre-built SLAM map).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, LifecycleNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('limo_car')

    drive_mode   = LaunchConfiguration('drive_mode')
    localization = LaunchConfiguration('localization')

    params_file = PythonExpression(["'", os.path.join(pkg, 'config', 'nav2_limo_'), drive_mode, ".yaml'"])
    map_yaml    = os.path.join(pkg, 'maps', 'thesis_map.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    is_amcl = IfCondition(PythonExpression(["'", localization, "' == 'amcl'"]))
    is_gt   = IfCondition(PythonExpression(["'", localization, "' == 'ground_truth'"]))

    declare_use_sim = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation clock')
    declare_drive_mode = DeclareLaunchArgument(
        'drive_mode', default_value='diff', description='Drive mode (ackermann | diff)')
    # AMCL's initial guess. Defaults match nav2_limo_diff.yaml's set_initial_pose
    # block, so launching this file standalone behaves exactly as before; when
    # nav_pick.launch.py drives it, these carry scene.yaml's robot.spawn_pose so the
    # seed and the actual spawn can no longer disagree.
    declare_init_x = DeclareLaunchArgument(
        'initial_pose_x', default_value='-2.0',
        description="AMCL initial pose X (ignored under localization:=ground_truth)")
    declare_init_y = DeclareLaunchArgument(
        'initial_pose_y', default_value='7.0',
        description="AMCL initial pose Y (ignored under localization:=ground_truth)")
    declare_init_yaw = DeclareLaunchArgument(
        'initial_pose_yaw', default_value='-1.5708',
        description="AMCL initial pose yaw (ignored under localization:=ground_truth)")

    declare_localization = DeclareLaunchArgument(
        'localization', default_value='ground_truth',
        description='ground_truth (static map->odom, sim) | amcl (LiDAR localization)')

    # ── Map server (always) ───────────────────────────────────────────────────
    map_server = LifecycleNode(
        package='nav2_map_server', executable='map_server', name='map_server',
        namespace='', output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}])

    # ── Localization option A: AMCL (real LiDAR localization) ─────────────────
    amcl = LifecycleNode(
        package='nav2_amcl', executable='amcl', name='amcl',
        namespace='', output='screen', condition=is_amcl,
        # The dict entries override the yaml (later parameter sources win), so the
        # seed follows the launch arguments rather than the frozen constants in
        # nav2_limo_diff.yaml. Nested yaml keys are addressed with dotted names.
        parameters=[params_file, {
            'use_sim_time':      use_sim_time,
            'set_initial_pose':  True,
            'initial_pose.x':    ParameterValue(LaunchConfiguration('initial_pose_x'),
                                                value_type=float),
            'initial_pose.y':    ParameterValue(LaunchConfiguration('initial_pose_y'),
                                                value_type=float),
            'initial_pose.z':    0.0,
            'initial_pose.yaw':  ParameterValue(LaunchConfiguration('initial_pose_yaw'),
                                                value_type=float),
        }])

    # ── Localization option B: ground truth (static map->odom identity) ───────
    # odom == world == map in sim (diff_drive odometry_source=WORLD), so identity
    # makes map->base = the robot's true pose, wherever it is placed.
    gt_map_to_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='gt_map_to_odom', output='screen', condition=is_gt,
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}])

    # ── Planner / Controller / Smoother / Behaviours / BT / Waypoint / VelSmooth
    def lnode(p, e, n):
        return LifecycleNode(package=p, executable=e, name=n, namespace='',
                             output='screen',
                             parameters=[params_file, {'use_sim_time': use_sim_time}])

    planner      = lnode('nav2_planner',           'planner_server',    'planner_server')
    controller   = lnode('nav2_controller',        'controller_server', 'controller_server')
    smoother     = lnode('nav2_smoother',          'smoother_server',   'smoother_server')
    behaviors    = lnode('nav2_behaviors',         'behavior_server',   'behavior_server')
    bt_nav       = lnode('nav2_bt_navigator',      'bt_navigator',      'bt_navigator')
    waypoint     = lnode('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower')
    vel_smoother = lnode('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother')

    # ── Lifecycle managers — one WITH amcl (amcl mode), one WITHOUT (gt mode) ──
    nav_nodes = ['planner_server', 'controller_server', 'smoother_server',
                 'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother']

    # Wait for map_server's OWN service to exist before starting the lifecycle
    # manager, instead of guessing a fixed delay. The old code waited a flat 6 s;
    # on a loaded machine (Gazebo + RViz + camera all starting together) that
    # guess sometimes wasn't enough, the lifecycle manager's change_state call
    # timed out before map_server had even registered its service, Nav2
    # bring-up aborted silently, and RViz showed no costmap with the robot
    # refusing to move — intermittent, because it depended on machine load at
    # that exact moment. This waits for the real event instead of the clock.
    wait_for_map_server = Node(
        package='limo_car', executable='wait_for_lifecycle_node',
        name='wait_for_map_server', output='screen',
        arguments=['map_server'])

    lifecycle_amcl_node = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen', condition=is_amcl,
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'bond_timeout': 0.0,
                     'node_names': ['map_server', 'amcl'] + nav_nodes}])

    lifecycle_gt_node = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen', condition=is_gt,
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'bond_timeout': 0.0,
                     'node_names': ['map_server'] + nav_nodes}])

    # Both candidate lifecycle managers are attached to the SAME wait process —
    # only one of the two actually starts, gated by is_amcl/is_gt on the Node
    # itself, exactly as before.
    start_lifecycle_managers = RegisterEventHandler(OnProcessExit(
        target_action=wait_for_map_server,
        on_exit=[lifecycle_amcl_node, lifecycle_gt_node]))

    return LaunchDescription([
        declare_use_sim, declare_drive_mode, declare_localization,
        declare_init_x, declare_init_y, declare_init_yaw,
        map_server,
        amcl, gt_map_to_odom,
        planner, controller, smoother, behaviors, bt_nav, waypoint, vel_smoother,
        wait_for_map_server, start_lifecycle_managers,
    ])
