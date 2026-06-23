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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, LifecycleNode


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
        parameters=[params_file, {'use_sim_time': use_sim_time}])

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

    lifecycle_amcl = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen', condition=is_amcl,
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'bond_timeout': 0.0,
                     'node_names': ['map_server', 'amcl'] + nav_nodes}])

    lifecycle_gt = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen', condition=is_gt,
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'bond_timeout': 0.0,
                     'node_names': ['map_server'] + nav_nodes}])

    return LaunchDescription([
        declare_use_sim, declare_drive_mode, declare_localization,
        map_server,
        amcl, gt_map_to_odom,
        planner, controller, smoother, behaviors, bt_nav, waypoint, vel_smoother,
        lifecycle_amcl, lifecycle_gt,
    ])
