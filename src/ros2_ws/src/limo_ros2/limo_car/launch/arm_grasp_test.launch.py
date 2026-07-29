"""ISOLATED arm grasp test — sim + robot + a reachable test box (NO driving).

This brings up the EXACT same simulation as the real pipeline
(ackermann_gazebo.launch.py: final_map.world + the LIMO+myCobot robot +
controllers + RViz), then spawns a thin pedestal with a blue 5 cm box right in
front of the parked arm — ~0.22 m ahead, comfortably inside the arm's reach.

The robot never drives. Pair this with:
    ros2 launch limo_cobot_moveit_config moveit.launch.py   # move_group
    ros2 launch limo_car arm_grasp_run.launch.py            # perception + grasp

Nothing here modifies the real map; the test box is spawned at runtime and the
robot stays at its normal spawn (-2, 7) facing -Y, so "forward" is world -Y.
Target placed 0.22 m in front of the arm  ->  world (-2, 6.78).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('limo_car')

    drive_mode_arg = DeclareLaunchArgument(
        'drive_mode', default_value='diff',
        description='Drive mode (ackermann | diff)')

    # use_rviz=false: MoveIt brings its own RViz in moveit.launch.py. Running a
    # second RViz here on top of gzclient starves the GPU and segfaults gzserver's
    # depth-camera renderer on spawn — so we skip it.
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'ackermann_gazebo.launch.py')),
        launch_arguments={
            'use_rviz': 'false',
            'drive_mode': LaunchConfiguration('drive_mode')
        }.items())

    table_sdf = os.path.join(pkg, 'worlds', 'test_table.sdf')
    box_sdf = os.path.join(pkg, 'worlds', 'test_box.sdf')
    spawn_py = '/opt/ros/humble/lib/gazebo_ros/spawn_entity.py'

    # Box on a thin TABLE at the arm's reachable top-down zone: base_link
    # (0.25, 0, 0.08) -> world (-2, 6.75, 0.21), robot at (-2,7) facing -Y.
    # Table top at z=0.20; the 2 cm box sits on it. Table is narrow in y so it
    # clears the robot's front. Realistic (not floating), and reachable.
    # Moved +0.04 m further out than the original 6.79: the robot's own body
    # collision (0.16 m half-extent) left only ~2 cm clearance to the table's
    # near face at 6.79, which in practice was enough to jitter/collide on
    # spawn settling. This gives ~6 cm clearance instead.
    spawn_table = ExecuteProcess(
        cmd=['/usr/bin/python3', spawn_py,
             '-file', table_sdf, '-entity', 'grasp_test_table',
             '-x', '-2.0', '-y', '6.75', '-z', '0.10'],   # 0.20-tall table -> top at 0.20
        output='screen')
    spawn_box = ExecuteProcess(
        cmd=['/usr/bin/python3', spawn_py,
             '-file', box_sdf, '-entity', 'grasp_test_box',
             '-x', '-2.0', '-y', '6.75', '-z', '0.215'],    # 3 cm box on the table top (z=0.20 + half 0.015)
        output='screen')

    # Pin the base so the arm's reaction can't roll it (drift) or launch it on a
    # contact. Starts after the robot has spawned + settled.
    # override_x/y pin the base at the KNOWN spawn (-2, 7.0) so the box lands at the
    # reachable ~0.21 m even if the base drifted on spawn (otherwise it can park ~8 cm
    # back -> box at 0.27 m -> beyond top-down reach).
    base_pin = Node(package='limo_car', executable='base_pin', name='base_pin',
                    output='screen',
                    parameters=[{'use_sim_time': True, 'override_x': -2.0, 'override_y': 7.0}])

    return LaunchDescription([
        drive_mode_arg,
        sim,
        # base_pin FIRST, well before the table spawns — it only needs ~3 s internally to
        # let the robot settle on its wheels (see base_pin.py). Previously this engaged at
        # t=12s while the table spawned at t=8s: with only ~2 cm nominal clearance between
        # the robot's own body and the table, that 4 s unpinned window let any contact
        # bounce the free-rolling base repeatedly (the "hitting it over and over" jitter).
        TimerAction(period=1.0, actions=[base_pin]),
        TimerAction(period=8.0, actions=[spawn_table]),
        TimerAction(period=10.0, actions=[spawn_box]),
    ])
