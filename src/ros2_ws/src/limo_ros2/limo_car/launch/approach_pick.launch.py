"""APPROACH + PICK — the full perception-driven demo (no hard-coded pose, no nav stack yet).

The robot spawns BACK from the table, drives up to it with the HSV visual-servo
(box_follower), parks at grasp range, then performs the calibrated perception-driven
top-down pick (arm_grasp_test reads /box_pose).

Run alongside move_group:
    Terminal 1:  ros2 launch limo_car approach_pick.launch.py
    Terminal 2:  ros2 launch limo_cobot_moveit_config move_group.launch.py

Sequence (TimerAction, tune the periods if your approach is faster/slower):
    sim + robot(back) -> spawn table/box -> perception + weld -> box_follower drives & parks
    -> base_pin holds the parked base -> grasp brain picks using the perceived /box_pose.

Tuning knobs:
  * spawn_y below: how far back the robot starts (7.4 ~= 0.6 m approach; 6.79 = table).
  * the t=24 / t=26 periods: must be AFTER box_follower has parked ("Target reached").
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('limo_car')
    sim_time = {'use_sim_time': True}

    drive_mode_arg = DeclareLaunchArgument(
        'drive_mode', default_value='diff',
        description='Drive mode (ackermann | diff)')

    # Robot starts BACK (y=7.4) so it must drive ~0.6 m up to the table at y=6.79.
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'ackermann_gazebo.launch.py')),
        launch_arguments={
            'spawn_y': '7.4',
            'drive_mode': LaunchConfiguration('drive_mode'),
        }.items())

    table_sdf = os.path.join(pkg, 'worlds', 'test_table.sdf')
    box_sdf = os.path.join(pkg, 'worlds', 'test_box.sdf')
    spawn_py = '/opt/ros/humble/lib/gazebo_ros/spawn_entity.py'

    spawn_table = ExecuteProcess(
        cmd=['/usr/bin/python3', spawn_py, '-file', table_sdf, '-entity', 'grasp_test_table',
             '-x', '-2.0', '-y', '6.79', '-z', '0.10'], output='screen')
    spawn_box = ExecuteProcess(
        cmd=['/usr/bin/python3', spawn_py, '-file', box_sdf, '-entity', 'grasp_test_box',
             '-x', '-2.0', '-y', '6.79', '-z', '0.215'], output='screen')

    perception = Node(package='limo_car', executable='box_pose_estimator',
                      name='box_pose_estimator', output='screen', parameters=[sim_time])
    attacher = Node(package='limo_car', executable='grasp_attacher',
                    name='grasp_attacher', output='screen', parameters=[sim_time])
    follower = Node(package='limo_car', executable='box_follower',
                    name='box_follower', output='screen', parameters=[sim_time])
    base_pin = Node(package='limo_car', executable='base_pin', name='base_pin',
                    output='screen', parameters=[sim_time])
    grasp = Node(package='limo_car', executable='arm_grasp_test',
                 name='arm_grasp_test', output='screen', parameters=[sim_time])

    return LaunchDescription([
        drive_mode_arg,
        sim,
        TimerAction(period=8.0,  actions=[spawn_table]),
        TimerAction(period=10.0, actions=[spawn_box]),
        TimerAction(period=11.0, actions=[perception, attacher]),
        TimerAction(period=12.0, actions=[follower]),   # drive up to the box and park
        TimerAction(period=24.0, actions=[base_pin]),   # hold the (now parked) base for the grasp
        TimerAction(period=26.0, actions=[grasp]),      # perception-driven top-down pick
    ])
