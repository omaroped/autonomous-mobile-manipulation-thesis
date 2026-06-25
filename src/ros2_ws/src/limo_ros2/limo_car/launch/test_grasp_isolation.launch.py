"""
test_grasp_isolation.launch.py — Fast gripper calibration environment.

Skips Nav2 entirely. Spawns robot pre-docked at world y=4.24 so
the box is already at x≈0.24 m in base_link (exactly the dock distance).

What launches:
  t=0   Gazebo + robot + controllers (robot pre-docked, box on table)
  t=8   MoveIt move_group
  t=15  grasp_attacher  (Gazebo weld service)
  t=15  smart_grasp     (auto-weld on bilateral contact)
  t=20  calib_pick      (N-shot grasp loop with Gazebo ground truth)

One iteration takes ~25 s (ready + hover + descend + close + lift + verify).
With calib_loops=10 you get a full friction/offset sweep in ~5 minutes.

Usage:
    ros2 launch limo_car test_grasp_isolation.launch.py
    ros2 launch limo_car test_grasp_isolation.launch.py loops:=20
    ros2 launch limo_car test_grasp_isolation.launch.py loops:=10 close_sec:=3.0

Live-tune (no relaunch — takes effect next iteration):
    ros2 param set /calib_pick grasp_off_x  0.010
    ros2 param set /calib_pick grasp_off_y -0.005
    ros2 param set /calib_pick grasp_off_z  0.000
    ros2 param set /calib_pick close_sec    3.0

Spawn geometry:
    Box at world (-2, 4.0, 0.12).  Robot spawns at world (-2, 4.24, 0),
    yaw=-π/2 (facing -Y toward table).  Box in base_link ≈ (0.24, 0, -0.02).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg    = get_package_share_directory('limo_car')
    moveit = get_package_share_directory('limo_cobot_moveit_config')

    loops     = LaunchConfiguration('loops',     default='5')
    close_sec = LaunchConfiguration('close_sec', default='4.0')
    use_gzclient = LaunchConfiguration('use_gzclient', default='true')

    loops_arg = DeclareLaunchArgument(
        'loops', default_value='5',
        description='Number of pick-and-verify trials to run')
    close_sec_arg = DeclareLaunchArgument(
        'close_sec', default_value='4.0',
        description='Gripper close duration (s). Slower = gentler contact.')
    use_gzclient_arg = DeclareLaunchArgument(
        'use_gzclient', default_value='true',
        description='Whether to launch Gazebo GUI')

    # ── 1. Gazebo — robot spawned at dock position (y=4.24), no driving needed
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'ackermann_gazebo.launch.py')),
        launch_arguments={
            'spawn_y':    '4.24',    # box at world y=4.0 → box at base_link x=0.24
            'use_rviz':   'false',
            'use_gzclient': use_gzclient,
            'drive_mode': 'diff',
        }.items())

    # ── 2. MoveIt — delay 8 s for controllers
    moveit_launch = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit, 'launch', 'moveit.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'use_rviz':     'false',   # no RViz in calibration mode
            }.items())])

    # ── 3. grasp_attacher — delay 15 s
    grasp_attacher = TimerAction(
        period=15.0,
        actions=[Node(
            package='limo_car',
            executable='grasp_attacher',
            name='grasp_attacher',
            output='screen',
            parameters=[{'use_sim_time': True}])])

    # ── 4. smart_grasp — delay 15 s
    smart_grasp = TimerAction(
        period=15.0,
        actions=[Node(
            package='limo_car',
            executable='smart_grasp',
            name='smart_grasp',
            output='screen',
            parameters=[{'use_sim_time': True}])])

    # ── 5. calib_pick — delay 20 s (move_group must be ready first)
    calib_pick = TimerAction(
        period=20.0,
        actions=[Node(
            package='limo_car',
            executable='calib_pick',
            name='calib_pick',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'loops':        ParameterValue(loops,     value_type=int),
                'close_sec':    ParameterValue(close_sec, value_type=float),
            }])])

    return LaunchDescription([
        loops_arg,
        close_sec_arg,
        use_gzclient_arg,
        gazebo,
        moveit_launch,
        grasp_attacher,
        smart_grasp,
        calib_pick,
    ])
