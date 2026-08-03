"""real_robot.launch.py — bring up the ENTIRE physical LIMO in one command.

Runs on the Jetson (~/limo_ros2_ws/real_robot/). Starts, in order:

  1. limo_base        — chassis driver (odom, /cmd_vel, /limo_status)
  2. ydlidar          — T-mini Pro with limo_tminipro.yaml (±100°, matches sim)
  3. orbbec camera    — DaBai DC1 colour + depth
  4. arm_bridge       — /joint_states + gripper commands → pymycobot serial

Everything was verified individually on 2026-08-03; this only composes them.

Usage:
    cd ~/limo_ros2_ws/real_robot
    ros2 launch ./real_robot.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    lidar_params = os.path.expanduser(
        '~/limo_ros2_ws/src/ydlidar_ros2_driver/params/limo_tminipro.yaml')

    base = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('limo_base'),
                     'launch', 'limo_base.launch.py')))

    lidar = TimerAction(period=3.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ydlidar_ros2_driver'),
                             'launch', 'ydlidar_launch.py')),
            launch_arguments={'params_file': lidar_params}.items())])

    # Use whichever DaBai launch file exists in this OrbbecSDK_ROS2 version.
    orbbec_share = get_package_share_directory('orbbec_camera')
    cam_launch = None
    for name in ('dabai_dc1.launch.py', 'dabai.launch.py', 'astra.launch.py'):
        p = os.path.join(orbbec_share, 'launch', name)
        if os.path.exists(p):
            cam_launch = p
            break
    if cam_launch is None:
        raise RuntimeError(f'no DaBai launch file found in {orbbec_share}/launch')
    camera = TimerAction(period=6.0, actions=[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(cam_launch))])

    # Plain python process, not a package node — the bridge lives beside this file.
    bridge = TimerAction(period=9.0, actions=[ExecuteProcess(
        cmd=['python3', os.path.join(os.path.dirname(__file__), 'arm_bridge.py')],
        output='screen')])

    return LaunchDescription([base, lidar, camera, bridge])
