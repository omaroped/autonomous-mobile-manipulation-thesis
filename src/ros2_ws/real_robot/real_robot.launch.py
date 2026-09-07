"""real_robot.launch.py — bring up the ENTIRE physical LIMO in one command.

Runs on the Jetson (~/limo_ros2_ws/real_robot/). Starts, in order:

  1. limo_base        — chassis driver (odom, /cmd_vel, /limo_status)
  2. ydlidar          — T-mini Pro with limo_tminipro.yaml (±100°, matches sim)
  3. orbbec camera    — DaBai DC1 colour + depth
  4. camera_tf        — static base_link -> camera transform (see WARNING below)
  5. arm_bridge       — /joint_states + gripper commands → pymycobot serial

Everything was verified individually on 2026-08-03; this only composes them.

*** CAMERA TF: TRANSLATION MEASURED 2026-08-26. ROTATION STILL ASSUMED. ***
Why this exists: until 2026-08-25 nothing in this launch published a transform linking
the camera to the rest of the robot, so box_pose_estimator's 3D positions were
computed against whatever TF2 defaulted to -- a box ~0.5 m away came out at x = 2.64 m.

The x/y/z defaults below are MEASURED on the physical robot with a tape measure:

    x  0.189 (base_link to bumper face) - 0.070 (bumper face back to lens) = 0.119 m
    y  camera sits on the chassis centre line                              = 0.000 m
    z  0.185 (floor to lens) - 0.145 (floor to base_link)                  = 0.040 m

Rotation is assumed level and forward-facing (0, 0, 0).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # MEASURED on the physical robot 2026-08-26 with tape measure.
    camera_offset_x_arg = DeclareLaunchArgument(
        'camera_offset_x', default_value='0.119',
        description='MEASURED. base_link -> camera lens, metres forward '
                    '(0.189 bumper - 0.070 measured back from the bumper face).')
    camera_offset_y_arg = DeclareLaunchArgument(
        'camera_offset_y', default_value='0.0',
        description='MEASURED. base_link -> camera, metres left.')
    camera_offset_z_arg = DeclareLaunchArgument(
        'camera_offset_z', default_value='0.040',
        description='MEASURED. base_link -> camera, metres up '
                    '(0.185 floor-to-lens - 0.145 floor-to-base_link).')
    camera_optical_frame_arg = DeclareLaunchArgument(
        'camera_optical_frame', default_value='camera_depth_optical_frame',
        description='Must equal the frame_id actually published in the depth '
                    'image header -- box_pose_estimator reads that header directly '
                    'and does not know this launch argument exists. Verify with: '
                    'ros2 topic echo /camera/depth/camera_info --field header.frame_id')

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

    # base_link -> camera, split into TWO transforms as the sim does (sensor.xacro),
    # not one, because they come from different kinds of knowledge:
    #
    #   1. camera_link: the physical MOUNT -- where the camera body sits relative to
    #      base_link, in ordinary robot axes (X forward, Y left, Z up). Translation
    #      measured 2026-08-26; rotation still assumed identity (level, forward
    #      facing) and unverified -- but kept independent of the second transform
    #      below, so measuring the tilt later touches only this one.
    #
    #   2. REP-103 optical rotation: X-right/Y-down/Z-forward is the standing
    #      convention every camera driver's *_optical_frame publishes images in
    #      (depth_camera_estimator's deprojection assumes it too). Zero translation,
    #      fixed rotation -- this is NOT something to measure, it is the same
    #      constant the sim uses (sensor.xacro depth_link joint,
    #      rpy="-1.570796 0 -1.570796"). Keeping it separate from step 1 means a
    #      future mount-offset correction only ever touches the transform that is
    #      actually in question.
    #
    # Started alongside the camera (not before it): box_pose_estimator's TF lookups
    # will simply retry/warn until these exist, which is exactly what was happening
    # before this fix -- with nothing linking the camera into the robot's TF tree,
    # its deprojection had nothing correct to compose against.
    camera_mount_tf = TimerAction(period=6.0, actions=[Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='camera_mount_tf', output='screen',
        arguments=[
            LaunchConfiguration('camera_offset_x'),
            LaunchConfiguration('camera_offset_y'),
            LaunchConfiguration('camera_offset_z'),
            # Rotation assumed identity: camera body level and forward-facing.
            # NOT measured -- a tilted mount would bias every depth reading. See the
            # module docstring.
            '0', '0', '0',
            'base_link', 'camera_link',
        ])])
    camera_optical_tf = TimerAction(period=6.0, actions=[Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='camera_optical_tf', output='screen',
        arguments=[
            '0', '0', '0', '-1.570796', '0', '-1.570796',
            'camera_link', LaunchConfiguration('camera_optical_frame'),
        ])])

    # Plain python process, not a package node — the bridge lives beside this file.
    bridge = TimerAction(period=9.0, actions=[ExecuteProcess(
        cmd=['python3', os.path.join(os.path.dirname(__file__), 'arm_bridge.py')],
        output='screen')])

    return LaunchDescription([
        camera_offset_x_arg, camera_offset_y_arg, camera_offset_z_arg,
        camera_optical_frame_arg,
        base, lidar, camera, camera_mount_tf, camera_optical_tf, bridge,
    ])
