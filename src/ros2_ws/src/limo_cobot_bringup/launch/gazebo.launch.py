"""
Gazebo simulation launch file for the LIMO Cobot mobile manipulator.

Launches:
  1. Gazebo with an optional world file
  2. robot_state_publisher (processes xacro → URDF)
  3. Spawns the robot entity in Gazebo
  4. Loads and activates ros2_control arm controllers

Usage:
  ros2 launch limo_cobot_bringup gazebo.launch.py
  ros2 launch limo_cobot_bringup gazebo.launch.py use_rviz:=true
  ros2 launch limo_cobot_bringup gazebo.launch.py world:=path/to/world.world

Author: Omar (Thesis - Mobile Manipulation for On-Demand Manufacturing)
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Set GAZEBO_MODEL_PATH so Gazebo knows where to find the package:// meshes
    mycobot_description_share = get_package_share_directory('mycobot_description')
    limo_car_share = get_package_share_directory('limo_car')
    gazebo_model_path = os.path.dirname(mycobot_description_share) + ':' + os.path.dirname(limo_car_share)
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ':' + gazebo_model_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path

    # --- Package paths ---
    pkg_share = FindPackageShare('limo_cobot_bringup')
    gazebo_ros_share = FindPackageShare('gazebo_ros')

    # --- Launch arguments ---
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Launch RViz alongside Gazebo'
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(get_package_share_directory('limo_cobot_bringup'), 'worlds', 'simple_warehouse.world'),
        description='Path to a Gazebo world file'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    use_rviz = LaunchConfiguration('use_rviz')
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')

    import subprocess

    # --- Process xacro to URDF ---
    pkg_share_dir = get_package_share_directory('limo_cobot_bringup')
    xacro_file_path = os.path.join(pkg_share_dir, 'urdf', 'limo_cobot.xacro')

    command_str = f"xacro {xacro_file_path} | python3 -c \"import sys, re; print(re.sub(r'<!--.*?-->', '', sys.stdin.read(), flags=re.DOTALL))\""
    result = subprocess.run(command_str, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"Failed to process xacro: {result.stderr}")
    robot_description_content = result.stdout

    # --- Robot State Publisher ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([gazebo_ros_share, 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={
            'world': world,
            'extra_gazebo_args': '--verbose'
        }.items(),
    )

    # --- Spawn robot in Gazebo ---
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'limo_cobot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2',
        ],
        output='screen',
    )

    # --- Load controllers (after spawn) ---
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen',
    )

    load_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mycobot_gripper_controller'],
        output='screen',
    )

    # Chain: spawn → load joint_state_broadcaster → load controllers
    delayed_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_broadcaster],
        )
    )

    delayed_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_arm_controller, load_gripper_controller],
        )
    )

    # --- Optional RViz ---
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'default.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        use_rviz_arg,
        world_arg,
        use_sim_time_arg,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        delayed_joint_state_broadcaster,
        delayed_arm_controller,
        rviz_node,
    ])
