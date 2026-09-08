"""real_arm_moveit.launch.py — Phase 1: MoveIt against MOCK hardware, no Gazebo,
no real arm involvement at all (arm_bridge.py is not started here — see Phase 2).

Starts, in order:
  1. robot_state_publisher   — fed real_arm.xacro (arm + gripper, mock ros2_control)
  2. controller_manager      — standalone ros2_control_node (no Gazebo to auto-start
                                one), same mycobot_controllers.yaml as sim, unchanged
  3. controller spawners     — joint_state_broadcaster, mycobot_arm_controller,
                                mycobot_gripper_controller — identical names/pattern
                                to ackermann_gazebo.launch.py
  4. MoveIt (moveit_real.launch.py) — move_group + optional RViz

Verification for this phase: RViz shows the arm, `ros2 control list_controllers`
shows mycobot_arm_controller active, a MoveIt-planned motion executes and
/joint_states updates — all with no physical arm connected, since the hardware
plugin is mock_components/GenericSystem, not the real one.

Usage:
    ros2 launch limo_car real_arm_moveit.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')

    limo_car_pkg = get_package_share_directory('limo_car')
    xacro_file = os.path.join(limo_car_pkg, 'gazebo', 'real_arm.xacro')
    controllers_yaml = os.path.join(limo_car_pkg, 'config', 'mycobot_controllers.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
    )

    # No Gazebo means no libgazebo_ros2_control.so to spawn this for us — start the
    # standalone ros2_control_node directly, fed the same robot_description and the
    # same controllers.yaml sim already uses.
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'robot_description': robot_description}, controllers_yaml],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30'])

    arm_controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['mycobot_arm_controller', '--controller-manager-timeout', '30'])

    gripper_controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['mycobot_gripper_controller', '--controller-manager-timeout', '30'])

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('limo_cobot_moveit_config'),
                         'launch', 'moveit_real.launch.py')),
        launch_arguments={'use_rviz': use_rviz}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        moveit,
    ])
