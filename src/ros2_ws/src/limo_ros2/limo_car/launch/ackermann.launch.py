# Autor: Zhui Li
# E-Mail: lz554113510@gmail.com
# Company: Institut für Intermodale Transport- und Logistiksysteme in Technische Universität Braunschweig
# Description: Diese py-Datei basiert auf Regeln und definiert Funktionen durch python,
# um den Launch der Simulation in Gazebo zu ermöglichen. Danach mit der Simulation in Gazebo kann man die weitere
# Forschung arbeiten. Diese Launch-Datei dient zu Ackermann-Type.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
<<<<<<< HEAD
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
=======
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro
>>>>>>> origin/master


def generate_launch_description():

    # um die Zeit zu simulieren
    use_sim_time = LaunchConfiguration('use_sim_time')
<<<<<<< HEAD
    drive_mode = LaunchConfiguration('drive_mode')
    use_camera = LaunchConfiguration('use_camera')
=======
>>>>>>> origin/master

    # finden path für Modell
    pkg_path = os.path.join(get_package_share_directory('limo_car'))
    xacro_file = os.path.join(pkg_path, 'gazebo', 'ackermann_with_sensor.xacro')
<<<<<<< HEAD

    # Process dynamically at runtime
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' drive_mode:=', drive_mode,
                 ' use_camera:=', use_camera]),
        value_type=str
    )

    # Erstellt ein robot_state_publisher Node
    params = {'robot_description': robot_description,
=======
    robot_description_config = xacro.process_file(xacro_file)

    # Erstellt ein robot_state_publisher Node
    params = {'robot_description': robot_description_config.toxml(),
>>>>>>> origin/master
            'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
<<<<<<< HEAD
        DeclareLaunchArgument(
            'drive_mode',
            default_value='diff',
            description='Drive mode (ackermann | diff)'),
        DeclareLaunchArgument(
            'use_camera',
            default_value='true',
            description='Spawn the depth camera. Set false to avoid the Gazebo GPU '
                        'depth-camera segfault when running with the GUI (driving/nav demos).'),
=======
>>>>>>> origin/master
        node_robot_state_publisher
    ])
