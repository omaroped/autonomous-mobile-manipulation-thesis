<<<<<<< HEAD
# Author: Zhui Li (Updated by Antigravity)
# Description: Launch file for the LIMO simulation in Gazebo using Ackermann-style steering.

import os
from os import environ, pathsep

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


from launch.conditions import IfCondition

def generate_launch_description():

    package_name = 'limo_car'

    world_config = LaunchConfiguration('world')
    rviz_config  = LaunchConfiguration('rvizconfig')

    pkg_path = get_package_share_directory(package_name)

    default_world_path      = os.path.join(pkg_path, 'worlds', 'final_map.world')
    default_rviz_config_path = os.path.join(pkg_path, 'rviz',   'gazebo.rviz')

    world_arg = DeclareLaunchArgument(
        'world', default_value=default_world_path,
        description='Absolute path to Gazebo world file')

    rviz_arg = DeclareLaunchArgument(
        'rvizconfig', default_value=default_rviz_config_path,
        description='Absolute path to rviz config file')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Whether to launch RViz')

    use_gzclient_arg = DeclareLaunchArgument(
        'use_gzclient', default_value='true',
        description='Whether to launch Gazebo client (GUI)')

    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y', default_value='7.0',
        description='Robot spawn Y (m). 7.0 = at the table (isolated grasp test); '
                    'larger (e.g. 7.6) = farther back so the robot drives up for an approach.')

    use_camera_arg = DeclareLaunchArgument(
        'use_camera', default_value='true',
        description='Spawn the depth camera. Set false for GUI driving/nav demos to avoid '
                    'the intermittent Gazebo GPU depth-camera segfault.')

    drive_mode_arg = DeclareLaunchArgument(
        'drive_mode', default_value='diff',
        description='Drive mode (ackermann | diff)')


    # ── Gazebo environment paths ──────────────────────────────────────────────
    # Plugins live in /opt/ros/humble/lib; models in gazebo_ros share dir.
    gazebo_ros_share  = get_package_share_directory('gazebo_ros')
    # mycobot and limo meshes use package:// (which Gazebo maps to model://) — Gazebo needs the
    # parent of their package share directories in GAZEBO_MODEL_PATH.
    mycobot_models = os.path.dirname(get_package_share_directory('mycobot_description'))
    limo_models = os.path.dirname(get_package_share_directory('limo_car'))
    model_path  = os.path.join(gazebo_ros_share, 'models') + pathsep + mycobot_models + pathsep + limo_models
    # Load our PATCHED gazebo_ros2_control plugin (workspace overlay) BEFORE the
    # system one, so Gazebo uses the build that no longer crashes passing
    # robot_description to controller_manager >= 2.x (see memory
    # "gazebo-ros2-control-eol-incompatibility").
    from ament_index_python.packages import get_package_prefix
    patched_gzros2_lib = os.path.join(get_package_prefix('gazebo_ros2_control'), 'lib')
    plugin_path = patched_gzros2_lib + pathsep + '/opt/ros/humble/lib'
    if 'GAZEBO_MODEL_PATH' in environ:
        model_path  += pathsep + environ['GAZEBO_MODEL_PATH']
    if 'GAZEBO_PLUGIN_PATH' in environ:
        plugin_path += pathsep + environ['GAZEBO_PLUGIN_PATH']

    gazebo_env = {
        'GAZEBO_MODEL_PATH':  model_path,
        'GAZEBO_PLUGIN_PATH': plugin_path,
        # ── Render Gazebo on the NVIDIA dGPU (NVIDIA Optimus / PRIME on-demand) ──
        # This is a dual-GPU laptop (RTX 3060 + AMD integrated). In on-demand mode
        # apps default to the AMD/Mesa GPU, where Gazebo's GPU depth-camera sensor
        # SEGFAULTS gzserver on spawn. Offloading rendering to the NVIDIA dGPU is
        # fast AND stable — no need for LIBGL_ALWAYS_SOFTWARE=1 (which ran but was
        # painfully slow). Affects both gzserver (sensor rendering) and gzclient (GUI).
        '__NV_PRIME_RENDER_OFFLOAD': '1',
        '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
        '__VK_LAYER_NV_optimus': 'NVIDIA_only',
    }

    # Launch gzserver directly so -s flags are separate arguments (not a
    # combined string), which is required for Gazebo to parse them correctly.
    gzserver_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_config,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
             # NOTE: libgazebo_ros_state.so is a WORLD plugin, loaded inside
             # final_map.world (NOT here with -s, which left its services dead).
        additional_env=gazebo_env,
        output='screen'
    )

    gzclient_cmd = ExecuteProcess(
        cmd=['gzclient'],
        additional_env=gazebo_env,
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_gzclient'))
    )


    mbot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_path, 'launch', 'ackermann.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'world': world_config,
            'drive_mode': LaunchConfiguration('drive_mode'),
            'use_camera': LaunchConfiguration('use_camera')
        }.items()
    )

    # ── Spawn robot in Gazebo ─────────────────────────────────────────────────
    # Use /usr/bin/python3 explicitly to bypass pyenv interception, which
    # would use a Python without rclpy and cause spawn_entity to hang silently.
    spawn_entity = ExecuteProcess(
        cmd=['/usr/bin/python3',
             '/opt/ros/humble/lib/gazebo_ros/spawn_entity.py',
             '-topic', 'robot_description',
             '-entity', 'mbot',
             # z≈0 so the wheels start essentially on the ground; with real diff-drive
             # physics the base settles onto its wheels under gravity (small, clean drop).
             '-x', '-2.0', '-y', LaunchConfiguration('spawn_y'), '-z', '0.0', '-Y', '-1.5708'],
        output='screen'
    )

    # ── RViz ─────────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        output='screen', arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')))

    # ── Controller spawners ───────────────────────────────────────────────────
    joint_state_broadcaster_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'])

    arm_controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['mycobot_arm_controller'])

    gripper_controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['mycobot_gripper_controller'])

    # Base drive: the gazebo_ros_diff_drive plugin (in ackermann.xacro) subscribes /cmd_vel and
    # publishes /odom + the odom->base_footprint TF directly — no separate controller/bridge.

    return LaunchDescription([
        world_arg,
        rviz_arg,
        spawn_y_arg,
        use_rviz_arg,
        use_gzclient_arg,
        drive_mode_arg,
        use_camera_arg,
        mbot,
        gzserver_cmd,
        gzclient_cmd,
        spawn_entity,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        rviz_node,
    ])
=======
# Autor: Zhui Li
# E-Mail: lz554113510@gmail.com
# Company: Institut für Intermodale Transport- und Logistiksysteme in Technische Universität Braunschweig
# Description: Diese py-Datei basiert auf Regeln und definiert Funktionen durch python,
# um den Launch der Simulation in Gazebo zu ermöglichen. Danach mit der Simulation in Gazebo kann man die weitere
# Forschung arbeiten. Diese Launch-Datei dient zu Ackermann-Type.

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    # definiert Path für Modell
    package_name = 'limo_car'
    world_file_path = 'worlds/empty_world.model'
    rviz_path = 'rviz/gazebo.rviz'

    pkg_path = os.path.join(get_package_share_directory(package_name))
    world_path = os.path.join(pkg_path, world_file_path)
    default_rviz_config_path = os.path.join(pkg_path, rviz_path)

    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # Position dafür, wo die Modelle herstellt werden
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.0'
    spawn_yaw_val = '0.0'

    mbot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch', 'ackermann.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'world': world_path}.items()
    )

    # Einbindung der Gazebo-Startdatei, die im gazebo_ros-Paket enthalten ist
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # laufen ein leere node aus den gazebo_ros package
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'mbot',
                                   '-x', spawn_x_val,
                                   '-y', spawn_y_val,
                                   '-z', spawn_z_val,
                                   '-Y', spawn_yaw_val],
                        output='screen')


    return LaunchDescription([
        mbot,
        gazebo,
        spawn_entity,
        rviz_arg,
        rviz_node
    ])
>>>>>>> origin/master
