import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('limo_car')
    sim_time = {'use_sim_time': True}

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
    
    follower = Node(package='limo_car', executable='box_follower',
                    name='box_follower', output='screen', parameters=[sim_time])

    return LaunchDescription([
        spawn_table,
        spawn_box,
        perception,
        follower,
    ])
