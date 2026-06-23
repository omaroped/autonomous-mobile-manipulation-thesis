from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    sim_time = {'use_sim_time': True}
    
    attacher = Node(package='limo_car', executable='grasp_attacher',
                    name='grasp_attacher', output='screen', parameters=[sim_time])
    
    # Pins the base in place wherever it parked (no overrides) so the arm doesn't push the chassis back.
    base_pin = Node(package='limo_car', executable='base_pin', name='base_pin',
                    output='screen', parameters=[sim_time, {'override_x': float('nan'), 'override_y': float('nan')}])
    
    grasp = Node(package='limo_car', executable='arm_grasp_test',
                 name='arm_grasp_test', output='screen', parameters=[sim_time])

    return LaunchDescription([
        attacher,
        base_pin,
        grasp,
    ])
