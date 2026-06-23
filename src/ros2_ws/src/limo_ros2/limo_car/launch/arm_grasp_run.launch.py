"""Run the ISOLATED grasp: perception + the arm_grasp_test node.

box_pose_estimator locates the parked test box (rock-solid since the base does
not move); arm_grasp_test reads /box_pose and runs the full top-down grasp.

Start AFTER:
    ros2 launch limo_car arm_grasp_test.launch.py            # sim + robot + test box
    ros2 launch limo_cobot_moveit_config moveit.launch.py    # move_group
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sim_time = {"use_sim_time": True}
    return LaunchDescription([
        Node(package="limo_car", executable="box_pose_estimator",
             name="box_pose_estimator", output="screen", parameters=[sim_time]),
        # the 'weld' helper — welds the box to the gripper on /grasp_attach
        Node(package="limo_car", executable="grasp_attacher",
             name="grasp_attacher", output="screen", parameters=[sim_time]),
        Node(package="limo_car", executable="arm_grasp_test",
             name="arm_grasp_test", output="screen", parameters=[sim_time]),
    ])
