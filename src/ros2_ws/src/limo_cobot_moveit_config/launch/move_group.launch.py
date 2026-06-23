"""move_group, launched explicitly so we can FORCE use_sim_time:=true.

The moveit_configs_utils helper (generate_move_group_launch) did not propagate
use_sim_time on this system, so move_group ran on wall-clock time while Gazebo
publishes sim time — it then "Failed to fetch current robot state" (timestamp
mismatch) and could not plan. Building the node ourselves guarantees sim time.
Also restricted to OMPL (no CHOMP default).
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("limo_cobot", package_name="limo_cobot_moveit_config")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,                       # <-- the fix
                "publish_robot_description_semantic": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "monitor_dynamics": False,
            },
        ],
    )
    return LaunchDescription([move_group_node])
