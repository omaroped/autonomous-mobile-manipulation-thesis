# Pass 2 — System Architecture Extraction

## Runtime Node Graph (Simplified)
```mermaid
graph TD
    subgraph Simulation
        Gazebo[Gazebo Classic]
        gz_ros_ctl[gazebo_ros2_control_plugin]
    end

    subgraph Perception
        cam[Depth Camera /image_raw] --> bpe[box_pose_estimator]
        bpe --> box_pose[/box_pose]
    end

    subgraph Navigation
        box_pose --> bf[box_follower]
        bf --> cmd_vel[/cmd_vel]
        cmd_vel --> Gazebo
    end

    subgraph Manipulation
        box_pose --> po[pick_orchestrator]
        po --> mg[move_group / MoveIt 2]
        mg --> jtc_action[/joint_trajectory_controller/follow_joint_trajectory]
        jtc_action --> gz_ros_ctl
        po --> grip_cmd[/gripper_controller/command]
        grip_cmd --> gz_ros_ctl
    end

    subgraph Utilities
        bp[base_pin] -.-> Gazebo
        ga[grasp_attacher] --> Gazebo
    end
```

## Package Responsibility Table
| Package | Primary Responsibility | Classification | Originality Score |
| :--- | :--- | :--- | :--- |
| `limo_car` | **Algorithmic Core:** Navigation (visual servoing), Perception (HSV/depth), and Orchestration (pick state machine). | Original | High |
| `limo_cobot_moveit_config` | **Manipulation Logic:** MoveIt 2 semantic model, collision scenes, and planning pipelines. | Original | High |
| `gazebo_ros2_control` | **Middleware Patch:** Modified `gazebo_ros2_control_plugin.cpp` to fix Humble controller-manager incompatibility. | Patched | Medium |
| `limo_ros2` | **Base Integration:** Host for URDF/Xacro models, launch files, and base-link kinematics. | Modified Vendor | Medium |
| `mycobot_ros2` | **Arm Integration:** Description, meshes, and joint-control interfaces for the myCobot 280. | Modified Vendor | Medium |

## Key Lifecycle Flow
1. **Navigate**: `box_follower` performs proportional control on visual error from the camera until the target is within ~0.22 m.
2. **Perceive**: `box_pose_estimator` segments the target in the depth image, deprojects to 3-D, and broadcasts a TF frame.
3. **Plan**: `pick_orchestrator` invokes `move_group` to plan a collision-aware trajectory to a pre-grasp pose above the box.
4. **Actuate**: `move_group` executes via `ros2_control`; `pick_orchestrator` triggers the gripper via topic-direct command.
5. **Finalize**: `grasp_attacher` ensures the physical grasp is "locked" in Gazebo for transport.
