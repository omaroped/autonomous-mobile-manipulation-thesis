# Pass 10 — The Exclusive Technical DNA Master Index

This is the definitive, 100% comprehensive inventory of the project's technical artifacts. Every file in the contribution packages has been audited for its role in the thesis.

## 1. Middleware: The C++ Source Patch
**File:** `src/gazebo_ros2_control/gazebo_ros2_control/src/gazebo_ros2_control_plugin.cpp`
*   **Role:** The project's most critical "War Story." 
*   **The Fix:**
    ```cpp
    // Removed injection of --param robot_description:=<urdf>
    // arguments.push_back(RCL_PARAM_FLAG);
    // arguments.push_back(rb_arg);
    ```
*   **The Logic:** Newer ROS 2 Humble `controller_manager` versions reject URDF strings passed as CLI parameter overrides. By removing this, the plugin is forced to wait for the `/robot_description` topic. This one change is the single reason the arm can be controlled in simulation.

## 2. Package: `limo_car` (The Integrated Hub)
This is the primary container for the thesis's simulation logic.

| Directory | File | Exclusive Role / Insight |
| :--- | :--- | :--- |
| **scripts/** | `limo_pick_place.py` | Integrated nav+pick state machine with J1/J5 kinematic bias. |
| | `pick_orchestrator.py` | Stage 1 pick using MoveIt Action interfaces (No `moveit_py`). |
| | `box_pose_estimator.py` | HSV+Depth deprojection with median-window filtering. |
| | `box_follower.py` | P-controller visual servoing for Ackermann steering. |
| | `arm_grasp_test.py` | Isolated grasp test implementing "Probing IK" (Frontal vs. Top-Down). |
| | `base_pin.py` | Physics stability helper: 50 Hz world-pose locking. |
| | `grasp_attacher.py` | Coordinate-follower "Pseudo-Weld" at 25 Hz. |
| | `reach_sweep.py` | Grid search script ($n=405$) that generated the thesis evaluation data. |
| | `base_joint_state_pub.py` | Bridging the LIMO base joints into the MoveIt robot state. |
| | `box_detector.py` | Isolated testing script for HSV mask calibration. |
| | `limo_mycobot_teleop.py` | Manual override controller for system stress-testing. |
| **gazebo/** | `ackermann_with_sensor.xacro` | The "Master Model": Merges LIMO, myCobot, and Gripper URDF trees. |
| | `mycobot_ros2_control.xacro` | Hardware abstraction layer defining the 6-joint "Mirror Gripper". |
| | `steering_dummy.xacro` | Simulation-only kinematic constraints for Ackermann joints. |
| **worlds/** | `final_map.world` | Standardized evaluation environment with table and target. |
| | `test_box.sdf` / `test_table.sdf` | Non-static Gazebo models for the "Pseudo-Weld" grasp. |
| **maps/** | `thesis_map.pgm` / `.yaml` | 2D occupancy grid for abandoned (but documented) SLAM phase. |

## 3. Package: `limo_cobot_moveit_config` (The Planning Brain)
*   **`config/limo_cobot.srdf`**: Contains the **Manual Collision Matrix** (Pass 8) which unlocked the arm's workspace.
*   **`config/ompl_planning.yaml`**: Configured for high-resolution (`0.005`) collision checking.
*   **`config/joint_limits.yaml`**: Fine-tuned velocity/acceleration caps to prevent J5/J6 torque spikes.

## 4. Package: `limo_ros2` (The Base Foundation)
*   **`urdf/limo_ackerman_base.xacro`**: Contains the **Wheel Dynamics** constants (`damping="0.1"`) that fixed the Ackermann drift bug.

## 5. Artifact Summary
*   **Total Scripted Logic:** 11 Python nodes covering perception, control, and orchestration.
*   **Total Model Complexity:** 3 Integrated Xacro trees with custom sensor positioning.
*   **Total Physics Patches:** 2 Simulation stability helpers + 1 Middleware C++ patch.

**Audit Status:** 100% Comprehensive. All artifacts have been traced from raw source to thesis contribution.
