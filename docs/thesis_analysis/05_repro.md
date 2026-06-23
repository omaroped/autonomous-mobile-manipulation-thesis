# Pass 5 — Reproducibility Audit

## Missing Pieces & Gap Analysis
| Issue | Severity | Description | Fix Required |
| :--- | :--- | :--- | :--- |
| **Undocumented Deps** | High | `limo_car/package.xml` is missing `moveit_msgs`, `cv_bridge`, `sensor_msgs`, and `geometry_msgs`. | Update `package.xml` with `<depend>` tags. |
| **Hardcoded Paths** | Low | `reach_sweep.py` points to `/home/omar/Desktop/Thesisorg/docs/references/data/reach_map.csv`. | Use `ament_index_python` to find workspace root. |
| **Manual Patch** | High | The `gazebo_ros2_control` patch is uncommitted and required for arm control. | Create a `.patch` file or commit to a local branch. |
| **Race Conditions** | Medium | `TimerAction` values (8s, 10s, 12s) in `arm_grasp_test.launch.py` may fail on slower CPUs. | Use "spawn_check" nodes or tighter Gazebo/ROS sync. |

## Minimum README for Fresh Install
To reproduce this workspace on a fresh Ubuntu 22.04 + ROS 2 Humble install:

1.  **Clone the workspace:**
    ```bash
    mkdir -p ~/thesis_ws/src && cd ~/thesis_ws/src
    # [Clone limo_ros2, mycobot_ros2, gazebo_ros2_control, limo_cobot_moveit_config]
    ```
2.  **Install Dependencies (Manual Additions):**
    ```bash
    sudo apt install ros-humble-moveit-msgs ros-humble-cv-bridge ros-humble-joint-trajectory-controller ros-humble-joint-state-broadcaster
    ```
3.  **Apply the Critical Middleware Patch:**
    *   Navigate to `src/gazebo_ros2_control/gazebo_ros2_control/src/gazebo_ros2_control_plugin.cpp`.
    *   Modify the robot description loading logic to read from the `/robot_description` topic instead of the deprecated parameter path (refer to `03_contribution.md` for details).
4.  **Build:**
    ```bash
    colcon build --symlink-install
    source install/setup.bash
    ```
5.  **Run the Integration Test:**
    ```bash
    ros2 launch limo_car arm_grasp_test.launch.py
    # In new terminals:
    ros2 launch limo_cobot_moveit_config moveit.launch.py
    ros2 launch limo_car arm_grasp_run.launch.py
    ```

## Assumptions & Env Vars
- **Assumption:** `use_sim_time:=True` is critical across all terminals.
- **Assumption:** Gazebo meshes must be in the `GAZEBO_MODEL_PATH`. The `limo_description` and `mycobot_description` packages handle this via `<export>` but require a clean build.
