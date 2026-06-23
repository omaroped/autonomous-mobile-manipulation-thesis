# Pass 3 — Contribution Isolation

## Codebase Classification
| Category | Definition | Subsystems | Approx. Volume |
| :--- | :--- | :--- | :--- |
| **(a) Third-party** | Unmodified vendor code. | `mycobot_ros2` (base), `limo_ros2` (base), `gazebo_ros2_control` (base). | ~20,000+ LOC |
| **(b) Integration** | Launch files, URDF tweaks, patches. | `gazebo_ros2_control_plugin.cpp` patch, Xacro dynamics, controller YAMLs. | ~500 LOC |
| **(c) Original** | Substantive algorithmic code. | `limo_car` scripts, MoveIt SRDF/Config. | **~2,550 LOC** |

## Original Substantive Core (Category C)
The "Technical Core" of the thesis consists of ~2,550 lines of Python and YAML, categorized as follows:

### 1. Collision-Aware Orchestration (~1000 LOC)
*   **Files:** `pick_orchestrator.py`, `arm_grasp_test.py`, `limo_pick_place.py`
*   **Contribution:** An asynchronous state machine using the MoveIt 2 `MoveGroup` Action Interface. It handles the "Navigate → Perceive → Planning Scene Update → Plan → Grasp → Place" sequence. Crucially, it manages the **planning scene** to prevent the arm from striking the table or the robot base.

### 2. Reactive Navigation (~170 LOC)
*   **Files:** `box_follower.py`
*   **Contribution:** A proportional control visual servoing node specifically tuned for the **Ackermann steering** geometry of the LIMO. It maps visual error (pixel offset) to steering angle and velocity, implementing a smooth deceleration curve for precision alignment at the 0.22 m grasp distance.

### 3. Perception & Depth Mapping (~317 LOC)
*   **Files:** `box_pose_estimator.py`, `box_detector.py`
*   **Contribution:** A classical perception pipeline that segments known objects via HSV colour-space filtering and deprojects 2-D centroids to 3-D world coordinates using camera intrinsics. It broadcasts the target as a TF frame for the manipulator.

### 4. MoveIt 2 Configuration (~450 LOC)
*   **Files:** `limo_cobot.srdf`, `ompl_planning.yaml`, `moveit_controllers.yaml`
*   **Contribution:** The semantic definition of the mobile manipulator, including self-collision matrices (critical for the arm-base interface), joint limits, and planning group definitions for the myCobot 280.

### 5. Middleware Fixes (~323 LOC)
*   **Files:** `gazebo_ros2_control_plugin.cpp` (patch), `grasp_attacher.py`, `base_pin.py`
*   **Contribution:** Critical "War Story" artifacts that make the simulation viable. The `gazebo_ros2_control` patch (23 lines) enables modern controller-manager compatibility, while `base_pin` and `grasp_attacher` compensate for Gazebo physics instabilities.

## Algorithmic Summary
The project does not invent a new planner; instead, it **engineers a robust integration** of Nav-Perception-MoveIt. The primary contribution is the **calibration and orchestration** of these disparate systems to solve the "reachability vs. base-positioning" constraint identified as the project's binding challenge.
