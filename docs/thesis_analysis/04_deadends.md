# Pass 4 — Dead-ends and Pivot Log

This log documents abandoned paths, unreachable code, and strategic scope reductions. These are framed as engineering decisions based on toolchain limitations or complexity-benefit analysis.

## 1. The "Heavyweight" Pipeline (MTC + PCL)
*   **Attempted:** Integration of MoveIt Task Constructor (MTC) for multi-stage planning and Point Cloud Library (PCL) for RANSAC-based object segmentation.
*   **Evidence:** Comments in `box_pose_estimator.py` ("no PCL / RANSAC / MTC") and `pick_orchestrator.py`.
*   **Reason for Pivot:** Toolchain incompatibility in ROS 2 Humble (MTC was not fully packaged) and the "brittleness" of PCL segmentation in a dynamic simulation.
*   **Engineering Decision:** Replaced with a "lightweight" HSV-depth deprojection pipeline and direct `MoveGroup` action calls.

## 2. Global Navigation (SLAM + Nav2)
*   **Attempted:** Full warehouse navigation using `slam_toolbox` and `nav2_stack`.
*   **Evidence:** `slam_mapping.launch.py` and `slam_toolbox.yaml` in `limo_car/launch/config`.
*   **Reason for Pivot:** The AgileX LIMO's Ackermann steering introduced significant odometry drift in Gazebo, making global map stability difficult.
*   **Engineering Decision:** Pivoted to **Reactive Visual Servoing**. Since the target box is locally visible from the start/intermediate positions, a global map was deemed unnecessary. This prioritizes "hand-off" precision over global path planning.

## 3. Physical Hardware Validation
*   **Attempted:** Deployment of the pipeline to the physical AgileX LIMO and myCobot 280.
*   **Evidence:** Hardware requirements and "Meeting and experiment templates" in the Mar 2 kickoff.
*   **Reason for Pivot:** Scope management. The "Simulation-based integration study" was formalized as the primary contribution to ensure a high-quality, verified pipeline within the 14-week window.
*   **Engineering Decision:** Deferred to "Future Work".

## 4. Multi-Orientation Grasping
*   **Attempted:** Grasping from the front, side, or at various tilts.
*   **Evidence:** `reach_sweep.py` grid search script.
*   **Findings:** The script revealed that "side-grasp" and "frontal" orientations had significantly lower reachability counts (likely due to self-collision with the base or kinematic limits).
*   **Engineering Decision:** Standardized on **Top-Down Grasping** to maximize the feasible workspace and reduce the degrees of freedom required for perception calibration.

## 5. Educational Course Website
*   **Attempted:** A React/Vite-based platform for teaching ROS 2/VLM concepts.
*   **Evidence:** `CHANGELOG.md` (Mar 15) and source files in `src/website` (if they exist).
*   **Reason for Pivot:** High effort/low reward for a technical engineering thesis.
*   **Engineering Decision:** Orphaned the website to focus exclusively on the `ros2_control` and MoveIt integration.
