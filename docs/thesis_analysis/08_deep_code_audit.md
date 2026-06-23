# Pass 8 — Deep Code-Level Audit

This report details the algorithmic and kinematic logic extracted directly from the project's source code, moving beyond high-level summaries.

## 1. Algorithmic Logic: Orchestration & Control

### A. The "Settling Logic" (State Machine Robustness)
In `pick_orchestrator.py`, the system addresses the "moving target" problem caused by the AgileX LIMO's Ackermann steering drift and Gazebo suspension rocking.
*   **Algorithm:** Variance-based stability check.
*   **Logic:** The node monitors the perceived box pose in the `base_link` frame. It only "locks" the target pose when the X and Y coordinates change by less than **0.015 m** over a **1.5-second** window.
*   **Contribution:** Prevents MoveIt from planning to a "stale" or "jittery" pose captured while the base was still decelerating.

### B. Smooth Gripper Ramping
To prevent "Gazebo explosions" (high-impulse contacts launching the robot), `pick_orchestrator.py` implements manual joint ramping.
*   **Code Snippet:**
    ```python
    for i in range(1, steps + 1):
        a = i / steps
        interp = [s + a * (v - s) for s, v in zip(start, values)]
        msg.data = [float(x) for x in interp]
        self._gripper.publish(msg)
    ```
*   **Logic:** Linearly interpolates between the current and target joint positions over **0.8 seconds** (20 steps).
*   **Contribution:** Replaces the standard "step" command with a rate-limited trajectory, ensuring physics stability during grasp.

### C. Ackermann-Aware Visual Servoing
`box_follower.py` implements a Proportional (P) controller tuned for the LIMO's steering geometry.
*   **Control Law:** 
    *   $v_x = \text{clamp}(\text{error}_{dist} \cdot 0.6, 0.06, 0.25)$
    *   $\omega_z = \text{error}_{heading} \cdot 1.0$
*   **The "Stiction" Constant:** A minimum velocity of **0.06 m/s** is enforced to ensure the robot doesn't get stuck due to simulation friction (stiction) before reaching the target.
*   **Target Distance:** Tuned to **0.095 m** (camera-to-box), which places the box at the optimal reachable distance for the myCobot 280.

## 2. Perception: Depth Deprojection & Latching

### A. Median Window Filtering
In `box_pose_estimator.py`, the depth extraction isn't a single pixel read (which is noisy).
*   **Logic:** Extracts a **5x5 pixel window** around the HSV centroid and calculates the **median** depth.
*   **Contribution:** Robustness against "depth holes" and edge noise common in Gazebo's simulated depth cameras.

### B. Pose Latching
The perception system publishes at 10 Hz but "latches" the last good detection.
*   **Logic:** If the blue box leaves the camera's FOV (e.g., when the robot is very close), the node continues to publish the last known 3-D pose in `base_link`.
*   **Contribution:** Allows MoveIt to complete a plan even if the "eyes" of the robot are blinded by proximity.

## 3. Kinematic Configuration & URDF

### A. Exact Mounting Offsets
*   **Arm Base:** `xyz=\"-0.03 0 0.02\" rpy=\"0 0 1.5708\"`. Mounted slightly rearward to clear the LiDAR.
*   **Depth Camera:** `xyz=\"0.1 0 0.065\"`. Positioned for a top-down view of the workspace.
*   **Gripper TCP:** Defined as a massless frame at the `gripper_base`. Offsets are handled by MoveIt planning.

### B. The "Mimic-Free" Gripper Strategy
In `mycobot_ros2_control.xacro`, the project abandons URDF `<mimic>` tags.
*   **Logic:** Gazebo Classic often fails to solve the constraint for mimic joints, resulting in "limp" fingers. 
*   **Implementation:** All 6 gripper joints are exposed as independent `position` command interfaces. The "mimic" behavior is implemented in the control software (Python) using mirrored signs: `[1, 1, -1, -1, -1, 1]`.

## 4. MoveIt 2: The Collision Matrix Contribution

### A. Unlocking the Workspace
The most significant "hidden" contribution is the manual edit to `limo_cobot.srdf`.
*   **Problem:** The MoveIt Setup Assistant failed to identify that the arm's wrist (`joint6`) and the gripper base pass safely *over* the LIMO's rear wheels and LiDAR. It flagged them as colliding, effectively "locking" the arm's workspace.
*   **Solution:** Manually added 24 `<disable_collisions>` pairs (lines 280-305 of `limo_cobot.srdf`) between the distal arm links and the base components.
*   **Result:** Enabled the "Stage 4" collision-aware planning that allows the arm to reach forward and down without triggering false-positive collision halts.
