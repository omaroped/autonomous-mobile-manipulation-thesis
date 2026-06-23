# Pass 7 — The Honest Engineering Memo

**TO:** Thesis Supervisor
**FROM:** Senior Robotics Auditor
**DATE:** June 13, 2026
**SUBJECT:** Forensic Audit of the LIMO + myCobot Mobile Manipulation Project

### Executive Summary
The project has successfully delivered a functional, end-to-end autonomous mobile manipulation pipeline in a ROS 2 Humble + Gazebo Classic simulation. The core contribution is a **collision-aware integrated picking system** that overcomes significant toolchain incompatibilities (EOL Gazebo plugins) and kinematic constraints.

### What Actually Works
*   **Precision Docking:** The visual servoing node (`box_follower`) handles the Ackermann base's constraints to park the robot within the ~0.05 m "reachable window" required for the myCobot's limited reach.
*   **Perception-to-TF:** A robust HSV-based depth deprojection pipeline converts raw camera data into transformable 3-D target frames.
*   **Collision-Aware Planning:** The MoveIt 2 implementation is "Stage 4" mature, meaning it actively manages the planning scene (e.g., adding the table as a collision object) to ensure safe arm trajectories.
*   **End-of-Life Patches:** The project successfully "resuscitated" the limp arm by patching the `gazebo_ros2_control` plugin at the source code level to restore controller compatibility.

### Technical Trade-offs & Scope Reduction
The project underwent a significant "Simplification Pivot" in April 2026. High-complexity components like the **MoveIt Task Constructor (MTC)** and **Point Cloud Library (PCL)** were abandoned. While this reduces the "algorithmic novelty," it significantly increased the **system reliability**. The current reactive pipeline is more stable and better suited for the high-drift Gazebo environment. Similarly, the pivot from global **Nav2** to local **Visual Servoing** was a pragmatic engineering decision to solve the docking precision problem.

### Primary Research Insight
The binding constraint of this specific robot combination is the **geometric overlap** between the base's stopping distance and the arm's reach. The thesis effectively demonstrates that for small-scale mobile manipulators, the "navigation-to-grasp hand-off" is the most critical and failure-prone phase, requiring tighter integration than what off-the-shelf global planners usually provide.

### Recommendation
The project is ready for defense. The student should focus the written thesis on the **integration engineering** and the **reachability analysis** (supported by the `reach_map.csv` data) rather than claiming novelty in planning algorithms. The "Stage 4" MoveIt pipeline is a substantive artifact that justifies the "Simulation-based integration study" scope.
