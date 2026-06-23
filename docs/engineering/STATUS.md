# Project Status, Roadmap & Progress Log (STATUS.md)

This document tracks the active engineering status, pending tasks, academic gaps, learning roadmap, and historical progress logs of the LIMO Cobot Mobile Manipulation project.

---

## 1. Active Status Board

| Subsystem / Function | Status | Evidence / Verification Method |
|---|---|---|
| **Gazebo Physics & URDF** | **Works** | Model spawns stably; no NaN crashes after inertia tensor limits fix. |
| **Controllers (Broadcasters/Joints)**| **Works** | `joint_state_broadcaster`, `arm_controller`, `gripper_controller` spawn and load active. |
| **Gripper Mimic Synch** | **Works** | Gripper linkage closes and opens symmetrically in simulation. |
| **MoveIt 2 IK (Within Envelope)**| **Works** | 22-seed search yields 100% solution rate within reachable boundaries. |
| **Perception (Close / Clean)** | **Partial** | Hough/RANSAC extracts model successfully when target is near and noise is low. |
| **Perception (Noisy / General)** | **Fragile** | Curvature limits and Euclidean filters fail under high noise. |
| **HSV Follower (Visual Servo)** | **Works** | Detects Gazebo Blue, drives base, and stops at `0.095 m` standoff distance. |
| **Top-Down Grasp (Stationary)** | **Works** | Executes `pre-grasp` → `descent` → `close` → `lift` from a stationary base. |
| **Simulation Attachment (Weld)**| **Works** | Live TF lookup (`odom→gripper_tcp`) — box correctly follows gripper after robot drives to table. Fixed 2026-06-23 (was broken: stale startup pose caused box to float). |
| **Navigation (Diff Drive / Nav2)** | **Works** | Nav2 + AMCL navigate to RViz goals autonomously. Clean map (0 noise pixels). LIDAR self-scan fixed. Fixed 2026-06-23. |
| **Placing Stage** | **Not Done** | `bridge_node._state_place()` contains a `TODO` stub. |
| **Full Integrated Cycle** | **In Progress** | nav→pick→backup demonstrated. Place stage still a TODO stub. |
| **Sim-to-Real Hardware Transfer**| **Not Started**| All testing has been completed in simulation; nothing has run on physical hardware. |

---

## 2. Academic & Engineering Gaps

### Academic Gaps
1. **Literature Synthesis:** Compile relevant literature covering ROS 2 mobile manipulation, Ackermann steering planners (TEB/Smac), RGB-D visual servoing, and sim-to-real transfer.
2. **Quantitative Benchmarking:** Execute repeated trial campaigns ($N \ge 20$) in simulation to record success rates, final docking errors, grasping repeatability, and cycle times for statistical reporting.
3. **Hardware Deployment:** Swap Gazebo simulation plugins for real hardware drivers (Jetson, myCobot baud rates, DaBai camera parameters) and run validation trials.

### Immediate Engineering Tasks
1. **Implement Place Stage:** Replicate the grasp sequence in reverse to support place actions.
2. **Wrist Yaw Misalignment Correction:** Adjust Joint 6 (wrist yaw) dynamically based on target box orientation to ensure gripper jaws align parallel to box faces.
3. **HSV Path Fallback:** Use the simple HSV visual servo tracker as a fallback for the C++ perception server if cluster noise cannot be filtered reliably.

---

## 3. Learning Roadmap

Follow these modules to understand the underlying robotics concepts:

### Module 1: ROS 2 computation Graph
* Command list: `ros2 node list`, `ros2 topic echo /odom`, `ros2 topic hz /scan`.
* Concept: Publish direct `geometry_msgs/Twist` values to `/cmd_vel` to drive the Ackermann chassis.

### Module 2: The Simulation launch Chain
* Process: URDF creation via Xacro → Spawning entity → Starting controllers → RViz visualization.
* Concept: Enforcing launch event handlers (`OnProcessExit`) to avoid controller spawn races.

### Module 3: URDF coordinate frames
* TF Tree: `map` → `odom` → `base_footprint` → `base_link` → sensor links.
* Concept: Understanding the elevation offset between `base_footprint` (ground level) and `base_link` (center of chassis).

---

## 4. Historical Progress Log

This log lists major achievements from the beginning of development.

* **[2026-03-08] Log and LaTeX Initialization:** Created `thesis_notebook.md` and generated the thesis template structure inside `thesis/`.
* **[2026-03-08] PDF Compilation Scripting:** Successfully compiled `main.pdf` and verified `cleanup.sh` helper scripts.
* **[2026-03-08] Writing Resources Collection:** Populated academic writing guidelines and HSRW formatting resource guides.
* **[2026-03-08] Repository Setup:** Configured `.gitignore` to omit LLM runtime outputs and generated the project README.
* **[2026-03-07] Gazebo World layout:** Set processing counter coordinate splits inside the simulation world layout.
* **[2026-03-07] Launch Script Bootstrapping:** Drafted initial launch scripts to spawn LIMO and myCobot arm.
* **[2026-03-07] Voice-to-Text Setup:** Configured `SpeechNote` dictation parameters to speed up chapter drafting.
* **[2026-06-11] MoveIt 2 Self-Collision Fix:** Discovered wrist-to-base collision checking omissions in SRDF and added exclusions to restore Cartesian planning.
* **[2026-06-12] Gripper Linkage Scale Fix:** Identified MoveIt COLLADA mesh scaling unit conflicts and applied the `scale="0.001"` fix.
* **[2026-06-14] Parameter Verification Audit:** Cross-checked and corrected wheelbase, sensor FOV, and clip limits to match physical specs.
* **[2026-06-15] Navigation Lifecycle and Docking:** Resolved Nav2 node collision heartbeats and implemented the two-stage visual docking algorithm.
* **[2026-06-23] Diff Drive Pipeline Fix Session:** Fixed 9 bugs — LIDAR 360°→120° self-scan fix, contaminated SLAM map rebuilt (0 noise pixels), Nav2 costmap min-range raised (arm self-marking as lethal), grasp_attacher stale startup pose replaced with live TF lookup, Gazebo physics destabilization reverted, wheel separation corrected 0.175→0.172m, RViz crash fixed (removed MoveIt panel), workspace ~/.bashrc contamination from Desktop/Thesis project removed. Full nav→pick→backup pipeline demonstrated. New files: `slam_wanderer.py`, `nav_monitor.rviz`, `nav2_limo_diff.yaml`, clean map.
