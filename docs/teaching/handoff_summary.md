# Master Handoff & Search Index Document

This document serves as a comprehensive, self-contained handoff guide and chronological search index for the **27,360-line conversation log** stored at:
`[conversation_log.md](file:///home/omar/Desktop/Thesisorg/docs/teaching/conversation_log.md)`

A new AI session can use this document to understand the project goal, trace every design decision, locate code fragments, and identify the exact line ranges in the log to retrieve context.

---

## 1. Goal and Background
*   **Thesis Topic:** Autonomous mobile-manipulation pick-and-place pipeline (Navigate $\rightarrow$ Perceive $\rightarrow$ Grasp $\rightarrow$ Place/Stack).
*   **Student:** Omar Alobaid (Matriculation Number: 29606).
*   **Supervisor:** Prof. Ronny Hartanto.
*   **Hardware Platform:**
    *   **Base:** AgileX **LIMO PRO** (car-like Ackermann base by default).
    *   **Arm:** Elephant Robotics **myCobot 280 M5** (6-DOF manipulator).
    *   **Gripper:** Custom adaptive parallel gripper.
    *   **Sensors:** **Orbbec DaBai** depth camera + **EAI T-mini Pro** LiDAR + Jetson Orin Nano (onboard computer).
*   **Software Stack:** ROS 2 Humble, Gazebo Classic 11, Nav2, MoveIt 2, `ros2_control`.
*   **Active Workspace:** `/home/omar/Desktop/Thesisorg/src/ros2_ws` (Referred to as **Thesisorg** or **ORG**).
*   **Inactive/Sibling Workspace:** `/home/omar/Desktop/Thesis` (An older, abandoned attempt using MoveIt Task Constructor (MTC) + Point Cloud Library (PCL) + RANSAC perception. It was too complex and brittle. Do **not** use it; it is kept only for reference).

---

## 2. Key Engineering & Design Decisions
1.  **Direct Control vs. MoveIt 2:** In Phase 1, the arm was driven via direct joint trajectory control (`limo_pick_place.py`). In Phase 4, the project transitioned to MoveIt 2 (`move_group` via `/move_action` interface) for collision-aware path planning, which prevents the arm from colliding with the chassis or table. MoveIt Task Constructor (MTC) was rejected as over-engineered.
2.  **Ackermann to Differential Drive Conversion:** The LIMO base URDF/Xacro (`ackermann.xacro`) was modified to support both modes, switchable via `drive_mode:=diff|ackermann`. Differential drive is preferred during execution because it allows the robot to rotate in place for lateral alignment.
3.  **Ground-Truth vs. AMCL Localization:** Configured ground-truth localization (`localization:=ground_truth`) via a static `map` $\rightarrow$ `odom` identity transform for fast testing, alongside real LiDAR-based AMCL (`localization:=amcl`).
4.  **Odometry Docking Hard-Stop (Camera Blind Spot):** The Orbbec DaBai camera is inaccurate closer than 0.30m (blind point-blank). Relying on live camera feedback at close range caused the robot to hit the table and climb it.
    *   *Solution:* The orchestrator deactivates Nav2 near the table, rotates in place to align laterally ($y \approx 0$), takes a median box reading at a safe distance (~0.60m back), and drives the remaining distance blindly via odometry, stopping at `STOP_DISTANCE = 0.24m`.
5.  **Arm Mount Joint Rotation (+90° Z-Axis):** The myCobot arm base link mount was rotated by 90° CCW around the Z-axis in URDF (`rpy="0 0 1.5708"` in `ackermann_with_sensor.xacro`) to make the arm's screen face to the side. Consequently, joint 1 now needs `J1 = -1.5708` ($-\pi/2$) to point the arm forward.
6.  **Simulation Grasp Weld:** Gazebo Classic's parallel contact solver is slippery. While the physical robot uses friction, the simulation utilizes a `grasp_attacher` plugin that welds the box to the gripper link upon contact to prevent the box from slipping.

---

## 3. Chronological Search Index for `conversation_log.md`

Use this map to search for specific logs, errors, and code blocks inside the 27,360-line log:

| Line Range | Phase / Topic | Key Content & Major Discoveries |
| :--- | :--- | :--- |
| **L1 – L1,647** | **Phase 1: Initial Setup, Gripper Orientation, & World Customization** | Discussion of Thesis vs. Thesisorg workspaces (L37-53). Sideways gripper bug resolved by setting J5=0 for top-down pickup (L54-122). Arm base rotated 90° CCW (L341-384). Arm base joint rotation in `ackermann_with_sensor.xacro` (rpy="0 0 1.5708") (L385-513). Crawl bug in `box_follower.py` resolved (L615-704). Node executable registration issues fixed in CMake (L707-954). |
| **L1,648 – L3,946** | **Phase 2: Resolving Gazebo Plugin Startup & Pyenv Interception** | Troubleshooting spawner and `controller_manager` timeouts (L955-1166). Resolving zombie processes (L1254-1373). Fixing `gazebo_ros2_control` system vs. world plugin types (L1405-1540). Rewriting `ackermann_gazebo.launch.py` to bypass `pyenv` by calling `/usr/bin/python3` explicitly and resolving Gazebo system flags (L1750-3946). |
| **L3,947 – L4,582** | **Phase 3: Narrowing Table Geometry, Ramping Gripper, and Arm Mount** | Fixing robot front hitting the table (crawling/stopping at 0.19m instead of 0.17m) by narrowing the table Y-depth to 0.15m (L3947-4100). Resolving gripper mimic joint issues by adding all 5 mimic joints to the YAML controller parameters. Shifting the arm base joint 3cm rearward (`xyz="-0.03 0 0.02"`) to improve alignment (L4100-4582). |
| **L4,583 – L9,123** | **Phase 4: MoveIt 2 Integration and Planning Scene Setup** | Implementing the `limo_cobot_moveit_config` package (SRDF, kinematics, controllers) (L4956-5483). Fixing EOL `gazebo_ros2_control` plugin compatibility with newer `controller_manager` 2.54 by patching the plugin source in the workspace (L6155-6756). Troubleshooting Cartesian planning failures and "Unable to sample any valid states for goal tree" errors (L7839-8364). |
| **L9,124 – L12,627** | **Phase 5: The Collision Matrix Breakthrough & Mesh Scale Fix** | Testing the arm on a fixed base in isolation (L9201-10115). **The Big Discovery:** Identifying the incomplete SRDF collision matrix where MoveIt registered collisions between the gripper base and LIMO wheels/laser (L10538-10652). **Gripper Scale Fix:** Finding that MoveIt ignored the COLLADA `<unit>` tag, causing the gripper to load 1000× too large (58m). Fixed with `scale="0.001"` in URDF (L11381-11526). Creating the `grasp_attacher.py` weld and `base_pin.py` nodes (L11527-12627). |
| **L12,628 – L19,349** | **Phase 6: Nav2 Navigation & Decimated Base Collisions** | Writing LaTeX thesis chapters (L12701-13836). Setting up Nav2 for Ackermann drive (`nav2_limo_ackermann.yaml`). Resolving lifecycle heartbeat crashes by setting `bond_timeout: 0.0` (L16711-17987). Decimating the complex LIMO chassis visual mesh to create a simplified `limo_base_collision.stl` (3144 triangles) to resolve Gazebo loading hangs (L17988-19220). Integrating visual-servo with Nav2 (L19221-19349). |
| **L19,350 – L26,707** | **Phase 7: Camera Blind Spot (Odometry Approach) & Diff-Drive** | Documenting the Orbbec DaBai 30cm close-range blind spot / bias and its sim-to-real implications (L19401-21027). Discussion of "perceive-then-blind-approach" (dock geometry) (L21028-22397). Deactivating Nav2's cmd_vel nodes during visual servoing to stop them from fighting the orchestrator's commands (L22398-23232). Fixing LiDAR self-occlusion by shrinking `laser_link` collision cylinder to 0.001m (L23233-24292). Converting the robot base to differential drive (`drive_mode:=diff`) (L24293-24880). Implementing visual docking Phase B odometry-measured hard stop at `STOP_DISTANCE = 0.24m` (L25164-26707). |
| **L26,708 – L27,360** | **Phase 8: Grasp Calibration Loop & Live Parameter Tuning** | Implementing `calib_loop` in `nav_pick_orchestrator.py` (L26807-26937). Testing the 8-iteration calibration loop and identifying base yaw misalignment as the cause of gripper landing off-center (L26938-27073). Setting default offsets to `grasp_off_x = 0.0`, `grasp_off_z = 0.0` (L27074-27167). Discussion on AMCL/RViz visualization and creating the local chat exporter script `export_chat.py` (L27168-27360). |

---

## 4. Key Files and Codebase Audit

All files are located relative to the workspace root `/home/omar/Desktop/Thesisorg/src/ros2_ws/src/`:

1.  **Orchestrator Node:** `[nav_pick_orchestrator.py](file:///home/omar/Desktop/Thesisorg/src/ros2_ws/src/limo_ros2/limo_car/scripts/nav_pick_orchestrator.py)`
    *   *Purpose:* Handles the state machine (Nav $\rightarrow$ Dock $\rightarrow$ Pin $\rightarrow$ Grasp $\rightarrow$ Lift $\rightarrow$ Back up) and contains the `calib_loop` method for live offset calibration.
    *   *Log Reference:* L26735, L26807, L26910.
2.  **Unified Robot Model:** `[ackermann_with_sensor.xacro](file:///home/omar/Desktop/Thesisorg/src/ros2_ws/src/limo_ros2/limo_car/gazebo/ackermann_with_sensor.xacro)`
    *   *Purpose:* Combines LIMO chassis, myCobot arm, adaptive gripper, camera, and LiDAR. Contains the arm base rotation offset (`rpy="0 0 1.5708"`) and the rearward placement (`xyz="-0.03 0 0.02"`).
    *   *Log Reference:* L385, L4552, L4700.
3.  **MoveIt 2 Semantic Configuration:** `[limo_cobot.srdf](file:///home/omar/Desktop/Thesisorg/src/ros2_ws/src/limo_cobot_moveit_config/config/limo_cobot.srdf)`
    *   *Purpose:* Defines the planning groups (`arm` and `gripper`), named poses (`home`, `ready`), and contains the critical `disable_collisions` matrix preventing false self-collisions between the gripper base and LIMO wheels.
    *   *Log Reference:* L10538, L10611, L12676.
4.  **Base Drive Configuration:** `[ackermann.xacro](file:///home/omar/Desktop/Thesisorg/src/ros2_ws/src/limo_ros2/limo_car/gazebo/ackermann.xacro)`
    *   *Purpose:* Supports switchable differential drive or Ackermann drive based on the `drive_mode` parameter.
    *   *Log Reference:* L24293, L24384, L26764.
5.  **LiDAR Collision Model:** `[sensor.xacro](file:///home/omar/Desktop/Thesisorg/src/ros2_ws/src/limo_ros2/limo_car/gazebo/sensor.xacro)`
    *   *Purpose:* Shrunk the LiDAR's `laser_link` collision cylinder to 0.001m to prevent the sensor rays from colliding with the robot body.
    *   *Log Reference:* L23233, L26773.
6.  **AMCL Navigation Configuration:** `[nav2_limo_diff.yaml](file:///home/omar/Desktop/Thesisorg/src/ros2_ws/src/limo_ros2/limo_car/config/nav2_limo_diff.yaml)`
    *   *Purpose:* Contains Nav2 parameters for AMCL. The initial pose configuration has `set_initial_pose: true` enabled to ensure proper localization.
    *   *Log Reference:* L24881, L26774.
7.  **Gazebo Physics World:** `[final_map.world](file:///home/omar/Desktop/Thesisorg/src/ros2_ws/src/limo_ros2/limo_car/worlds/final_map.world)`
    *   *Purpose:* Contains the warehouse map, the 10cm brown pickup table, the 4cm vivid blue `target_box` model, and the `gazebo_ros_state` world plugin needed for base pinning and the weld.
    *   *Log Reference:* L4020, L12670, L27074.

---

## 5. Next Steps and Action Items

1.  **Wrist Rotation (Yaw Misalignment Correction):**
    *   *Context (L26938-27073):* The visual docking centering phase (Phase A) rotates the base to align with the box, but leaves a small residual yaw error. Because the gripper's top-down orientation is fixed, the fingers pinch the box at an angle (one finger on the edge, one in the middle), leading to drops.
    *   *Action:* Retrieve the robot's orientation relative to the box (either from `/odom` or the visual `/box_pose` yaw angle) and apply a rotation to joint 6 (wrist yaw) during planning to keep the fingers parallel to the box's faces.
2.  **Weld Restoration:**
    *   *Context (L27074-27167):* Welding was disabled (`USE_WELD = False` in `nav_pick_orchestrator.py`) to test Gazebo's physical friction, which proved unstable.
    *   *Action:* Once grasp offsets are verified, change `USE_WELD` back to `True` to allow successful pickup and transport simulation.
3.  **Place and Stack Phase:**
    *   *Action:* Expand `nav_pick_orchestrator.py` to add the placing sequence. Once the box is picked and the arm is in the `home` position, the robot should navigate to a second pre-defined table and place/stack the box at a fixed, hard-coded position.
4.  **RViz Learning Configuration:**
    *   *Action:* Create a pre-configured `nav_learning.rviz` config file that automatically loads the `/map`, `/scan`, `/particle_cloud` (AMCL particles), and `/plan` (Nav2 path) displays.

---

## 6. Execution Command Playbook

Do **not** execute commands directly. Provide them to the user to copy-paste.

### Command A: Clean Stale Gazebo/DDS Processes (Run on simulation hang or crash)
```bash
pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f nav_pick
rm -f /dev/shm/fastrtps_* /dev/shm/sem.* 2>/dev/null
ros2 daemon stop; ros2 daemon start
```

### Command B: Build and Run Calibration Loop (8 iterations)
```bash
cd ~/Desktop/Thesisorg/src/ros2_ws
colcon build --packages-select limo_car
source install/setup.bash
ros2 launch limo_car nav_pick.launch.py drive_mode:=diff calib_loops:=8
```

### Command C: Live Offset Calibration (Run in Terminal 2 while loop is running)
```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/Thesisorg/src/ros2_ws/install/setup.bash
ros2 param set /nav_pick_orchestrator grasp_off_x 0.0     # - = back, + = forward
ros2 param set /nav_pick_orchestrator grasp_off_y 0.013   # + = left, - = right
ros2 param set /nav_pick_orchestrator grasp_off_z 0.0     # - = down, + = up
```

### Command D: Launch Dedicated Navigation / AMCL Learning Session
Run each block in a separate terminal:
```bash
# Terminal 1: Simulation (diff mode, rviz off)
cd ~/Desktop/Thesisorg/src/ros2_ws && source install/setup.bash
ros2 launch limo_car ackermann_gazebo.launch.py drive_mode:=diff use_rviz:=false
```
```bash
# Terminal 2: Nav2 with real AMCL localization
cd ~/Desktop/Thesisorg/src/ros2_ws && source install/setup.bash
ros2 launch limo_car nav2_limo.launch.py drive_mode:=diff localization:=amcl
```
```bash
# Terminal 3: RViz
cd ~/Desktop/Thesisorg/src/ros2_ws && source install/setup.bash
rviz2
```
