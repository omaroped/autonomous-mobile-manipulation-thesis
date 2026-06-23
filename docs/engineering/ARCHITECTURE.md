# System Architecture & Design (ARCHITECTURE.md)

This document serves as the authoritative, code-grounded reference for the system design and physical integration of the LIMO Cobot Mobile Manipulation system.

---

## 1. The Two-Workspace Reality

This project is developed across **two sibling workspaces** with distinct design lineages:

| Workspace | Path | Lineage | State & Capability |
|---|---|---|---|
| **Thesis** | `~/Desktop/Thesis` | MoveIt 2 + MoveIt Task Constructor (MTC) + PCL perception server | Advanced perception and inverse kinematics (IK) research. **End-to-end pipeline not fully closed** (the placing stage is stubbed, and perception remains fragile under noisy conditions). |
| **Thesisorg** | `~/Desktop/Thesisorg` | Direct controller + HSV visual servoing + MoveGroup pick | Simpler, robust control pipeline. **The parts that actually run end-to-end** (follow → stop → top-down pick) live here. |

### Technical Summary
* The **Thesis** workspace is where the perception/planning sophistication was built.
* The **Thesisorg** workspace is where a working follow-and-pick demo was achieved.
* High-level results (100% success, 43s cycle time) represent best-case integrated simulation scenarios and require specific coordinate alignments rather than a single plug-and-play run of either workspace's `main` branch.

---

## 2. Platform & System Summary

The platform consists of the **Agilex LIMO PRO** mobile base combined with the Elephant Robotics **myCobot 280 M5** robotic arm, an adaptive parallel-jaw gripper, an Orbbec DaBai depth camera, an EAI T-mini Pro 2D LiDAR, and a HI226 IMU, driven by an NVIDIA Jetson Orin Nano (8GB).

### Hardware Specifications

| Component | Parameter | Value / Specification |
|---|---|---|
| **Chassis (LIMO PRO)** | Dimensions (L×W×H) | 322 × 220 × 251 mm |
| | Wheelbase | 200 mm |
| | Tread (Track Width) | 175 mm |
| | Mass / Payload | 4.8 kg / 4.0 kg |
| | Max Speed | 1.0 m/s (no-load); planner config cap: `max_vel_x = 0.55 m/s` |
| | Kinematics | Ackermann (car-like), 4WD Diff, Track, Mecanum. (Thesis uses **Ackermann**; min radius = 0.4m) |
| **Manipulator (myCobot 280)** | Degree of Freedom | 6 DoF |
| | Reach / Payload | 280 mm / 250 g |
| | Repeatability / Serial | ±0.5 mm / `/dev/ttyTHS1` @ 1,000,000 baud |
| **Gripper (Adaptive)** | Type | Parallel linkage, 110 g, max clamp 150 g, jaw travel 20–45 mm |
| **LiDAR (EAI T-mini Pro)** | Ranging Range | 0.02 – 12.0 m (360° coverage, 6–12 Hz) |
| **Depth Camera (Orbbec DaBai)** | Depth Range / FOV | 0.30 – 3.0 m / Depth: H 67.9° × V 45.3° |
| **Onboard Compute** | System | Jetson Orin Nano (8GB LPDDR5, Ubuntu 20.04 native) |
| **Inventory** | Barcode Tag | HSRW inventory tag `3692838` |

---

## 3. Physical Mounting & Calibration

The physical placement of the arm on the mobile chassis has been calibrated to eliminate mounting adapters and ensure exact kinematic alignment:

* **Direct Mount:** The square mounting adapter (`g_base`) was removed to align with the real-world hardware. The arm base is mounted directly flush to the top deck of the LIMO PRO.
* **Kinematic Transform:** The fixed joint `arm_mount_joint` in `limo_cobot.xacro` is defined with the origin:
  ```xml
  <origin xyz="-0.03 0.0 0.025" rpy="0 0 1.5708" />
  ```
* **Arm Base Rotation (+90° Z-Axis):** The myCobot arm base link mount is rotated by 90° CCW around the Z-axis. Consequently, joint 1 requires a setting of $J_1 = -1.5708$ ($-\pi/2$) to point the arm forward.
* **SRDF Base Link:** The MoveIt 2 chain base in `limo_cobot.srdf` is configured to start at `joint1` instead of `g_base`.

---

## 4. Software Architecture and Data Flow

The system operates in **ROS 2 Humble** and **Gazebo Classic 11**. Nodes communicate via topics, services, and actions.

```
       ┌──────────────────────── GAZEBO (gzserver) ────────────────────────┐
       │  physics + the simulated robot + the camera + the controllers      │
       └──┬──────────────────┬───────────────────┬──────────────────▲───────┘
          │ publishes        │ publishes         │ publishes        │ subscribes
          │ camera images    │ /joint_states     │ /clock           │ arm / gripper cmds
          ▼                  ▼                   ▼                  │
   ┌─────────────┐    ┌───────────────────────────────────────┐     │
   │ Perception  │    │     pick_orchestrator / Bridge        │     │
   │ (HSV+depth) │    │  - Coordinates the state machine      │     │
   │             │ ◄──┤  - Inserts collision objects (table)  │     │
   └──────┬──────┘    │  - Manages visual docking / approach  │     │
          │ /box_pose └──┬──────────────────┬─────────────────┘     │
          ▼              │ sends Goal       │ publishes gripper cmds│
   ┌─────────────┐       ▼ via /move_action └───────────────────────┼───────┘
   │  move_group │ ◄─────┘                                          │
   │  (MoveIt 2) ├──────────────────────────────────────────────────┘
   └─────────────┘ sends Joint Trajectory action
```

### Core Software Components

1. **Simulator & ros2_control:** Gazebo simulates the environment and loads plugins (`libgazebo_ros_camera.so`, `libgazebo_ros_ray_sensor.so`, `libgazebo_ros_imu_sensor.so`). The `gazebo_ros2_control` interface manages command arbitration for joints and wheels.
2. **Perception:**
   * *Thesis:* `get_planning_scene_server.cpp` processes the raw point cloud from `/depth/points`, performs crop filters, plane segmentation, normal estimation, and region-growing clustering to segment target shape models (RANSAC).
   * *Thesisorg:* `box_follower.py` segments the target in the HSV color space (Gazebo Blue), picks the closest contour, and uses a median depth filter to calculate coordinates.
3. **Motion Planning:** MoveIt 2 (`move_group`) acts as the path search engine. It solves IK via the KDL kinematics plugin and searches paths via OMPL/RRTConnect. Adjacent self-collisions are managed via the semantic robot description (`limo_cobot.srdf`).
4. **Task Orchestration (Orchestrator/Bridge):**
   * *Thesis:* `bridge_node.py` runs an 8-state FSM (`INIT` → `NAV_TO_PICK` → `DETECT` → `PICK` → `NAV_TO_PLACE` → `PLACE` → `NAV_HOME` → `DONE`). Note: `_state_place()` is a stub.
   * *Thesisorg:* `nav_pick_orchestrator.py` handles the Visual Docking Phase (go-to-goal waypoint alignment, steering-centering, active line-following, and odometry-measured blind approach), then invokes MoveGroup to perform top-down grasp actions.

---

## 5. Authored ROS 2 Packages

All code is integrated under the following packages:
* **`limo_car` (`Thesisorg`):** Core robot bringup launcher, URDF/Xacro models, navigation maps, and the visual docking/picking orchestrators (`nav_pick_orchestrator.py`, `box_follower.py`).
* **`limo_cobot_bridge` (`Thesis`):** Coordinating FSM action client wrapper.
* **`limo_cobot_tasks` (`Thesis`):** Contains `moveit_client.py` and the 22-seed IK solving strategy.
* **`hello_mtc_with_perception` (`Thesis`):** C++ Point Cloud Library perception server and MoveIt Task Constructor nodes.
* **`limo_cobot_moveit_config`:** Semantic planning groups, kinematics configurations, and collision avoidance matrix parameters.
