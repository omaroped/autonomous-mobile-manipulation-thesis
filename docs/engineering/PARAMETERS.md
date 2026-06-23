# Verified Robot Parameters & Configuration (PARAMETERS.md)

This document contains a catalog of every physically meaningful parameter, sensor specification, and software configuration value used across the LIMO Cobot simulation and hardware integration.

---

## 1. Chassis and Kinematic Base

These parameters describe the physical chassis of the Agilex LIMO PRO base.

| Parameter | URDF Model Value | Verified Hardware Spec | Status / Note |
|---|---|---|---|
| **LIMO Total Mass** | `~4.8 kg` | `4.8 kg` | ✅ Confirmed by summing link inertias. |
| **Base Mass (Chassis Box)**| `2.1557 kg` | `—` | ✅ Reverted from 8.0 kg (real-faithful). |
| **Base Collision Box** | `0.13 × 0.12 × 0.10 m` | `0.322 × 0.220 × 0.251 m` | ⚠️ Simplified for collision checking. |
| **Wheelbase** | `0.20 m` | `0.20 m` | ✅ Matches physical chassis. |
| **Wheel Track (Tread)** | `0.175 m` | `0.175 m` | ✅ Corrected from legacy `0.14 m`. |
| **Wheel Radius** | `0.045 m` | `0.045 m` (⌀ 90 mm) | ✅ Verified. |
| **Min Turning Radius** | `0.40 m` | `0.40 m` | ✅ Car-like Ackermann constraint. |

---

## 2. Sensor Configurations

### Depth Camera: Orbbec DaBai

The depth camera is mounted forward. The real camera has a blind spot that must be handled in software.

| Parameter | Simulation Value | Verified Hardware Spec | Status / Action |
|---|---|---|---|
| **Horizontal FOV** | `1.185 rad (67.9°)` | `67.9° H`, `45.3° V` | ✅ Corrected from legacy `80°`. |
| **Resolution** | `640 × 400` | `640 × 400` (depth) | ✅ Matches sensor output. |
| **Near Clip** | `0.05 m` | `0.30 m` | ⚠️ **Critical Sim-to-Real Gap:** Sim uses `0.05m` to allow close-range grasp tracking. The physical camera is blind closer than `0.30m`. The hardware must use a perceive-then-approach docking strategy. |
| **Far Clip** | `3.0 m` | `3.0 m` | ✅ Corrected from legacy `8.0 m`. |
| **Update Rate** | `30 Hz` | `30 Hz` | ✅ Configured. |

### LiDAR: EAI T-mini Pro

| Parameter | Simulation Value | Verified Hardware Spec | Status / Action |
|---|---|---|---|
| **Angular FOV** | `±3.14159 rad (360°)`| `360°` | ✅ Corrected from legacy `240°` (±120°). |
| **Ranging Range** | `0.02 – 12.0 m` | `0.02 – 12.0 m` | ✅ Corrected from legacy `0.2 – 8.0 m`. |
| **Samples** | `666` | `360° / 0.54° ≈ 666` | ✅ Matches angular resolution. |
| **Update Rate** | `8 Hz` | `6 – 12 Hz` | ✅ Verified. |

---

## 3. Manipulator and Gripper

These values specify the Elephant Robotics myCobot 280 M5 manipulator and its parallel adaptive gripper.

| Parameter | URDF Model Value | Verified Hardware Spec | Status / Action |
|---|---|---|---|
| **Arm Reach** | `280 mm` | `280 mm` | ✅ Sum of link lengths. |
| **Payload Capacity** | `—` | `250 g` | ✅ Target block is 20 g (well within range). |
| **Gripper Mass** | `110 g` | `110 g` | ✅ Model matches spec. |
| **Gripper Travel** | `20 – 45 mm` | `20 – 45 mm` | ✅ Replaces incorrect handover dossier numbers. |
| **Grasp TCP Offset** | `26 mm` | `—` | ✅ Verified lateral centering calibration. |
| **Test Box Size** | `30 mm cube` | `30 mm` | ⚠️ Sim-safety: Changed from 20 mm to 30 mm to clear the gripper's minimum clamp limit (20 mm). |

---

## 4. Legitimate Simulation Choices (Not Real Specs)

These parameters are configured to stabilize Gazebo's ODE physics engine and are not representative of physical limits:

* **Friction Coefficients:** Table and box friction coefficients (`mu1`, `mu2`) are set to `10.0` (sim-tuned to prevent sliding during arm retraction).
* **Soft Contacts:** Contact parameter inputs (`kp = 1e5`, `kd = 50.0`, `max_vel = 0.01`, `min_depth = 0.001`) are set in the SDF links to prevent impulse explosions when the gripper contacts surfaces.
* **Speed Scaling:** Velocity and acceleration scales (`VEL_SCALE`, `ACC_SCALE`) are set to `0.2` (for safety during testing).

---

## 5. Software Configurations (YAML Files)

### Navigation Parameters (`bridge_params.yaml`)
* `pick_zone`: `(0.60, -0.22)` m
* `place_zone`: `(-1.0, 0.5)` m
* `cruise_speed`: `0.30` m/s
* `approach_speed`: `0.12` m/s
* `nav_to_pick_timeout`: `90` s
* `nav_to_place_timeout`: `60` s
* `nav_to_home_timeout`: `75` s
* `nav_xy_tolerance`: `0.05` m
* `nav_yaw_tolerance`: `0.087` rad (~5°)
* `perception_timeout`: `10` s
* `target_dimensions`: `[0.03, 0.03, 0.03]` m (3 cm cube)

### Manipulation Parameters (`mtc_params.yaml`)
* `arm_group`: `arm`
* `gripper_frame`: `gripper_tcp`
* `gripper_open`: `0.15` (float values/joints configuration)
* `gripper_closed`: `-0.20`
* `grasp_frame_transform`: `[0, 0.026, 0, 1.2, 0, 0]`
* `place_pose`: `[0.28, 0.12, 0.025, 0, 0, 0]`
* `approach`: `0.005 – 0.15` m
* `lift_lower`: `0.04 – 0.12` m
* `grasp_pose_angle_delta`: `0.2618` rad (15°)
* `grasp_pose_max_ik_solutions`: `8`
* `cartesian_step_size`: `0.002` m

### Perception Parameters (`perception_params.yaml`)
These parameters have been relaxed to accommodate Gazebo depth noise (which is between 3 mm and 8 mm):

| Parameter | Legacy Value | Relaxed Value (Applied) | Reason |
|---|---|---|---|
| `ransac_distance_threshold` | `0.001` m | `0.008` m | Absorbs sensor depth noise. |
| `line_curvature_threshold` | `0.0011` | `0.010` | Prevents flat box face curvature rejection. |
| `inlier_threshold` | `85` points | `30` points | 3 cm box has fewer points per face. |
| `line_max_clusters` | `2` | `3` | Handles edge fragmentation due to noise. |
| `circle_max_clusters` | `2` | `3` | Same reason. |
| `min_cluster_size` | `100` points | `50` points | Prevents small object rejection. |
| `num_iterations` | `5` | `8` | Improves Hough vote convergence. |
| `ransac_max_iterations` | `1000` | `100` | Reclaims server performance. |
| `curvature_threshold` (region) | `0.2` | `0.5` | Prevents object surface exclusion. |
| `smoothness_threshold` | `20.0` | `25.0` | Allows surface variance. |
| `plane z_tolerance` | `—` | `0.15` m | Set to cover 15 cm base link offset. |

---

## 6. Manipulator Joint Limits & Named States

### Joint Position Limits (Radians)
* **Joint 1:** `[-2.932, 2.932]`
* **Joint 2:** `[-2.443, 2.443]`
* **Joint 3:** `[-2.618, 2.618]`
* **Joint 4:** `[-2.618, 2.618]`
* **Joint 5:** `[-2.705, 2.793]`
* **Joint 6:** `[-3.142, 3.142]`

### Named Planning States
* **`home`** = `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`
* **`ready`** = `[0.0, -0.5, -1.2, 1.7, 0.0, 0.0]` (top-down ready pose)
