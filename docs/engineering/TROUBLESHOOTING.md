# Troubleshooting and Problem-Resolution Log (TROUBLESHOOTING.md)

This document catalogs every critical engineering challenge, software bug, and physics engine instability encountered during the integration of the LIMO Cobot Mobile Manipulator system. Each entry details the symptom, root cause, and resolution.

---

## 1. Primary Problem-Resolution Log (The Engineering Log)

### 1.1 Gazebo Physics Collapse (NaN Arm Explosion)
* **Symptom:** On startup, the myCobot arm breaks apart, flies off-screen, and `robot_state_publisher` spams `NaN` transform warnings.
* **Root Cause:** The manufacturer-provided URDF moment of inertia values for `joint2` violated the rigid-body triangle inequality ($I_{xx} + I_{yy} \ge I_{zz} \implies 0.0001 + 0.0001 < 0.0006$). The ODE solver evaluates this invalid tensor and returns undefined states, causing physics collapse.
* **Fix:** Corrected the moments of inertia in `mycobot_280_m5_gazebo.xacro` to be mathematically stable and positive-definite:
  ```xml
  <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001" />
  ```

### 1.2 Parallel Gripper Linkage Sagging
* **Symptom:** Gripper fingers drift apart, lag behind the master joint, and disconnect visually under gravity.
* **Root Cause:** Gazebo Classic ignores URDF `<mimic>` tags for passive joints, treating them as free-spinning.
* **Fix:** Registered mimic constraints inside the `<ros2_control>` block:
  ```xml
  <joint name="gripper_left3_to_gripper_left1">
    <param name="mimic">gripper_controller</param>
    <param name="multiplier">-1.0</param>
  </joint>
  ```
  This forces the hardware interface to calculate and drive the passive joints at 1000 Hz.

### 1.3 Controller-Manager Spawner Race Conditions
* **Symptom:** Command outputs "Controller manager not available" or "hardware interface not available" and the arm remains limp.
* **Root Cause:** Spawner processes were launched before the Gazebo server finished loading the robot description and registering the interface.
* **Fix:** Enforced sequential execution in `gazebo.launch.py` using `OnProcessExit` event handlers:
  `Spawn robot entity` → `Load joint_state_broadcaster` → `Load controllers`.

### 1.4 Arm-Chassis Base Link Misalignment
* **Symptom:** MoveIt plans succeed, but joint commands cause collisions, or the arm base sits offset from the LIMO chassis.
* **Root Cause:** The URDF placed the arm base joint (`joint1`) relative to an adapter block (`g_base`) that is not present on the real robot.
* **Fix:** Deleted `g_base` geometry. Mounted `joint1` directly on the LIMO top plate (`0.025m` elevation offset) and reconfigured `limo_cobot.srdf` to start the arm chain from `joint1`.

### 1.5 Wheel Creep at Zero Velocity Command
* **Symptom:** Mobile base slowly drifts or rotates when no velocity commands are published.
* **Root Cause:** Physics solver round-off errors with no wheel joint friction/damping parameters.
* **Fix:** Configured wheel joint dynamics:
  ```xml
  <dynamics damping="0.1" friction="0.05" />
  ```

### 1.6 MoveIt 2 Self-Collision Failures (Waist Rotation)
* **Symptom:** MoveIt fails to compute Cartesian plans (waits or folds to home) when the arm faces forward.
* **Root Cause:** The auto-generated collision matrix omitted wrist links (`gripper_base`, `joint6`, `joint6_flange`) against base wheels, LiDAR, and camera, causing phantom self-collision checks.
* **Fix:** Added missing `<disable_collisions>` pairs in `limo_cobot.srdf` (reduced collision pairs to 125).

### 1.7 rclpy Service Call Deadlock
* **Symptom:** `/compute_ik` and object attach/detach calls hang/timeout, while CLI runs instantly.
* **Root Cause:** Using `call_async(req)` + `while not future.done(): sleep()` on the main thread blocked the callback executor under Humble's `MultiThreadedExecutor`.
* **Fix:** Transitioned to synchronous client calls (`client.call(req)`) with a background spin thread in `moveit_client.py`.

### 1.8 Navigation Timeout Aborts
* **Symptom:** `NAV_TO_PICK` transitions abort mid-traverse.
* **Root Cause:** A long traverse (9.5m from spawn) exceeded the hardcoded 30s timeout when driving at 0.12 m/s.
* **Fix:** Implemented dynamic timeout computation (`_compute_nav_timeout = distance / cruise_speed + 25 s`) and split cruise speed (0.30 m/s) from docking approach speed (0.12 m/s).

### 1.9 Wheel Slip & Odometry Drift
* **Symptom:** Odometry diverges from ground truth during driving; arm plans from a shifted pose.
* **Root Cause:** Simulated wheel slip in Gazebo propagates encoder drift.
* **Fix:** Utilized `/gazebo/model_states` ground truth in simulation. *Note: Real hardware requires closed-loop AMCL/SLAM and visual tag feedback.*

### 1.10 Depth Camera Near-Clipping
* **Symptom:** Empty point cloud at close range; perception server fails.
* **Root Cause:** Default depth camera near-clip is 0.15m; the box sits closer at grasp range.
* **Fix (Sim-only):** Lowered near-clip in URDF to 0.05m. *Note: Real camera blind spot is 0.30m (see PARAMETERS.md).*

### 1.11 Camera Below Table Top Height
* **Symptom:** Camera sees the vertical face of the table, but target box top is invisible.
* **Root Cause:** Camera center sat lower than the 25 cm table top.
* **Fix:** Lowered table height to 16 cm in the Gazebo world.

### 1.12 C++ Perception Server Double Mapping Crash
* **Symptom:** `GetPlanningScene` service crashes with `std::out_of_range` index errors.
* **Root Cause:** The segmentation logic in `object_segmentation.cpp` applied `projection_map.at()` twice during RANSAC and inlier filtering.
* **Fix:** Replaced double lookup with a single lookup signature and updated the maps.

### 1.13 Curvature and Cluster Parameter Rejections
* **Symptom:** Object segmentation returns `success: false` and extracts 0 objects.
* **Root Cause:** Curvature tolerances (`line_curvature_threshold = 0.0011`) and RANSAC thresholds were too tight for noisy simulation depth clouds.
* **Fix:** Relaxed parameters (curvature threshold to `0.010`, inlier threshold from `85` to `30`).

### 1.14 Inverse Kinematics Local Minima timeouts
* **Symptom:** KDL solver returns `-31` (NO_IK_SOLUTION) inside valid reachable zones.
* **Root Cause:** The default Newton-Raphson solver fell into local minima; the 0.05s timeout was too restrictive.
* **Fix:** Extended solver timeout to 0.5s and implemented a 22-seed IK search strategy (using reachability seeds).

### 1.15 Wrist Gimbal-Lock Singularity
* **Symptom:** IK calls fail for targets within 3 cm of the robot's longitudinal axis ($y \approx 0$).
* **Root Cause:** Singularity when Joint 1 aligns with a flat wrist configuration.
* **Fix:** Clamped lateral target offset to `y_rel = sign(y) * max(0.050, |y|)` (still within the 8 cm gripper jaw width).

### 1.16 Ackermann Steering Limit Cycles
* **Symptom:** Base orbits target at close range (~0.15m) without settling.
* **Root Cause:** Heading error corrections demanded steering angles exceeding the Ackermann turn limit.
* **Fix:** Implemented a two-stage visual dock: (1) align to a stand-off waypoint, (2) center wheels, (3) execute 1D line-following.

### 1.17 Box Pose Capture Timing
* **Symptom:** Arm plans target a far pose mid-approach.
* **Root Cause:** The orchestrator captured target coordinates before the base settled.
* **Fix:** Added a velocity settling check before locking in box coordinates.

### 1.18 Floor-Level Grasping Failures
* **Symptom:** Top-down plan is unreachable for objects sitting on the floor.
* **Root Cause:** Kinematic height constraints: a 280 mm arm on a 15 cm base cannot extend vertically to the floor.
* **Fix:** Elevated targets onto a 16 cm support platform.

### 1.19 Impulsive Gripper Snap
* **Symptom:** Gripper closure kicks the box out of alignment.
* **Root Cause:** Gripper joint inputs were stepped instantaneously.
* **Fix:** Ramped joint commands over 20 discrete increments.

### 1.20 Gripper Orientation Pitch Error
* **Symptom:** Grasp targets reached, but fingers land horizontal instead of vertical.
* **Root Cause:** Using Joint 5 at 1.57 rad.
* **Fix:** Set Joint 5 to 0.0 rad and maintained `J2 + J3 + J4 = -pi/2` for vertical grasping.

---

## 2. Point-Cloud Verification Protocol

If the perception service (`/get_planning_scene_mycobot`) returns `success: false`:

1. **Verify Depth Cloud Flow:**
   ```bash
   ros2 topic hz /depth/points
   # Verify frequency is >15 Hz.
   ```
2. **Launch Server standalone with Parameters:**
   ```bash
   ros2 run hello_mtc_with_perception get_planning_scene_server \
     --ros-args --params-file src/ros2_ws/src/limo_cobot_bridge/config/perception_params.yaml
   ```
3. **Trigger Test Service Call:**
   ```bash
   ros2 service call /get_planning_scene_mycobot mycobot_interfaces/srv/GetPlanningScene \
     "{target_shape: 'box', target_dimensions: [0.03, 0.03, 0.03]}"
   ```
4. **Check /tmp/ Debug PCD files:**
   Inspect `/tmp/5_objects_cloud_debug_cloud.pcd`. If it is empty, plane segmentation is consuming the object. Increase `z_tolerance` or adjust crop parameters.

---

## 2. Session 2026-06-23 — Differential Drive Pipeline Fixes

Nine bugs discovered and resolved while integrating the full autonomous pick-and-place pipeline (navigate → dock → pick → retract).

### 1.21 RViz Hard Crash on nav_pick.rviz
* **Symptom:** RViz segfaults immediately on startup with `nav_pick.rviz`. No error in terminal beyond the crash.
* **Root Cause:** The MoveIt `MotionPlanning` RViz panel loaded the robot model `limo_ackermann` at startup, which crashed the GPU driver on this hardware (dual AMD iGPU / NVIDIA dGPU setup).
* **Fix:** Created `limo_car/rviz/nav_monitor.rviz` — an identical copy of `nav_pick.rviz` with the entire `moveit_rviz_plugin/MotionPlanning` display block and its `MotionPlanning: collapsed: false` window entry removed. Contains: RobotModel, TF, LaserScan, Map, Global/Local Costmap, Path, Camera (RGB).

### 1.22 LIDAR Self-Scan — Robot Scanning Its Own Chassis
* **Symptom:** LIDAR dots visible trailing behind and around the robot in RViz; scan lines clearly hitting the chassis body.
* **Root Cause:** A previous AI agent changed the sensor FOV from ±120° to full ±360° "to match EAI T-mini Pro spec". At ±108–120°, rear beams hit the chassis sides at 6–7cm range. The `min_range` was also set to 0.02m (2cm), well below the 6–7cm chassis hit distance, so self-hits were reported as real obstacles.
* **Fix:** Restored FOV to ±120° forward arc (`min_angle=-2.094`, `max_angle=2.094` rad). Raised `min_range` to 0.10m — chassis self-hits at 6–7cm now fall below the sensor floor. Reference: `omaroped/workSPACELIMO` uses the same ±120° config. File: `limo_car/gazebo/sensor.xacro`.

### 1.23 SLAM Map Contaminated with 55 Fake Obstacle Pixels
* **Symptom:** Global and local costmaps showed pink circular blobs scattered across the free space. Every one of them was artificial.
* **Root Cause:** The SLAM map (`thesis_map.pgm`) was built while the 360° self-scan (issue 1.22) was active. Each chassis self-hit was baked as an occupied cell into the PGM file. When Nav2 loaded this map into the static layer, all 55 fake walls were treated as real obstacles.
* **Fix:** Fixed issue 1.22 first, then re-ran a full SLAM session using `slam_wanderer.py` (autonomous reactive wanderer). New map: 238×358 cells, 0 isolated noise pixels, 97.2% free space, 100% coverage. Files: `limo_car/maps/thesis_map.pgm`, `thesis_map.yaml`.

### 1.24 Wheel Separation Incorrect — Odometry Drift
* **Symptom:** Robot odometry drifted in circles during straight-line navigation; turn radius was slightly off.
* **Root Cause:** `limo_car/gazebo/ackermann.xacro` had `wheel_separation: 0.175m`. The real LIMO PRO tread (front axle center-to-center) is 172mm. The reference repo `omaroped/workSPACELIMO` also uses 0.172m.
* **Fix:** Changed `wheel_separation` from `0.175` to `0.172` in `limo_car/gazebo/ackermann.xacro`.

### 1.25 Robot Flipped Forward During Navigation
* **Symptom:** Immediately after starting to drive, the rear of the robot lifted off the ground, front dipped, rear wheels spun fast, front wheels turned slowly. Robot nearly tipped over.
* **Root Cause:** A Gazebo physics block copied from the reference repo included `<contact_max_correcting_vel>0.1</contact_max_correcting_vel>`. The Gazebo Classic default is 100 m/s — this setting is 1000× lower. Our robot has a high center-of-mass (cobot arm on top). At 0.1 m/s correcting velocity, the contact solver became unstable for a heavy elevated chassis, causing the forward-flip instability.
* **Fix:** Removed the physics block entirely from `limo_car/worlds/final_map.world`. Gazebo defaults are stable for our robot.

### 1.26 Nav2 Navigation — Robot Rotated Then Stopped, Never Drove to Goal
* **Symptom:** After setting a Nav2 goal in RViz, the robot rotated in place to face the goal (normal), then stopped and never moved forward. The path was visible in green but the controller didn't follow it.
* **Root Cause:** Two compounding causes:
  1. `obstacle_min_range: 0.22m` and `raytrace_min_range: 0.22m` in both costmaps. The arm and arm base extend 0.25–0.35m from the robot's center, so they were continuously marking the robot's own surrounding area as lethal obstacles. Nav2 couldn't find a valid path that didn't start inside a lethal cell.
  2. `use_rotate_to_heading: true` in the controller — the robot correctly rotates to face the goal before driving. This looks like "stuck" but is normal diff-drive behavior (expect 4–5 seconds of rotation first).
  3. AMCL `laser_min_range: 0.02` didn't match the sensor (0.10m), causing spurious scan hits to feed into localization.
* **Fix:** Raised `obstacle_min_range` and `raytrace_min_range` from 0.22m → 0.35m in both `local_costmap` and `global_costmap` sections. Set AMCL `laser_min_range: 0.10`. File: `limo_car/config/nav2_limo_diff.yaml`.

### 1.27 Workspace Contamination from Unrelated Desktop/Thesis Project
* **Symptom:** `ros2 run limo_car ...` showed duplicate package warnings. `steering_wheel_joint` errors appeared at launch (a joint from the Ackermann steering project, not diff-drive). MoveIt config from wrong project was loaded.
* **Root Cause:** `~/.bashrc` contained an auto-source line at line 145 that sourced `~/Desktop/Thesis/install/setup.bash`. That is a completely separate ROS 2 project (different robot, different packages). Its overlay was silently injected into every terminal session, polluting the package database and overriding configs.
* **Fix:** Commented out the auto-source line in `~/.bashrc`. Added a named alias `thesis_source` for intentional use when that project is needed explicitly. The `~/Desktop/Thesisorg` project (this project) is never linked to `~/Desktop/Thesis` in any way.

### 1.28 Grasp Weld Broken After Robot Navigation — Box Floated Away
* **Symptom:** After the robot navigated to the table and the grasp was triggered, the box appeared to teleport to a wrong location in the world instead of staying in the gripper.
* **Root Cause:** `grasp_attacher.py::capture_base()` called `/get_entity_state` on the robot model **once at startup** and stored the result as `_B_pos`. The gripper world position was then always computed as: `gripper_pos = _B_pos + R_startup × TF(base_footprint → gripper_tcp)`. After the robot drove ~3m to the table, `_B_pos` was completely stale (startup position). The computed gripper position was wrong, so the box was placed at an incorrect world location.
* **Fix:** Replaced the startup-captured `_B_pos` with a live TF lookup: `tf_buffer.lookup_transform('odom', 'gripper_tcp', rclpy.time.Time())`. With `odometry_source=1` (WORLD mode), the `odom` frame IS the Gazebo world frame, so this lookup always gives the correct gripper world position regardless of where the robot has driven. File: `limo_car/scripts/grasp_attacher.py`.

### 1.29 AMCL Localization Unstable During Rotation (Downstream of 1.22 + 1.23)
* **Symptom:** When the robot rotated, the `map→odom` TF from AMCL jumped erratically. The robot appeared to teleport on the map.
* **Root Cause:** Compounding effect of two upstream issues: (a) the live scan contained self-hit points (issue 1.22), adding 20–30 spurious readings per scan; (b) the static map layer had 55 fake obstacle cells (issue 1.23). AMCL scan-match was comparing a corrupted scan against a corrupted map — the particle filter couldn't converge.
* **Fix:** No separate AMCL tuning needed. Resolved entirely by fixing issues 1.22 and 1.23. Clean scan + clean map → stable localization.

---

## 3. Large Reference Logs

For diagnostic history, refer to:
* **[Orchestrating LIMO Mobile Manipulation.md](file:///home/omar/Desktop/Thesisorg/docs/references/docs/Orchestrating%20LIMO%20Mobile%20Manipulation.md):** Logs mesh scaling corrections (1000x scaling error) and spawner dependencies.
* **[Fixing LIMO Autonomous Navigation Failures.md](file:///home/omar/Desktop/Thesisorg/docs/references/docs/Fixing%20LIMO%20Autonomous%20Navigation%20Failures.md):** Logs TEB/hybrid planner setup and passive mimic controller overrides.
