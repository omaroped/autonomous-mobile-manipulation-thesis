# Master Plan — Rebuild the Thesis Simulation & Code From Scratch (Clean-Room)

**Audience:** a fresh AI agent (and the student) with **no prior context** on the previous
attempt. This document is the complete specification: architecture decisions, hard-won
lessons to obey, a phased build order, and measurable acceptance tests.

**Golden rule:** build the *foundation* clean, but treat the "Hard-Won Lessons" (§3) as
**mandatory constraints** — they are solutions already paid for in pain. Do not rediscover them.

---

## ⛔ CLEAN-ROOM RULE — NON-NEGOTIABLE, READ FIRST

You must build **everything yourself, from zero, in a brand-new workspace** (e.g.
`~/thesis_v2_ws`). You are **strictly forbidden** from reading, opening, listing, `cat`/`grep`-ing,
copying, importing, or "migrating" **any** code from the previous attempt or **any existing
project folder on this machine**, including but not limited to:
`~/Desktop/Thesisorg/src/ros2_ws`, `limo_ros2`, `limo_car`, `mycobot_ros2`,
`limo_cobot_moveit_config`, the sibling `~/Desktop/Thesis`, and any `_archive*`/backup folders.

Do **not** look at them "just for reference." Pulling from the old implementation is **cheating**
and defeats the entire purpose of this rebuild — it is exactly the shortcut that produced the
mess this plan replaces. If you catch yourself about to open an existing project folder, **STOP**.

**The ONLY permitted inputs are:**
1. **This document.** The §3 lessons are *written knowledge* (text, not code) — that is the
   legitimate, intended knowledge transfer.
2. **Fresh clones of official PUBLIC upstream packages** from their *original* source (e.g. the
   manufacturer's robot-description repos, official ROS 2 / MoveIt / Nav2 packages), pulled new
   into your own workspace — **never** copied from the user's modified folders.

Everything else — URDF, launch files, perception, MoveIt config, orchestration, tests — you
**author yourself** from the spec in this document.

---

## 0. Project context

- **Goal:** an autonomous **mobile-manipulation** robot that **navigates → docks at a table →
  picks a box → stacks boxes vertically**, evaluated as a **Sim-to-Real reliability study**
  (stack success / drop rate, placement error, failure recovery).
- **Hardware (real robot):** AgileX **LIMO PRO** base (Mecanum mode), Elephant Robotics
  **myCobot 280 M5** 6-DOF arm, adaptive parallel gripper, **Orbbec DaBai** depth camera,
  **EAI T-mini Pro** LiDAR, Jetson Orin Nano onboard.
- **LIMO PRO dimensions (from the manual — use these, do NOT guess):** wheel base **0.20 m**,
  tread **0.175 m**, wheel radius **0.045 m**.
- **Software baseline:** **ROS 2 Humble**, **MoveIt 2**, **Nav2**, `ros2_control`.
- **Dev machine:** HP Omen laptop, **dual-GPU (NVIDIA RTX 3060 6 GB + AMD iGPU, PRIME
  on-demand)**, Ubuntu 22.04. This machine's quirks are documented in §3 — they are real.

---

## 1. DECISION GATES — resolve BEFORE any code (with the student/professor)

The previous attempt failed mainly by *not* deciding these first. Do not start Phase 1 until
these are answered in writing.

1. **Simulator.** Recommendation: **Gazebo Sim (Fortress)** with `ros_gz` + `gz_ros2_control`.
   Rationale: it has a **native MecanumDrive** system, it is the supported successor to Gazebo
   Classic (which is EOL Jan 2025 and has *no* usable mecanum), and MoveIt/Nav2 carry over.
   Fallback: Gazebo Classic + **differential drive** (real physics, but not mecanum).
2. **Drive mode.** Mecanum (professor's stated preference) — but ONLY meaningful with a
   simulator that models it (gate #1). If staying in Classic, **differential drive** is the
   honest choice (real wheel physics; mecanum in Classic is a kinematic fake — see §3.L1).
3. **Sim2Real scope.** Is the contribution the **manipulation/stacking** reliability (base is a
   transport abstraction) or also **locomotion/odometry fidelity**? This decides how much base
   realism you need. Write it down; the thesis narrative depends on it.
4. **Definition of "done"** for the sim demo: e.g. "stack 3 boxes, ≥X% success over N trials,
   with logged failures." Quantify it now.

---

## 2. Target architecture (once gates are decided)

```
ros2_ws/
  src/
    limo_description/      # URDF/xacro: base + arm + gripper + sensors (sim-agnostic core)
    limo_gazebo/           # sim: world, spawn, drive plugin, sensor bridges, launch
    limo_moveit_config/    # MoveIt 2 SRDF/kinematics/controllers (sim-agnostic)
    limo_nav/              # Nav2 params, maps, localization
    limo_perception/       # box detection (depth → /box_pose)
    limo_pipeline/         # orchestrator: nav → dock → pick → stack; metrics; recovery
    limo_bringup/          # top-level launches that compose the above
```
Keep **sim-specific** plugins isolated in `limo_gazebo` so the simulator can be swapped without
touching MoveIt/Nav2/perception. (Previous attempt mixed drive/sensor plugins into the base
xacro — that coupling caused most of the churn.)

---

## 3. HARD-WON LESSONS — mandatory constraints (the gold; obey all)

**Build / environment**
- **L-BUILD1 (pyenv breaks CMake):** the machine's `python3` is a **pyenv shim** that makes
  `colcon`/CMake fail with *"Could NOT find Python3 (missing: Interpreter)"* on a clean
  configure. Always build with `--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`, or
  `pyenv shell system` first. Put this in a `build.sh`.
- **L-BUILD2:** any node script's shebang must be `#!/usr/bin/python3` (not `env python3`) or it
  picks up the pyenv Python without `rclpy`.

**GPU / simulator stability (this laptop)**
- **L-GPU1 (dual-GPU):** render the sim on the **NVIDIA dGPU** via PRIME offload
  (`__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia`). On the AMD iGPU the GPU
  camera sensor **segfaults the server** on spawn. Bake the env into the launch.
- **L-GPU2 (don't wedge it):** repeated `kill -9` of the sim server **leaks GPU contexts and
  wedges the driver**, after which it segfaults on *every* launch until a **reboot**. Shut the
  sim down cleanly; if it starts crashing deterministically, reboot.
- **L-GPU3 (camera is the load):** the depth camera is the heaviest GPU consumer and the usual
  crash source. Provide a **`use_camera:=false`** launch toggle so driving/nav demos run without
  it; only enable the camera for perception/grasp work. Two heavy GUIs (sim GUI + RViz) on 6 GB
  also contend — run RViz on NVIDIA too, or headless.

**Robot model**
- **L-MODEL1 (collision mesh):** the manufacturer base mesh is huge (~7.5k faces / ~50 MB) and
  makes MoveIt/physics choke. **Decimate** to a lightweight collision mesh (~3k faces).
- **L-MODEL2 (gripper scale):** the gripper COLLADA uses a `<unit>` tag MoveIt ignores → loads
  **1000× too large**. Set explicit `scale="0.001"` in the URDF.
- **L-MODEL3 (mimic joints):** Gazebo's mimic-joint support is unreliable — drive **all** gripper
  finger joints explicitly with mirrored command signs, not via `<mimic>`.
- **L-MODEL4 (arm mount):** if the arm base is rotated 90° about Z for layout, joint 1 needs a
  −π/2 offset to face forward. Decide arm placement once and document it.
- **L-MODEL5 (LiDAR self-hit):** shrink the `laser_link` collision to ~0.001 m or the scan rays
  hit the robot's own body.

**MoveIt**
- **L-MOVEIT1 (collision matrix):** the SRDF `disable_collisions` matrix MUST be complete, or
  MoveIt reports false self-collisions (gripper vs wheels/laser) and refuses to plan. Generate it
  carefully and verify.
- **L-MOVEIT2:** prefer plain `move_group` top-down grasp planning over MoveIt Task Constructor
  (MTC was over-engineered and brittle for this task).

**Perception / grasp / docking**
- **L-PERCEP1 (camera blind spot):** the DaBai depth camera is unreliable closer than ~0.30 m.
  Do NOT rely on live camera feedback at close range. Instead: perceive the box at a safe
  distance (~0.6 m), then drive a **measured (odometry) blind approach** to a fixed stop
  distance (~0.24 m). This prevents the robot climbing the table.
- **L-GRASP1 (sim weld):** Gazebo's parallel-grip contact is slippery — boxes fly out. Use a
  **grasp-weld** (fixed joint between gripper and box on contact) as a sim aid; keep it
  toggleable to also test pure friction.
- **L-DOCK1:** with a holonomic/diff base, dock by aligning laterally (strafe or rotate) then
  driving straight in — keeps the short-reach arm able to reach the box.

**Drive fidelity (the big one)**
- **L-DRIVE1:** **Mecanum has no faithful model in Gazebo Classic.** A friction-emulation
  (driven wheels + anisotropic `fdir1` friction) was measured to translate & strafe but **could
  not rotate** in any config. The kinematic `planar_move` plugin works for all 3 DOF but ignores
  wheel physics & vertical gravity (fake). **Conclusion: if mecanum fidelity matters, use a
  simulator that models it (Gazebo Sim / Webots / Isaac), not Classic.**
- **L-DRIVE2 (testing):** measure motion against the **true simulator model pose**, NOT the
  controller-computed `/odom` (odom can report motion that didn't physically happen).

---

## 4. Phased build order — each phase ends with a MEASURED acceptance test

> Build incrementally. Do **not** proceed to the next phase until the current one passes its
> test. Commit after each green phase.

**Phase 0 — Workspace & build hygiene**
- Create the package skeleton (§2), a `build.sh` (with L-BUILD1), a `.gitignore` (ignore
  `build/ install/ log/ __pycache__/ *.bak node_modules/`), and a README.
- *Accept:* `./build.sh` succeeds clean from an empty `build/`.

**Phase 1 — Robot model (sim-agnostic)**
- URDF/xacro: base (manual dims, decimated collision L-MODEL1), arm, gripper (L-MODEL2/3),
  sensors (camera, LiDAR L-MODEL5, IMU). No drive/sensor *plugins* yet — pure structure.
- *Accept:* loads in RViz from `/robot_description`; TF tree complete; no link without a frame.

**Phase 2 — Simulation bring-up + drive**
- Add the chosen sim's drive + sensor bridges in `limo_gazebo` (PRIME offload L-GPU1,
  `use_camera` toggle L-GPU3, spawn at rest height). Wire `/cmd_vel` + `/odom` + TF.
- *Accept (measured, L-DRIVE2 vs true model pose):* forward, strafe (if mecanum), and rotate
  match commanded velocities within ~10%; robot planted at rest (<2 cm drift); `/clock` stable;
  no segfault across 5 launches.

**Phase 3 — Perception**
- Depth → box pose on `/box_pose` (median-filtered). Respect L-PERCEP1 (far-range only).
- *Accept:* published box pose within a few cm of ground truth at ~0.6 m; degrades gracefully <0.3 m.

**Phase 4 — MoveIt grasp (arm on a fixed base first)**
- SRDF + collision matrix (L-MOVEIT1), top-down grasp (L-MOVEIT2), gripper open/close, weld
  (L-GRASP1). Test in isolation before adding the base.
- *Accept:* arm plans + executes a top-down pick of a box at a known pose, ≥8/10 trials.

**Phase 5 — Navigation (Nav2)**
- Map, localization (ground-truth for dev + AMCL for realism), planner/controller matched to the
  drive mode.
- *Accept:* robot drives to a 2D goal and stops within tolerance; no TF/lifecycle errors.

**Phase 6 — Orchestration (nav → dock → pick)**
- State machine: navigate → blind odometry dock (L-PERCEP1) → pin base → grasp → retract.
- *Accept:* full nav-to-pick succeeds ≥7/10 from a fixed start.

**Phase 7 — Stacking**
- Place onto a hardcoded foundation; z grows one box-height per level; loop N boxes.
- *Accept:* stacks 2–3 boxes; logs per-box success.

**Phase 8 — Error recovery + metrics**
- Camera-verify after each place; detect drop/misplacement; retry or flag. Log stack-success /
  drop-rate / placement-error to CSV.
- *Accept:* induced failures are detected and logged; metrics CSV produced over N trials.

**Phase 9 — Sim2Real + write-up**
- Run the same metrics on the real robot; compare; document the gap. Honest **limitations
  section** (esp. base-drive fidelity per L-DRIVE1).
- *Accept:* sim-vs-real comparison table + threats-to-validity written.

---

## 5. Testing methodology (apply throughout)
- Measure against **true model pose** (L-DRIVE2), not computed odom.
- Headless tests for physics/logic (`use_gzclient:=false`); GUI only when watching.
- One change at a time; re-measure; keep a results log. (The previous attempt changed multiple
  variables at once and couldn't attribute effects.)
- Script acceptance tests (small `rclpy` nodes) so they're repeatable, not eyeballed.

## 6. Human-in-the-loop checkpoints (cannot be automated)
- Grasp offset calibration; stack foundation coords; AMCL initial pose; any real-robot step;
  GPU reboots (L-GPU2). The agent must **stop and ask** at these, not guess-loop.

## 7. Scope & honest limitations to state in the thesis
- Whichever drive model: document its fidelity (L-DRIVE1). If kinematic, say so and scope
  Sim2Real to manipulation. If a real-physics drive (diff/Gazebo-Sim mecanum), claim accordingly.
- Sim weld (L-GRASP1) is a sim aid, not real grasping — contrast with real friction.

## 8. Deliverables
- Buildable workspace (§2), per-phase acceptance test scripts + results log, launch files with
  the `use_camera`/headless toggles, metrics CSVs, and a limitations section.

---

### First action for the agent
1. **Acknowledge the CLEAN-ROOM RULE** above: you will NOT read or copy any existing project
   folder on this machine; you build everything yourself in a new workspace.
2. Confirm the §1 decision gates are answered. If not, STOP and ask the student — do not start
   building on an undecided foundation.
3. Then execute Phases 0→9, stopping at each acceptance test and at every §6 checkpoint.
