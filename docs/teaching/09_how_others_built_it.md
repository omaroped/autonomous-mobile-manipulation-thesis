# 09 — How Others Built This Exact Robot (and what we learn)

We are not the first to put a **myCobot 280** on an **AgileX LIMO**. It is in fact an *official
product* (the **AgileX LIMO COBOT**), and there are two well-documented public lineages for
building the software. This chapter summarises *how they did it*, with real package and method
names, and — most importantly — **what we should take from each.** Scope: LIMO + myCobot 280 and
close variants (LIMO PRO, myCobot 280 M5/Pi, 320). Focus: integration/URDF/ros2_control,
MoveIt & grasping, and perception. (Sources at the bottom.)

> **The one-sentence takeaway:** the *manufacturer's own* integration is **simpler** than the
> MTC + point-cloud approach the sibling project (`~/Desktop/Thesis`) attempted and got stuck on —
> it uses **fiducial markers + the direct arm API + a small state machine.** That strongly
> validates our "simple beats stuck" choice, and points to a concrete upgrade for our perception.

---

## 9.1 The landscape at a glance

| Source | Stack | Perception | Arm control | Navigation | Relationship to us |
|---|---|---|---|---|---|
| **AgileX LIMO COBOT** (the product) | ROS 1 Noetic / ROS 2 Foxy, Jetson Orin Nano | — | API or MoveIt | gmapping+Nav | Our hardware is a real, supported combo. |
| **Elephant Robotics official** (Hackster A/B) | ROS 1 | **STag markers + `solvePnP`** | **`pymycobot` direct API** | gmapping, move_base, DWA/TEB, pure_pursuit | The *simple* path — closest in spirit to ours. |
| **automaticaddison `mycobot_ros2`** | ROS 2 Humble/Jazzy + Gazebo | **PCL** (RANSAC plane + Hough shapes) → CollisionObjects | **MoveIt 2 + MTC** | (arm-focused) | The *deep* path — **this is the sibling project's lineage.** |
| **Ours (`Thesisorg`)** | ROS 2 Humble + Gazebo Classic | **HSV colour + depth** | **MoveIt 2 via `/move_action`** | reactive visual-servo (Nav2 later) | A deliberate middle: simpler than MTC, real planning. |

---

## 9.2 It is an official product: the AgileX LIMO COBOT

AgileX sells the exact combination as the **LIMO COBOT**: a **LIMO PRO** base + **myCobot 280 M5**
arm + **NVIDIA Jetson Orin Nano**, marketed to universities for *both* navigation and manipulation.
Documented support is **ROS 1 Noetic and ROS 2 Foxy**, with tutorials for mapping, navigation,
localization, and MoveIt motion planning. **What this means for us:** the platform, kinematics,
and use-case are validated; their docs (`limo_ros2`, `limo_pro_doc`) are a reliable reference for
the *base*, and we are free to modernise the software to ROS 2 Humble (which we did).

## 9.3 Lineage A — the manufacturer's own integration (the *simple* path)

From Elephant Robotics' official build series ("LIMO & myCobot — A New Era of Robotic Integration"
A & B, and "Exploring LIMOCOBOT…"). Three modules: **LIMO PRO**, **machine vision**, **the arm**.

- **Hardware:** myCobot 280 M5 + **Adaptive Gripper** + **Camera Flange 2.0** (a 2-D camera at the
  wrist — *eye-in-hand*).
- **Perception = fiducial markers, not colour or deep learning.** They place **STag markers**
  on/near objects and recover full **6-DOF pose** with OpenCV's `cv2.solvePnP` (returns a rotation
  vector `rvec` + translation `tvec`). Libraries: `ManfredStoiber/stag-python`, `bbenligiray/stag`.
- **Arm control = the direct API (`pymycobot`),** *not* MoveIt: `send_angles(angles, speed)`,
  `send_coords(coords, speed, mode)` (Cartesian end-effector pose), `set_gripper_value(v, speed)`.
  MoveIt is offered as the optional "more flexible" alternative.
- **Navigation (ROS 1):** `gmapping` SLAM → `move_base` + `amcl`, with **DWA** (global) and **TEB**
  (local) planners; fixed-route patrol via `agilex_pure_pursuit` (`record_path` then `pure_pursuit`).
- **Integration = a simple state machine:** LIMO patrols a recorded route → **stops when it sees a
  marker** → waits while the arm grasps → moves to the next point. Wired with four nodes (image
  recognition → `/marker_detected`; control; task execution → `/task_status`; navigation).

**What we learn from Lineage A (this is the valuable part):**
1. **Markers fix the two perception problems we fought.** A fiducial gives a *robust, full 6-DOF
   pose* (position **and** orientation) directly — no HSV tuning, no near-clip colour-blob
   fragility, and the orientation it returns is exactly what our grasp needs. For the **real
   robot** especially, a printed STag on the box is the pragmatic, manufacturer-blessed way to get
   a reliable grasp pose. Worth adopting (sim and hardware).
2. **The direct `pymycobot` API is a legitimate alternative to MoveIt on hardware** — simpler,
   deterministic, and what the vendor ships. A good fallback if MoveIt-on-real-arm proves fiddly.
3. **Their orchestration shape — *patrol → detect → grasp → continue* — is exactly the FSM our
   `pick_orchestrator` should grow into** for the full pipeline. Clean template.

## 9.4 Lineage B — automaticaddison `mycobot_ros2` (the *deep* path = the sibling's lineage)

This is the canonical **ROS 2 + MoveIt 2** myCobot codebase, and — confirmed by the file names —
**it is where the sibling project's `get_planning_scene_server.cpp` and `object_segmentation.cpp`
came from.** Packages: `mycobot_description` (URDF), `mycobot_gazebo` (RGBD depth-camera sim),
`mycobot_moveit_config`, `mycobot_moveit_demos`, `mycobot_mtc_demos`,
**`mycobot_mtc_pick_place_demo`**, `mycobot_interfaces`, `mycobot_system_tests`, `docker`; branches
for **Humble** and **Jazzy**.

- **Perception = a full PCL pipeline** behind a custom service
  `mycobot_interfaces/srv/GetPlanningScene`: plane segmentation (RANSAC + surface normals) →
  normals/curvature/**RSD** estimation → region-growing **cluster extraction** →
  `object_segmentation` (projects each cluster onto the plane, votes in a **Hough** space for
  **boxes/cylinders**, RANSAC-fits, returns pose + dimensions). The server
  (`get_planning_scene_server.cpp`) turns detections into `moveit_msgs::CollisionObject`s
  (`box_0`, `cylinder_0`, plus the support surface) and returns a `PlanningSceneWorld`.
- **Planning = MoveIt Task Constructor** (`mtc_node.cpp`): open gripper → move to (vision-derived)
  pre-grasp → approach → close → lift → move to place → lower → open → retreat. Grasp poses come
  from the **live perception**, not a grasp database.
- **Topics/config:** camera on `/camera_head/color/image_raw` and
  `/camera_head/depth/color/points`; tuned via `get_planning_scene_server.yaml` (RANSAC/RSD/
  curvature/cluster thresholds) and `mtc_node_params.yaml` (timeouts, scaling, approach distances).

**What we learn from Lineage B:**
1. It is the **reference implementation** to consult whenever we extend MoveIt — the
   *service-returns-a-PlanningSceneWorld* pattern is a clean way to feed perception into planning.
2. **But its complexity is precisely what stalled the sibling project.** PCL + Hough + RSD + MTC is
   a lot of moving parts to debug. Treat this repo as a **lookup reference, not a thing to adopt
   wholesale** — consistent with chapter 08's "simple beats stuck."
3. Their **`mycobot_gazebo` ros2_control + RGBD setup** is worth reading for our integration focus:
   it is the same `gazebo_ros2_control` + depth-camera-plugin pattern we use, done canonically.

## 9.5 The piece we will need for the real robot: hand-eye calibration

Our simulation *knows* the camera-to-arm transform exactly (we declare it in the URDF). The **real**
eye-in-hand camera does not — its true pose on the flange must be **measured**. This is **hand-eye
calibration**, and it is non-optional for phase two:
- **Eye-in-hand workflow:** offline, solve for the fixed **camera → end-effector** transform;
  at runtime, compose it with the robot's forward kinematics (**end-effector → base**) to put every
  camera detection into `base_link` — exactly the frame our grasp planner expects.
- **Ready ROS 2 tooling:** `shengyangzhuang/handeye_calibration_ros2` (camera-agnostic, has a sim
  demo), `lixiny/Handeye-Calibration-ROS`; Elephant ships a `camera_calibration.py` for the intrinsics.
- **Why it matters:** a few millimetres / degrees of uncalibrated camera error is the difference
  between a grasp and a miss on the real arm. Budget a calibration step before the first real pick.

---

## 9.6 What this means for OUR project — concrete recommendations

1. **Adopt fiducial markers (STag/ArUco) for the target's 6-DOF pose** — the single highest-value
   idea here. It directly removes the orientation ambiguity and near-clip colour fragility from
   chapters 02–03, and it is the manufacturer's own method. Do it at least for the real robot;
   consider it in sim too (it also makes the sim→real transfer cleaner).
2. **Grow `pick_orchestrator` into the *patrol → detect → grasp → continue* FSM** that both lineages
   converge on.
3. **Keep `pymycobot` direct control in our back pocket** as a simpler real-arm path if MoveIt
   execution on hardware is troublesome.
4. **Plan a hand-eye calibration step** for phase two; pull in a ready ROS 2 package rather than
   rolling our own.
5. **Use automaticaddison's repo as a reference, not a dependency** — read its `mycobot_gazebo`
   (ros2_control/RGBD) and the GetPlanningScene service pattern, but resist re-importing the full
   PCL/MTC stack that stalled the sibling.
6. **Port the official nav recipe to Nav2 terms** when we add navigation: their DWA/TEB/AMCL/patrol
   maps onto Nav2's controller/planner/AMCL/waypoint-follower.

---

## 9.7 Sources

- AgileX LIMO COBOT product & docs — [global.agilex.ai](https://global.agilex.ai/products/limo-corot) · [datasheet PDF](https://static.generation-robots.com/media/limo-cobot-informations.pdf) · [limo_ros2](https://github.com/agilexrobotics/limo_ros2) · [limo_pro_doc](https://github.com/agilexrobotics/limo_pro_doc)
- Elephant Robotics official integration — [Integration (A)](https://www.hackster.io/Elephant-Robotics-Official/limo-mycobot-a-new-era-of-robotic-integration-a-c5aea8) · [Integration (B)](https://www.hackster.io/Elephant-Robotics-Official/enhancing-tasks-with-limo-mycobot-integration-b-b5e26b) · [Exploring LIMOCOBOT](https://www.hackster.io/Elephant-Robotics-Official/exploring-limocobot-in-enhanced-realistic-settings-788d52) · [LIMO Cobot page](https://www.elephantrobotics.com/en/limo-cobot-en/)
- myCobot ROS 2 (official) — [elephantrobotics/mycobot_ros2](https://github.com/elephantrobotics/mycobot_ros2)
- myCobot ROS 2 + MoveIt 2 + MTC + perception (the deep reference) — [automaticaddison/mycobot_ros2](https://github.com/automaticaddison/mycobot_ros2) · [Pick & place with perception tutorial](https://automaticaddison.com/pick-and-place-task-using-moveit-2-and-perception-ros2-jazzy/)
- Fiducial markers — [stag-python](https://github.com/ManfredStoiber/stag-python) · [bbenligiray/stag](https://github.com/bbenligiray/stag)
- Hand-eye calibration (ROS 2) — [shengyangzhuang/handeye_calibration_ros2](https://github.com/shengyangzhuang/handeye_calibration_ros2) · [lixiny/Handeye-Calibration-ROS](https://github.com/lixiny/Handeye-Calibration-ROS)

---

## 9.8 Deep-research prompt (paste into a specialised search AI for a second, wider pass)

```text
You are a robotics research assistant. I am building an autonomous MOBILE MANIPULATOR:
an AgileX LIMO (Ackermann mobile base) with an Elephant Robotics myCobot 280 6-DOF arm and an
adaptive gripper, in ROS 2 Humble + Gazebo, with the goal of deploying to the REAL hardware
(sim-to-real). I want to learn EXACTLY how other people have built the same/similar system.

Scope: AgileX LIMO / LIMO PRO + myCobot 280 (M5/Pi) or 320; the AgileX "LIMO COBOT" product;
also any myCobot-on-a-mobile-base build. Prioritise, in this order:
  1) INTEGRATION & URDF / ros2_control: how the arm is mounted and described, the xacro/URDF,
     the gazebo_ros2_control setup, controllers.yaml, joint/mimic handling for the gripper.
  2) MoveIt & GRASPING: SRDF/planning groups, IK solver choice, pick-and-place pipelines
     (MoveIt Task Constructor vs direct move_group), grasp pose generation.
  3) PERCEPTION: camera choice, and the detection method — fiducial markers (STag/ArUco +
     solvePnP), classical PCL segmentation (RANSAC/Hough), or learned detectors — and how the
     detected pose is transformed into the arm's base frame (incl. hand-eye calibration).

For EACH source you find, report: (a) the exact GitHub repo / URL, (b) ROS version & whether it's
sim or real hardware, (c) the concrete packages/nodes/topics/launch files and key config
parameters, (d) the perception and the grasp method specifically, (e) what is reusable and what
the known pitfalls are. Strongly prefer primary sources with actual code/config over blog summaries.
Include: official AgileX & Elephant Robotics docs/tutorials, automaticaddison's mycobot_ros2,
university theses/papers, ROS Discourse/Answers threads, and YouTube build series (with the repo
they reference). Finish with a ranked shortlist of the 5 most useful resources to copy ideas from,
and a list of concrete techniques I should adopt for a sim-first-then-real myCobot+LIMO pick-and-place.
```
