# Project Topics & Notes — Discussion Guide

## 1. The project in one breath
An autonomous **mobile manipulator** — AgileX **LIMO** (Ackermann) base + Elephant Robotics **myCobot 280** (6-DOF) arm + adaptive gripper — that autonomously **navigates to a target box, picks it** with a collision-aware top-down grasp (MoveIt 2), and **places it**, in **ROS 2 Humble + Gazebo Classic** simulation.
Pipeline: **navigate → perceive → grasp → place.**

## 2. Technical topics / building blocks (what the thesis covers)
- **Robot modelling** — combined URDF/Xacro (LIMO base + myCobot arm + adaptive gripper + sensors); `ros2_control` integration.
- **Simulation** — Gazebo Classic warehouse world; depth camera + LiDAR + IMU; physics.
- **Navigation** — reactive **visual-servo** control on an Ackermann base: drive to the target from camera feedback, no global map.
- **Perception** — RGB-D: HSV colour segmentation + depth **deprojection** (camera intrinsics) → 3-D object pose → **TF** into the arm frame.
- **Motion planning** — MoveIt 2: semantic model (SRDF), KDL IK, **OMPL/RRTConnect**, **collision-aware planning scene**, top-down grasp.
- **Manipulation control** — `ros2_control` trajectory controller (arm) + position controller (gripper); gripper driven directly by topic.
- **Orchestration** — a pick state machine (navigate → perceive → add table collision → pre-grasp → grasp → close → lift) talking to `move_group` via the action interface.
- **Evaluation** — metrics: grasp success rate, final-pose accuracy, planning time, path length; reachability vs base/workspace geometry.

## 3. Engineering challenges solved (war stories — strong talking points)
- **End-of-life toolchain incompatibility** — Gazebo Classic's `gazebo_ros2_control` (frozen/EOL) was incompatible with the upgraded `controller_manager`: it passes the robot description a deprecated way the new version rejects → no controllers, limp arm. **Patched the plugin from source** in the workspace.
- **MoveIt 2 brought up from scratch** — solved: missing mobile-base joint states (MoveIt needs a *complete* state), simulation-time sync, and the planner silently defaulting to CHOMP instead of OMPL.
- **Gripper actuation** — finger joints were defined `fixed` (un-actuatable) → controller wouldn't activate. Made them revolute/driven, then **smoothed** the close to avoid impulse instability.
- **Camera TF mismatch** — the depth image was stamped with a frame absent from the TF tree → perception couldn't transform. Fixed the frame naming.
- **Physics instability ("robot flying")** — old open-loop hard-coded poses drove the gripper into the table edge → huge impulse. Replaced by collision-aware planning that refuses such moves.

## 4. Research findings / notes (the insights = thesis contributions)
- **Reachability is the binding constraint (headline).** The myCobot 280's ~0.28 m top-down reach + a camera mounted ~0.10 m forward put the object near/beyond reach at a safe stop distance — a **narrow feasible window** governed by three coupled parameters: **arm mounting, base stop distance, workspace layout.**
- **Navigation→grasp hand-off.** Base-positioning precision *directly* determines graspability — a mobile-manipulation-specific problem absent in fixed-base manipulation.
- **Ackermann limitation.** Can't turn in place → a residual lateral offset on side-placed targets at close range.
- **Collision-aware planning eliminates a whole failure class** (the table-strike explosion).
- **Simple beats complex.** A deliberately lightweight design (classical perception, direct planning) proved more robust and debuggable than a heavy pipeline (an earlier MoveIt-Task-Constructor + PCL attempt never reached a working pick).

## 5. Key design decisions & rationale (the "why")
- **move_group action interface** (not `moveit_py`) — `moveit_py` isn't packaged for Humble; the action interface is pure-Python, no extra deps.
- **HSV + depth perception** (not PCL/RANSAC or YOLO) — robust, transparent, sufficient for a known object; avoids the brittleness that sank the earlier attempt.
- **Reactive visual servoing** (not SLAM/Nav2) — the target is locally visible; no map needed.
- **Gripper by topic, outside MoveIt** — keeps the working gripper path simple; MoveIt plans only the arm.
- **No MoveIt Task Constructor** — overkill for a single top-down pick.

## 6. Current status
- ✅ **Working & verified:** simulation, navigation, perception (3-D box pose), MoveIt arm planning + execution, gripper (open/close, smoothed). The **full pipeline runs end-to-end**.
- 🔧 **In progress:** completing the physical grasp — gated by the reachability geometry (workspace / stop-distance tuning) and final grasp calibration (gripper TCP offset, top-down orientation).

## 7. Open items / next steps
- Resolve reach via the **workspace** (shallower/nearer table, closer safe stop) — keeping the arm centred.
- **Calibrate the grasp** (gripper TCP offset, top-down orientation) so the gripper lands on the box.
- Improve **base centring** (lateral alignment) for the Ackermann at close range.
- Add the **place** phase (carry → drop-off) and return-home.
- *(Optional)* transfer to **physical hardware**.

## 8. Artifacts produced
- **Code** — `limo_car` package (nodes: `box_follower`, `box_pose_estimator`, `pick_orchestrator`, `base_joint_state_pub`, teleop); `limo_cobot_moveit_config` package (MoveIt config + launches); patched `gazebo_ros2_control`.
- **Docs** — `THESIS_EXPOSE.md` (proposal), `PROGRESS_SUMMARY.md` (status), `PROJECT_NOTES.md` (this guide).
