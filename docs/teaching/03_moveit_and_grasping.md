# 03 — MoveIt 2 and Grasping: how the arm plans, and how it picks

This is the heart of the manipulation side. Two halves:
- **A. Motion planning** — how MoveIt turns "put the gripper *there*" into a safe joint motion.
- **B. Grasping** — how we actually pick the box up, including the simulation-physics fight.

---

## A. MoveIt 2 — the motion planner

**MoveIt** is the standard ROS framework for arm motion planning. Its one job: given the arm's
current pose and a target for the gripper, find a **trajectory** (a smooth sequence of joint
angles over time) that reaches the target **without colliding** with the robot itself or the
world. The running planner is the node **`move_group`**.

> **Why we talk to MoveIt over an *action*, not the Python API.** The Python bindings
> (`moveit_py`) are **not packaged for ROS 2 Humble**. So our brain talks to `move_group`
> through its standard **`/move_action`** action interface using plain `rclpy`. Pure Python,
> no extra dependencies. (See `pick_orchestrator.py`.)

### A.1 The four ingredients MoveIt needs

1. **The robot model (URDF)** — the shapes, joints, and limits (where the parts are, how they
   move). Comes from the Xacro/URDF.
2. **The semantic model (SRDF)** — *meaning* on top of the URDF:
   `config/limo_cobot.srdf`. It defines:
   - **Planning groups**: which joints form "the arm." Ours is the chain
     `joint1 → … → gripper_tcp` named `arm`. MoveIt only plans this group.
   - **Named poses**: e.g. `ready`, `home` (joint presets).
   - **The collision matrix** (`<disable_collisions>`): which pairs of links are *allowed* to
     be near each other (because they're adjacent, or never actually touch). **This file caused
     the biggest bug in the project — see §A.5.**
3. **The IK solver** (`config/kinematics.yaml`) — the **K**DL plugin that answers "what joint
   angles put the gripper at this pose?"
4. **The planner** (OMPL) — `RRTConnect`, which *searches* for a collision-free path.

### A.2 Inverse Kinematics (IK) — "where do the joints go?"

**Forward kinematics**: given joint angles, compute where the gripper ends up (easy, one
answer). **Inverse kinematics**: given a desired gripper pose, compute the joint angles
(hard, may have many solutions or none). MoveIt uses an IK solver (KDL) to turn a Cartesian
target into joint angles. You can call it directly — invaluable for debugging:
```bash
# "Can the arm put gripper_tcp at this pose?"  error_code 1 = yes, -31 = no solution
ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK "{...}"
```

### A.3 OMPL & RRTConnect — "find a *path*, not just an endpoint"

IK gives an *endpoint* (goal joint angles). The planner must still find a **path** from the
current pose to that endpoint that doesn't pass through a collision on the way. `RRTConnect`
grows two random trees (one from start, one from goal) until they meet — fast and reliable for
arms. If start and goal are both valid but *every path between them* hits a collision, it
reports **"unable to solve."** (That exact message, with a valid goal, is what pointed us at a
*path* collision — see §A.5.)

### A.4 The planning scene & collision checking

MoveIt keeps a **planning scene**: the robot plus any obstacles we tell it about (e.g. we add
the table as a collision box so it won't drive the arm through it). Before executing *any*
plan, MoveIt checks **every** step against this scene and **refuses** anything in collision.
This is a feature — it's what prevents the arm smashing into the table — but it means a *wrong*
collision setup makes it refuse *valid* motions. Two services let you interrogate it:
```bash
ros2 service call /check_state_validity moveit_msgs/srv/GetStateValidity "{...}"  # is this pose in collision? with what?
```

### A.5 ⭐ The bug that froze the arm for ~10 runs (the most important lesson)

**Symptom:** the arm executed *joint*-space goals (like `ready`) fine, but **every** Cartesian
("go to this pose") goal failed with a generic failure. For many runs we wrongly blamed reach,
then IK, then the gripper.

**The diagnosis path (this is the model to copy):**
1. `/compute_ik` → **succeeded**. So IK works; the goal is reachable. *Not reach, not IK.*
2. `/check_state_validity` on the start and goal → **both valid**, collision-free.
3. Tried joint goals directly: every goal with **`joint1` (the waist) = 0** *succeeded*; every
   goal with **waist ≠ 0** *failed*. A crisp, reproducible pattern.
4. Read `move_group`'s own log: *"contact between `gripper_base` and a wheel/steer/laser."*
5. Interpolated the path with `/check_state_validity` and printed the contacts → the gripper
   was being reported as colliding with the **LIMO's own wheels/steers/laser/camera** as the
   waist rotated to face forward.

**The cause:** the SRDF collision matrix disabled the gripper *fingers* against the base parts,
but **never disabled `gripper_base` / `joint6` / `joint6_flange`** against them. So the moment
the arm turned to face forward, MoveIt *hallucinated* a collision with the robot's own body and
**refused every plan**. The arm could only reach poses with the waist at 0 (folded straight
up = the `ready` pose) — which is *exactly* what we saw.

**The fix:** add the missing `<disable_collisions>` lines for the wrist/gripper links against
the base appendages. One configuration file; ~25 lines.

**The lessons (worth more than the fix):**
- A symptom ("arm won't move") can be many layers away from its cause ("one missing line in a
  config file").
- *Prove* each layer with the right tool (`/compute_ik`, `/check_state_validity`, the planner's
  log) instead of guessing.
- An incomplete auto-generated collision matrix silently blocks planning while joint goals
  still work — a trap worth remembering for any MoveIt project.

---

## B. Grasping — the surprisingly hard part

Once MoveIt can move the arm, you'd think gripping is trivial. It is not — and *most* of the
difficulty is **simulation physics**, not robotics.

### B.1 Reachability is position *and orientation* (the second big lesson)

We "knew" the box at 0.20 m was unreachable — because we tested **one** gripper orientation
(`quaternion (1,0,0,0)`). When we asked the IK solver across *many* orientations, the truth
appeared: the arm reaches the box at 0.20 m **straight down with a different wrist roll**
`(0.707, 0.707, 0, 0)`. The first roll over-twists the wrist when the arm is extended; the
second doesn't. **The "wall" was an artefact of fixing one orientation parameter to a bad
value.** Always reason about reach with the *full* orientation, including roll.

### B.2 Know your gripper's geometry (measure it, don't assume)

Using `tf2_echo` to measure the finger links, we found this gripper's **fingers point along its
own +y axis and are only ~2.7 cm long.** That single fact explained a confusing failure: our
"top-down" orientation pointed the *gripper's z* down, but the *fingers* (its +y) stuck out
**sideways**, hovering *over* the box instead of around it. The correct top-down grasp points
the gripper's **+y straight down** (`quaternion (−0.707, 0, 0, 0.707)`). *Measure the tool
frame before you command it.*

### B.3 ⭐ The "robot flies into the sky" — Gazebo contact instability

**What you see:** the open gripper descends, touches the table edge, and the *whole robot*
launches several metres into the air.

**Why:** Gazebo models bodies as rigid. When a *position*-controlled finger is told to move
*into* a solid table, the solver sees two solids overlapping and applies an enormous repulsive
force to separate them. Because the arm is bolted to a **light, un-anchored base**, that force
doesn't just jitter the gripper — it throws the entire robot. This is a famous, well-known
Gazebo pain point. **It is a simulation artefact, not a robotics failure.**

Two fixes, both used:
- **Don't drive into geometry.** Stop the descent so the fingers straddle the *upper* part of
  the box, clear of the table; and/or add the table to the planning scene so MoveIt refuses to
  enter it.
- **Anchor the base.** `base_pin.py` reads the robot's settled world pose once and re-asserts it
  at 50 Hz via `/set_entity_state`, so the base cannot creep (the "box drifts while static"
  effect) or be launched. *A real robot is parked/braked while grasping, so this is legitimate.*

### B.4 ⭐ Grasping a rigid object: the "weld" (attach-on-grasp)

Because real friction grasping is so unstable in Gazebo, the **standard, accepted** technique
in essentially every ROS/Gazebo pick-and-place demo is **attach-on-grasp**: when the gripper is
genuinely around the object, you *rigidly bond* the object to the gripper so it rides with the
arm, and release it on open. You stop relying on simulated squeezing. This is *not* cheating —
it is the field's answer to a known simulation limitation — **provided you only attach at the
real grasp moment** (gripper actually around the object), which keeps it honest.

Our implementation (`grasp_attacher.py`) had to work around a real plugin limitation we
discovered by testing: Gazebo's state plugin can only address whole **models** in the **world**
frame — it cannot reference a **link**. So the weld works in world coordinates:
1. The base is pinned, so its world pose `B` is known and constant.
2. **TF** gives `base_footprint → gripper_tcp`, so the gripper's world position is `B ∘ TF`.
3. On attach, we record the box's offset from the gripper; then at 25–50 Hz we set the box's
   world pose to follow the gripper. Release on open.

(The maths of that composition is in `06_math_models.md`.)

### B.5 The grasp sequence (the state machine)

The brain (`arm_grasp_test.py` / `pick_orchestrator.py`) runs a fixed sequence, each step a
MoveIt goal or a gripper/weld command:

```
ready  →  open gripper  →  add table to planning scene
      →  pre-grasp (hover above the box, fingers down)
      →  grasp     (descend so fingers straddle the box)
      →  WELD on   (bond box to gripper)  →  close gripper (cosmetic)
      →  lift      (box rises with the arm)
      →  place     (move to drop-off, lower)
      →  open gripper  →  WELD off (release)
      →  ready
```

Each Cartesian step is sent as a `/move_action` goal; MoveIt plans + executes it; then the next
step runs. Gripper open/close are *ramped* over ~0.8 s (a one-shot position jump imparts a big
impulse that can kick the box or destabilise the sim).

**Next:** `04_navigation.md` — how the *base* drives to the box.
