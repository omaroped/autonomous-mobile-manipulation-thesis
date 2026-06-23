# The LIMO + myCobot Mobile Manipulator — Teaching Guide

> **Who this is for.** A beginner who has never touched ROS 2, Gazebo, MoveIt, or robotics.
> If you read this folder front to back, you will understand *this entire project* —
> not just *what* we built, but *why* each piece exists, *how* the pieces talk to each
> other, the *concepts* behind them, the *maths*, the *tools*, and every *bug we hit and
> how we reasoned our way out*. By the end you should be able to point at any part of the
> system and say "this connects to that, which is why changing X breaks Y."

This guide is deliberately written as a *teaching* document, not a dry reference. Every
concept is explained from zero, with the real file names and commands from this repository
so you can go and look. Where we made a mistake, we keep the mistake **and** the fix,
because the reasoning is the most valuable thing here.

---

## 0. How to read this folder

Read in order the first time; after that, jump around.

| File | What you'll learn |
|------|-------------------|
| **`README.md`** (this file) | The map: what the robot is, the 4-layer mental model, the node/topic data-flow, and how everything connects. |
| **`01_foundations_ros2_gazebo_tf.md`** | The vocabulary: ROS 2 (nodes, topics, services, actions, parameters), Gazebo (`gzserver`/`gzclient`, plugins, worlds, why processes persist), the launch chain, and TF (coordinate frames). |
| **`02_perception.md`** | How a camera image becomes "the box is *here*": HSV colour, depth, the de-projection maths, and the transform into the arm's frame. |
| **`03_moveit_and_grasping.md`** | How the arm decides to move and how it grasps: MoveIt (SRDF, IK, OMPL, the planning scene & collision matrix), the grasp sequence, the simulation-physics "explosion," and the *weld*. |
| **`04_navigation.md`** | How a mobile robot drives to a goal: Nav2 concepts (costmaps, planners, controllers) and the simpler *reactive visual-servo* approach this project uses. |
| **`05_war_stories_and_playbook.md`** | Every real bug, the diagnosis, the fix, and the lesson — plus the terminal command playbook and a glossary. This is the gold. |
| **`06_math_models.md`** | The maths in one place: de-projection, quaternions & orientation, reachability, kinematics, and the weld's coordinate composition. |
| **`07_terminal_and_introspection.md`** | **Become fluent in the live system**: every command to find nodes/topics/services, watch data flow, inject messages, prove reach/collisions, inspect TF, and a 90-second "read the whole robot" lab + cheat sheet. |
| **`08_lessons_from_the_sibling_project.md`** | What we mined from the earlier `~/Desktop/Thesis` attempt: confirmed bugs, a *cleaner* base-drift fix (wheel damping), the gripper-mimic fix, moment-of-inertia validity, and why "simple beats stuck." |
| **`09_how_others_built_it.md`** | **How other people built this exact robot** (LIMO + myCobot 280): the official AgileX LIMO COBOT, Elephant Robotics' marker-based integration, automaticaddison's MoveIt 2 + MTC + PCL repo (the sibling's lineage), hand-eye calibration for the real robot — with concrete takeaways + a deep-research prompt. |
| **`diagrams/`** | Rendered figures (SVG + PNG from Graphviz): the loop, the data-flow, the TF tree, the grasp state machine. |

---

## 1. What is this project, in one breath?

An **autonomous mobile manipulator** that **navigates to a box, picks it up, and places it**.
The real target is **physical hardware** — an AgileX LIMO + myCobot 280 that **already sits in
our lab**. We develop and prove the *entire* pipeline in **simulation first** (ROS 2 Humble +
Gazebo Classic) because that is safe, fast, free, and repeatable; once it works in simulation,
the *same* ROS 2 system is deployed onto the real robot. **Simulation is phase one, not the
destination.**

It is built from three pieces of hardware, each modelled in software so we can develop against
them before risking the real ones:

- an **AgileX LIMO** — a small 4-wheeled mobile base driven *Ackermann*-style (like a car: it
  steers, it cannot spin on the spot);
- an **Elephant Robotics myCobot 280** — a tiny **6-DOF** (six-joint) robot arm bolted on
  top of the LIMO;
- an **adaptive gripper** on the end of the arm.

The job, as a pipeline: **navigate → perceive → grasp → place.**

```
        ┌──────────┐   ┌───────────┐   ┌─────────┐   ┌────────┐
        │ NAVIGATE │ → │  PERCEIVE │ → │  GRASP  │ → │ PLACE  │
        │ drive to │   │ find the  │   │ pick it │   │ put it │
        │ the box  │   │ box in 3D │   │   up    │   │ down   │
        └──────────┘   └───────────┘   └─────────┘   └────────┘
```

### The two phases (why simulation comes first)

This project is a **simulation-to-reality (sim-to-real)** effort, in two phases:

1. **Simulate (where we are now).** Build the LIMO, the arm, the camera, and the world inside
   Gazebo and get the full *navigate → perceive → grasp → place* pipeline working. Mistakes here
   cost nothing: no stripped servos, no crashed arm, and an instant reset (`pkill gzserver`). This
   is the safe place to make — and understand — every mistake in this guide.
2. **Deploy to the real robot (the goal).** The *same* ROS 2 nodes, topics, TF tree, and MoveIt
   configuration run on the physical LIMO + myCobot 280 in the lab. What changes is only the
   **bottom layer**: Gazebo's physics and sensor plugins are swapped for the real hardware drivers
   — a real depth camera, real motors, real `ros2_control`. The perception, planning, and
   orchestration layers are reused unchanged.

The gap between the two is the **reality gap** — friction, sensor noise, latency, calibration,
and lighting that the simulator does not perfectly capture. This is exactly why several "bugs" in
this guide (the contact "explosion," the creeping base, the rolling box) are flagged as
*simulation artefacts*: they vanish on hardware, while new real-world effects appear in their
place. Building in sim first is what lets us meet the real robot already knowing the pipeline is
sound.

---

## 2. The mental model: four jobs, on a loop

*Every* robot like this is really just **four jobs repeating**:

```
   SENSE  ───►  THINK / LOCATE  ───►  PLAN  ───►  ACT  ───┐
 (cameras,        (perception:        (MoveIt /    (motor    │
  encoders)        where is it?)       Nav2)       controllers)│
     ▲                                                          │
     └──────────────── the world changed; sense again ◄─────────┘
```

If you ever feel lost, ask: *which of these four boxes am I looking at, and what is it
waiting for as input?* Almost every bug in this project was one box not getting the input it
expected from the box before it.

In *our* system each job is performed by a specific program (a **ROS 2 node**) or by Gazebo:

| Job                                  | Performed by                               | Where it lives                    |
| ------------------------------------ | ------------------------------------------ | --------------------------------- |
| SENSE — simulate the world + sensors | **Gazebo** (`gzserver`)                    | the simulator                     |
| LOCATE — find the box in 3-D         | **`box_pose_estimator`**                   | a perception node we wrote        |
| THINK — sequence the pick            | **`pick_orchestrator`** / `arm_grasp_test` | the "brain" node we wrote         |
| PLAN — compute a safe arm motion     | **`move_group`**                           | MoveIt 2                          |
| ACT — drive the joints               | **`mycobot_arm_controller`**               | `ros2_control`, inside `gzserver` |
| DRIVE — move the base                | **`box_follower`** (or Nav2 later)         | a node we wrote                   |

---

## 3. The data-flow: who publishes what, who listens

This is the single most important diagram in the project. Arrows are **ROS 2 topics**
(streams of messages) or **actions** (goal → result). Read it top to bottom.

> 📊 **Rendered figures** are in [`diagrams/`](diagrams/): `pipeline.svg` (the loop),
> `system_architecture.svg` (this data-flow, in colour), `tf_tree.svg` (the frame tree), and
> `grasp_state_machine.svg` (the pick sequence). The ASCII version below is the same picture.

```
        ┌──────────────────────── GAZEBO (gzserver) ────────────────────────┐
        │  physics + the simulated robot + the camera + the controllers      │
        └──┬──────────────────┬───────────────────┬──────────────────▲───────┘
 publishes │        publishes │         publishes │       subscribes  │
  camera → │   /joint_states →│          /clock → │  arm trajectory ← │
 /rgb/image_raw               │ (the sim's time)  │  gripper cmds   ← │
 /depth_camera/depth/image_raw│                   │                   │
 /depth_camera/depth/camera_info                  │                   │
        │                     │                   │                   │
        ▼                     │                   │                   │
 ┌───────────────────┐        │                   │                   │
 │ box_pose_estimator │  PERCEPTION               │                   │
 │ pixels+depth → 3D  │        │                   │                   │
 │ → transform (TF)   │        │                   │                   │
 └─────────┬──────────┘        │                   │                   │
           │ publishes /box_pose                   │                   │
           ▼                    │                   │                   │
 ┌────────────────────────────┐ │                   │                   │
 │ pick_orchestrator   (BRAIN) │ subscribes /box_pose                   │
 │ "where is it → what to do"  │ │                   │                   │
 └───┬───────────────────┬─────┘ │                   │                   │
     │ sends a GOAL        │ publishes gripper cmds ──┼───────────────────┘
     │ via /move_action    │      (Float64MultiArray) │
     ▼  (a ROS 2 *action*) └──────────────────────────┘
 ┌───────────────────┐
 │   move_group       │  MoveIt 2  —  subscribes /joint_states  ◄── (knows where the arm IS)
 │   IK + OMPL search │
 │   + planning scene │
 └─────────┬──────────┘
           │ sends a JOINT TRAJECTORY (another action) to…
           ▼
 ┌───────────────────────────┐
 │ mycobot_arm_controller     │  ros2_control, *inside* gzserver → physically moves the joints
 └───────────────────────────┘
```

Two things beginners always miss, made explicit:

1. **`move_group` is not blind to the arm.** It constantly reads `/joint_states` to know
   the arm's *current* position — that is the "start state" for every plan. (It *is* blind to
   the **box** unless perception feeds it `/box_pose`. In the isolated grasp test we
   temporarily replaced perception with a hard-coded box position to debug the arm alone.)
2. **Nothing happens continuously.** The brain sends *one goal*; MoveIt plans *once* from the
   current state; the controller executes; then the next goal is sent. It is a sequence of
   discrete steps, not a live servo.

---

## 4. The golden rules this project taught us (read these first)

These are the hard-won principles. Each is expanded in
`05_war_stories_and_playbook.md`, but internalise them now:

1. **Prove, don't assume.** Most of the lost time in this project was spent chasing a
   *symptom* with a guessed cause. The breakthroughs always came from a *measurement*
   (`ros2 topic echo`, `/compute_ik`, `/check_state_validity`) that turned a guess into a
   fact. When you think "it's probably X," go and *check* X before acting.
2. **A symptom can be far from its cause.** "The arm won't move" was, for ~10 runs, *not* a
   reach problem, an IK problem, or a hardware problem — it was **one missing line in a
   collision-configuration file**. Follow the data to the *real* cause.
3. **Change one variable at a time.** When two things are wrong at once (e.g. the gripper
   pointed the wrong way *and* descended onto the table), fixing one and re-running tells you
   far more than changing five things and hoping.
4. **Reachability is position *and* orientation.** A point the arm can reach with one wrist
   roll can be impossible with another. Never reason about "can it reach (x,y,z)" without the
   orientation.
5. **Simulation ≠ reality.** Several of the hardest problems (the gripper "exploding" the
   robot into the sky, the box rolling off, the un-braked base creeping) are *simulation*
   artefacts, not robotics failures. Knowing which is which saves enormous time. And because the
   **real robot is the goal** (phase two), this distinction matters twice over: sim-only artefacts
   will vanish on hardware, while new real-world effects (sensor noise, latency, calibration,
   friction) will appear in their place.

---

## 5. Where the real code lives (so you can go look)

- Robot + sim + everything: `src/ros2_ws/src/limo_ros2/limo_car/`
  - perception: `scripts/box_pose_estimator.py`
  - brain (full pipeline): `scripts/pick_orchestrator.py`
  - brain (isolated arm test): `scripts/arm_grasp_test.py`
  - drive-to-box: `scripts/box_follower.py`
  - sim helpers: `scripts/base_pin.py`, `scripts/grasp_attacher.py`, `scripts/base_joint_state_pub.py`
  - the robot description: `gazebo/`, `urdf/` (Xacro files)
  - the world: `worlds/final_map.world`
- MoveIt configuration: `src/ros2_ws/src/limo_cobot_moveit_config/`
  - the semantic model + collision matrix: `config/limo_cobot.srdf`
  - the IK solver choice: `config/kinematics.yaml`
- The arm itself (myCobot 280): `src/ros2_ws/src/mycobot_ros2/mycobot_description/urdf/mycobot_280_m5/`

Now open `01_foundations_ros2_gazebo_tf.md`.
