# 01 — Foundations: ROS 2, Gazebo, the Launch Chain, and TF

This chapter gives you the vocabulary. Nothing else in the project makes sense without it.
Take your time; everything later refers back here.

---

## 1.1 ROS 2 — the "operating system" for the robot

ROS 2 (Robot Operating System 2) is **not** an operating system. It is a **framework for many
small programs to talk to each other** over a network, even across machines. The whole design
philosophy is: *lots of small single-purpose programs, loosely connected*, instead of one
giant program. That is why our system is a dozen little nodes, not one big script.

There are exactly **four** ways those programs communicate. Learn these four and you can read
any ROS system.

### Node
A **node** is one running program with a name. `box_pose_estimator`, `move_group`,
`mycobot_arm_controller` are all nodes. See them live with:
```bash
ros2 node list
```

### Topic (a stream — "publish / subscribe")
A **topic** is a named channel carrying a stream of messages of one type. A node **publishes**
to it; any number of nodes **subscribe**. The publisher does not know or care who listens.
- Example: Gazebo publishes the camera image on the topic `/rgb/image_raw`
  (type `sensor_msgs/Image`). `box_pose_estimator` subscribes to it.
- Use it for *continuous data*: sensor readings, robot state, velocity commands.
```bash
ros2 topic list                 # all channels
ros2 topic echo /joint_states   # watch one channel's messages
ros2 topic info /joint_states -v # who publishes / subscribes, and the type
ros2 topic hz /clock            # how fast messages arrive
```

### Service (a question — "request / response")
A **service** is a one-shot call: you send a request, you get one response, then it's done.
Like a function call between programs.
- Example: `/compute_ik` — "give me joint angles to put the gripper at this pose."
- Use it for *quick queries or commands* that return immediately.
```bash
ros2 service list
ros2 service call /get_entity_state gazebo_msgs/srv/GetEntityState "{name: 'mbot'}"
```

### Action (a long job — "goal / feedback / result")
An **action** is for jobs that take *time* and that you might want to watch or cancel: you send
a **goal**, you get **feedback** while it runs, and a **result** at the end.
- Example: `/move_action` — "plan and execute a motion to this pose" (takes seconds).
- This is exactly how the brain commands MoveIt, and how MoveIt commands the arm controller.

### Parameter (a setting)
A **parameter** is a named setting a node reads at startup or runtime — e.g. `use_sim_time`,
or the IK solver's `kinematics_solver_timeout`.
```bash
ros2 param list /move_group
ros2 param get /move_group robot_description_kinematics.arm.kinematics_solver
```

> **Why this matters for us:** when the arm "did nothing," the right question was *which
> interface failed?* — was the **action** (`/move_action`) rejected, was the **topic**
> (`/joint_states`) missing, was the **service** (`/compute_ik`) returning an error? Naming
> the interface is half the diagnosis.

### `use_sim_time` — a subtlety that bit us
Normally ROS uses your computer's wall clock. But in simulation, time is whatever Gazebo says
on the `/clock` topic (it can run faster or slower than real time). **Every** node that needs
to agree on "now" must set the parameter `use_sim_time: true`, or it will compare a sim
timestamp to a wall-clock timestamp, decide the data is "from the future/past," and silently
fail. MoveIt failing to "fetch the current robot state" was exactly this.

---

## 1.2 Gazebo — the simulator (and the `gzserver` / `gzclient` split)

**Gazebo Classic** is the physics simulator. It models gravity, friction, collisions, motors,
and sensors (cameras, LiDAR, IMU) so we can develop the whole robot without the real hardware.

Crucially, Gazebo is **two separate programs**:

| Program | Role | Has the world state? | Has graphics? |
|---------|------|----------------------|---------------|
| **`gzserver`** | the **server**: runs physics, holds every model + pose, hosts plugins/ROS bridges | **YES** | no (headless) |
| **`gzclient`** | the **client**: a viewer window that draws what the server has | no | yes |

**This is why "closing Gazebo" doesn't reset anything.** Clicking the window's X kills
`gzclient` (the viewer). `gzserver` keeps running headless — still simulating, still holding
the box wherever it fell, still answering services. Re-launching attaches a *new* viewer to
the *old* world (and spawns a *second* box on top). The only true reset is to kill the server:
```bash
pkill -9 -f gzserver        # THIS is the reset. Not the window's X.
```

### Gazebo plugins — and the bug they caused
Gazebo's behaviour is extended by **plugins** (shared libraries, `.so` files). There are
different *kinds*, and **using the wrong kind silently breaks things**:

- **System plugins** (`gzserver -s some.so`): load before the world; used for core ROS bridges
  like `libgazebo_ros_init.so` (clock) and `libgazebo_ros_factory.so` (spawn/delete).
- **World plugins** (written *inside* the `.world` file as `<plugin .../>`): attached to the
  world; this is what `libgazebo_ros_state.so` (the `/get_entity_state` + `/set_entity_state`
  services) must be.
- **Model / sensor plugins**: attached to a model or sensor (e.g. the camera plugin that
  publishes `/rgb/image_raw`, the `ros2_control` plugin that runs the controllers).

> **War story (full detail in chapter 05):** we needed `/set_entity_state` for the grasp
> "weld." We loaded `libgazebo_ros_state.so` with `-s` (the *system-plugin* slot). It loaded
> just enough to **advertise the service names**, but — being a *world* plugin loaded in the
> wrong slot — it never attached to the world, so the services **were listed but never
> answered**. The fix was to load it correctly, as a `<plugin>` inside `final_map.world`.
> Lesson: *plugin kind matters; "the service exists" ≠ "the service works."*

### Spawning, and why the box persisted
Static scenery lives *in* the world file. But we add the **test box and table at runtime**
with `spawn_entity.py` (which talks to the factory plugin). Because they are injected into the
*running* `gzserver`, and because `gzserver` outlives the window, those objects — and any
`set_entity_state` we did to them — **persist until the server is killed**. That is the full
explanation of "the box stayed on the floor after I re-ran it."

---

## 1.3 The launch chain — from text files to a moving robot

You never start a dozen nodes by hand. A **launch file** (Python, `*.launch.py`) starts and
configures groups of nodes. Our isolated grasp test uses three:

```bash
# Terminal 1 — the simulation + robot + controllers (+ the test box/table + base pin)
ros2 launch limo_car arm_grasp_test.launch.py
# Terminal 2 — MoveIt's planner (move_group)
ros2 launch limo_cobot_moveit_config moveit.launch.py
# Terminal 3 — perception + the brain (the grasp sequence)
ros2 launch limo_car arm_grasp_run.launch.py
```

What Terminal 1 actually does, step by step (this is the "launch chain"):
1. **Xacro → URDF.** The robot is written in **Xacro** (an XML macro language) split across
   many files for reuse. At launch it is *expanded* into one big **URDF** (the full robot
   description) and published on the `/robot_description` topic.
2. **`gzserver`** starts with the world (`final_map.world`) and the system plugins.
3. **`spawn_entity`** reads `/robot_description` and creates the robot inside Gazebo.
4. The **`ros2_control`** plugin inside the robot starts the **controllers** (see §1.4).
5. Our extra test nodes (spawn the box/table, `base_pin`) start on timers.

The build step that connects your edited source to what actually runs:
```bash
cd ~/Desktop/Thesisorg/src/ros2_ws
colcon build --packages-select limo_car        # compile/install your package
source install/setup.bash                       # make this terminal "see" the new build
```
> **Gotcha we hit repeatedly:** editing a file changes nothing until you `colcon build` *and*
> `source install/setup.bash` in the terminal you launch from. "I changed it but it behaves
> the same" almost always means an un-built or un-sourced terminal.

---

## 1.4 `ros2_control` — turning "go to this angle" into motion

The arm's joints are driven by the **`ros2_control`** framework, which runs *inside* `gzserver`
via a plugin. It provides **controllers** — small modules that convert high-level commands into
joint motion and report joint state back:

- **`joint_state_broadcaster`** — *reads* every joint's angle and **publishes `/joint_states`**.
  This is the feedback that lets `move_group` know where the arm is. (If this isn't publishing
  the arm joints, MoveIt is blind to the arm.)
- **`mycobot_arm_controller`** (a *JointTrajectoryController*) — *accepts* a time-stamped
  trajectory (a list of joint angles over time) via an action, and drives the joints to follow
  it. This is what MoveIt's plan is handed to.
- **`mycobot_gripper_controller`** (a *position controller*) — accepts raw joint positions on a
  topic (`/mycobot_gripper_controller/commands`, a `Float64MultiArray`). We drive the gripper
  *outside* MoveIt for simplicity.

Check controllers live:
```bash
ros2 control list_controllers     # should show all three "active"
```
> **War story:** after a system upgrade, the EOL `gazebo_ros2_control` plugin passed the robot
> description to the newer controller-manager in a format it rejected → **no controllers** → a
> limp, flopping arm. The fix was to patch the plugin to read the description from a *topic*
> instead. Lesson: *version mismatches at a boundary are a whole class of bug.*

---

## 1.5 TF — the shared map of "where is everything?"

A robot is a tree of rigid parts, each with its own **coordinate frame** (its own origin and
x/y/z axes). **TF** (transform) is a live, time-stamped publication of *where every frame is
relative to its parent*. Together they form a tree:

```
world → base_footprint → base_link ─┬─► joint1 → joint2 → … → gripper_tcp   (the arm)
                                     └─► depth_camera_link                   (the camera)
```

Why you cannot do robotics without it: perception finds the box in the **camera's** frame, but
the arm thinks in **`base_link`**. TF is what lets you ask *"that point the camera saw — where
is it in the arm's frame?"* and get an exact answer. It is also how the grasp "weld" found the
gripper's pose in the world.

Conventions (REP-103): axes are **x = forward, y = left, z = up**, lengths in **metres**,
angles in **radians**. Orientations are stored as **quaternions** (4 numbers) — see
`06_math_models.md` for why and how.

Inspect TF live:
```bash
ros2 run tf2_ros tf2_echo base_link gripper_tcp     # where is the gripper, in the arm frame?
ros2 run tf2_tools view_frames                       # save a PDF picture of the whole tree
```

> We used `tf2_echo` to *measure the real gripper geometry* — discovering that the fingers
> point along the gripper's **+y** axis and are only ~2.7 cm long, which explained why a
> "top-down" grasp had the fingers pointing the wrong way. TF is a measurement tool, not just
> plumbing.

**Next:** `02_perception.md` — how a picture becomes a 3-D position.
