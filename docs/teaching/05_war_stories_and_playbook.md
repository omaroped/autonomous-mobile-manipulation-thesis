# 05 — War Stories, the Command Playbook, and Glossary

This is the most valuable chapter for a beginner, because it shows *how to think* when things
break. Each story has the same shape: **Symptom → How we proved the cause → The fix → The
lesson.** The lessons generalise far beyond this project.

---

## 5.1 The war stories

### 1. The arm did nothing for ~10 runs (the incomplete collision matrix)
- **Symptom:** joint goals (`ready`) worked, but **every** "go to this pose" goal failed; the
  arm always folded up and stopped.
- **Proof:** `/compute_ik` succeeded (reach OK), `/check_state_validity` said start+goal were
  valid, joint goals failed *only when the waist ≠ 0*, and the planner's log named a contact
  between `gripper_base` and a wheel. Interpolating the path and printing contacts confirmed a
  *phantom* self-collision.
- **Fix:** add the missing `<disable_collisions>` lines (`gripper_base`/`joint6`/`joint6_flange`
  vs the wheels/steers/laser/camera/IMU) in `config/limo_cobot.srdf`.
- **Lesson:** the cause can be many layers from the symptom. An auto-generated collision matrix
  can be silently incomplete and block *all* Cartesian planning while joint goals still work.
  **Diagnose with services and logs, not guesses.**

### 2. "It can't reach the box" — actually the wrong wrist roll
- **Symptom:** the box at 0.20 m looked unreachable.
- **Proof:** we asked `/compute_ik` across *many* orientations; straight-down with one roll
  failed, straight-down with a 90°-rotated roll reached **all the way out**.
- **Fix:** use the reachable roll.
- **Lesson:** **reachability = position AND orientation (including roll).** Fixing one
  orientation parameter to a bad value makes the whole workspace *look* smaller than it is.

### 3. The gripper hovered over the box instead of around it (tool-frame geometry)
- **Symptom:** the "top-down" gripper came down beside/over the box, fingers not around it.
- **Proof:** `tf2_echo` on the finger links showed the fingers point along the gripper's **+y**
  axis (not its z), and are only ~2.7 cm long.
- **Fix:** orient so the gripper's **+y points straight down** (`(−0.707,0,0,0.707)`).
- **Lesson:** **measure the tool frame before commanding it.** "Top-down" is meaningless until
  you know which axis the fingers are on.

### 4. The whole robot flew into the sky (rigid-contact explosion)
- **Symptom:** the gripper touched the table and the robot launched metres up.
- **Proof:** the box-pose readings went to garbage; it happened exactly on table contact.
- **Fix:** stop the descent above the table (and add the table to the planning scene); **pin the
  base** so a contact force can't launch it.
- **Lesson:** this is a **simulation** artefact (rigid bodies + a light un-anchored base), not a
  robotics failure. Know which of your problems are physics-of-the-simulator.

### 5. The box drifted while "static," and stayed on the floor after re-running
- **Symptom:** the box reading crept (0.21 → 0.25 m) though the box was static; and after a
  re-run the box was still on the floor.
- **Proof:** the *base* was creeping under the arm's reaction force (un-braked wheels);
  separately, the test box is spawned into the *running* `gzserver`, which outlives the GUI.
- **Fix:** `base_pin` holds the base; and the only true reset is `pkill gzserver`.
- **Lesson:** **closing the Gazebo window ≠ resetting the world** (`gzclient` vs `gzserver`).
  And an un-braked base is a real disturbance source.

### 6. The "weld" service was listed but never answered (wrong plugin kind)
- **Symptom:** `ros2 service list` showed `/get_entity_state`, but every call timed out;
  `base_pin` even crashed trying to use it.
- **Proof:** the sim was running and unpaused, only one `gzserver`, the plugin *was* in the
  command — yet no response.
- **Fix:** `libgazebo_ros_state.so` is a **world** plugin; we'd loaded it with `-s` (the
  *system*-plugin slot). Move it into `final_map.world` as a `<plugin>`.
- **Lesson:** **plugin *kind* matters**, and **"the service is advertised" ≠ "the service
  works."** Test the actual call.

### 7. The state plugin can't reference a link (design around your tools)
- **Symptom:** the weld's "box relative to the gripper link" call always returned `success=false`.
- **Proof:** every gripper-link name failed; whole-model + world-frame queries succeeded.
- **Fix:** compute the gripper's world pose from **TF + the pinned base pose**, and move the box
  in world coordinates instead.
- **Lesson:** when a tool can't do exactly what you want, **find what it *can* do and compose**
  the rest yourself.

### 8. EOL `gazebo_ros2_control` vs the newer controller-manager
- **Symptom:** after an upgrade, no controllers loaded; the arm flopped under gravity.
- **Proof:** the controller-manager rejected the way the old plugin passed the robot
  description.
- **Fix:** patch the plugin (from source) to read the description from a topic.
- **Lesson:** version mismatches *at a boundary between components* are a whole class of bug.
  Patching a dependency from source in your workspace is a legitimate tool.

### 9. MoveIt "failed to fetch current robot state" (sim time)
- **Symptom:** planning failed to read the arm's state.
- **Proof:** nodes were comparing wall-clock timestamps to sim-clock timestamps.
- **Fix:** force `use_sim_time: true` on `move_group`.
- **Lesson:** in simulation, *everyone* must agree time comes from `/clock`.

---

## 5.2 The diagnostic method (the meta-lesson)

Notice the pattern in every story above:

1. **Name the failing interface** (topic? service? action? which node?).
2. **Prove each layer with a tool**, working along the data flow: `ros2 topic echo/hz`,
   `ros2 service call /compute_ik`, `/check_state_validity`, `tf2_echo`, the node's own log.
3. **Find a crisp, reproducible pattern** ("fails only when waist ≠ 0"). A pattern points at a
   cause.
4. **Fix the cause, not the symptom**, then re-run changing *one* thing.

If you do only this, you will out-debug most people.

---

## 5.3 The command playbook

### Reset & run the isolated grasp test (3 terminals)
```bash
# 0) the ONLY true reset — kills the server, not just the window
pkill -9 -f "gzserver|gzclient|move_group|rviz2|robot_state_publisher|spawn_entity|box_pose|arm_grasp|base_pin|grasp_attacher" ; sleep 3

# 1) sim + robot + table + box + base pin
cd ~/Desktop/Thesisorg/src/ros2_ws && source install/setup.bash
ros2 launch limo_car arm_grasp_test.launch.py        # wait for "base pinned — holding pose"

# 2) the planner
ros2 launch limo_cobot_moveit_config moveit.launch.py # wait for "You can start planning now!"

# 3) perception + the grasp brain
ros2 launch limo_car arm_grasp_run.launch.py
```

### Build & source (after editing code)
```bash
cd ~/Desktop/Thesisorg/src/ros2_ws
colcon build --packages-select limo_car --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash      # in EVERY terminal that will use the new build
```

### Inspect the live system
```bash
ros2 node list                                   # who's running
ros2 topic list ; ros2 topic echo /box_pose       # channels + watch one
ros2 topic hz /joint_states                       # is the arm state flowing?
ros2 service list                                 # services
ros2 control list_controllers                     # are the controllers active?
ros2 run tf2_ros tf2_echo base_link gripper_tcp    # where is the gripper?
```

### Prove reach / collisions (MoveIt)
```bash
ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK "{...}"          # can it reach a pose?
ros2 service call /check_state_validity moveit_msgs/srv/GetStateValidity "{...}"  # is a pose in collision?
```

---

## 5.4 Glossary

| Term | Meaning |
|------|---------|
| **ROS 2** | Framework for many small programs (nodes) to communicate. |
| **Node** | One running program with a name. |
| **Topic** | A named stream; publish/subscribe; for continuous data. |
| **Service** | A request/response call; for quick queries/commands. |
| **Action** | A goal/feedback/result interface; for long jobs. |
| **Parameter** | A node's setting (e.g. `use_sim_time`). |
| **`gzserver` / `gzclient`** | Gazebo's headless *server* (the actual simulation) / its GUI *viewer*. |
| **Plugin** | A `.so` library extending Gazebo; *kinds*: system / world / model / sensor. |
| **URDF / Xacro** | The robot description / the macro language it's written in. |
| **TF** | Live tree of coordinate-frame transforms ("where is everything"). |
| **`ros2_control`** | Framework running the joint controllers. |
| **JointStateBroadcaster** | Controller that publishes `/joint_states` (joint feedback). |
| **JointTrajectoryController** | Controller that executes a planned trajectory. |
| **MoveIt / `move_group`** | The arm motion-planning framework / its node. |
| **SRDF** | Semantic robot description: groups, named poses, **collision matrix**. |
| **IK / FK** | Inverse / Forward Kinematics (pose→angles / angles→pose). |
| **OMPL / RRTConnect** | The planning library / the path-search algorithm. |
| **Planning scene** | MoveIt's model of the robot + obstacles, for collision checks. |
| **De-projection** | Pixel + depth + intrinsics → a 3-D point. |
| **HSV** | Hue/Saturation/Value colour space (robust colour filtering). |
| **Quaternion** | 4-number representation of an orientation. |
| **`/cmd_vel`** | Velocity command topic for the base (`geometry_msgs/Twist`). |
| **Odometry** | The robot's own estimate of its motion (drifts over time). |
| **Nav2 / SLAM / AMCL** | Navigation stack / live mapping / localisation on a map. |
| **Costmap** | Map inflated with safety margins for planning. |
| **Ackermann** | Car-like steering (cannot turn on the spot). |
| **Attach-on-grasp ("weld")** | Bonding the object to the gripper in sim to avoid contact instability. |

**Next:** `06_math_models.md` — the equations, in one place.
