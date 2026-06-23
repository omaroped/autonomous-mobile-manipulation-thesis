# 07 — Terminal & Introspection Mastery (become fluent in the live system)

This chapter turns you into someone who can sit at a running robot, type a handful of commands,
and **know exactly what is connected to what, what is flowing, and where it stops.** That skill
is worth more than memorising any single fix — it is how every bug in chapter 05 was actually
found. Work through it with the simulation running.

> **Two setup facts that catch everyone:**
> 1. Every new terminal must be told where ROS and your build are:
>    ```bash
>    source /opt/ros/humble/setup.bash                 # ROS itself
>    cd ~/Desktop/Thesisorg/src/ros2_ws && source install/setup.bash   # YOUR packages
>    ```
>    If `ros2` "can't find" your node or package, you forgot one of these.
> 2. `ros2 ...` commands talk to a background **daemon** that caches the graph. If a list looks
>    stale, refresh it: `ros2 daemon stop` (it auto-restarts), or add `--no-daemon`.

---

## 7.1 The four "what exists?" commands (your eyes on the system)

```bash
ros2 node list        # every running program (node)
ros2 topic list       # every stream
ros2 service list     # every request/response endpoint
ros2 action list      # every long-job endpoint
ros2 param list       # (per node) every setting
```
Filter with `grep`. This is the reflex for "is the thing I expect even running?":
```bash
ros2 node list  | grep -i move
ros2 topic list | grep -i box
ros2 service list | grep -i entity_state
```

## 7.2 The "tell me about *one* thing" commands (`info`, `type`, `show`)

Once you've found a name, interrogate it:
```bash
# WHO is on this topic, and what message does it carry?
ros2 topic info /box_pose -v        # -v lists every publisher & subscriber node + QoS
ros2 topic type /box_pose           # -> geometry_msgs/msg/PoseStamped

# WHAT FIELDS does that message have? (so you know what you can read/echo)
ros2 interface show geometry_msgs/msg/PoseStamped

# WHAT does a node connect to? (every topic/service/action it pub/subs/offers)
ros2 node info /box_pose_estimator
```
`ros2 topic info -v` is the command that answers *"is anyone actually listening to this?"* — the
fastest way to spot a broken wire (a publisher with zero subscribers, or vice-versa).

## 7.3 Watching the data actually flow (`echo`, `hz`, `bw`)

```bash
ros2 topic echo /box_pose              # print every message (Ctrl-C to stop)
ros2 topic echo /box_pose --once       # just one, then exit
ros2 topic hz /joint_states            # how many messages per second? (is it alive?)
ros2 topic bw /rgb/image_raw           # bandwidth (heavy topics like images)
ros2 topic echo /joint_states --field name   # only one field of the message
```
The single most useful debugging habit in this whole project: **follow the data forward** —
`echo` the camera, then `/box_pose`, then watch `/move_action` — and find the *first* place the
data stops or goes wrong. That is your bug's location.

## 7.4 Poking the system on purpose (`pub`, `service call`)

You can *inject* messages and *call* services by hand — invaluable for isolating a layer:
```bash
# Manually open the gripper (test the actuation path with no brain involved)
ros2 topic pub --once /mycobot_gripper_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.15,0.15,-0.15,-0.15,-0.15,0.15]}"

# Manually drive the base forward a touch
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"

# Ask Gazebo where a model is (the kind of call the weld depends on)
ros2 service call /get_entity_state gazebo_msgs/srv/GetEntityState "{name: 'mbot'}"
```
> Lesson from chapter 05: a service can be **listed** but not **answer** — always *call* it to
> prove it works, don't trust `service list` alone.

## 7.5 The MoveIt diagnostic services (prove reach & collisions)

These two turned ~10 wasted runs into a 5-minute diagnosis. Learn them.
```bash
# "Can the arm put gripper_tcp at this pose?"  result error_code: 1 = yes, -31 = no IK solution
ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK \
  "{ik_request: {group_name: 'arm', ik_link_name: 'gripper_tcp', pose_stamped: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.2, y: 0.0, z: 0.1}, orientation: {w: 1.0}}}}}"

# "Is this configuration in collision, and with what?"
ros2 service call /check_state_validity moveit_msgs/srv/GetStateValidity \
  "{group_name: 'arm', robot_state: {joint_state: {name: [...], position: [...]}}}"
```
For anything heavier than a one-liner, write a tiny `rclpy` Python script (see the `/tmp/*.py`
probes we used) — it's the same idea, just easier to build the request.

## 7.6 Coordinate frames (`tf2_echo`, `view_frames`)

```bash
ros2 run tf2_ros tf2_echo base_link gripper_tcp   # live transform between two frames
ros2 run tf2_tools view_frames                     # writes frames.pdf: a picture of the whole tree
```
We *measured the real gripper geometry* with `tf2_echo` (fingers along +y, ~2.7 cm) — TF tools
are measurement instruments, not just plumbing.

## 7.7 The controllers (`ros2 control`)

```bash
ros2 control list_controllers                # all should read "active"
ros2 control list_hardware_interfaces        # the command/state interfaces ros2_control sees
```
If `mycobot_arm_controller` isn't `active`, MoveIt's plan has nothing to execute (and a "flopping
arm" usually means the controllers never loaded — chapter 05, story 8).

## 7.8 Visual tools (when text isn't enough)

```bash
rqt_graph                       # a live picture of nodes & topics (the auto-generated version of our diagram)
ros2 run rqt_image_view rqt_image_view   # see camera / the perception debug mask
rviz2                            # 3D view: the robot, TF frames, /box_pose marker, planned paths
ros2 run rqt_console rqt_console # a searchable window of all log messages
```
`rqt_graph` is the fastest way to *see* a missing connection; RViz is the fastest way to *see*
whether the arm is going where you think it is.

## 7.9 Record & replay (`ros2 bag`) — capture a bug once, study it offline

```bash
ros2 bag record /box_pose /joint_states /tf      # record selected topics to a folder
ros2 bag record -a                                # record EVERYTHING
ros2 bag info <folder>                            # what's inside
ros2 bag play <folder>                            # replay it (drives subscribers as if live)
```
If something fails intermittently, **record it**, then replay and `echo` at your leisure.

## 7.10 Gazebo's own CLI (the simulator, separate from ROS)

Gazebo Classic has its own transport, parallel to ROS:
```bash
gz model --list                       # models in the world
gz model -m mbot -p                    # pose of a model
gz topic -l                            # Gazebo (not ROS) topics
gz stats                               # real-time factor: is the sim keeping up?
```
And the process truth from chapter 01:
```bash
pgrep -a gzserver        # the SERVER (the actual simulation) — killing this is the only real reset
pgrep -a gzclient        # the GUI window — closing it changes nothing about the world
```

## 7.11 Build & environment (close the loop on your edits)

```bash
cd ~/Desktop/Thesisorg/src/ros2_ws
colcon build --packages-select limo_car --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash                  # in EVERY terminal that runs the new build
ros2 pkg list | grep limo                   # is the package even visible?
ros2 pkg executables limo_car               # what runnable nodes does it install?
```
> "I changed the file but nothing changed" = you didn't `colcon build`, or didn't `source` the
> terminal you launched from. This is the #1 beginner trap.

---

## 7.12 Guided lab: read this whole robot in 90 seconds

With the three launch terminals running, do this top-to-bottom and you will have a complete,
*proven* mental model of the live system:

```bash
ros2 node list                                   # 1. who's alive?
ros2 topic hz /rgb/image_raw                      # 2. is the camera sensing?
ros2 topic echo /box_pose --once                  # 3. did perception locate the box?
ros2 topic hz /joint_states                       # 4. is the arm reporting its state?
ros2 control list_controllers                     # 5. are the controllers active?
ros2 run tf2_ros tf2_echo base_link gripper_tcp    # 6. where is the gripper, really?
ros2 node info /move_group | grep -i action        # 7. is /move_action offered?
ros2 service call /get_entity_state gazebo_msgs/srv/GetEntityState "{name: 'mbot'}"  # 8. is Gazebo answering?
```
Each line tests one box of the Sense→Locate→Plan→Act loop. When something is wrong, the line
that surprises you is pointing at the broken layer. **That is the entire skill.**

## 7.13 One-page cheat sheet

| Goal | Command |
|------|---------|
| What's running? | `ros2 node list` |
| What streams exist? | `ros2 topic list` |
| Who's on a topic? | `ros2 topic info /T -v` |
| What's in a message? | `ros2 interface show <type>` |
| Watch data | `ros2 topic echo /T` · `ros2 topic hz /T` |
| Inject data | `ros2 topic pub --once /T <type> "{...}"` |
| Call a service | `ros2 service call /S <type> "{...}"` |
| A node's connections | `ros2 node info /N` |
| Can the arm reach? | `ros2 service call /compute_ik ...` |
| Where is a frame? | `ros2 run tf2_ros tf2_echo A B` |
| Controllers ok? | `ros2 control list_controllers` |
| See the graph | `rqt_graph` · `rviz2` |
| Record/replay | `ros2 bag record -a` · `ros2 bag play <f>` |
| True sim reset | `pkill -9 -f gzserver` |
| Rebuild + see it | `colcon build … && source install/setup.bash` |

You now have the same toolkit that found every bug in this project. Read `05_war_stories…` again
with this chapter in hand and you'll see each diagnosis is just these commands in sequence.
