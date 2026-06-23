# 04 — Navigation: how the base drives to the box

The arm can only grasp what's within ~25 cm of it, so the **base must first drive the robot up
to the box.** There are two ways to do this, and understanding the difference is itself a
valuable lesson in "right tool for the job."

---

## 4.1 The two philosophies

| | **Reactive visual servoing** (what we use first) | **Nav2 (full autonomous navigation)** |
|---|---|---|
| Idea | "I can *see* the box; drive toward it." | "I have a *map*; plan a route to a goal coordinate." |
| Needs a map? | No | Yes (SLAM or a pre-built map) |
| Needs the goal to be visible? | Yes | No |
| Handles obstacles in the way? | No | Yes (global + local planning) |
| Complexity | A few dozen lines | A large, configured stack |

This project starts with the **reactive** approach because the box is locally visible and the
scene is simple — the simplest thing that works. Nav2 is the planned upgrade for obstacle
avoidance and goal-driven driving.

---

## 4.2 Reactive visual servoing (`box_follower.py`)

The control loop, every camera frame:
1. Find the blue box in the image (same HSV idea as perception) → its pixel column and its
   depth (distance).
2. Two errors drive two commands:
   - **distance error** → forward speed: drive faster when far, stop at the target stand-off.
   - **heading error** (box left/right of image centre) → steering.
3. Publish a **`geometry_msgs/Twist`** on **`/cmd_vel`** (`linear.x` = forward speed,
   `angular.z` = turn rate). Gazebo's drive plugin turns that into wheel motion.

```bash
ros2 topic echo /cmd_vel        # watch the velocity commands
```

### The Ackermann constraint (why steering ≠ a tank)
The LIMO here is driven **Ackermann**-style — like a car. It **cannot turn on the spot**; it
must move forward to change heading. So a box that ends up *to the side* leaves a **residual
lateral offset** the base can't simply rotate away — a real, hardware-specific limitation that a
differential-drive (tank) robot wouldn't have. This matters for the grasp: *base-positioning
precision directly determines whether the arm can reach the box* — a problem that doesn't exist
for a fixed-base arm. This is one of the project's genuine findings.

### `/cmd_vel` and the drive plugin
`/cmd_vel` is the universal "how to move" channel for mobile robots. A Gazebo **drive plugin**
(model plugin in the robot's description) subscribes to it and applies the right wheel/steer
velocities, and publishes **odometry** (`/odom`) — the robot's estimate of its own motion.

---

## 4.3 Nav2 concepts (the planned upgrade) — explained from zero

Even though we haven't wired Nav2 in yet, you should understand its pieces, because they are
the standard vocabulary of mobile robotics and they're in the thesis plan.

- **Map** — a 2-D grid of "free / occupied / unknown." Either built live by **SLAM**
  (Simultaneous Localisation And Mapping — the robot maps the world *and* tracks itself in it
  using the LiDAR) or loaded from a saved file.
- **Localisation (AMCL)** — "where am I on the map?", by matching live LiDAR to the map.
- **Costmaps** — the map, inflated with a safety margin around obstacles, so the robot keeps
  clearance. Two of them:
  - **Global costmap** — the whole known area, for long-range route planning.
  - **Local costmap** — a small window around the robot, updated fast from live sensors, for
    dodging things that appear suddenly.
- **Global planner** — computes a full route from here to the goal on the global costmap
  (e.g. A*).
- **Local planner / controller** — follows that route while reacting to the local costmap,
  emitting the actual `/cmd_vel`. (It must respect the Ackermann constraint — no in-place
  spins.)
- **Behaviour tree** — the "brain" that sequences it all (plan → follow → recover if stuck).

The mental model: **global planner = the route on the map; local controller = the steering that
follows it while avoiding surprises.** Both ultimately produce the same `/cmd_vel` our reactive
node produces today — so the grasp side doesn't care which drives the base.

### Odometry and drift (a concept that bites everyone)
`/odom` is *dead reckoning* — adding up wheel motion. It is smooth but **drifts** over time
(small errors accumulate; wheels slip). That's why a map + localisation exists: to *correct*
the drift. In simulation we also saw the base **creep** under the arm's reaction force because
the wheels were un-braked — handled for the grasp test by pinning the base (chapter 03/05).

**Next:** `05_war_stories_and_playbook.md` — the complete list of bugs, fixes, lessons, and the
command playbook.
