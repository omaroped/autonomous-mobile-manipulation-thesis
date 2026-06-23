# How the Robot Navigates, Knows Where It Is, and Finds the Box — Explained Simply

A plain-language teaching report answering: how navigation works, what is hard-coded vs.
sensed, the "identical places" problem (your professor's point), which topics talk to which,
and what each window/launch is for.

---

## 0. The one-paragraph summary
The robot has a **pre-built map** of the room. It always tries to answer *"where am I on this
map?"* (**localization**, done by a node called **AMCL** using the LiDAR). When you give it a
**goal** on the map, **Nav2** draws a path and drives there (**path planning + following**).
Getting to the *table* is done with **known map coordinates** (we tell it "the workstation is
here"). Finding the *exact box on the table* is done with the **camera** (perception). So it's a
**hybrid: coarse position is map-based, fine position is camera-based.**

---

## 1. "Where am I?" — Localization (the foundation of everything)

Think of the robot waking up in a building it has a **floor plan** (map) of.

- **map** = the floor plan (we built it earlier with SLAM; it's `thesis_map`).
- **odom** = the robot counting its own wheel turns from where it started ("I rolled 2 m forward,
  turned 30°"). Always available, but **drifts** over time (wheels slip, etc.).
- **AMCL** = the node that answers *"where am I on the floor plan?"*. It works like this:
  1. We **tell it the starting spot once** (`set_initial_pose` → "you're at (−2, 7) facing −Y").
  2. It scatters hundreds of **guess particles** around that spot.
  3. It compares the **live LiDAR scan** to the map's walls. Guesses where the scan lines up with
     real walls get rewarded; bad guesses die. The surviving cluster = the robot's position.
  4. As the robot drives, **odom** moves the particles and the **LiDAR keeps correcting** them.
- The output is the **`map → odom` transform** — literally "here is the robot on the map," updated
  continuously. **If this transform doesn't exist, nothing else works** (that's the "frame map
  does not exist" error we hit).

### ⭐ The "identical places" problem (your professor's point)
AMCL figures out position by **matching the LiDAR scan to the map walls**. If two parts of the
room look **identical** (same wall shapes — e.g., a long plain corridor, or two identical corners),
the scan matches **both** equally well, and AMCL **can't tell which one it's in** → it may "jump"
to the wrong place → the robot drives to the wrong spot.

**The fix (what your professor meant):** make the environment **visually/geometrically distinct** —
different corner shapes, a pillar here, an angled wall there, or **fiducial markers** (printed
tags). Then every location produces a **unique scan**, and AMCL is never ambiguous. This is a real,
documented design point for your thesis: *a feature-rich map = reliable localization.*

> **Important about moving the robot:** if you drag the robot in Gazebo, AMCL still believes the old
> spot (we only told it once). Then it drives blind and dodges phantom walls. To move it, you must
> re-tell AMCL (RViz "2D Pose Estimate") — or use **ground-truth localization** in sim (see §8).

---

## 2. "How do I get there?" — Navigation & path planning (Nav2)

Once the robot knows where it is, you give it a **goal pose** on the map. **Nav2** then:

1. **Costmaps** — it paints the map into a grid: black = walls/obstacles (from the static map +
   live LiDAR), with a safety "halo" (inflation) around them so it doesn't clip corners.
   - **Global costmap** = the whole map (for planning the route).
   - **Local costmap** = a small window around the robot (for dodging things in real time).
2. **Global planner** (`NavfnPlanner`) — draws the **shortest free path** from the robot to the
   goal across the global costmap (like a GPS route). This is the line you see in RViz.
3. **Controller** (`RegulatedPurePursuit`) — actually **drives** the robot along that path, sending
   `/cmd_vel` (speed + turn). With **differential drive** it can rotate in place to face the path.
4. **Recovery behaviours** — if stuck, it spins, backs up, or re-plans.

So "navigate the entire map" = *plan a path on the costmap, then follow it,* re-checking constantly.

---

## 3. "Where exactly is the box?" — Hard-coded vs. camera

This is the part that confused you, so here's the clear split:

| Question | How it's done now | Alternative |
|---|---|---|
| **Which table to go to?** | **Hard-coded map coordinate** — we tell Nav2 "drive to (−2, 4.6)," the known workstation. | Explore the room and detect tables (much harder; not needed for a known station). |
| **Where's the box on the table?** | **Camera (perception)** — the depth camera sees the blue box, computes its 3-D position (`/box_pose`), and the arm grasps *that exact spot*. | Hard-code the box pose too (only works if it never moves). |
| **Final inch alignment** | **Camera + visual docking** — the robot rotates/creeps until the box is centred in front of it. | — |

So today: **drive to the table = known coordinate; pick the box = camera.** That's the sweet spot —
you don't need full exploration, but the grasp still adapts to where the box actually is. This is a
completely standard "known workstation" design used in real warehouses.

---

## 4. Two tables: pick from A, place at B — how does it know?

- **Table A and Table B are known locations** on the map → two **hard-coded waypoints**. The robot
  navigates A → (pick) → B → (place). Nav2 handles the driving between them.
- **The place spot can be hard-coded** (your professor literally said *"hard-code the place"*): we
  tell the arm "put it at these coordinates on table B." Simple and reliable — good for stacking,
  where you place at fixed stack positions.
- **If table B were *not* hard-coded**, the robot would need to *recognise* it — e.g., a **fiducial
  marker** on the table, or detecting the table with the camera/LiDAR. That's extra work and only
  needed if the layout changes. For your thesis scope, **known waypoints + hard-coded place** is the
  right, defensible choice.
- **"Identical areas" again:** this is why the professor wants distinct places — so the robot never
  confuses table A's area with table B's area during localization.

---

## 5. Who talks to whom — the topics (data flow)

```
  LiDAR  ──/scan──────────────►  AMCL  ──► map→odom transform (where am I)
  wheels ──/odom──────────────►  AMCL  ┘            │
  map    ──/map───────────────►  costmaps ◄─/scan──┘
                                   │
  goal ─► Nav2 (planner+controller) ──/cmd_vel──► robot wheels (drive)
                                   │
  RGB+depth camera ──/rgb/image_raw, /depth/...──► perception ──/box_pose──► orchestrator
  orchestrator ──/cmd_vel (docking), /move_action (arm via MoveIt), /grasp_attach (weld)
```
Key topics:
- **`/scan`** (LiDAR) → AMCL (localize) **and** costmaps (obstacles).
- **`/odom`** (diff_drive plugin) → AMCL (motion).
- **`/tf`** → the chain `map → odom → base_link → … → gripper`; everything positions itself with this.
- **`/cmd_vel`** → the one "drive" command the wheels obey (Nav2 sends it while navigating; the
  orchestrator sends it while docking — only one should drive at a time).
- **`/box_pose`** → the camera's answer for "where's the box," used by the arm.
- **`/move_action`** → the arm asks MoveIt to plan a motion.

---

## 6. The windows & the launches — what each is and when to run it

**Two kinds of windows:**
- **Gazebo (gzclient)** = the **real simulated world** — physics, the actual robot, the actual box.
  "What's truly happening."
- **RViz** = the **robot's mind** — what it *believes*: the map, its localization, costmaps, the
  planned path, the laser scan, the arm's plan. "What the robot thinks is happening."
  *(Comparing the two is how you verify it's correct — e.g., the scan in RViz should line up with
  the walls.)*

**The launches (each starts one part of the system):**
| Launch | Starts | Run it when |
|---|---|---|
| `ackermann_gazebo.launch.py` | The **simulation** (world + robot + sensors + wheels) | Always — it's the base of everything |
| `moveit.launch.py` | The **arm brain** (MoveIt motion planning) + its RViz | When you need the **arm** (any grasp) |
| `nav2_limo.launch.py` | The **navigation stack** (map, AMCL, planner, controller) | When you need the robot to **drive itself** |
| `arm_grasp_test` + `arm_grasp_run` | A fixed test box + the grasp brain | To test **picking only** (no driving) |
| `nav_pick.launch.py` | **Everything together** + the orchestrator | The **full demo**: navigate → pick |

**Do you run them all?** Only what the test needs:
- Test the **arm** → Gazebo + MoveIt + grasp (3 windows/terminals).
- Test **navigation** → Gazebo + Nav2 + RViz.
- **Full pipeline** → Gazebo + Nav2 + MoveIt + orchestrator.
We split them into separate terminals so that **if one part misbehaves you can see exactly which**,
and restart just that part — much easier to debug than one giant launch.

---

## 7. What's hard-coded today vs. what could be smarter (honest status)
- **Hard-coded:** the workstation/table coordinates (nav goal), the place position, the map itself.
- **Sensed (adaptive):** the robot's live position (AMCL), obstacles (costmaps), and the **exact box
  position** (camera).
- **Could be added later (future work / sim-to-real):** fiducial markers for unambiguous places, a
  wrist camera for true eye-in-hand grasp refinement, and table/box recognition if the layout isn't
  fixed. Your professor's "non-identical places" note is precisely the cheap, high-value upgrade to
  make localization rock-solid — worth doing and documenting.

---

### TL;DR for your thesis
> The robot localizes with **AMCL + LiDAR-against-a-known-map**, navigates with **Nav2
> (plan a path on a costmap, then follow it)**, drives to **known table coordinates**, and uses the
> **camera to find and grasp the exact box**. Placing is at a **fixed (hard-coded) spot**.
> Reliable localization needs a **visually distinct map** (the professor's point) — identical areas
> confuse the LiDAR matching.
