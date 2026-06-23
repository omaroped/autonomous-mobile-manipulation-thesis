# Stacking Experiment: Full Implementation Plan

**Goal:** Robot picks boxes one by one from Table 1, stacks them vertically on Table 2,
returns to Table 1, repeats. Run N=20 iterations. Measure how pose drift accumulates
and find at what iteration the arm can no longer pick successfully.

---

## 1. What This Experiment Proves (Academic Value)

| Metric | What it measures | Thesis contribution |
|---|---|---|
| Position error at T1 per iteration | Navigation drift accumulating over cycles | Quantifies robot's repeatability limit |
| Grasp success rate vs. iteration | At what drift magnitude picking fails | Defines the "operational envelope" |
| Stack Z-error per layer | Placement precision under accumulated drift | Characterises the place stage accuracy |
| Sim vs. hardware drift profiles | The sim-to-real gap in odometry | Core sim-to-real contribution |

The **breaking point** — the iteration N* where the arm first fails to pick — is a
concrete, measurable thesis result that no prior work on this specific platform reports.

---

## 2. World Layout (What to Build in Gazebo)

```
      [TABLE 1]             [TABLE 2]
      source                target/stack
      ┌───────┐             ┌───────┐
      │ □ □ □ │             │       │  ← boxes stack up here
      │ □ □ □ │             └───────┘
      └───────┘
          ↑                     ↑
      ~3m forward           ~1m left of T2 goal
      (current world)       (NEW — add to world)

  ROBOT starts here (original position)
```

- Table 1: already in world (the existing pickup table, `final_map.world`)
- Table 2: **new** — add a second table ~1.5–2m to the right of Table 1
- Boxes: multiple identical boxes on Table 1 in a row

### World file changes needed
In `limo_car/worlds/final_map.world`:
```xml
<!-- Table 2 — stack target -->
<model name="table2">
  <pose>2.5 0.0 0 0 0 0</pose>   <!-- adjust XY to suit your map -->
  <static>true</static>
  <link name="link">
    <collision name="col">
      <geometry><box><size>0.6 0.4 0.32</size></box></geometry>
    </collision>
    <visual name="vis">
      <geometry><box><size>0.6 0.4 0.32</size></box></geometry>
      <material><ambient>0.4 0.2 0.0 1</ambient></material>
    </visual>
  </link>
</model>

<!-- Extra source boxes on Table 1 (spawn in a row, X offsets) -->
<!-- box2, box3, box4 ... same as existing box but offset in Y -->
```

---

## 3. Implementation: Step by Step

### Step 1 — Implement Place Stage in Orchestrator

File: `src/ros2_ws/src/limo_ros2/limo_car/scripts/nav_pick_orchestrator.py`

Add to `NAMED_STATES`:
```python
'place_hover': [0.0, -0.3, -0.5, 0.8, 0.0, 0.0],  # arm up, ready to lower over stack
```

Add constants:
```python
TABLE2_POSE_X   = 2.5    # world X of Table 2 centre
TABLE2_POSE_Y   = 0.0    # world Y of Table 2 centre
TABLE2_HEIGHT   = 0.32   # table top Z (metres)
BOX_HEIGHT      = 0.06   # height of one box (metres)
PLACE_CLEARANCE = 0.04   # how high above stack top the gripper hovers before lowering

# Table 2 Nav2 goal (odom frame = world frame in sim)
TABLE2_NAV_GOAL = (2.0, 0.0, 0.0)   # (x, y, yaw) — stop in front of T2
TABLE1_NAV_GOAL = (0.0, 0.0, math.pi) # (x, y, yaw) — return to T1 facing it
```

Add method `place_on_stack(self, stack_level: int) -> bool`:
```python
def place_on_stack(self, stack_level: int) -> bool:
    """Lower held box onto the stack at Table 2. stack_level=0 means first box."""
    self._wait_for_move_group()

    # Target Z = table top + boxes already placed + half new box
    place_z = TABLE2_HEIGHT + stack_level * BOX_HEIGHT + BOX_HEIGHT / 2.0
    hover_z = place_z + PLACE_CLEARANCE

    # Move to hover position above stack
    quat = [0.0, 1.0, 0.0, 0.0]  # gripper pointing down
    ok = self.go_pose(TABLE2_POSE_X, TABLE2_POSE_Y, hover_z, quat, 'place_hover')
    if not ok:
        self.get_logger().error(f'place_hover failed at level {stack_level}')
        return False

    # Lower to place position
    ok = self.go_pose(TABLE2_POSE_X, TABLE2_POSE_Y, place_z, quat, 'place_lower')
    if not ok:
        self.get_logger().error(f'place_lower failed at level {stack_level}')
        return False

    # Open gripper + detach weld
    self.open_gripper()
    self.unpin_base()  # detaches the weld

    time.sleep(0.3)

    # Lift arm away
    ok = self.go_pose(TABLE2_POSE_X, TABLE2_POSE_Y, hover_z + 0.1, quat, 'place_retreat')
    return True
```

Add method `navigate_to(self, x, y, yaw, label) -> bool`:
```python
def navigate_to(self, x: float, y: float, yaw: float, label: str) -> bool:
    """Generic Nav2 goal. Reuses the same action client as navigate_to_table()."""
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp = self.get_clock().now().to_msg()
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    q = euler_to_quaternion(0, 0, yaw)   # helper already exists
    goal.pose.pose.orientation.z = q[2]
    goal.pose.pose.orientation.w = q[3]
    # same send/wait pattern as navigate_to_table()
    ...
```

---

### Step 2 — Implement the Iteration Loop

Replace the single-cycle `run()` with a multi-cycle `run_stacking(n_cycles=20)`:

```python
def run_stacking(self, n_cycles: int = 20):
    self.get_logger().info(f'=== STACKING EXPERIMENT: {n_cycles} cycles ===')

    # Data log: one row per iteration
    log = []   # list of dicts

    self._wait_for_move_group()
    self.go_named('travel')

    for i in range(n_cycles):
        self.get_logger().info(f'--- Iteration {i+1}/{n_cycles} ---')
        row = {'iteration': i + 1, 'pick_success': False, 'place_success': False,
               'nav_t1_error_m': None, 'nav_t2_error_m': None,
               'box_pose_x': None, 'box_pose_y': None, 'box_pose_z': None}

        # 1. Navigate to Table 1
        ok = self.navigate_to(*TABLE1_NAV_GOAL, 'nav_T1')
        if not ok:
            self.get_logger().error(f'Navigation to T1 failed at iteration {i+1}')
            break

        # 2. Visual dock + get box pose
        box = self.get_box_pose()   # returns (x,y,z) or None
        if box is None:
            self.get_logger().warning(f'Box not detected at iteration {i+1} — PICK FAILED')
            log.append(row)
            break
        row['box_pose_x'], row['box_pose_y'], row['box_pose_z'] = box

        # 3. Pick
        pick_ok = self.grasp_and_retract(*box)
        row['pick_success'] = pick_ok
        if not pick_ok:
            self.get_logger().warning(f'Grasp FAILED at iteration {i+1}')
            log.append(row)
            break

        # 4. Navigate to Table 2
        self.go_named('travel')  # fold arm before driving
        ok = self.navigate_to(*TABLE2_NAV_GOAL, 'nav_T2')
        if not ok:
            self.get_logger().error(f'Navigation to T2 failed at iteration {i+1}')
            break

        # 5. Place on stack
        place_ok = self.place_on_stack(stack_level=i)
        row['place_success'] = place_ok

        log.append(row)
        self.get_logger().info(f'Iteration {i+1} complete. Pick={pick_ok}, Place={place_ok}')

    # Save log to CSV
    self._save_log(log)
    self.get_logger().info('=== STACKING EXPERIMENT COMPLETE ===')
```

---

### Step 3 — Data Collection: CSV Logger

```python
import csv, datetime

def _save_log(self, log: list):
    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    path = f'/home/omar/Desktop/Thesisorg/data/stacking_run_{ts}.csv'
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=log[0].keys())
        writer.writeheader()
        writer.writerows(log)
    self.get_logger().info(f'Log saved: {path}')
```

**Per-iteration data captured:**

| Column | Description |
|---|---|
| `iteration` | Cycle number (1-based) |
| `pick_success` | True/False |
| `place_success` | True/False |
| `box_pose_x/y/z` | Detected box pose from perception (camera) |
| `nav_t1_error_m` | Distance from T1 goal to actual robot pose (from odometry) |
| `nav_t2_error_m` | Distance from T2 goal to actual robot pose |

To get ground-truth error in simulation: compare nav final pose (from `/odom`) against
the programmed goal pose. Since `odom == world` in sim, this is perfect ground truth.
On hardware, compare against the map-frame pose published by AMCL.

---

### Step 4 — Box Source Management

Boxes on Table 1 must be pre-positioned. Two approaches:
1. **Simple**: spawn N boxes at known offsets on Table 1, pick one per iteration
   - Requires changing perception target each cycle (shift Y target by `i * BOX_WIDTH`)
2. **Realistic**: spawn all boxes overlapping, let them settle, pick the front one
   - The HSV detector naturally picks the nearest/largest contour (already works)

Approach 1 is used for experiment repeatability and precise error analysis.

---

## 4. Metrics and Analysis

### Primary Metric: Pick Success Rate vs. Iteration

Plot: X = iteration number, Y = pick success (binary scatter + rolling average)

Expected shapes:
- Simulation: flat at 100% (perfect odom, zero drift) → confirms pipeline is correct
- Hardware: degrades after N* iterations → quantifies the drift limit

### Secondary Metric: Navigation Position Error

After each `navigate_to(T1_GOAL)` call, read the actual robot pose from `/odom` or
`/amcl_pose`. Compute Euclidean distance to the goal:

```
nav_error_i = sqrt((actual_x - goal_x)^2 + (actual_y - goal_y)^2)
```

Plot: X = iteration, Y = nav_error_i — expect drift in real hardware

### Tertiary: Stack Z-Error

After each place, check placed box height (from Gazebo `/model_states` in sim, or
measured physically). Expected = `TABLE2_HEIGHT + i * BOX_HEIGHT + BOX_HEIGHT/2`.
Z-error = |measured - expected|.

---

## 5. Thesis Experiment Section Structure

When writing Chapter 5, results section, use this structure:

```
5.4 Multi-Cycle Stacking Experiment
    5.4.1 Setup (two tables, N=20 cycles, data collection method)
    5.4.2 Simulation Results (pick 100%, drift=0, baseline established)
    5.4.3 Hardware Results (pick degrades at iteration N*, drift profile)
    5.4.4 Sim-to-Real Comparison (drift in sim=0 vs hardware=measured)
    5.4.5 Discussion (what limits the system: nav drift, perception, arm reach?)
```

---

## 6. Implementation Order (Days)

| Day | Task |
|---|---|
| Day 1 | Add Table 2 to world file, add extra boxes to Table 1 |
| Day 2 | Implement `place_on_stack()` — test with arm only (no nav) |
| Day 3 | Implement `navigate_to()` generic goal, test two-table navigation |
| Day 4 | Implement `run_stacking()` loop + CSV logger |
| Day 5 | Full 5-iteration test in simulation — verify stack builds correctly |
| Day 6–7 | Run 20-iteration simulation experiment, collect data |
| Day 8 | Generate plots (matplotlib), validate data quality |
| Day 9+ | Hardware deployment |
