# Perception Pipeline — Verification & Troubleshooting Guide

## 0. Apply the Fixes

```bash
# Copy fixed files into your workspace (adjust paths as needed)
cp object_segmentation.cpp \
   ~/Desktop/Thesis/src/ros2_ws/src/mycobot_ros2/hello_mtc_with_perception/src/

cp perception_params.yaml \
   ~/Desktop/Thesis/src/ros2_ws/src/limo_cobot_bridge/config/

# Rebuild only the affected packages
cd ~/Desktop/Thesis/src/ros2_ws
colcon build --symlink-install \
  --packages-select hello_mtc_with_perception mycobot_interfaces
source install/setup.bash
```

---

## 1. Confirm Point-Cloud Data is Flowing

```bash
# Check that the camera is publishing points
ros2 topic hz /depth/points
# Expected: ~15-30 Hz

ros2 topic echo /depth/points --no-arr --once
# Expected: header with frame_id set, width/height > 0
```

If `/depth/points` is empty or not publishing, check the camera plugin in
your Gazebo world/URDF — depth cameras in Gazebo Classic need the
`libgazebo_ros_depth_camera.so` plugin with `<pointCloudTopicName>`.

---

## 2. Verify the Perception Server is Running

```bash
ros2 node list | grep get_planning_scene
# Expected: /get_planning_scene_server

ros2 service list | grep mycobot
# Expected: /get_planning_scene_mycobot
```

If the service is missing, launch the server:
```bash
ros2 run hello_mtc_with_perception get_planning_scene_server \
  --ros-args --params-file \
  ~/Desktop/Thesis/src/ros2_ws/src/limo_cobot_bridge/config/perception_params.yaml
```

---

## 3. Call the Service and Inspect the Response

```bash
ros2 service call /get_planning_scene_mycobot \
  mycobot_interfaces/srv/GetPlanningScene \
  "{target_shape: 'box', target_dimensions: [0.03, 0.03, 0.03]}"
```

### What to look for in the terminal running the server:

**Good signs:**
```
Plane and object segmentation successful
Support plane cloud size: XXXX
Objects cloud size: XXXX          ← must be > 0
Successfully estimated normals... ← must succeed
Node '...' successfully extracted N clusters  ← N > 0
Clustered models:
  Line clusters:   N              ← N > 0 for a box
  Circle clusters: N
Selected top model:
  Type: Line  Votes: ...
Added box: id=box_0 ...
===== Service response filled successfully =====
```

**Failure signs and fixes:**

| Log message | Root cause | Fix |
|---|---|---|
| `Objects cloud size: 0` | Plane segmentation consuming object pts | Lower `z_tolerance`; check `crop_min_z` |
| `failed to extract any clusters` | `min_cluster_size` too large | Lower to 30–50 |
| `Clustered models: Line 0 / Circle 0` | `inlier_threshold` too high or `ransac_distance_threshold` too small | Already fixed in params; verify params are loaded |
| `success: false` + `target_object_id empty` | Object detected but shape mismatch | Check `target_shape` matches what the algorithm outputs |

---

## 4. Visualize in RViz (optional but highly recommended)

Add these displays in RViz:

```
Fixed Frame: base_link

PointCloud2  → /depth/points          (raw input)
MarkerArray  → /planning_scene         (collision objects)
```

To publish the planning scene from the service response into MoveIt:

```bash
# In a separate terminal — apply the returned scene
ros2 service call /apply_planning_scene moveit_msgs/srv/ApplyPlanningScene \
  "$(ros2 service call /get_planning_scene_mycobot \
     mycobot_interfaces/srv/GetPlanningScene \
     "{target_shape: 'box', target_dimensions: [0.03, 0.03, 0.03]}" \
     | python3 -c 'import sys,yaml; d=yaml.safe_load(sys.stdin.read()); print(d["scene_world"])')"
```

Or use the PCD debug files that the server writes to `/tmp/`:
```bash
# Visualise intermediate clouds
ros2 run pcl_ros pcd_to_pointcloud \
  --ros-args -p file_name:=/tmp/5_objects_cloud_debug_cloud.pcd \
             -p frame_id:=base_link -p interval:=1.0
```

Check that `/tmp/5_objects_cloud_debug_cloud.pcd` actually contains the
object points (not just an empty file). If it is empty, the plane
segmentation is eating the object — reduce `z_tolerance`.

---

## 5. Iterative Parameter Tuning (if still getting 0 objects)

Work through these in order — each step builds on the previous:

### Step 5a — Find the right `inlier_threshold`
```bash
# Count how many points a cluster actually has:
# Look for "successfully extracted N clusters" in the server log.
# Each cluster size is printed just before object segmentation.
# Set inlier_threshold to ~20% of the smallest cluster size.
```

### Step 5b — Find the right `ransac_distance_threshold`
```bash
# Save the objects cloud from /tmp/5_objects_cloud_debug_cloud.pcd
# Open in CloudCompare (free tool). Select points on a flat face of the box.
# Compute the RMS distance to a fitted plane → that value is your noise floor.
# Set ransac_distance_threshold to 2× that value.
```

### Step 5c — Verify curvature values
```bash
# Flat surfaces should have curvature ≈ 0.002-0.010 in simulation.
# If line_curvature_threshold is below the actual curvature, all points
# are rejected. The fix sets it to 0.010 which handles this.
```

### Step 5d — Check cluster fragmentation
```bash
# If line_max_clusters=1 and noise breaks a single box edge into 2 clusters,
# the model is rejected. The fix raises this to 3.
```

---

## 6. End-to-End State Machine Test

Once `success: true` is confirmed from the service:

```bash
# Terminal 1 — Gazebo + MoveIt
ros2 launch limo_cobot_bringup gazebo.launch.py
ros2 launch limo_cobot_moveit_config gazebo_moveit.launch.py

# Terminal 2 — Perception server
ros2 run hello_mtc_with_perception get_planning_scene_server \
  --ros-args --params-file \
  ~/Desktop/Thesis/src/ros2_ws/src/limo_cobot_bridge/config/perception_params.yaml

# Terminal 3 — Bridge (state machine)
ros2 run limo_cobot_bridge bridge_node \
  --ros-args --params-file \
  ~/Desktop/Thesis/src/ros2_ws/src/limo_cobot_bridge/config/bridge_params.yaml
```

Watch the bridge log for state transitions:
```
State: INIT
State: NAV_TO_PICK
State: DETECT
MTC perception: box_0 at (X, Y, Z)   ← perception working
State: PICK
...
Bridge finished: COMPLETE
```

---

## 7. Summary of All Changes Made

### `object_segmentation.cpp` — 3 bugs fixed

**Bug 1 — Double projection_map lookup (silent data corruption)**
- `fitLineRANSAC` and `fitCircleRANSAC` now store **2D projected_cloud indices** only.
  The `projection_map` parameter is kept in the signature for API compatibility but
  is no longer used inside these functions.
- `filterLineInliers` and `filterCircleInliers` now perform the **single correct**
  `projection_map.at(idx_2d)` lookup to reach the original 3D point.

**Bug 2 — Inlier removal compared wrong index spaces**
- `inliers_to_remove` is now built as a set of **2D projected_cloud indices**
  (by inverting the projection_map over the 3D removal set).
- The removal loop correctly compares `i` (2D loop counter) against this set.

**Bug 3 — Stale projection_map after point removal**
- After compacting `projected_cloud`, a **new `projection_map` is built from scratch**
  mapping new 2D indices → original 3D indices.
- The initial map is saved and restored at the start of each outer iteration.

### `perception_params.yaml` — thresholds relaxed for simulation

| Parameter | Old | New | Reason |
|---|---|---|---|
| `ransac_distance_threshold` | 0.001 | 0.008 | Gazebo depth noise is 3–8 mm |
| `line_curvature_threshold` | 0.0011 | 0.010 | Flat box faces have curvature ~0.002–0.008 |
| `inlier_threshold` | 85 | 30 | A 3 cm box has far fewer than 85 pts per face |
| `line_max_clusters` | 2 | 3 | Noise fragments edges into 3 pieces |
| `circle_max_clusters` | 2 | 3 | Same reason |
| `min_cluster_size` | 100 | 50 | Small objects produce fewer points |
| `num_iterations` | 5 | 8 | More iterations → more Hough votes → better winner selection |
| `curvature_threshold` (region growing) | 0.2 | 0.5 | Don't reject object surfaces |
| `smoothness_threshold` | 20.0 | 25.0 | Allow slightly more surface variation |
