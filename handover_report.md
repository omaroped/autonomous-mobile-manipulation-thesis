# LIMO myCobot Mobile Manipulator Pick-and-Place: Expert Handover Report

This report provides a detailed overview of the master thesis project, what has been implemented, what is currently working, what bottlenecks remain, and how to resolve them.

---

## 1. Project Overview & Architecture

The objective of this project is to implement a complete, autonomous mobile manipulation pipeline using the **AgileX LIMO** mobile robot base, equipped with a **myCobot 280 M5** 6-DOF robotic arm and an adaptive gripper. The entire system is simulated in **Gazebo Classic** (using a Simple Warehouse environment) and planned/controlled via **ROS 2 Humble / MoveIt 2 / MoveIt Task Constructor (MTC)**.

The autonomous state machine is orchestrated by a coordinator node (`limo_cobot_bridge`) and follows this cycle:
```
[Start/Home] ──> Navigate to Pick Zone ──> Detect Object (Perception Server)
                     │
                     ▼
[End/Home] <── Navigate to Home <── Place Object <── Navigate to Place Zone <── Pick Object (MTC)
```

---

## 2. Important Files

Here are the key files in the workspace essential for understanding and resolving the pipeline's behavior:

### Coordination & Parameters
* **Bridge Orchestrator:** [`bridge_node.py`](file:///home/omar/Desktop/Thesis/src/ros2_ws/src/limo_cobot_bridge/limo_cobot_bridge/bridge_node.py) - Implements the main ROS 2 action/service client state machine.
* **Bridge Configuration:** [`bridge_params.yaml`](file:///home/omar/Desktop/Thesis/src/ros2_ws/src/limo_cobot_bridge/config/bridge_params.yaml) - Defines target coordinates, tolerances, and navigation zone waypoints.
* **MTC Task Parameters:** [`mtc_params.yaml`](file:///home/omar/Desktop/Thesis/src/ros2_ws/src/limo_cobot_bridge/config/mtc_params.yaml) - Contains link names, joint tolerances, gripper controllers, and search parameters.

### Perception & Object Detection
* **Perception Server Node:** [`get_planning_scene_server.cpp`](file:///home/omar/Desktop/Thesis/src/ros2_ws/src/mycobot_ros2/hello_mtc_with_perception/src/get_planning_scene_server.cpp) - Service server that processes point clouds to isolate support planes and identify target objects.
* **Segmentation Algorithms:** [`object_segmentation.cpp`](file:///home/omar/Desktop/Thesis/src/ros2_ws/src/mycobot_ros2/hello_mtc_with_perception/src/object_segmentation.cpp) - Contains the 2D projected line/circle RANSAC and Hough transform voting algorithms.
* **Perception Settings:** [`perception_params.yaml`](file:///home/omar/Desktop/Thesis/src/ros2_ws/src/limo_cobot_bridge/config/perception_params.yaml) - Configures crop boxes, normal/curvature estimation, Euclidean clustering, and RANSAC thresholds.

### Planning & Execution
* **MTC Planning Node:** [`mtc_node.cpp`](file:///home/omar/Desktop/Thesis/src/ros2_ws/src/mycobot_ros2/hello_mtc_with_perception/src/mtc_node.cpp) - Constructs the MTC stages (Generator, Propagator, Connectors) for Pick and Place trajectories.
* **Gazebo Launch:** [`gazebo.launch.py`](file:///home/omar/Desktop/Thesis/src/ros2_ws/src/limo_cobot_bringup/launch/gazebo.launch.py) - Launches the Gazebo world, spawns the robot and objects.
* **MoveIt Launch:** [`gazebo_moveit.launch.py`](file:///home/omar/Desktop/Thesis/src/ros2_ws/src/limo_cobot_moveit_config/launch/gazebo_moveit.launch.py) - Launches MoveIt 2 controllers and planning scene monitors.

---

## 3. What is Working (Going Well)

1. **Workspace & Compilation:**
   * Sourcing `/opt/ros/humble/setup.bash` and `install/setup.bash` yields a **100% clean compilation** with `colcon build --symlink-install`.
2. **Stable Simulation Launch:**
   * Custom launch wrapper with `LIBGL_ALWAYS_SOFTWARE=1` prevents Gazebo and RViz OpenGL crashes.
   * Node launches successfully, and physics unpauses automatically.
3. **Robot URDF & Mesh Visibility:**
   * Mesh scales and descriptions for the `mycobot_280` and the adaptive gripper have been resolved. The complete combined robot renders properly in Gazebo and RViz.
4. **Perception Support Plane Segmentation:**
   * The camera point cloud from `/depth/points` is successfully received.
   * Ground/support plane segmentation works properly, isolating the floor/table (z-tolerance expanded to 0.35 to account for LIMO's `base_link` height offset of ~15cm).
   * The support surface collision object is successfully added to the MoveIt planning scene.

---

## 4. What is Wrong (The Current Bottlenecks)

### Perception Server Fails to Detect Objects (`Segmented 0 objects`)
Although the plane is segmented, object classification fails because the code extracts **0 line/circle models** from the clustered point clouds. This is caused by three distinct issues in [`object_segmentation.cpp`](file:///home/omar/Desktop/Thesis/src/ros2_ws/src/mycobot_ros2/hello_mtc_with_perception/src/object_segmentation.cpp):

1. **Double Mapping Bug:**
   * In `fitLineRANSAC` / `fitCircleRANSAC`, indices are already mapped to original cluster indices via `projection_map.at(i)`.
   * In `filterLineInliers` / `filterCircleInliers`, the code performs `projection_map.at(idx)` *again*. This causes index lookup corruption or `std::out_of_range` exceptions when points are removed.
2. **Incorrect Point Removal in Iterative RANSAC:**
   * When removing inliers to search for the next model:
     `if (inliers_to_remove.find(i) == inliers_to_remove.end())`
     It checks `i` (the index in `projected_cloud`) against `inliers_to_remove` (which contains original indices in the 3D `cluster`). These indices do not align, leading to incorrect point removal.
   * `projection_map` is never rebuilt or updated as points are removed, meaning shifted indices in `projected_cloud` map to wrong points in the original cloud.
3. **Restrictive RANSAC & Filtering Parameters:**
   * `ransac_distance_threshold` is set to `0.001` (1 mm) in [`perception_params.yaml`](file:///home/omar/Desktop/Thesis/src/ros2_ws/src/limo_cobot_bridge/config/perception_params.yaml). Depth noise in simulation prevents points on a small 3cm box from fitting a line within a 1mm tolerance.
   * `line_curvature_threshold` (`0.0011`) is extremely tight.
   * The filter functions immediately reject a line/circle if Euclidean clustering returns more than `line_max_clusters` (`2`). Noise can easily fragment a single edge into 3 small clusters, discarding valid models.

---

## 5. Proposed Next Steps Plan

1. **Fix the Perception Mapping Code:**
   * Refactor `filterLineInliers` and `filterCircleInliers` to bypass the redundant `projection_map.at(idx)` lookup (since indices are already mapped in `fitLineRANSAC` / `fitCircleRANSAC`).
   * Fix the RANSAC inner loop to correctly track shifted indices or rebuild `projection_map` during point removal.
2. **Tune Perception Thresholds:**
   * Increase `ransac_distance_threshold` to `0.005` or `0.01` (5–10 mm) to absorb simulation sensor noise.
   * Relax `line_curvature_threshold` and `line_max_clusters`.
3. **Verify Planning Scene Insertion:**
   * Verify that `/get_planning_scene_mycobot` returns `success: true` and correctly identifies the target object.
4. **End-to-End Pipeline Execution:**
   * Execute the stationary pick-and-place task, then transition to full integrated mobile navigation.

---

## 6. Prompt for Future AI/Expert

Copy and paste the prompt below into a new chat session to continue working:

```text
I am developing an autonomous pick-and-place pipeline in ROS 2 Humble for a LIMO mobile manipulator equipped with a myCobot 280 arm.
The simulation is running in Gazebo Classic. The current roadblock is that the perception service (/get_planning_scene_mycobot) fails with "success: false" and "Segmented 0 objects from the point cloud clusters".

I have located index mapping bugs and overly restrictive parameters in the custom object segmentation logic in get_planning_scene_server.cpp and object_segmentation.cpp (in hello_mtc_with_perception).

Please:
1. Examine object_segmentation.cpp to resolve the double mapping bug where projection_map is applied multiple times, and fix the index tracking/removal logic when filtering out inliers between RANSAC iterations.
2. Review and relax the thresholds in perception_params.yaml (e.g., ransac_distance_threshold = 0.001 is too tight for simulated depth data; line_curvature_threshold and cluster limits are too restrictive).
3. Test and verify the perception service via:
   ros2 service call /get_planning_scene_mycobot mycobot_interfaces/srv/GetPlanningScene "{target_shape: 'box', target_dimensions: [0.03, 0.03, 0.03]}"
4. Once perception works, assist in executing the pick-and-place state machine orchestrated by bridge_node.py.
```
