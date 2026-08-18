"""
nav_pick.launch.py — ONE COMMAND to run the full autonomous pick-and-place.

Starts (in dependency order):
  1. Gazebo simulation  (ackermann_gazebo.launch.py — sim + robot + controllers)
  2. MoveIt move_group  (moveit.launch.py)
  3. Nav2 stack         (nav2_limo.launch.py — map, AMCL, planner, controller)
  4. box_pose_estimator (perception → /box_pose)
  5. grasp_attacher     (LEGACY kinematic weld — only if use_physics_grasp:=false)
  6. base_pin           (Gazebo pose-hold service)
  7. nav_pick_orchestrator (the mission node — waits for everything above)

RViz is launched with nav_pick.rviz (all required displays pre-configured).

Usage:
    ros2 launch limo_car nav_pick.launch.py
"""

import os
import re

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetLaunchConfiguration
)
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


class SceneGeometryError(RuntimeError):
    """The scene is not physically achievable, or the world file disagrees."""


def load_scene(pkg):
    """Read config/scene.yaml, derive the docking geometry, and validate it.

    This is the ONLY place docking distances are computed. Everything the
    orchestrator and the estimator need is derived here and passed down as ROS
    parameters, so the geometry cannot drift out of sync between them again.

    Raises SceneGeometryError if the scene cannot physically work, rather than
    letting the robot discover it by pushing a table for 30 s or throwing the
    box across the room.
    """
    with open(os.path.join(pkg, 'config', 'scene.yaml')) as f:
        s = yaml.safe_load(f)

    rb, pt, kt = s['robot'], s['place_table'], s['pickup_table']
    bumper, reach, clear = rb['bumper_x'], rb['arm_reach'], rb['bumper_clearance']
    ground_z = rb['base_link_ground_z']
    side = pt['side']

    # ── Derive ───────────────────────────────────────────────────────────────
    # Stop with base_link one arm-reach from the table centre, so the arm can
    # place AT the centre rather than being silently clamped short of it.
    dock_range   = reach - side / 2.0        # base_link -> tag (tag is on the face)
    place_y_dock = pt['centre_map'][1] + reach   # map y where the robot stops
    stop_dist    = reach                     # pickup dock: base_link -> box

    # Table top height, in base_link frame, for the arm's z target.
    # WORLD top_z - base_link_ground_z. The two tables have different world
    # heights (0.14 pickup, 0.10 place), so this MUST be computed per table.
    # Bug found 2026-08-04: place_box() was reusing the PICKUP table's
    # perceived surface height for the PLACE table (no perception happens at
    # place time) -- releasing the box ~6 cm above a table only 8x8 cm across,
    # which bounced/rolled it off onto the floor. Both affected runs still
    # reported place_success=True: metrics_20260803_195910.csv (err_xy 10 cm)
    # and metrics_20260803_195111.csv (err_xy 89 cm), both with box world
    # z=0.020 -- resting on the floor, not the table.
    place_surface_base_z  = pt['top_z'] - ground_z
    pickup_surface_base_z = kt['top_z'] - ground_z

    # ── Validate ─────────────────────────────────────────────────────────────
    if dock_range < bumper + clear:
        raise SceneGeometryError(
            f"place_table.side = {side:.3f} m is too large.\n"
            f"  dock_range = arm_reach - side/2 = {dock_range:.3f} m\n"
            f"  but the bumper needs at least {bumper:+.3f} + {clear:.3f} = "
            f"{bumper + clear:.3f} m.\n"
            f"  The robot would be commanded INSIDE the table.\n"
            f"  Max feasible side = {2 * (reach - bumper - clear):.3f} m.")

    kt_face = kt['depth'] / 2.0
    if stop_dist - kt_face < bumper + clear:
        raise SceneGeometryError(
            f"pickup_table.depth = {kt['depth']:.3f} m is too large: the bumper "
            f"would sit {bumper + clear - (stop_dist - kt_face):.3f} m inside it.")

    # ── Cross-check against the world file, which Gazebo actually loads ──────
    # scene.yaml cannot drive Gazebo, so the two must be kept in step by hand.
    # Catch the mismatch here instead of three phases into a run.
    world = open(os.path.join(pkg, 'worlds', 'final_map.world')).read()
    m = re.search(r"<model name='place_table'>.*?<box><size>([\d.]+) ([\d.]+) ([\d.]+)</size>",
                  world, re.S)
    if m:
        wx, wy, wz = (float(v) for v in m.groups())
        if abs(wx - side) > 1e-6 or abs(wy - side) > 1e-6 or abs(wz - pt['top_z']) > 1e-6:
            raise SceneGeometryError(
                f"scene.yaml and final_map.world disagree about place_table:\n"
                f"  scene.yaml : {side} x {side} x {pt['top_z']}\n"
                f"  world file : {wx} x {wy} x {wz}\n"
                f"  Change BOTH together.")

    s['derived'] = {
        'dock_range':             dock_range,
        'place_y_dock':           place_y_dock,
        'stop_distance':          stop_dist,
        'bumper_gap':             dock_range - bumper,
        'place_surface_base_z':   place_surface_base_z,
        'pickup_surface_base_z':  pickup_surface_base_z,
    }
    print(f"[scene] place table {side*100:.0f} cm square, top {pt['top_z']*100:.0f} cm | "
          f"dock_range {dock_range:.3f} m | bumper gap {dock_range - bumper:.3f} m | "
          f"arm reaches table centre ✓")
    print(f"[scene] place surface at {place_surface_base_z:+.4f} m in base_link "
          f"(was silently reusing pickup's {pickup_surface_base_z:+.4f} m — "
          f"a {abs(place_surface_base_z - pickup_surface_base_z)*100:.1f} cm error)")
    return s


def generate_launch_description():
    pkg      = get_package_share_directory('limo_car')
    moveit   = get_package_share_directory('limo_cobot_moveit_config')
    scene    = load_scene(pkg)
    place    = scene['place_table']
    pickup   = scene['pickup_table']
    derived  = scene['derived']

    # 2026-08-10: derived['dock_range'] leaves only ~1.1 cm of bumper clearance
    # (dock_range - bumper_x), just 1 mm above load_scene()'s own hard-coded
    # minimum (bumper_x + bumper_clearance). That margin is smaller than one
    # control tick's travel at PLACE_DOCK_MAX_FWD, so the chassis was hitting the
    # table. Pad the ACTUAL commanded stop distance by 1 cm without touching the
    # scene-derived value load_scene() validates -- the box lands ~1 cm off dead
    # centre (toward the robot) instead, well within the 8x8 cm table for a 4x4 cm box.
    PLACE_DOCK_EXTRA_CLEARANCE = 0.01
    dock_range_padded = derived['dock_range'] + PLACE_DOCK_EXTRA_CLEARANCE

    spawn_y = LaunchConfiguration('spawn_y', default='7.0')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_gzclient = LaunchConfiguration('use_gzclient', default='true')
    drive_mode = LaunchConfiguration('drive_mode', default='diff')
    calib_loops = LaunchConfiguration('calib_loops', default='0')
    place_stack_levels = LaunchConfiguration('place_stack_levels', default='1')
    # Derived in load_scene() from arm_reach and place_table.side. Overridable
    # on the command line for experiments, but the default is always consistent
    # with the scene -- it can no longer be a stale literal.
    dock_range = LaunchConfiguration('dock_range', default=str(dock_range_padded))
    metrics_csv = LaunchConfiguration('metrics_csv', default='')
    use_physics_grasp = LaunchConfiguration('use_physics_grasp', default='true')
    base_pin_enabled = LaunchConfiguration('base_pin_enabled', default='true')

    # ── Argument declarations ─────────────────────────────────────────────────
    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y', default_value='7.0',
        description='Robot spawn Y. Default 7.0 → 3 m from the table (nav problem).')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Whether to launch RViz')

    use_gzclient_arg = DeclareLaunchArgument(
        'use_gzclient', default_value='true',
        description='Whether to launch Gazebo client (GUI)')

    drive_mode_arg = DeclareLaunchArgument(
        'drive_mode', default_value='diff',
        description='Drive mode (ackermann | diff)')

    calib_loops_arg = DeclareLaunchArgument(
        'calib_loops', default_value='0',
        description='0 = run once (normal demo). N>0 = calibration mode: after docking, '
                    'repeat the grasp N times, auto-resetting the box, reading grasp_off_* '
                    'params live each iteration so the offset can be tuned without relaunching.')

    place_stack_levels_arg = DeclareLaunchArgument(
        'place_stack_levels', default_value='1',
        description='Number of boxes to pick+stack at the place table in one run '
                    '(the world has 3 distinct stack_box_N boxes on the pickup table).')

    dock_range_arg = DeclareLaunchArgument(
        'dock_range', default_value=str(dock_range_padded),
        description=f"AprilTag dock stop distance (m). DERIVED from config/scene.yaml "
                    f"as arm_reach - place_table.side/2 = {derived['dock_range']:.3f}, "
                    f"plus a {PLACE_DOCK_EXTRA_CLEARANCE*1000:.0f} mm safety pad added "
                    f"2026-08-10 (the raw derived value left only "
                    f"{derived['bumper_gap']*1000:.0f} mm of bumper clearance -- too "
                    f"tight, the chassis was hitting the table). Effective bumper gap "
                    f"now ~{(derived['bumper_gap'] + PLACE_DOCK_EXTRA_CLEARANCE)*1000:.0f} mm. "
                    f"Do not hardcode: a value below "
                    f"{scene['robot']['bumper_x']:.3f} m is physically unreachable and "
                    f"the robot will grind its wheels against the table.")

    metrics_csv_arg = DeclareLaunchArgument(
        'metrics_csv', default_value='',
        description='Path for the per-cycle metrics CSV. Empty = auto-name under data/.')

    use_physics_grasp_arg = DeclareLaunchArgument(
        'use_physics_grasp', default_value='true',
        description='true  = grasping is done by libgazebo_grasp_plugin (a REAL ODE fixed '
                    'joint created on finger contact, released when the gripper opens past '
                    'release_position). grasp_attacher and smart_grasp are NOT started.\n'
                    'false = legacy kinematic weld: grasp_attacher teleports the box to '
                    'follow the gripper via /set_entity_state, driven by /grasp_attach.\n'
                    'The two mechanisms MUST NOT run together — the teleport fights the '
                    'joint solver. This one flag switches both the launch graph and the '
                    "orchestrator's release logic, so they cannot disagree.")

    base_pin_enabled_arg = DeclareLaunchArgument(
        'base_pin_enabled', default_value='true',
        description='true = base is teleport-held at 50 Hz during arm motion (default). '
                    'false = pin_base() is a no-op — experiment to see whether/how much '
                    'the base actually creeps under arm reaction forces with nothing '
                    'holding it.')

    # Preserve the real top-level use_rviz BEFORE the gazebo include can touch it.
    # ackermann_gazebo.launch.py declares its OWN 'use_rviz' argument too, and the
    # 'use_rviz': 'false' passed into it below overwrites the shared launch-context
    # value for that name — LaunchConfiguration substitutions resolve lazily, so
    # moveit_launch's read of 'use_rviz' 8 s later was silently picking up 'false'
    # instead of this top-level flag, and RViz never launched. Confirmed via a
    # minimal reproduction (same TimerAction pattern alone launched RViz fine —
    # only broke once the gazebo include with its own 'use_rviz' arg was added).
    preserve_use_rviz = SetLaunchConfiguration('moveit_use_rviz', use_rviz)

    # ── 1. Gazebo + robot + controllers ──────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'ackermann_gazebo.launch.py')),
        launch_arguments={
            'spawn_y': spawn_y,
            'use_rviz': 'false',
            'use_gzclient': use_gzclient,
            'drive_mode': drive_mode,
        }.items())


    # ── 2. MoveIt (move_group) — delay 8 s so controllers come up first ──────
    moveit_launch = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit, 'launch', 'moveit.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'use_rviz': LaunchConfiguration('moveit_use_rviz'),
                'rviz_config': os.path.join(pkg, 'rviz', 'nav_monitor.rviz'),
            }.items())])

    # ── 3. Nav2 — delay 12 s so the robot is spawned + controllers ready ─────
    nav2_launch = TimerAction(
        period=12.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'nav2_limo.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'drive_mode': drive_mode,
            }.items())])

    # ── 4. box_pose_estimator — delay 15 s ───────────────────────────────────
    box_estimator = TimerAction(
        period=15.0,
        actions=[Node(
            package='limo_car',
            executable='box_pose_estimator',
            name='box_pose_estimator',
            output='screen',
            parameters=[{'use_sim_time': True}])])

    # ── 4b. tag_dock_estimator — delay 15 s (parallel with box_estimator) ────
    # Detects AprilTag 36h11 on the place table, publishes /place_tag_pose and
    # /place_drop_point. Requires the same camera topics as box_pose_estimator.
    tag_estimator = TimerAction(
        period=15.0,
        actions=[Node(
            package='limo_car',
            executable='tag_dock_estimator',
            name='tag_dock_estimator',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                # ALL from config/scene.yaml -- never write literals here. The
                # drop point is tag_position - normal * (table_side/2), so a
                # stale table_side aims the arm past the table entirely.
                'tag_size':    place['tag_size'],
                'table_side':  place['side'],
                'table_top_z': place['top_z'],
            }])])

    # ── 5. grasp_attacher — LEGACY, only when use_physics_grasp:=false ───────
    #    The kinematic weld. Superseded by libgazebo_grasp_plugin (declared in
    #    gazebo/mycobot_ros2_control.xacro), which makes a real ODE fixed joint.
    #    Kept as a working fallback, NOT deleted: if the physics joint misbehaves,
    #    `use_physics_grasp:=false` restores the exact pipeline that produced the
    #    two successful runs on 2026-07-02.
    #
    #    Why it cannot run alongside the plugin: it calls /set_entity_state at
    #    ~50 Hz to teleport the box onto the gripper, while ODE solves the fixed
    #    joint at 1000 Hz. Both would write the box pose every step and fight.
    grasp_attacher = TimerAction(
        period=15.0,
        condition=UnlessCondition(use_physics_grasp),
        actions=[Node(
            package='limo_car',
            executable='grasp_attacher',
            name='grasp_attacher',
            output='screen',
            parameters=[{'use_sim_time': True}])])

    # ── 6. smart_grasp — LEGACY, only when use_physics_grasp:=false ──────────
    #    Watches the finger contact sensors and publishes /grasp_attach True when
    #    both fingers touch — i.e. it is the *trigger* for the weld above, and is
    #    meaningless without it. The C++ plugin does contact detection internally
    #    (grasp_count_threshold), so this node is redundant under physics grasp.
    smart_grasp = TimerAction(
        period=15.0,
        condition=UnlessCondition(use_physics_grasp),
        actions=[Node(
            package='limo_car',
            executable='smart_grasp',
            name='smart_grasp',
            output='screen',
            parameters=[{'use_sim_time': True}])])

    # ── 7. nav_pick_orchestrator — delay 25 s (Nav2 fully active) ────────────
    orchestrator = TimerAction(
        period=25.0,
        actions=[Node(
            package='limo_car',
            executable='nav_pick_orchestrator',
            name='nav_pick_orchestrator',
            output='screen',
            parameters=[{'use_sim_time': True,
                          'calib_loops': ParameterValue(calib_loops, value_type=int),
                          'place_stack_levels': ParameterValue(place_stack_levels, value_type=int),
                          'dock_range': ParameterValue(dock_range, value_type=float),
                          'metrics_csv': metrics_csv,
                          # ── scene geometry, all from config/scene.yaml ──────
                          # These replace TABLE_MAP_X/Y, PLACE_MAP_Y_DOCK,
                          # NAV_PLACE_X/Y/YAW and STOP_DISTANCE, which used to be
                          # module constants duplicating the world file.
                          'table_map_x':      float(place['centre_map'][0]),
                          'table_map_y':      float(place['centre_map'][1]),
                          'place_map_y_dock': float(derived['place_y_dock']),
                          'nav_place_x':      float(place['nav_approach'][0]),
                          'nav_place_y':      float(place['nav_approach'][1]),
                          'nav_place_yaw':    float(place['nav_approach'][2]),
                          'nav_goal_x':       float(pickup['nav_approach'][0]),
                          'nav_goal_y':       float(pickup['nav_approach'][1]),
                          'nav_goal_yaw':     float(pickup['nav_approach'][2]),
                          'stop_distance':    float(derived['stop_distance']),
                          # base_link-frame table surface heights (see load_scene).
                          # place_box() has no live perception of the place table --
                          # unlike pickup, nothing looks at it before release -- so
                          # this scene-derived constant IS the source of truth, not
                          # a fallback.
                          'place_surface_base_z':  float(derived['place_surface_base_z']),
                          'pickup_surface_base_z': float(derived['pickup_surface_base_z']),
                          # World-frame top_z, for comparing against Gazebo ground
                          # truth in the metrics harness (world frame, not base_link).
                          'place_table_top_z_world': float(place['top_z']),
                          # Same flag that gates the two legacy nodes above, so the
                          # orchestrator's release logic can never disagree with which
                          # grasp mechanism is actually running.
                          'use_physics_grasp': ParameterValue(
                              use_physics_grasp, value_type=bool),
                          'base_pin_enabled': ParameterValue(
                              base_pin_enabled, value_type=bool)}])])

    return LaunchDescription([
        spawn_y_arg,
        use_rviz_arg,
        use_gzclient_arg,
        drive_mode_arg,
        calib_loops_arg,
        place_stack_levels_arg,
        dock_range_arg,
        metrics_csv_arg,
        use_physics_grasp_arg,
        base_pin_enabled_arg,
        preserve_use_rviz,
        gazebo,
        moveit_launch,
        nav2_launch,
        box_estimator,
        tag_estimator,
        grasp_attacher,
        smart_grasp,
        orchestrator,
    ])
