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
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetLaunchConfiguration,
    OpaqueFunction
)
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


class SceneGeometryError(RuntimeError):
    """The scene is not physically achievable, or the world file disagrees."""


class LocalizationConfigError(RuntimeError):
    """The localization and odometry settings cannot work together."""


def validate_localization(localization, odometry_source):
    """Refuse configurations that degrade silently instead of failing.

    ENCODER odometry (odometry_source=0) integrates real wheel rotation, so /odom
    drifts exactly as it does on hardware. Ground-truth localization publishes a
    FIXED zero map->odom transform, which is only correct while odom cannot drift.
    Put the two together and the drift accumulates behind a frozen transform with
    nothing to correct it: the robot's believed map pose separates from reality,
    slowly, and navigation goals go progressively wrong. Nothing errors -- the run
    just gets worse the longer it lasts, which is the most expensive kind of bug to
    chase.

    AMCL is what makes ENCODER usable: it matches the laser against the map and
    corrects map->odom as the drift accumulates. So ENCODER is allowed only with
    AMCL, and that pairing is the configuration that actually rehearses hardware.
    """
    if str(odometry_source) == '0' and localization != 'amcl':
        raise LocalizationConfigError(
            f"odometry_source:=0 (ENCODER) requires localization:=amcl, "
            f"but localization is '{localization}'.\n"
            f"  ENCODER odometry drifts like the real robot. Ground-truth "
            f"localization publishes a FIXED map->odom transform and cannot correct "
            f"that drift, so the robot's believed position would separate from "
            f"reality with nothing to catch it.\n"
            f"  For the honest hardware rehearsal:\n"
            f"    ros2 launch limo_car nav_pick.launch.py "
            f"localization:=amcl odometry_source:=0\n"
            f"  For the default perfect-localization sim, leave both unset.")


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
    # The box no longer sits at the table centre -- pickup_table.box_offset moves it
    # toward the robot -- so the arm's target is that much closer than full reach.
    # Previously this line read `stop_dist = reach`, which commanded every grasp at
    # the arm's measured maximum (see the box_offset note in scene.yaml).
    box_offset   = kt.get('box_offset', 0.0)
    stop_dist    = reach - box_offset        # pickup dock: base_link -> box

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

    # Distance from base_link to the table's NEAR FACE at the dock.
    #
    # stop_dist is base_link -> BOX, and the box is box_offset in front of the table
    # centre, so the centre is at stop_dist + box_offset and the near face is half a
    # table depth in front of that. With box_offset = 0 this reduces to the original
    # `stop_dist - depth/2`. Getting this wrong the other way would silently approve a
    # dock that drives the chassis into the table.
    #
    # Note the dock itself is UNCHANGED by box_offset: the approach is box-relative,
    # so moving the box toward the robot moves the robot's stopping point toward it by
    # the same amount, and the bumper gap stays exactly what it was (11 mm).
    kt_face = kt['depth'] / 2.0 - box_offset
    if stop_dist - kt_face < bumper + clear:
        raise SceneGeometryError(
            f"pickup dock puts the bumper inside the table.\n"
            f"  box at {stop_dist:.3f} m, table near face at "
            f"{stop_dist - kt_face:.3f} m, bumper needs "
            f"{bumper + clear:.3f} m.\n"
            f"  Reduce pickup_table.depth or pickup_table.box_offset.")

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

    sp = rb.get('spawn_pose', {'x': -2.0, 'y': 7.0, 'yaw': -1.5708})

    s['derived'] = {
        'spawn_x':                float(sp['x']),
        'spawn_y':                float(sp['y']),
        'spawn_yaw':              float(sp['yaw']),
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


def _check_localization_config(context, *args, **kwargs):
    """Resolve the two settings and validate them before anything starts.

    Has to be an OpaqueFunction: LaunchConfiguration values do not exist while the
    launch description is being built, only once the context is populated. Raising
    here aborts the launch cleanly, before Gazebo or Nav2 come up.
    """
    validate_localization(
        LaunchConfiguration('localization').perform(context),
        LaunchConfiguration('odometry_source').perform(context))
    return []


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
    # 0.0 since 2026-08-25 (was 0.01). scene.yaml derives dock_range = arm_reach -
    # side/2 = 0.200 m precisely so the arm can reach the table CENTRE; the 10 mm pad
    # pushed the commanded dock to 0.210, which puts the drop point BEYOND arm_reach
    # and violates the constraint scene.yaml validates against. reach_map.csv shows
    # that at place height (z = -0.025) and the observed lateral offset, essentially
    # one column (x = 0.22) is reachable -- so the padded target sat outside the
    # measured workspace, and the place retract had no near IK branch to return
    # (measured 2026-08-25: all 3 samples came back 2.189 rad from the arm's actual
    # configuration, the move was refused, and the Cartesian fallback built a
    # 246-waypoint 41.5 rad path).
    #
    # The bumper clearance the pad was protecting is restored by biasing the DROP
    # POINT inward instead (see PLACE_NEAR_EDGE_BIAS in the orchestrator), which
    # moves the arm's target without moving the chassis closer to the table.
    PLACE_DOCK_EXTRA_CLEARANCE = 0.0
    dock_range_padded = derived['dock_range'] + PLACE_DOCK_EXTRA_CLEARANCE

    # Defaults come from scene.yaml's robot.spawn_pose, so there is one place to
    # change the start pose and the Gazebo spawn, AMCL's seed and everything derived
    # from it move together. Override all three on the command line to test off-spawn
    # behaviour:  ros2 launch limo_car nav_pick.launch.py spawn_y:=5.5
    spawn_x   = LaunchConfiguration('spawn_x',   default=str(derived['spawn_x']))
    spawn_y   = LaunchConfiguration('spawn_y',   default=str(derived['spawn_y']))
    spawn_yaw = LaunchConfiguration('spawn_yaw', default=str(derived['spawn_yaw']))
    # Which localization the whole pipeline runs. This was NOT passed through to
    # nav2_limo.launch.py until 2026-08-25, so the include always took that file's
    # own default (ground_truth) and AMCL was unreachable from the full pipeline --
    # every navigation result so far was measured with a perfect map->odom transform.
    localization = LaunchConfiguration('localization', default='ground_truth')
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
    # Which box to grasp when several are visible: 'rightmost' (deterministic,
    # correct for multi-box stacking) or 'nearest' (the old single-box default).
    target_policy = LaunchConfiguration('target_policy', default='rightmost')
    use_physics_grasp = LaunchConfiguration('use_physics_grasp', default='true')
    base_pin_enabled = LaunchConfiguration('base_pin_enabled', default='true')

    # ── Argument declarations ─────────────────────────────────────────────────
    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x', default_value=str(derived['spawn_x']),
        description='Robot spawn X (map frame). Default from scene.yaml robot.spawn_pose.')
    spawn_yaw_arg = DeclareLaunchArgument(
        'spawn_yaw', default_value=str(derived['spawn_yaw']),
        description='Robot spawn yaw, rad (map frame). Default from scene.yaml.')
    odometry_source_arg = DeclareLaunchArgument(
        'odometry_source', default_value='1',
        description='Wheel odometry: 1 = WORLD (exact, the sim default), '
                    '0 = ENCODER (drifts like the real robot). ENCODER requires '
                    'localization:=amcl and the launch refuses the other combination.')
    localization_arg = DeclareLaunchArgument(
        'localization', default_value='ground_truth',
        description='ground_truth = static identity map->odom (perfect, sim only). '
                    'amcl = real LiDAR localization against the map. Use amcl with '
                    'odometry_source:=0 for the honest hardware rehearsal.')

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

    target_policy_arg = DeclareLaunchArgument(
        'target_policy', default_value='rightmost',
        description="Which box to grasp when several are visible: 'rightmost' "
                    "(deterministic, for multi-box stacking) or 'nearest'.")

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
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_yaw': spawn_yaw,
            'odometry_source': LaunchConfiguration('odometry_source'),
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
                # Passthrough added 2026-08-25. Without it this include silently took
                # nav2_limo.launch.py's own default and AMCL could not be reached from
                # the full pipeline at all.
                'localization': localization,
                # AMCL's seed is the SAME pose Gazebo spawns the robot at, so moving
                # the spawn can no longer leave the filter believing something else.
                'initial_pose_x': spawn_x,
                'initial_pose_y': spawn_y,
                'initial_pose_yaw': spawn_yaw,
            }.items())])

    # ── 4. box_pose_estimator — delay 15 s ───────────────────────────────────
    box_estimator = TimerAction(
        period=15.0,
        actions=[Node(
            package='limo_car',
            executable='box_pose_estimator',
            name='box_pose_estimator',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                # 'rightmost' since 2026-08-25, when stack_box_1/2 returned to the
                # world. With three identical boxes at the SAME distance from the
                # robot, the default 'nearest' policy picks by depth -- which is
                # ambiguous between them and flips on perception noise, so the robot
                # could re-target mid-approach. 'rightmost' orders them
                # deterministically by base_link y, and because each pick removes a
                # box, the remaining ones present a new rightmost every cycle with no
                # counter to keep in sync.
                'target_policy': target_policy,
            }])])

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
        spawn_x_arg,
        spawn_y_arg,
        spawn_yaw_arg,
        localization_arg,
        odometry_source_arg,
        # Validate the pair BEFORE any node starts.
        OpaqueFunction(function=_check_localization_config),
        use_rviz_arg,
        use_gzclient_arg,
        drive_mode_arg,
        calib_loops_arg,
        target_policy_arg,
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
