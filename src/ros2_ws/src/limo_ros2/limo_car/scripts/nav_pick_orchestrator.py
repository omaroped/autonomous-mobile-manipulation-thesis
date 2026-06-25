#!/usr/bin/python3
"""
nav_pick_orchestrator.py — full autonomous navigate-and-pick sequence.

Sequence (one-shot, runs to completion):
  1. Navigate  — Nav2 NavigateToPose → approach goal in front of the table
  2. Pin base  — Gazebo /set_entity_state holds the base steady for grasping
  3. Perceive  — wait for a stable /box_pose (settle-then-capture)
  4. Grasp     — calibrated MoveIt top-down pick (arm_grasp_test logic):
                   ready → pre_grasp → grasp → weld → gentle close → lift
  5. Retract   — go_named('ready')
  6. Back up   — publish cmd_vel reverse for BACKUP_SEC seconds

Prerequisites (run first):
    ros2 launch limo_car nav_pick.launch.py
    ros2 launch limo_cobot_moveit_config moveit.launch.py

All the MoveIt / gripper logic is ported verbatim from arm_grasp_test.py (proven).
The only addition is the Nav2 action client at the front and the cmd_vel reverse
at the end.

Design notes:
  - base_pin logic is INLINE here (no separate node) — the orchestrator owns the
    50 Hz hold timer and cancels it before publishing cmd_vel reverse.
  - AMCL initial pose is set programmatically (spawn is always −2, 7).
  - If Nav2 is not running the node fails fast with a clear error.
"""

import math
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import (PoseStamped, Pose, Point,
                                PoseWithCovarianceStamped, Twist)
from std_msgs.msg import Float64MultiArray, Bool
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry

from nav2_msgs.action import NavigateToPose

from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, JointConstraint,
    PositionConstraint, OrientationConstraint, BoundingVolume,
    CollisionObject, PlanningScene,
)
from shape_msgs.msg import SolidPrimitive

from gazebo_msgs.srv import GetEntityState, SetEntityState
from gazebo_msgs.msg import EntityState

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition


# ── Navigation goal ───────────────────────────────────────────────────────────
# Robot stops so the arm base (~0.22 m ahead of body centre) is ~0.20 m from box.
# Table at (−2, 4); approach from +Y → stop at y ≈ 4.22.
# Fine-tune NAV_GOAL_Y if the arm can't reach on first run.
NAV_GOAL_X   = -2.0
NAV_GOAL_Y   =  4.75    # Nav2 stops here; visual_docking() handles the final 0.5 m.
                         # Must be > pickup_table inflated north edge (4.10+0.45=4.55) + margin.
NAV_GOAL_YAW = -1.5708  # −π/2 → robot faces −Y (toward the table)

NAV_TIMEOUT_SEC = 120.0  # generous — 3 m at 0.25 m/s = 12 s, but allow replanning

# ── Backup ────────────────────────────────────────────────────────────────────
BACKUP_VEL_X = -0.15   # m/s reverse
BACKUP_SEC   =  2.5    # back up ~0.38 m

# ── Gripper ───────────────────────────────────────────────────────────────────
GRIPPER_OPEN  = [ 0.15,  0.15, -0.15, -0.15, -0.15,  0.15]
GRIPPER_GRASP = [-0.20, -0.20,  0.20,  0.20,  0.20, -0.20]   # gentle partial close
GRIPPER_TOPIC = '/mycobot_gripper_controller/commands'

# Angle at which we fire the weld directly if smart_grasp hasn't triggered.
# Estimated from box geometry: 35 mm box, finger gap at open=43.4 mm, at grasp=31.8 mm.
# Linear interpolation: contact at θ ≈ -0.10 rad.  Fire at -0.10 so fingers are
# just touching the box when the weld attaches — no force overshoot.
WELD_FALLBACK_ANGLE = -0.10

# Sim "weld" attaches the box rigidly to the gripper (a Gazebo grasp aid). Set False to
# test whether the gripper physically holds the box on its own (real-grip test).
USE_WELD = False  # real physics grasp — no artificial attachment

# ── MoveIt frames / group ─────────────────────────────────────────────────────
PLANNING_FRAME = 'base_link'
TCP_LINK       = 'gripper_tcp'
ARM_GROUP      = 'arm'
ARM_JOINTS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
              'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
NAMED_STATES = {
    'home':   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'ready':  [0.0, -0.5, -0.6, 1.1, 0.0, 0.0],
    # Travel pose: joint2 tilts arm forward, joint3+4 fold it DOWN so the arm's
    # centre of mass is low and close to the body. This minimises the pendulum
    # torque on joint2 (horizontal-axis hinge) during navigation, stopping the
    # vibration that occurs when the arm is straight up (home = worst case).
    'travel': [0.0,  1.2, -0.6, -0.6, 0.0, 0.0],
}

# ── Grasp geometry (calibrated, from arm_grasp_test) ─────────────────────────
TOPDOWN_QUAT = (-0.7071, 0.0, 0.0, 0.7071)
GRASP_OFFSET = (0.0, 0.013, 0.0)   # perception → grasp target offset (calibrated: x/z pulled to box centre; y kept)
FIXED_BOX    = (0.28, 0.015, 0.08)     # fallback if perception unavailable
PRE_ABOVE    = 0.07                    # TCP above box centre at pre-grasp
GRASP_ABOVE  = 0.06                    # TCP above box centre at grasp

# Table collision in planning scene (add it so MoveIt routes around it)
TABLE_SIZE     = (0.10, 0.32, 0.10)   # metres (world: 0.30×0.08×0.10; pad Y/X slightly)
BOX_HALF_H     = 0.02                  # half of 4 cm box

# ── Stacking geometry (vertical box-stacking — thesis target task) ────────────
# The robot picks a box and places it on a HARDCODED foundation (per supervisor scope),
# then stacks each subsequent box on top (z grows by one box height per level).
# All three numbers are LIVE params (stack_x/stack_y/stack_surface_z) — calibrate them
# in sim with `ros2 param set` exactly like the grasp offset, NO relaunch needed.
BOX_HEIGHT       = 2 * BOX_HALF_H      # 0.04 m — full box height = one stack level
STACK_X_DEFAULT  =  0.26               # base_link x of the stack foundation (≈ pick reach)
STACK_Y_DEFAULT  = -0.12               # base_link y — offset to the side of the pick spot
STACK_SURFACE_Z  =  0.06               # base_link z of the foundation surface top (table top)
STACK_HOVER      =  0.10               # TCP hover above the current stack top before placing
STACK_COUNT_DEF  =  0                  # 0 = legacy single pick+backup; N>0 = stack N boxes

# ── Place table (world: -2, 0, 0.05 — same corridor, 4 m south of pickup) ────
NAV_PLACE_X    = -4.0        # place table moved to open area west of the divider
NAV_PLACE_Y    = -1.28       # Nav2 stops here (table at y=−2, dock at y=−1.76, +0.48 m buffer)
NAV_PLACE_YAW  = -1.5708    # same −π/2 (robot faces −Y, arm reaches toward table)
PLACE_DOCK_M   =  0.24      # final dock distance from table centre (same as pick)
PLACE_NAV_TO_DOCK = 0.48    # odometry drive from NAV_PLACE_Y to dock position
PLACE_BOX_Z    =  0.08      # box centre z in base_link (same table height as pick)
PLACE_HOVER    =  0.12      # TCP height above box centre while hovering
PLACE_ABOVE    =  0.04      # TCP height above box centre when setting box down

# ── Planning ──────────────────────────────────────────────────────────────────
PLAN_ATTEMPTS = 10
PLAN_TIME_SEC =  5.0
VEL_SCALE     =  0.2
ACC_SCALE     =  0.2
MOVEIT_SUCCESS = 1

# ── Base pin ──────────────────────────────────────────────────────────────────
PIN_ROBOT_NAME = 'mbot'


class NavPickOrchestrator(Node):

    def __init__(self):
        super().__init__('nav_pick_orchestrator')

        # Nav2
        self._nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # AMCL initial pose publisher
        self._init_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # MoveIt
        self._move  = ActionClient(self, MoveGroup, '/move_action')
        self._scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene')

        # Gripper
        self._gripper      = self.create_publisher(Float64MultiArray, GRIPPER_TOPIC, 10)
        self._attach_pub   = self.create_publisher(Bool, '/grasp_attach', 10)
        self._last_gripper = [0.0] * 6
        self._weld_active  = False
        self.create_subscription(Bool, '/grasp_attach', self._weld_cb, 10)

        # Perception
        self._latest_box = None
        self.create_subscription(PoseStamped, '/box_pose', self._box_cb, 10)

        # Odometry (for the blind final approach — drive a measured distance when the
        # real-spec camera can no longer see the box closer than 0.30 m)
        self._odom = None
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        # Box (x,y,z) in base_link recorded at the dock: x from the odometry-measured
        # distance (TRUE), y/z from the accurate far perception. Used for the grasp so
        # we don't aim at the camera's inflated close-range distance.
        self._dock_box = None

        # ── Calibration knobs (live, tune with `ros2 param set` — NO relaunch) ──
        self.declare_parameter('grasp_off_x', GRASP_OFFSET[0])
        self.declare_parameter('grasp_off_y', GRASP_OFFSET[1])
        self.declare_parameter('grasp_off_z', GRASP_OFFSET[2])
        self.declare_parameter('calib_loops', 0)     # 0 = normal single run; N = repeat the grasp N times
        self.declare_parameter('calib_pause', 6.0)   # seconds to observe between iterations
        self.declare_parameter('box_name', 'target_box')
        self.declare_parameter('box_reset_xyz', [-2.0, 4.0, 0.12])  # where to respawn the box each iter

        # ── Stacking knobs (live, tune with `ros2 param set` — NO relaunch) ──────
        self.declare_parameter('stack_count', STACK_COUNT_DEF)   # >0 → stack this many boxes
        self.declare_parameter('stack_x', STACK_X_DEFAULT)       # base_link x of foundation
        self.declare_parameter('stack_y', STACK_Y_DEFAULT)       # base_link y of foundation
        self.declare_parameter('stack_surface_z', STACK_SURFACE_Z)  # base_link z of surface top

        # cmd_vel (for backup)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # base_pin (Gazebo services)
        self._get_state = self.create_client(GetEntityState, '/get_entity_state')
        self._set_state = self.create_client(SetEntityState, '/set_entity_state')
        self._pin_pose  = None
        self._pin_timer = None

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _box_cb(self, msg):
        self._latest_box = msg

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self._odom = (p.x, p.y)

    def _odom_dist_since(self, x0, y0):
        """Straight-line distance the base has driven since (x0, y0) in odom."""
        if self._odom is None:
            return None
        return math.hypot(self._odom[0] - x0, self._odom[1] - y0)

    def _offset(self):
        """Current grasp offset (live ROS params — tunable without relaunch)."""
        return (self.get_parameter('grasp_off_x').value,
                self.get_parameter('grasp_off_y').value,
                self.get_parameter('grasp_off_z').value)

    def reset_box(self):
        """Respawn the target box on the table (known world pose) for the next pick."""
        name = self.get_parameter('box_name').value
        x, y, z = self.get_parameter('box_reset_xyz').value
        st = EntityState()
        st.name = name
        st.pose.position = Point(x=float(x), y=float(y), z=float(z))
        st.pose.orientation.w = 1.0
        st.reference_frame = 'world'
        req = SetEntityState.Request(); req.state = st
        if self._set_state.wait_for_service(timeout_sec=3.0):
            fut = self._set_state.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
            self.get_logger().info(f'reset box "{name}" → ({x:.2f}, {y:.2f}, {z:.2f})')

    # ── Step 1: Navigate ──────────────────────────────────────────────────────

    def _publish_initial_pose(self):
        """Tell AMCL where the robot starts (spawn pose)."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = -2.0
        msg.pose.pose.position.y =  7.0
        msg.pose.pose.position.z =  0.0
        # yaw −π/2 → quaternion (0, 0, sin(−π/4), cos(−π/4)) = (0, 0, −0.7071, 0.7071)
        msg.pose.pose.orientation.z = -0.7071
        msg.pose.pose.orientation.w =  0.7071
        # diagonal covariance — position known within ±0.5 m, yaw within ±0.2 rad
        msg.pose.covariance[0]  = 0.25
        msg.pose.covariance[7]  = 0.25
        msg.pose.covariance[35] = 0.04

        # Wait for AMCL to subscribe to /initialpose
        self.get_logger().info('Waiting for AMCL to subscribe to /initialpose...')
        for _ in range(30):
            if self._init_pose_pub.get_subscription_count() > 0:
                self.get_logger().info('AMCL subscribed! Publishing initial pose...')
                break
            time.sleep(0.5)
            # Spin to allow discovery to proceed
            rclpy.spin_once(self, timeout_sec=0.1)

        for _ in range(10):    # publish several times so AMCL receives it
            msg.header.stamp = self.get_clock().now().to_msg()
            self._init_pose_pub.publish(msg)
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().info('published AMCL initial pose at (−2, 7, yaw=−π/2)')

    def _activate_nav2_cmdvel(self):
        """Re-activate the Nav2 cmd_vel nodes (in case a previous mission
        deactivated them for docking) so navigation can drive again."""
        for node in ('controller_server', 'velocity_smoother', 'behavior_server',
                     'collision_monitor'):
            cli = self.create_client(ChangeState, f'/{node}/change_state')
            if not cli.wait_for_service(timeout_sec=3.0):
                continue
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_ACTIVATE
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)

    def navigate_to_table(self):
        self.get_logger().info('=== Step 1: Navigate to table ===')
        self._activate_nav2_cmdvel()
        self._publish_initial_pose()
        time.sleep(1.0)   # give AMCL a moment to digest the initial pose

        if not self._nav.wait_for_server(timeout_sec=30.0):
            self.get_logger().error(
                'NavigateToPose action server not available — '
                'is nav2_limo.launch.py running?')
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(NAV_GOAL_X)
        goal.pose.pose.position.y = float(NAV_GOAL_Y)
        # Yaw NAV_GOAL_YAW → quaternion
        half = NAV_GOAL_YAW / 2.0
        goal.pose.pose.orientation.z = math.sin(half)
        goal.pose.pose.orientation.w = math.cos(half)

        self.get_logger().info(
            f'sending Nav2 goal → ({NAV_GOAL_X:.2f}, {NAV_GOAL_Y:.2f}, '
            f'yaw={NAV_GOAL_YAW:.3f})')

        gh = None
        for attempt in range(15):
            send_fut = self._nav.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_fut, timeout_sec=10.0)
            if send_fut.done():
                gh = send_fut.result()
                if gh is not None and gh.accepted:
                    self.get_logger().info('Nav2 goal accepted — driving to table…')
                    break
            self.get_logger().warn(f'Nav2 goal rejected or timed out (attempt {attempt+1}/15) — waiting for bt_navigator to become active...')
            end_time = self.get_clock().now() + Duration(seconds=2.0)
            while self.get_clock().now() < end_time:
                rclpy.spin_once(self, timeout_sec=0.1)
        else:
            self.get_logger().error('Nav2 goal rejected after 15 attempts — aborting')
            return False
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=NAV_TIMEOUT_SEC)
        if not res_fut.done():
            self.get_logger().error(f'Nav2 did not complete within {NAV_TIMEOUT_SEC} s')
            return False

        res = res_fut.result()
        if res.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f'Nav2 navigation failed with status: {res.status}')
            return False

        self.get_logger().info('Nav2 navigation complete ✓')
        return True

    # ── Nav2 cmd_vel hand-off ─────────────────────────────────────────────────

    def _silence_nav2_cmdvel(self):
        """Deactivate the Nav2 nodes that publish /cmd_vel so the orchestrator can
        own the topic during docking. collision_monitor publishes safety-stop zeros
        when it receives no input on cmd_vel_smoothed (after smoother deactivates),
        which overrides direct dock-drive commands."""
        for node in ('collision_monitor', 'velocity_smoother', 'controller_server',
                     'behavior_server'):
            cli = self.create_client(ChangeState, f'/{node}/change_state')
            if not cli.wait_for_service(timeout_sec=3.0):
                self.get_logger().warn(f'{node}/change_state unavailable — skipping')
                continue
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_DEACTIVATE
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            self.get_logger().info(f'deactivated {node} (released /cmd_vel)')

    # ── Step 1.5: Visual docking ──────────────────────────────────────────────

    def _read_box_live(self):
        """One fresh /box_pose reading (x, y) in base_link, or None if missing /
        out of plausible range. Used by the docking control loops (needs to be
        responsive, so single readings — not the 8-sample median)."""
        rclpy.spin_once(self, timeout_sec=0.05)
        p = self._latest_box
        if p is None or p.header.frame_id != PLANNING_FRAME:
            return None
        x, y = p.pose.position.x, p.pose.position.y
        if x < 0.10 or x > 1.5:
            return None
        return x, y

    def visual_docking(self):
        """MECHANISM 1 — land, align, then approach (differential drive).

        Because the base is now diff-drive (can rotate in place), we do this in
        two clean phases instead of the old Ackermann steer-while-rolling:
          Phase A  rotate in place until the box is centred straight ahead (|y|→0)
                   — i.e. the robot FACES the box squarely.
          Phase B  drive straight in to the grasp distance, with small rotation
                   trims to stay centred.
        Entering Phase B already aligned means the box ends up at (~TARGET_X, ~0)
        in base_link — square and centred — which is what makes the top-down grasp
        reliable.
        """
        self.get_logger().info('=== Step 1.5: Land → align → approach (diff) ===')
        # Hand /cmd_vel over from Nav2 to us, or the dock crawls/stalls.
        self._silence_nav2_cmdvel()

        TARGET_X = 0.28     # final grasp distance from base_link
        X_TOL    = 0.02     # forward tolerance
        Y_TOL    = 0.03     # lateral-centre tolerance
        K_ROT    = 1.8      # rotation gain  (rad/s per m of lateral error)
        K_FWD    = 0.8      # forward gain   (m/s per m of distance error)
        MAX_ROT  = 0.5
        MIN_FWD, MAX_FWD = 0.06, 0.16

        def clamp(v, lo, hi):
            return max(lo, min(hi, v))

        # ── Phase A: rotate in place to centre the box ──────────────────────
        self.get_logger().info('Phase A: rotating to face the box…')
        ok = 0
        deadline = time.time() + 15.0
        while time.time() < deadline:
            b = self._read_box_live()
            if b is None:
                self._cmd_vel_pub.publish(Twist())   # stop; never coast on a lost box
                self.get_logger().info('waiting for /box_pose…', throttle_duration_sec=1.0)
                time.sleep(0.1); continue
            x, y = b
            self.get_logger().info(f'align: lateral={y:+.3f} (x={x:.3f})',
                                   throttle_duration_sec=0.5)
            if abs(y) < Y_TOL:
                ok += 1
                if ok >= 3:
                    self.get_logger().info('aligned — box centred ahead'); break
            else:
                ok = 0
            t = Twist(); t.angular.z = clamp(K_ROT * y, -MAX_ROT, MAX_ROT)
            self._cmd_vel_pub.publish(t); time.sleep(0.05)
        self._cmd_vel_pub.publish(Twist()); time.sleep(0.3)

        # ── Phase B: distance-measured HARD STOP (reliable, can't climb the table)
        # The old camera-feedback stop was fragile: it only stopped when forward AND
        # lateral error settled *at the same time*, so it often never fired and the
        # robot crept into the table and climbed it. Instead: take ONE solid box
        # reading while the camera is still accurate (robot is ~0.6 m back, far beyond
        # the near-clip), then drive a MEASURED distance by ODOMETRY and STOP HARD at
        # STOP_DISTANCE. This is pure geometry — it stops in front of the table every
        # time, regardless of the camera at close range.
        STOP_DISTANCE = 0.24    # box distance from base_link at the dock. CALIBRATION KNOB:
                                # lower = robot front closer to the table + box more reachable
                                # (but too low climbs the table); higher = safer but arm can't
                                # reach. 0.24 ≈ front ~3-4 cm from the table, box within reach.

        box0 = self.get_box_xyz(samples=6, timeout=8.0)
        if box0 is not None:
            d0 = box0[0]
            # After driving straight to STOP_DISTANCE, the box sits at base_link
            # x≈STOP_DISTANCE (TRUE), with the accurate far-range y/z unchanged.
            self._dock_box = (STOP_DISTANCE, box0[1], box0[2])
        else:
            b = self._read_box_live()
            d0 = b[0] if b else FIXED_BOX[0]
            self._dock_box = None
            self.get_logger().warn(f'Phase B: no median reading — using d0={d0:.3f} m')

        drive = max(0.0, d0 - STOP_DISTANCE)
        self.get_logger().info(
            f'Phase B: box at {d0:.3f} m → driving {drive:.3f} m to stop at {STOP_DISTANCE:.2f} m')

        # wait for a fresh odom sample, record the start point
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._odom is not None:
                break
        ox, oy = self._odom if self._odom is not None else (0.0, 0.0)

        deadline = time.time() + 25.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            disp = self._odom_dist_since(ox, oy)
            disp = 0.0 if disp is None else disp
            remaining = drive - disp
            self.get_logger().info(
                f'approach: driven {disp:.3f}/{drive:.3f} m  (box ≈ {d0 - disp:.3f} m)',
                throttle_duration_sec=0.5)
            if remaining <= 0.0:
                self.get_logger().info(
                    f'HARD STOP — docked ~{STOP_DISTANCE:.2f} m from box, table not climbed')
                break
            t = Twist()
            t.linear.x = clamp(0.6 * remaining, 0.06, 0.14)   # ease off near the stop
            self._cmd_vel_pub.publish(t); time.sleep(0.05)
        else:
            self.get_logger().warn('approach timed out — stopping')

        self._cmd_vel_pub.publish(Twist()); time.sleep(0.5)

    # ── Step 2: Pin base ──────────────────────────────────────────────────────

    def pin_base(self):
        """Capture current robot world pose and hold it at 50 Hz."""
        self.get_logger().info('=== Step 2: Pinning base ===')
        if not self._get_state.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn('/get_entity_state unavailable — base will not be pinned')
            return False
        req = GetEntityState.Request()
        req.name = PIN_ROBOT_NAME
        req.reference_frame = 'world'
        fut = self._get_state.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        if res is None or not res.success:
            self.get_logger().warn('Could not read robot pose — continuing without pin')
            return False
        self._pin_pose = res.state.pose
        p = self._pin_pose.position
        self.get_logger().info(f'pinning base at world ({p.x:.3f}, {p.y:.3f})')
        self._pin_timer = self.create_timer(0.02, self._hold_base)   # 50 Hz
        return True

    def unpin_base(self):
        """Cancel the base-pin timer so cmd_vel can drive freely."""
        if self._pin_timer is not None:
            self._pin_timer.cancel()
            self._pin_timer = None
            self.get_logger().info('base pin released')

    def _hold_base(self):
        if self._pin_pose is None:
            return
        st = EntityState()
        st.name = PIN_ROBOT_NAME
        st.pose = self._pin_pose
        st.reference_frame = 'world'
        req = SetEntityState.Request()
        req.state = st
        self._set_state.call_async(req)

    # ── Step 3: Perceive box ──────────────────────────────────────────────────

    def get_box_xyz(self, samples=8, timeout=20.0):
        self.get_logger().info('=== Step 3: Perceiving box ===')
        readings = []
        deadline = time.time() + timeout
        while time.time() < deadline and len(readings) < samples:
            self._latest_box = None
            t0 = time.time()
            while self._latest_box is None and time.time() < t0 + 1.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            p = self._latest_box
            if p is not None and p.header.frame_id == PLANNING_FRAME:
                readings.append((p.pose.position.x,
                                 p.pose.position.y,
                                 p.pose.position.z))
        if len(readings) < 3:
            self.get_logger().warn(
                'Not enough /box_pose readings — using FIXED_BOX fallback')
            return None
        box = (statistics.median(r[0] for r in readings),
               statistics.median(r[1] for r in readings),
               statistics.median(r[2] for r in readings))
        self.get_logger().info(
            f'box @ base_link {tuple(round(v, 3) for v in box)} '
            f'(median of {len(readings)} readings)')
        return box

    # ── Gripper helpers ───────────────────────────────────────────────────────

    def _weld_cb(self, msg: Bool):
        self._weld_active = msg.data

    def set_gripper(self, values, label, duration=0.8, steps=20):
        start = self._last_gripper
        for i in range(1, steps + 1):
            a = i / steps
            interp = [s + a * (v - s) for s, v in zip(start, values)]
            msg = Float64MultiArray()
            msg.data = [float(x) for x in interp]
            self._gripper.publish(msg)
            rclpy.spin_once(self, timeout_sec=duration / steps)
        self._last_gripper = list(values)
        self.get_logger().info(f'gripper → {label}')
        time.sleep(0.3)

    def close_until_contact(self):
        """Close one step at a time; stop the instant smart_grasp fires the weld.

        Prevents the ODE "explosion" (box launching sideways) that happens when a
        fixed closing angle keeps building contact force after the box is gripped.
        Also works for any object size — large objects stop the fingers early,
        small objects allow fingers to close further, no code change needed.

        Safety floor: never closes past GRIPPER_GRASP even without contact.
        """
        self._weld_active = False   # clear stale state before the new attempt

        j0    = self._last_gripper[0]
        floor = GRIPPER_GRASP[0]    # -0.20 rad hard limit

        step_rad = 0.005            # 0.5 deg per tick — very smooth
        step_sec = 0.08             # 80 ms per tick

        steps_taken = 0
        while j0 > floor:
            j0 = max(floor, j0 - step_rad)
            t  = (GRIPPER_OPEN[0] - j0) / (GRIPPER_OPEN[0] - GRIPPER_GRASP[0])

            cmd = Float64MultiArray()
            cmd.data = [float(o + t * (c - o))
                        for o, c in zip(GRIPPER_OPEN, GRIPPER_GRASP)]
            self._gripper.publish(cmd)
            self._last_gripper = cmd.data[:]
            steps_taken += 1

            rclpy.spin_once(self, timeout_sec=step_sec)

            if self._weld_active:
                self.get_logger().info(
                    f'[close] smart_grasp weld at j0={j0:.3f} rad  '
                    f'({steps_taken * step_sec:.1f} s)  HOLDING')
                break

            # Fallback: if smart_grasp hasn't fired by the estimated contact angle,
            # fire the weld directly.  Prevents the box from sliding away while waiting
            # for a contact sensor that may not detect the collision.
            if j0 <= WELD_FALLBACK_ANGLE and not self._weld_active:
                self.get_logger().info(
                    f'[close] weld FALLBACK at j0={j0:.3f} rad '
                    f'(smart_grasp silent — firing directly)')
                self.attach(True)
                break
        else:
            if not self._weld_active:
                self.get_logger().warn(
                    '[close] Reached floor -0.20 rad — weld never fired.')

        # Back off fingers 2 steps from contact so the finger force doesn't fight
        # the weld constraint — this eliminates the post-pick shaking.
        if self._weld_active:
            j_back = self._last_gripper[0] + 0.010
            t_back = max(0.0, (GRIPPER_OPEN[0] - j_back) /
                         (GRIPPER_OPEN[0] - GRIPPER_GRASP[0]))
            back_cmd = Float64MultiArray()
            back_cmd.data = [float(o + t_back * (c - o))
                             for o, c in zip(GRIPPER_OPEN, GRIPPER_GRASP)]
            self._gripper.publish(back_cmd)
            self._last_gripper = back_cmd.data[:]
            rclpy.spin_once(self, timeout_sec=0.15)
            self.get_logger().info('[close] fingers backed off 0.01 rad — contact force removed')

        time.sleep(0.3)

    def attach(self, on: bool):
        msg = Bool()
        msg.data = bool(on)
        self._attach_pub.publish(msg)
        self.get_logger().info(f'/grasp_attach → {on}')
        end = time.time() + 0.7
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    # ── MoveIt helpers ────────────────────────────────────────────────────────

    @staticmethod
    def _joint_constraints(values, tol=0.01):
        c = Constraints()
        for name, val in zip(ARM_JOINTS, values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(val)
            jc.tolerance_above = tol
            jc.tolerance_below = tol
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        return c

    @staticmethod
    def _pose_constraints(x, y, z, quat, pos_tol=0.025, ori_tol=0.1):
        c = Constraints()
        target = Pose()
        target.position = Point(x=float(x), y=float(y), z=float(z))
        target.orientation.x, target.orientation.y, \
            target.orientation.z, target.orientation.w = (float(q) for q in quat)

        pc = PositionConstraint()
        pc.header.frame_id = PLANNING_FRAME
        pc.link_name = TCP_LINK
        region = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [pos_tol]
        region.primitives.append(sphere)
        region.primitive_poses.append(target)
        pc.constraint_region = region
        pc.weight = 1.0
        c.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id = PLANNING_FRAME
        oc.link_name = TCP_LINK
        oc.orientation = target.orientation
        oc.absolute_x_axis_tolerance = ori_tol
        oc.absolute_y_axis_tolerance = ori_tol
        oc.absolute_z_axis_tolerance = ori_tol
        oc.weight = 1.0
        c.orientation_constraints.append(oc)
        return c

    def _send(self, constraints, label, plan_only=False, plan_time=PLAN_TIME_SEC):
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = ARM_GROUP
        req.goal_constraints = [constraints]
        req.num_planning_attempts = PLAN_ATTEMPTS
        req.allowed_planning_time = plan_time
        req.max_velocity_scaling_factor = VEL_SCALE
        req.max_acceleration_scaling_factor = ACC_SCALE
        goal.request = req
        goal.planning_options.plan_only = plan_only

        send_fut = self._move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=15.0)
        if not send_fut.done():
            self.get_logger().error(f'[{label}] goal not accepted within 15 s')
            return False
        gh = send_fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error(f'[{label}] goal rejected')
            return False
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=30.0)
        if not res_fut.done():
            self.get_logger().error(f'[{label}] no result within 30 s')
            return False
        ok = res_fut.result().result.error_code.val == MOVEIT_SUCCESS
        self.get_logger().info(f'[{label}] {"OK" if ok else "FAILED"}')
        return ok

    def _wait_for_move_group(self, timeout=60.0):
        self.get_logger().info('waiting for move_group…')
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._move.wait_for_server(timeout_sec=2.0):
                self.get_logger().info('move_group connected')
                return True
        return False

    def go_named(self, name):
        return self._send(self._joint_constraints(NAMED_STATES[name]), name)

    def go_pose(self, x, y, z, q, label):
        self.get_logger().info(f'[{label}] planning…')
        return self._send(self._pose_constraints(x, y, z, q), label)

    def add_table(self, bx, by, bz):
        """Register the pickup table as a MoveIt collision object."""
        table_top = bz - BOX_HALF_H
        co = CollisionObject()
        co.header.frame_id = PLANNING_FRAME
        co.id = 'pickup_table'
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = list(TABLE_SIZE)
        pose = Pose()
        pose.position = Point(x=float(bx), y=float(by),
                              z=float(table_top - TABLE_SIZE[2] / 2.0))
        pose.orientation.w = 1.0
        co.primitives.append(prim)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects.append(co)
        if self._scene.wait_for_service(timeout_sec=5.0):
            fut = self._scene.call_async(ApplyPlanningScene.Request(scene=scene))
            rclpy.spin_until_future_complete(self, fut)
            self.get_logger().info(f'added table collision @ z={pose.position.z:.3f}')

    def remove_table(self):
        co = CollisionObject()
        co.header.frame_id = PLANNING_FRAME
        co.id = 'pickup_table'
        co.operation = CollisionObject.REMOVE
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects.append(co)
        if self._scene.wait_for_service(timeout_sec=5.0):
            fut = self._scene.call_async(ApplyPlanningScene.Request(scene=scene))
            rclpy.spin_until_future_complete(self, fut)
            self.get_logger().info('removed table collision → retreat can plan freely')

    # ── Step 4–5: Grasp + Retract ─────────────────────────────────────────────

    def grasp_and_retract(self, bx, by, bz):
        """MECHANISM 2 — iterative top-down grasp.

        Instead of a single 1 cm descent, we:
          1. go to a HOVER point well above the box (top-down),
          2. re-perceive (median → kills the frame-to-frame drift), re-centre the
             target, and only descend once the estimate is STABLE,
          3. descend straight down to the calibrated grasp height, weld, close,
             lift, retract.

        Note: the depth camera is base-mounted (not on the wrist). Hovering the arm
        can occlude the box, so re-perception may return nothing — in that case we
        gracefully keep the (median-filtered) estimate from Step 3 and descend,
        i.e. at worst this equals the old behaviour, at best it's more precise.
        A wrist camera would make this a true eye-in-hand loop (sim-to-real note).
        """
        self.get_logger().info('=== Steps 4–5: Iterative top-down grasp ===')
        q = TOPDOWN_QUAT
        HOVER      = 0.12     # TCP this far above box centre while aligning
        GRASP_Z    = GRASP_ABOVE   # calibrated grasp height (0.06 above centre)
        STABLE_TOL = 0.012   # estimate "settled" when it shifts < 1.2 cm
        MAX_ITERS  = 3

        if not self._wait_for_move_group():
            return False
        if not self.go_named('ready'):
            return False
        self.set_gripper(GRIPPER_OPEN, 'open')
        self.add_table(bx, by, bz)

        tx, ty, tz = bx, by, bz   # dock-geometry target (already incl. GRASP_OFFSET)

        # ── hover above the box (single shot, no close-range re-perception) ──
        # At the dock the base camera OVER-READS the distance, so re-perceiving here
        # would push the goal past the arm's reach and make IK fail (the bug we hit).
        # The odometry-measured dock distance is the reliable target, so we trust it.
        if not self.go_pose(tx, ty, tz + HOVER, q, 'hover'):
            self.get_logger().error('hover pose failed — aborting')
            return False

        # ── descend straight down to the grasp point ────────────────────────
        if not self.go_pose(tx, ty, tz + GRASP_Z, q, 'grasp'):
            self.get_logger().error('grasp pose failed — aborting')
            return False

        # Adaptive close: steps 0.5 deg at a time, stops the moment smart_grasp
        # detects bilateral contact and fires the weld.  No fixed end angle —
        # works for any object size and eliminates the ODE "explosion" that
        # happened when a fixed -0.20 rad target kept building contact force
        # after the box was already gripped.
        if USE_WELD:
            self.attach(True)
        else:
            self.get_logger().info('USE_WELD=False — smart_grasp handles weld automatically')
        self.close_until_contact()

        # Remove table collision so we can lift without phantom table-gripper collisions
        self.remove_table()

        # Lift — retry 3× (OMPL can be flaky on the first attempt)
        lifted = False
        for attempt in range(3):
            if self.go_pose(tx, ty, tz + HOVER, q, f'lift (try {attempt + 1})'):
                lifted = True
                break
            time.sleep(0.5)
        if not lifted:
            self.get_logger().warn('lift attempts failed — continuing to retract anyway')

        # Retract to home
        self.go_named('home')
        self.get_logger().info('=== Grasp + Retract complete ===')
        return True

    # ── Step 5b: Place onto the stack ───────────────────────────────────────────

    def _stack_xy(self):
        """Live foundation (x, y) in base_link — tune with `ros2 param set`."""
        return (float(self.get_parameter('stack_x').value),
                float(self.get_parameter('stack_y').value))

    def place_on_stack(self, level):
        """Place the currently-held box onto the stack at `level` (0 = foundation).

        Top-down placement at the hardcoded foundation (stack_x, stack_y). The resting
        surface for this box is the foundation top plus `level` full box heights, so each
        box lands on the one below. We mirror the grasp geometry: at grasp the TCP sat
        GRASP_ABOVE above the box centre, so to rest the box bottom on the surface the TCP
        target is surface + BOX_HALF_H + GRASP_ABOVE. Hover → descend → release → lift.

        Assumes the box is still held (welded or gripped) and the arm is at 'home'.
        """
        self.get_logger().info(f'=== Step 5b: Place on stack — level {level} ===')
        q = TOPDOWN_QUAT
        sx, sy = self._stack_xy()
        surface_z = float(self.get_parameter('stack_surface_z').value)

        rest_surface = surface_z + level * BOX_HEIGHT          # top of the stack so far
        place_z      = rest_surface + BOX_HALF_H + GRASP_ABOVE  # TCP height to release at
        hover_z      = rest_surface + BOX_HALF_H + STACK_HOVER

        if not self.go_named('ready'):
            return False
        # Approach from above so we descend straight onto the stack (avoids clipping it).
        if not self.go_pose(sx, sy, hover_z, q, f'stack hover L{level}'):
            self.get_logger().error('stack hover failed — aborting place')
            return False
        if not self.go_pose(sx, sy, place_z, q, f'stack place L{level}'):
            self.get_logger().error('stack place failed — aborting place')
            return False

        # Release: drop the weld first (if used) so physics takes over, then open fingers.
        if USE_WELD:
            self.attach(False)
        self.set_gripper(GRIPPER_OPEN, 'release')
        time.sleep(0.4)   # let the box settle before retracting

        # Lift straight up off the placed box, then retract home.
        for attempt in range(3):
            if self.go_pose(sx, sy, hover_z, q, f'stack retract L{level} (try {attempt + 1})'):
                break
            time.sleep(0.5)
        self.go_named('home')
        self.get_logger().info(f'=== Box placed at stack level {level} ✓ ===')
        return True

    # ── Step 6: Back up ───────────────────────────────────────────────────────

    def back_up(self):
        self.get_logger().info('=== Step 6: Backing up ===')
        self.unpin_base()
        twist = Twist()
        twist.linear.x = BACKUP_VEL_X
        end = time.time() + BACKUP_SEC
        while time.time() < end:
            self._cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        # Stop
        self._cmd_vel_pub.publish(Twist())
        self.get_logger().info(f'backed up ~{abs(BACKUP_VEL_X) * BACKUP_SEC:.2f} m ✓')

    # ── Place: navigate to second table ──────────────────────────────────────

    def navigate_to_place_table(self):
        self.get_logger().info('=== Place Step 1: Navigate to place table ===')
        self._activate_nav2_cmdvel()

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(NAV_PLACE_X)
        goal.pose.pose.position.y = float(NAV_PLACE_Y)
        half = NAV_PLACE_YAW / 2.0
        goal.pose.pose.orientation.z = math.sin(half)
        goal.pose.pose.orientation.w = math.cos(half)

        gh = None
        for attempt in range(15):
            send_fut = self._nav.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_fut, timeout_sec=10.0)
            if send_fut.done():
                gh = send_fut.result()
                if gh is not None and gh.accepted:
                    self.get_logger().info('Nav2 goal accepted — driving to place table…')
                    break
            self.get_logger().warn(f'Place nav goal rejected (attempt {attempt + 1}/15) — retrying…')
            end = self.get_clock().now() + Duration(seconds=2.0)
            while self.get_clock().now() < end:
                rclpy.spin_once(self, timeout_sec=0.1)
        else:
            self.get_logger().error('Could not reach place table — aborting place')
            return False

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=NAV_TIMEOUT_SEC)
        if not res_fut.done() or res_fut.result().status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error('Navigation to place table failed')
            return False

        self.get_logger().info('Arrived at place table approach — docking by odometry…')
        self._silence_nav2_cmdvel()
        time.sleep(1.0)   # let velocity_smoother fully deactivate before we own /cmd_vel

        # Wait for a fresh odom reading
        for _ in range(40):
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._odom is not None:
                break

        if self._odom is None:
            self.get_logger().warn('No odom available — skipping dock drive')
            return True

        _, oy = self._odom

        # Absolute target: robot centre 0.24 m north of place table centre (y = -2.0).
        # This is robust against Nav2 stopping anywhere within the ±0.25 m goal tolerance.
        dock_target_y = -2.0 + PLACE_DOCK_M   # = -1.76

        if oy <= dock_target_y:
            self.get_logger().info(f'Nav2 delivered robot past dock target (y={oy:.3f}) — no drive needed')
            self._cmd_vel_pub.publish(Twist())
            return True

        self.get_logger().info(
            f'Place dock: driving south to y={dock_target_y:.3f} (now y={oy:.3f}, '
            f'need {oy - dock_target_y:.3f} m)')
        deadline = time.time() + 25.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            cur_y = self._odom[1] if self._odom else oy
            if cur_y <= dock_target_y:
                self.get_logger().info(f'Place dock complete ✓ (y={cur_y:.3f})')
                break
            t = Twist()
            t.linear.x = 0.10   # drive speed (m/s) — slightly faster than old 0.08
            self._cmd_vel_pub.publish(t)
            time.sleep(0.05)
        self._cmd_vel_pub.publish(Twist())
        time.sleep(0.5)
        return True

    # ── Place: lower box onto table and release ───────────────────────────────

    def place_box(self):
        self.get_logger().info('=== Place Step 2: Placing box on place table ===')
        q  = TOPDOWN_QUAT
        px = PLACE_DOCK_M   # 0.24 m ahead in base_link
        py = 0.0
        pz = PLACE_BOX_Z    # 0.08 m — same table height as pick

        if not self.go_named('ready'):
            self.get_logger().warn('ready pose failed — attempting place anyway')

        # Hover above place target
        if not self.go_pose(px, py, pz + PLACE_HOVER, q, 'place hover'):
            self.get_logger().error('place hover IK failed — releasing here')
            self.attach(False)
            self.set_gripper(GRIPPER_OPEN, 'release')
            return False

        # Descend to set box on table surface
        if not self.go_pose(px, py, pz + PLACE_ABOVE, q, 'place set'):
            self.get_logger().error('place set IK failed — releasing here')
            self.attach(False)
            self.set_gripper(GRIPPER_OPEN, 'release')
            return False

        # Release: weld first so physics takes over, then open fingers
        self.attach(False)
        time.sleep(0.2)
        self.set_gripper(GRIPPER_OPEN, 'release')
        time.sleep(0.5)   # let box settle

        # Lift clear of the placed box
        for attempt in range(3):
            if self.go_pose(px, py, pz + PLACE_HOVER, q,
                            f'place retract (try {attempt + 1})'):
                break
            time.sleep(0.3)

        self.go_named('home')
        self.get_logger().info('=== Box placed on place table ✓ ===')
        return True

    # ── Main sequence ─────────────────────────────────────────────────────────

    def calib_loop(self, base, n):
        """Repeat the grasp N times, auto-resetting the box between picks and reading the
        grasp offset from LIVE ROS params each iteration — so the offset can be tuned
        without relaunching anything:
            ros2 param set /nav_pick_orchestrator grasp_off_y <metres>
            ros2 param set /nav_pick_orchestrator grasp_off_x <metres>
        """
        self.get_logger().info(f'═══ CALIBRATION MODE — {n} grasp iterations ═══')
        self.get_logger().info('tune live, e.g.:  '
                               'ros2 param set /nav_pick_orchestrator grasp_off_y 0.0')
        if not self._wait_for_move_group():
            return
        pause = self.get_parameter('calib_pause').value
        for i in range(n):
            self.reset_box()
            time.sleep(0.7)   # let the box settle on the table
            ox, oy, oz = self._offset()
            bx, by, bz = base[0] + ox, base[1] + oy, base[2] + oz
            self.get_logger().info(
                f'── iter {i + 1}/{n}: base {tuple(round(v, 3) for v in base)} + '
                f'offset ({ox:.3f}, {oy:.3f}, {oz:.3f}) → grasp ({bx:.3f}, {by:.3f}, {bz:.3f}) ──')
            self.grasp_and_retract(bx, by, bz)
            self.get_logger().info(
                f'iter {i + 1} done — look at gripper vs box, then adjust the offset param')
            time.sleep(pause)
        self.get_logger().info('═══ calibration loop complete ═══')

    def run_stack(self, base, n):
        """Pick `n` boxes (auto-resetting the box at the pickup spot between picks) and
        stack them vertically on the hardcoded foundation. Assumes the robot is already
        docked and pinned. Picks reuse the same dock geometry as calib_loop, so the box
        must respawn at the same pickup location each iteration (box_reset_xyz)."""
        self.get_logger().info(f'═══ STACKING MODE — {n} boxes ═══')
        for level in range(n):
            if level > 0:
                self.reset_box()        # respawn the next box at the pickup spot
                time.sleep(0.7)         # let it settle on the table
            ox, oy, oz = self._offset()
            bx, by, bz = base[0] + ox, base[1] + oy, base[2] + oz
            self.get_logger().info(
                f'── box {level + 1}/{n}: pick ({bx:.3f}, {by:.3f}, {bz:.3f}) ──')
            if not self.grasp_and_retract(bx, by, bz):
                self.get_logger().error(f'pick failed at level {level} — aborting stack')
                break
            if not self.place_on_stack(level):
                self.get_logger().error(f'place failed at level {level} — aborting stack')
                break
        self.back_up()
        self.get_logger().info('═══ stacking complete ═══')

    def run(self):
        self.get_logger().info('══════════════════════════════════════════════')
        self.get_logger().info('  Nav-Pick Orchestrator — STARTING SEQUENCE   ')
        self.get_logger().info('══════════════════════════════════════════════')

        # 0. Travel pose — wait for MoveIt first, then fold arm DOWN before driving so
        #    joint2 (horizontal hinge) doesn't act as a pendulum during navigation.
        self._wait_for_move_group()
        self.go_named('travel')

        # 1. Navigate
        if not self.navigate_to_table():
            self.get_logger().error('Navigation failed — aborting')
            return

        time.sleep(0.5)   # settle

        # 1.5. Visual Docking
        self.visual_docking()

        # 2. Pin base
        self.pin_base()
        time.sleep(1.0)   # let the pin settle before moving the arm

        # 3. Grasp-target BASE (pre-offset). Prefer the DOCK geometry (x = odometry-
        #    measured true distance) — the close-range camera over-reads at the dock.
        base = self._dock_box if self._dock_box is not None else self.get_box_xyz()
        if base is None:
            base = FIXED_BOX
            self.get_logger().warn(f'perception fallback → FIXED_BOX {FIXED_BOX}')

        # CALIBRATION MODE: loop the grasp (box auto-resets) so the offset can be tuned
        # live with `ros2 param set` — no relaunch needed.
        n = self.get_parameter('calib_loops').value
        if n and n > 0:
            self.calib_loop(base, n)
            return

        # STACKING MODE: pick + stack `stack_count` boxes vertically (thesis task).
        stack_n = self.get_parameter('stack_count').value
        if stack_n and stack_n > 0:
            self.run_stack(base, stack_n)
            return

        # 4+5. Grasp + Retract (single run)
        ox, oy, oz = self._offset()
        bx, by, bz = base[0] + ox, base[1] + oy, base[2] + oz
        self.get_logger().info(
            f'grasp target = {tuple(round(v, 3) for v in base)} + offset '
            f'({ox:.3f}, {oy:.3f}, {oz:.3f}) → ({bx:.3f}, {by:.3f}, {bz:.3f})')
        self.grasp_and_retract(bx, by, bz)

        # 5b. If box is welded (grasp succeeded) — navigate to place table and place
        if self._weld_active:
            self.get_logger().info('Weld active — proceeding to place table')
            self.unpin_base()              # MUST release pin before driving — pin vs Nav2 = violent shake
            self.go_named('ready')         # arm elevated (box above LiDAR scan plane — no phantom obstacles)
            # Back away from pickup table before Nav2 plans.
            # Robot is parked 0.24 m from the table facing it — the table is dead ahead.
            # Nav2 can't plan a path with an obstacle right in front of the robot; it
            # would attempt to rotate/drive forward and hit the table instead.
            self.get_logger().info('Clearing pickup table — reversing 0.6 m before Nav2…')
            _twist = Twist()
            _twist.linear.x = BACKUP_VEL_X   # -0.15 m/s
            _t_end = time.time() + 4.0        # 0.15 × 4.0 = 0.60 m clearance
            while time.time() < _t_end:
                self._cmd_vel_pub.publish(_twist)
                rclpy.spin_once(self, timeout_sec=0.1)
            self._cmd_vel_pub.publish(Twist())
            time.sleep(0.3)
            if self.navigate_to_place_table():
                self.pin_base()
                self.place_box()
        else:
            self.get_logger().warn('Weld not active after grasp — skipping place step')

        # 6. Back up from wherever the robot stopped
        self.back_up()

        self.get_logger().info('══════════════════════════════════════════════')
        self.get_logger().info('  SEQUENCE COMPLETE ✓                         ')
        self.get_logger().info('══════════════════════════════════════════════')


def main(args=None):
    rclpy.init(args=args)
    node = NavPickOrchestrator()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.unpin_base()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
