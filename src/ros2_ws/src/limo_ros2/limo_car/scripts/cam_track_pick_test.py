#!/usr/bin/python3
"""
cam_track_pick_test.py — standalone camera-track-and-pick test.

The robot uses ONLY its camera. No Nav2, no map, no hardcoded coordinates.

  Phase 1 — Visual approach
    The camera continuously detects the box and reports its 3D position in
    base_link via /box_pose.  The robot drives toward the box using bearing
    and range as the sole control signals.

    Stopping criterion: the robot stops at the LAST position where the box is
    still reliably detected.  As the robot gets closer, the depth sensor
    approaches its minimum reliable range (near-clip).  This manifests as
    consecutive dropped detections.  The moment that happens, the robot stops
    and records the last known box position — that is the grasp position.

    A hard range floor (STOP_RANGE) acts as a secondary safety stop in case
    the depth sensor never drops out before the robot reaches the box.

  Phase 2 — Pick
    From the stopped position, the arm performs a top-down grasp at the last
    known box coordinates.  A re-sample (median of N fresh readings) refines
    the estimate while the robot is stationary.

Run prerequisites:
    - Gazebo + robot + controllers running
    - MoveIt running
    - box_pose_estimator running  (publishes /box_pose)
    - grasp_attacher running      (handles /grasp_attach)
    - smart_grasp running         (contact-triggered weld)

    ros2 run limo_car cam_track_pick_test
"""

import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Twist, Pose, Point
from std_msgs.msg import Float64MultiArray, Bool
from shape_msgs.msg import SolidPrimitive

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, JointConstraint,
    PositionConstraint, OrientationConstraint, BoundingVolume,
)

# ── Arm configuration ─────────────────────────────────────────────────────────
PLANNING_FRAME = 'base_link'
TCP_LINK       = 'gripper_tcp'
ARM_GROUP      = 'arm'

ARM_JOINTS = [
    'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
    'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6',
]

NAMED_STATES = {
    'ready': [0.0, -0.5, -0.6, 1.1, 0.0, 0.0],
    'home':  [0.0,  0.0,  0.0, 0.0, 0.0, 0.0],
}

TOPDOWN_QUAT  = (-0.7071, 0.0, 0.0, 0.7071)   # TCP pointing straight down
HOVER_ABOVE   =  0.12    # metres above box centre during hover
GRASP_ABOVE   =  0.06    # metres above box centre at grasp

GRIPPER_OPEN  = [ 0.15,  0.15, -0.15, -0.15, -0.15,  0.15]
GRIPPER_GRASP = [-0.20, -0.20,  0.20,  0.20,  0.20, -0.20]
GRIPPER_TOPIC = '/mycobot_gripper_controller/commands'

PLAN_ATTEMPTS  = 10
PLAN_TIME_SEC  =  5.0
VEL_SCALE      =  0.2
ACC_SCALE      =  0.2
MOVEIT_SUCCESS =  1

# ── Visual approach parameters ────────────────────────────────────────────────
# Alignment
Y_TOL    = 0.03   # lateral tolerance to consider the box centred (m)
K_ROT    = 1.8    # rotation gain: rad/s per metre of lateral error
MAX_ROT  = 0.50   # maximum rotation speed (rad/s)

# Forward approach
K_FWD    = 0.60   # forward gain: m/s per metre of remaining range
MIN_FWD  = 0.05   # minimum forward speed while approaching
MAX_FWD  = 0.14   # maximum forward speed while approaching

# Stopping
STOP_RANGE        = 0.40   # stop here — arm grasp distance, well inside camera reliable range
NEAR_CLIP_RANGE   = 0.50   # start watching for drop-outs below this range
NEAR_CLIP_MISSES  = 2      # consecutive missed frames → near-clip hit → stop


class CamTrackPickTest(Node):

    def __init__(self):
        super().__init__('cam_track_pick_test')

        self._latest_box   = None
        self._weld_active  = False
        self._last_gripper = [0.0] * 6

        self.create_subscription(PoseStamped, '/box_pose',    self._box_cb,  10)
        self.create_subscription(Bool,        '/grasp_attach', self._weld_cb, 10)

        self._cmd_vel_pub = self.create_publisher(Twist,            '/cmd_vel',    10)
        self._gripper_pub = self.create_publisher(Float64MultiArray, GRIPPER_TOPIC, 10)
        self._attach_pub  = self.create_publisher(Bool,             '/grasp_attach', 10)

        self._move  = ActionClient(self, MoveGroup, '/move_action')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _box_cb(self, msg: PoseStamped):
        self._latest_box = msg

    def _weld_cb(self, msg: Bool):
        self._weld_active = msg.data

    # ── Box reading ───────────────────────────────────────────────────────────

    def _read_box(self):
        """One fresh /box_pose reading. Returns (x, y, z) in base_link or None.

        Waits up to 150 ms for a new message. Returns None if the reading is
        missing, in the wrong frame, or outside a plausible distance range.
        """
        self._latest_box = None
        t0 = time.time()
        while time.time() < t0 + 0.15:
            rclpy.spin_once(self, timeout_sec=0.05)
        p = self._latest_box
        if p is None or p.header.frame_id != PLANNING_FRAME:
            return None
        x, y, z = p.pose.position.x, p.pose.position.y, p.pose.position.z
        if x < 0.10 or x > 2.5:   # sanity check: not behind the robot, not infinitely far
            return None
        return x, y, z

    def _stable_box(self, samples=6, timeout=6.0):
        """Median-filtered box position from N valid readings.

        Used after the robot is stationary to get a refined grasp target
        without frame-to-frame depth noise.
        """
        readings = []
        deadline = time.time() + timeout
        while time.time() < deadline and len(readings) < samples:
            b = self._read_box()
            if b:
                readings.append(b)
        if len(readings) < 3:
            return None
        return (
            statistics.median(r[0] for r in readings),
            statistics.median(r[1] for r in readings),
            statistics.median(r[2] for r in readings),
        )

    # ── Phase 1: visual approach ──────────────────────────────────────────────

    def visual_approach(self):
        """Drive toward the box using only the camera.

        Returns the last valid (x, y, z) of the box in base_link at the
        moment the robot stops, or None if the box was never found.
        """
        self.get_logger().info('═══ Phase 1: visual approach ═══')

        def stop():
            self._cmd_vel_pub.publish(Twist())

        def clamp(v, lo, hi):
            return max(lo, min(hi, v))

        # ── Step A: rotate in place until the box is centred (bearing → 0) ──
        self.get_logger().info('Step A: rotating to face the box…')
        ok_count = 0
        deadline = time.time() + 20.0
        while time.time() < deadline:
            b = self._read_box()
            if b is None:
                stop()
                self.get_logger().info('waiting for /box_pose…', throttle_duration_sec=1.5)
                time.sleep(0.1)
                continue
            x, y, z = b
            self.get_logger().info(
                f'align  lateral={y:+.3f} m   range={x:.3f} m',
                throttle_duration_sec=0.5)
            if abs(y) < Y_TOL:
                ok_count += 1
                if ok_count >= 3:
                    self.get_logger().info('box centred ✓')
                    break
            else:
                ok_count = 0
            cmd = Twist()
            cmd.angular.z = clamp(K_ROT * y, -MAX_ROT, MAX_ROT)
            self._cmd_vel_pub.publish(cmd)
            time.sleep(0.05)
        stop()
        time.sleep(0.3)

        # ── Step B: drive forward, stop at near-clip boundary ───────────────
        # As the robot closes in, the depth sensor will hit its minimum reliable
        # range and readings will start dropping out.  The moment NEAR_CLIP_MISSES
        # consecutive frames are lost while the range is already short, the robot
        # stops and returns the last known position.
        self.get_logger().info('Step B: approaching box — monitoring for near-clip…')
        last_valid = None
        miss_streak = 0
        deadline = time.time() + 30.0

        while time.time() < deadline:
            b = self._read_box()

            if b is None:
                if last_valid is not None and last_valid[0] < NEAR_CLIP_RANGE:
                    miss_streak += 1
                    self.get_logger().info(
                        f'near-clip dropout {miss_streak}/{NEAR_CLIP_MISSES}  '
                        f'last range={last_valid[0]:.3f} m')
                    if miss_streak >= NEAR_CLIP_MISSES:
                        stop()
                        self.get_logger().info(
                            f'near-clip boundary — stopping.  '
                            f'box last seen @ {tuple(round(v, 3) for v in last_valid)}')
                        return last_valid
                stop()
                time.sleep(0.05)
                continue

            miss_streak = 0
            last_valid  = b
            x, y, z     = b

            # Secondary hard stop: reached arm's grasp distance
            if x <= STOP_RANGE:
                stop()
                self.get_logger().info(
                    f'hard stop: range {x:.3f} ≤ {STOP_RANGE} m  '
                    f'box @ ({x:.3f}, {y:.3f}, {z:.3f})')
                return b

            remaining = x - STOP_RANGE
            cmd = Twist()
            cmd.linear.x  = clamp(K_FWD * remaining, MIN_FWD, MAX_FWD)
            cmd.angular.z = clamp(K_ROT * y,          -MAX_ROT, MAX_ROT)
            self._cmd_vel_pub.publish(cmd)
            self.get_logger().info(
                f'approach  range={x:.3f}  remaining={remaining:.3f}  '
                f'lateral={y:+.3f}',
                throttle_duration_sec=0.4)
            time.sleep(0.05)

        stop()
        self.get_logger().warn('approach timed out')
        return last_valid

    # ── MoveIt helpers ────────────────────────────────────────────────────────

    def _wait_for_move_group(self, timeout=30.0):
        self.get_logger().info('waiting for move_group…')
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._move.wait_for_server(timeout_sec=2.0):
                self.get_logger().info('move_group ready ✓')
                return True
        self.get_logger().error('move_group not available')
        return False

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
        (target.orientation.x, target.orientation.y,
         target.orientation.z, target.orientation.w) = (float(q) for q in quat)

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

    def _send(self, constraints, label):
        goal = MoveGroup.Goal()
        req  = MotionPlanRequest()
        req.group_name                      = ARM_GROUP
        req.goal_constraints                = [constraints]
        req.num_planning_attempts           = PLAN_ATTEMPTS
        req.allowed_planning_time           = PLAN_TIME_SEC
        req.max_velocity_scaling_factor     = VEL_SCALE
        req.max_acceleration_scaling_factor = ACC_SCALE
        goal.request = req

        sf = self._move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sf, timeout_sec=15.0)
        if not sf.done():
            self.get_logger().error(f'[{label}] goal send timeout')
            return False
        gh = sf.result()
        if gh is None or not gh.accepted:
            self.get_logger().error(f'[{label}] goal rejected')
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=30.0)
        if not rf.done():
            self.get_logger().error(f'[{label}] no result within 30 s')
            return False
        ok = rf.result().result.error_code.val == MOVEIT_SUCCESS
        self.get_logger().info(f'[{label}] {"OK ✓" if ok else "FAILED ✗"}')
        return ok

    def go_named(self, name):
        return self._send(self._joint_constraints(NAMED_STATES[name]), name)

    def go_pose(self, x, y, z, q, label):
        return self._send(self._pose_constraints(x, y, z, q), label)

    # ── Gripper helpers ───────────────────────────────────────────────────────

    def set_gripper(self, values, label, steps=20, duration=0.8):
        start = self._last_gripper
        for i in range(1, steps + 1):
            a = i / steps
            interp = [s + a * (v - s) for s, v in zip(start, values)]
            msg = Float64MultiArray()
            msg.data = [float(v) for v in interp]
            self._gripper_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=duration / steps)
        self._last_gripper = list(values)
        self.get_logger().info(f'gripper → {label}')
        time.sleep(0.3)

    def attach(self, on: bool):
        msg = Bool()
        msg.data = bool(on)
        self._attach_pub.publish(msg)
        end = time.time() + 0.6
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().info(f'weld → {on}')

    # ── Phase 2: pick ─────────────────────────────────────────────────────────

    def pick(self, bx, by, bz):
        """Top-down grasp at (bx, by, bz) in base_link.

        Sequence: ready → open → hover → descend → weld → close → lift → home.
        """
        self.get_logger().info(
            f'═══ Phase 2: pick at ({bx:.3f}, {by:.3f}, {bz:.3f}) ═══')
        q = TOPDOWN_QUAT

        if not self._wait_for_move_group():
            return False

        self.go_named('ready')
        self.set_gripper(GRIPPER_OPEN, 'open')

        if not self.go_pose(bx, by, bz + HOVER_ABOVE, q, 'hover'):
            self.get_logger().error('hover IK failed — aborting pick')
            return False

        if not self.go_pose(bx, by, bz + GRASP_ABOVE, q, 'grasp'):
            self.get_logger().error('grasp IK failed — aborting pick')
            return False

        # Weld attaches the box rigidly to the gripper (Gazebo grasp aid)
        self.attach(True)
        self.set_gripper(GRIPPER_GRASP, 'close')
        time.sleep(0.3)

        lifted = False
        for attempt in range(3):
            if self.go_pose(bx, by, bz + HOVER_ABOVE, q, f'lift (try {attempt + 1})'):
                lifted = True
                break
            time.sleep(0.4)
        if not lifted:
            self.get_logger().warn('lift failed — box may still be held')

        self.go_named('home')
        self.get_logger().info('═══ Pick complete ✓ ═══')
        return True

    # ── Entry point ───────────────────────────────────────────────────────────

    def run(self):
        self.declare_parameter('pick_only', False)
        pick_only = self.get_parameter('pick_only').value

        self.get_logger().info('══════════════════════════════════════════')
        self.get_logger().info('  Camera-track-and-pick test — starting   ')
        self.get_logger().info(f'  Mode: {"PICK ONLY" if pick_only else "APPROACH + PICK"}')
        self.get_logger().info('══════════════════════════════════════════')

        # box_z: override the camera's z estimate with a known table height.
        # The depth camera reliably measures x (forward distance) and y (lateral),
        # but z (vertical) is noisy at close range — the camera is only 3 cm above
        # base_link, so a box on the table appears near zero or negative in base_link.
        # Set this to the known box-centre height above base_link (default: 0.08 m).
        self.declare_parameter('box_z', 0.08)
        box_z_override = float(self.get_parameter('box_z').value)

        if pick_only:
            # Skip navigation — robot is already in position.
            # Read a stable box position and go straight to the arm.
            self.get_logger().info('pick_only mode — reading box position…')
            box = self._stable_box(samples=6, timeout=8.0)
            if box is None:
                self.get_logger().error('No box detected on /box_pose — aborting')
                return
            bx, by, bz = box
            bz = box_z_override
            self.get_logger().info(
                f'Box at ({bx:.3f}, {by:.3f}) z={bz:.3f} (fixed) — proceeding to pick')
        else:
            # Phase 1: drive toward the box using only the camera
            box = self.visual_approach()
            if box is None:
                self.get_logger().error('Box never found — aborting')
                return
            bx, by, bz = box
            self.get_logger().info(
                f'Stopped at box position: ({bx:.3f}, {by:.3f}, {bz:.3f}) in base_link')

            # Re-sample while stationary for a more accurate grasp target
            self.get_logger().info('Re-sampling box position (robot stationary)…')
            stable = self._stable_box(samples=6, timeout=5.0)
            if stable is not None:
                bx, by, bz = stable
                self.get_logger().info(
                    f'Refined position: ({bx:.3f}, {by:.3f}, {bz:.3f})')
            else:
                self.get_logger().warn('Re-sample failed — using last approach reading')

        # Phase 2: pick
        self.pick(bx, by, bz)


def main(args=None):
    rclpy.init(args=args)
    node = CamTrackPickTest()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
