#!/usr/bin/python3
"""
calib_pick.py — Isolated gripper calibration loop (no navigation).

The robot is spawned pre-docked by test_grasp_isolation.launch.py.
This node runs N pick-and-verify cycles automatically:

  1. Reset box to spawn position via /set_entity_state
  2. Open gripper, arm to ready
  3. Hover TCP above box (target from BOX_BASE_X/Y/Z + live offsets)
  4. Descend to grasp height, close gripper (slow)
  5. Lift arm
  6. Query /get_entity_state — did the box actually rise? → PASS / FAIL
  7. Print per-trial result; loop until done
  8. Print summary table

Ground truth: box world-Z must reach BOX_SUCCESS_Z (default 0.18 m).
Box starts at world Z=0.12 m, so it must rise >6 cm.  This cannot be
faked by the weld — if smart_grasp fires the weld and the arm lifts,
the box will reach 0.20+ m.  If the weld never fires (fingers missed),
the box stays at 0.12 m → FAIL.

Live-tune between iterations (no relaunch):
    ros2 param set /calib_pick grasp_off_x  0.010  # shift target forward (+X)
    ros2 param set /calib_pick grasp_off_y -0.005  # shift target laterally (+Y)
    ros2 param set /calib_pick grasp_off_z  0.000  # shift target up/down (+Z)
    ros2 param set /calib_pick close_sec    3.0    # gripper close speed (s)

Usage:
    ros2 run limo_car calib_pick
    ros2 run limo_car calib_pick --ros-args -p loops:=20 -p close_sec:=3.0
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import Point, Pose
from shape_msgs.msg import SolidPrimitive

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, JointConstraint,
    PositionConstraint, OrientationConstraint, BoundingVolume,
)
from gazebo_msgs.srv import GetEntityState, SetEntityState
from gazebo_msgs.msg import EntityState


# ── Grasp geometry (base_link frame, read from logs at dock position) ─────────
# From last run: box perceived at (0.240, -0.012, -0.021) in base_link.
# Use grasp_off_x/y/z params to shift without relaunch.
BOX_BASE_X = 0.240
BOX_BASE_Y = 0.000   # start centred; tune y with grasp_off_y
BOX_BASE_Z = -0.021

HOVER_ABOVE = 0.12   # TCP above box Z during hover
GRASP_ABOVE = 0.0    # TCP above box centre at grasp. 0 since 2026-08-01: gripper_tcp was
                       # moved to the real grasp point (ackermann_with_sensor.xacro),
                       # so the tool goes straight to the box centre. Was 0.06, which
                       # was silently correcting for the TCP sitting on the palm.
LIFT_ABOVE  = 0.16   # TCP above box Z during lift (verify phase)

# ── Gripper ───────────────────────────────────────────────────────────────────
GRIPPER_OPEN  = [ 0.15,  0.15, -0.15, -0.15, -0.15,  0.15]
GRIPPER_GRASP = [-0.20, -0.20,  0.20,  0.20,  0.20, -0.20]
GRIPPER_TOPIC = '/mycobot_gripper_controller/commands'

# close_until_contact: step this many radians per tick, pause this many seconds.
# Stops as soon as smart_grasp fires the weld — no fixed end angle.
# Safe floor: never closes past GRIPPER_GRASP even without contact.
CLOSE_STEP_RAD = 0.005   # 0.5 deg per tick (very smooth)
CLOSE_STEP_SEC = 0.08    # 80 ms per tick → max 5.6 s to close fully

# ── MoveIt ────────────────────────────────────────────────────────────────────
TOPDOWN_QUAT = (-0.7071, 0.0, 0.0, 0.7071)
PLANNING_FRAME = 'base_link'
TCP_LINK       = 'gripper_tcp'
ARM_GROUP      = 'arm'
ARM_JOINTS     = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
                  'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
READY_J = [0.0, -0.5, -0.6, 1.1, 0.0, 0.0]
HOME_J  = [0.0,  0.0,  0.0, 0.0, 0.0, 0.0]
MOVEIT_SUCCESS = 1
VEL_SCALE = 0.2
ACC_SCALE = 0.2

# ── Success gate ──────────────────────────────────────────────────────────────
BOX_NAME        = 'target_box'
BOX_RESET_WORLD = [-2.0, 4.0, 0.12]   # world frame: on table, centred
BOX_SUCCESS_Z   = 0.18                 # world Z — box must clear this to count


class CalibPick(Node):

    def __init__(self):
        super().__init__('calib_pick')

        self.declare_parameter('loops',        5)
        self.declare_parameter('grasp_off_x',  0.0)
        self.declare_parameter('grasp_off_y',  0.013)
        self.declare_parameter('grasp_off_z',  0.0)
        self.declare_parameter('close_sec',    4.0)
        self.declare_parameter('box_name',     BOX_NAME)

        self._move       = ActionClient(self, MoveGroup, '/move_action')
        self._grip       = self.create_publisher(Float64MultiArray, GRIPPER_TOPIC, 10)
        self._last_grip  = [0.0] * 6
        self._get_state  = self.create_client(GetEntityState, '/get_entity_state')
        self._set_state  = self.create_client(SetEntityState, '/set_entity_state')

        # Track weld state so close_until_contact knows when to stop
        self._weld_active = False
        self.create_subscription(Bool, '/grasp_attach', self._weld_cb, 10)

    # ── Param helpers ─────────────────────────────────────────────────────────

    def _offset(self):
        return (self.get_parameter('grasp_off_x').value,
                self.get_parameter('grasp_off_y').value,
                self.get_parameter('grasp_off_z').value)

    # ── Gazebo helpers ────────────────────────────────────────────────────────

    def reset_box(self):
        name = self.get_parameter('box_name').value
        x, y, z = BOX_RESET_WORLD
        st = EntityState()
        st.name = name
        st.pose.position = Point(x=float(x), y=float(y), z=float(z))
        st.pose.orientation.w = 1.0
        st.reference_frame = 'world'
        req = SetEntityState.Request()
        req.state = st
        if self._set_state.wait_for_service(timeout_sec=5.0):
            fut = self._set_state.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
            self.get_logger().info(f'box reset → world ({x:.2f}, {y:.2f}, {z:.2f})')

    def check_success(self):
        """Query Gazebo for box world Z. Returns (z_value, bool) or None on error."""
        name = self.get_parameter('box_name').value
        req = GetEntityState.Request()
        req.name = name
        req.reference_frame = 'world'
        if not self._get_state.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('/get_entity_state unavailable')
            return None
        fut = self._get_state.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        if not fut.done() or not fut.result().success:
            self.get_logger().error('GetEntityState failed')
            return None
        z = fut.result().state.pose.position.z
        return z, z >= BOX_SUCCESS_Z

    # ── Gripper ───────────────────────────────────────────────────────────────

    def _weld_cb(self, msg: Bool):
        self._weld_active = msg.data

    def close_until_contact(self):
        """Close one step at a time; stop the instant smart_grasp fires the weld.

        This prevents the "explosion" that happens when a fixed closing angle
        keeps building contact force after the box is already gripped.
        It also handles any object size: the gripper stops at whatever opening
        fits the object, never pushing past the first bilateral contact.

        Safety floor: never closes past GRIPPER_GRASP even with no contact
        (protects fingers from grinding against each other on a missed pick).
        """
        self._weld_active = False   # clear any stale weld state before closing

        j0 = self._last_grip[0]     # current joint-0 angle (starts at +0.15 open)
        floor = GRIPPER_GRASP[0]    # -0.20 rad — never go past this

        steps_taken = 0
        while j0 > floor:
            j0 = max(floor, j0 - CLOSE_STEP_RAD)

            # Linear interpolation across all 6 joints: t=0 fully open, t=1 fully closed
            t = (GRIPPER_OPEN[0] - j0) / (GRIPPER_OPEN[0] - GRIPPER_GRASP[0])
            cmd = Float64MultiArray()
            cmd.data = [float(o + t * (c - o))
                        for o, c in zip(GRIPPER_OPEN, GRIPPER_GRASP)]
            self._grip.publish(cmd)
            self._last_grip = cmd.data[:]
            steps_taken += 1

            rclpy.spin_once(self, timeout_sec=CLOSE_STEP_SEC)

            if self._weld_active:
                self.get_logger().info(
                    f'[close] Contact → weld fired  j0={j0:.3f} rad  '
                    f't={t:.2f}  ({steps_taken * CLOSE_STEP_SEC:.1f} s elapsed)  HOLDING')
                break
        else:
            if not self._weld_active:
                self.get_logger().warn(
                    f'[close] Reached floor {floor:.2f} rad — no bilateral contact. '
                    f'Box too small, missed, or contact sensors not firing.')

        time.sleep(0.3)

    def set_gripper(self, target, label, duration=None, steps=None):
        close_sec = float(self.get_parameter('close_sec').value)
        dur   = duration if duration is not None else close_sec
        steps = steps    if steps    is not None else max(20, int(dur * 20))
        start = self._last_grip[:]
        for i in range(1, steps + 1):
            a = i / steps
            msg = Float64MultiArray()
            msg.data = [float(s + a * (v - s)) for s, v in zip(start, target)]
            self._grip.publish(msg)
            rclpy.spin_once(self, timeout_sec=dur / steps)
        self._last_grip = list(target)
        self.get_logger().info(f'gripper → {label}  ({dur:.1f} s)')
        time.sleep(0.3)

    # ── MoveIt ────────────────────────────────────────────────────────────────

    def _send(self, constraints, label):
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = ARM_GROUP
        req.goal_constraints = [constraints]
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = VEL_SCALE
        req.max_acceleration_scaling_factor = ACC_SCALE
        goal.request = req
        fut = self._move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        if not fut.done() or not fut.result().accepted:
            self.get_logger().error(f'[{label}] rejected')
            return False
        r = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self, r, timeout_sec=30.0)
        ok = r.result().result.error_code.val == MOVEIT_SUCCESS
        self.get_logger().info(f'[{label}] {"OK" if ok else "FAILED"}')
        return ok

    def go_joints(self, joints, label):
        c = Constraints()
        for name, val in zip(ARM_JOINTS, joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(val)
            jc.tolerance_above = 0.02
            jc.tolerance_below = 0.02
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        return self._send(c, label)

    def go_pose(self, x, y, z, q, label, pos_tol=0.025, ori_tol=0.1):
        tg = Pose()
        tg.position = Point(x=float(x), y=float(y), z=float(z))
        tg.orientation.x, tg.orientation.y = float(q[0]), float(q[1])
        tg.orientation.z, tg.orientation.w = float(q[2]), float(q[3])

        pc = PositionConstraint()
        pc.header.frame_id = PLANNING_FRAME
        pc.link_name = TCP_LINK
        bv = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [pos_tol]
        bv.primitives.append(sp)
        bv.primitive_poses.append(tg)
        pc.constraint_region = bv
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = PLANNING_FRAME
        oc.link_name = TCP_LINK
        oc.orientation = tg.orientation
        oc.absolute_x_axis_tolerance = ori_tol
        oc.absolute_y_axis_tolerance = ori_tol
        oc.absolute_z_axis_tolerance = ori_tol
        oc.weight = 1.0

        c = Constraints()
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        return self._send(c, label)

    # ── One trial ─────────────────────────────────────────────────────────────

    def one_trial(self, n):
        ox, oy, oz = self._offset()
        tx = BOX_BASE_X + ox
        ty = BOX_BASE_Y + oy
        tz = BOX_BASE_Z + oz

        self.get_logger().info(
            f'\n┌── Trial {n} ─────────────────────────────────────\n'
            f'│  grasp target : ({tx:.3f}, {ty:.3f}, {tz:.3f}) base_link\n'
            f'│  offset applied: ({ox:+.3f}, {oy:+.3f}, {oz:+.3f})\n'
            f'│  close_sec    : {self.get_parameter("close_sec").value:.1f} s\n'
            f'└────────────────────────────────────────────────')

        # Open gripper and go to ready
        self.set_gripper(GRIPPER_OPEN, 'open', duration=0.8, steps=20)
        if not self.go_joints(READY_J, 'ready'):
            return None, 'IK failed at ready'

        # Hover above box
        if not self.go_pose(tx, ty, tz + HOVER_ABOVE, TOPDOWN_QUAT, 'hover'):
            return None, f'IK failed at hover ({tx:.3f}, {ty:.3f}, {tz + HOVER_ABOVE:.3f})'

        # Descend to grasp height
        if not self.go_pose(tx, ty, tz + GRASP_ABOVE, TOPDOWN_QUAT, 'grasp'):
            return None, 'IK failed at grasp descent'

        # Adaptive close — stops the moment both fingers contact the box.
        # Prevents the "explosion" from force buildup past first contact.
        self.close_until_contact()
        time.sleep(0.3)   # brief settle after weld fires

        # Lift
        lifted = False
        for attempt in range(3):
            if self.go_pose(tx, ty, tz + LIFT_ABOVE, TOPDOWN_QUAT, f'lift({attempt+1})'):
                lifted = True
                break
            time.sleep(0.3)

        if not lifted:
            self.get_logger().warn('lift failed — box might still have moved slightly')

        # Ground-truth check via Gazebo
        time.sleep(0.5)
        result = self.check_success()
        if result is None:
            return None, 'Gazebo GetEntityState unavailable'

        box_z, passed = result
        rise = box_z - BOX_RESET_WORLD[2]
        return passed, f'box_z={box_z:.4f} m  (rose {rise*100:.1f} cm)'

    # ── Main loop ─────────────────────────────────────────────────────────────

    def run(self):
        loops = self.get_parameter('loops').value

        self.get_logger().info('Waiting for /move_action…')
        self._move.wait_for_server(timeout_sec=60.0)
        self.get_logger().info('move_group connected')

        results = []   # list of (passed: bool|None, detail: str)

        self.get_logger().info(
            f'\n╔══════════════════════════════════════════════════╗\n'
            f'║  CalibPick  —  {loops} trial(s)                       ║\n'
            f'║  Tune live (no relaunch):                        ║\n'
            f'║    ros2 param set /calib_pick grasp_off_x 0.010 ║\n'
            f'║    ros2 param set /calib_pick grasp_off_y 0.005 ║\n'
            f'║    ros2 param set /calib_pick close_sec   3.0   ║\n'
            f'╚══════════════════════════════════════════════════╝')

        for i in range(1, loops + 1):
            self._set_state.wait_for_service(timeout_sec=5.0)
            self.reset_box()
            time.sleep(1.0)   # let physics settle after teleport

            passed, detail = self.one_trial(i)
            results.append((passed, detail))

            if passed is True:
                self.get_logger().info(f'  → ✓ PASS   {detail}')
            elif passed is False:
                self.get_logger().warn(f'  → ✗ FAIL   {detail}')
            else:
                self.get_logger().error(f'  → ? ERROR  {detail}')

            if i < loops:
                time.sleep(1.5)

        # Summary
        passes   = sum(1 for p, _ in results if p is True)
        fails    = sum(1 for p, _ in results if p is False)
        errors   = sum(1 for p, _ in results if p is None)
        rate     = passes / (passes + fails) * 100 if (passes + fails) > 0 else 0

        self.get_logger().info(
            f'\n╔══════════════════════════════════════╗\n'
            f'║  SUMMARY  ({loops} trials)              ║\n'
            f'║  ✓ PASS  : {passes:>2}  |  '
            f'✗ FAIL  : {fails:>2}  |  ? ERR: {errors:>2} ║\n'
            f'║  Success rate: {rate:.0f}%                  ║\n'
            f'╚══════════════════════════════════════╝\n'
            f'Per-trial breakdown:')
        for j, (p, d) in enumerate(results, 1):
            sym = '✓' if p is True else ('✗' if p is False else '?')
            self.get_logger().info(f'  Trial {j:02d}: {sym}  {d}')

        # Return to home
        self.set_gripper(GRIPPER_OPEN, 'open', duration=0.8, steps=20)
        self.go_joints(HOME_J, 'home')


def main(args=None):
    rclpy.init(args=args)
    node = CalibPick()
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
