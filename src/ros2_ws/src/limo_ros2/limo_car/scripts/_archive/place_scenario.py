#!/usr/bin/python3
"""
place_scenario.py — Scenario 2: robot starts with box already gripped.

Proves the gripper can hold the box through navigation and execute a clean
place, without depending on the pick pipeline at all.

Sequence:
  1. Arm → ready pose, gripper opens
  2. TF lookup: get gripper_tcp world position
  3. Teleport box to gripper_tcp (box appears between the open fingers)
  4. Weld fires immediately → box rigidly attached to gripper_tcp
  5. Gripper closes (adaptive, stops on contact — no explosion)
  6. Arm → travel pose (folded, safe for driving — box follows via weld)
  7. Navigate to table (Nav2 → same dock as pick scenario)
  8. Arm → ready → hover above table → lower to place height
  9. Open gripper + release weld → box rests on table
  10. Arm → ready → travel

Run AFTER:
  ros2 launch limo_car nav_pick.launch.py
  (or a minimal launch without the nav_pick_orchestrator)
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

import tf2_ros

from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped, Twist
from action_msgs.msg import GoalStatus
from shape_msgs.msg import SolidPrimitive

from nav2_msgs.action import NavigateToPose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, JointConstraint,
    PositionConstraint, OrientationConstraint, BoundingVolume,
)
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition


# ── Config ────────────────────────────────────────────────────────────────────
BOX_NAME = 'target_box'

TOPDOWN_QUAT = (-0.7071, 0.0, 0.0, 0.7071)

# Nav2 dock goal (same approach as pick scenario)
NAV_X   = -2.0
NAV_Y   =  4.60
NAV_YAW = -1.5708

# Place position in base_link at dock (from pick scenario logs)
PLACE_X     =  0.240
PLACE_Y     =  0.003
PLACE_Z_BOX = -0.021   # box centre z in base_link when resting on table
HOVER_ABOVE =  0.10    # TCP above box centre during approach hover
PLACE_ABOVE =  0.0    # TCP above box centre at grasp. 0 since 2026-08-01: gripper_tcp was
                       # moved to the real grasp point (ackermann_with_sensor.xacro),
                       # so the tool goes straight to the box centre. Was 0.06, which
                       # was silently correcting for the TCP sitting on the palm.

# Gripper
GRIPPER_OPEN  = [ 0.15,  0.15, -0.15, -0.15, -0.15,  0.15]
GRIPPER_GRASP = [-0.20, -0.20,  0.20,  0.20,  0.20, -0.20]
GRIPPER_TOPIC = '/mycobot_gripper_controller/commands'

CLOSE_STEP_RAD = 0.005
CLOSE_STEP_SEC = 0.08

# MoveIt
ARM_JOINTS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
              'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
READY_J  = [0.0, -0.5, -0.6,  1.1, 0.0, 0.0]
TRAVEL_J = [0.0,  1.2, -0.6, -0.6, 0.0, 0.0]

PLANNING_FRAME = 'base_link'
TCP_LINK       = 'gripper_tcp'
ARM_GROUP      = 'arm'
MOVEIT_SUCCESS = 1
VEL_SCALE = 0.2
ACC_SCALE = 0.2


class PlaceScenario(Node):

    def __init__(self):
        super().__init__('place_scenario')

        self._move    = ActionClient(self, MoveGroup, '/move_action')
        self._nav     = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._grip    = self.create_publisher(Float64MultiArray, GRIPPER_TOPIC, 10)
        self._weld    = self.create_publisher(Bool, '/grasp_attach', 10)
        self._cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self._amcl    = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        self._set_state = self.create_client(SetEntityState, '/set_entity_state')

        self._last_grip   = list(GRIPPER_OPEN)
        self._weld_active = False
        self.create_subscription(Bool, '/grasp_attach', self._weld_cb, 10)

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

    # ── Weld ──────────────────────────────────────────────────────────────────

    def _weld_cb(self, msg: Bool):
        self._weld_active = msg.data

    def set_weld(self, on: bool):
        msg = Bool()
        msg.data = on
        self._weld.publish(msg)
        self._weld_active = on
        time.sleep(0.4)
        self.get_logger().info(f'weld → {"ON" if on else "OFF"}')

    # ── Box teleport ──────────────────────────────────────────────────────────

    def teleport_box_to_gripper(self):
        """Look up gripper_tcp in world (odom) frame and teleport box there."""
        self.get_logger().info('Looking up gripper_tcp TF…')
        for _ in range(30):
            try:
                tf = self._tf_buffer.lookup_transform(
                    'odom', TCP_LINK,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=1.0))
                break
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.2)
        else:
            self.get_logger().error('Could not get gripper_tcp TF — aborting')
            return False

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        z = tf.transform.translation.z
        self.get_logger().info(f'gripper_tcp in odom: ({x:.3f}, {y:.3f}, {z:.3f})')

        st = EntityState()
        st.name = BOX_NAME
        st.pose.position = Point(x=x, y=y, z=z)
        st.pose.orientation.w = 1.0
        st.reference_frame = 'world'
        req = SetEntityState.Request()
        req.state = st

        if not self._set_state.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/set_entity_state unavailable')
            return False
        fut = self._set_state.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        self.get_logger().info(f'box teleported to gripper_tcp position')
        time.sleep(0.3)
        return True

    # ── Gripper ───────────────────────────────────────────────────────────────

    def set_gripper(self, target, label, duration=0.8, steps=20):
        start = self._last_grip[:]
        for i in range(1, steps + 1):
            a = i / steps
            msg = Float64MultiArray()
            msg.data = [float(s + a * (v - s)) for s, v in zip(start, target)]
            self._grip.publish(msg)
            rclpy.spin_once(self, timeout_sec=duration / steps)
        self._last_grip = list(target)
        self.get_logger().info(f'gripper → {label}')
        time.sleep(0.3)

    def close_until_contact(self):
        """Close step by step; stop the instant the weld fires (bilateral contact)."""
        self._weld_active = False
        j0    = self._last_grip[0]
        floor = GRIPPER_GRASP[0]
        steps = 0
        while j0 > floor:
            j0 = max(floor, j0 - CLOSE_STEP_RAD)
            t  = (GRIPPER_OPEN[0] - j0) / (GRIPPER_OPEN[0] - GRIPPER_GRASP[0])
            cmd = Float64MultiArray()
            cmd.data = [float(o + t * (c - o))
                        for o, c in zip(GRIPPER_OPEN, GRIPPER_GRASP)]
            self._grip.publish(cmd)
            self._last_grip = cmd.data[:]
            steps += 1
            rclpy.spin_once(self, timeout_sec=CLOSE_STEP_SEC)
            if self._weld_active:
                self.get_logger().info(
                    f'Contact confirmed at j0={j0:.3f} rad ({steps * CLOSE_STEP_SEC:.1f} s) — holding')
                break
        else:
            self.get_logger().warn('Reached close limit — no contact detected')
        time.sleep(0.3)

    # ── MoveIt ────────────────────────────────────────────────────────────────

    def _send(self, constraints, label):
        goal = MoveGroup.Goal()
        req  = MotionPlanRequest()
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
            self.get_logger().error(f'[{label}] goal rejected')
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
            jc.joint_name      = name
            jc.position        = float(val)
            jc.tolerance_above = 0.02
            jc.tolerance_below = 0.02
            jc.weight          = 1.0
            c.joint_constraints.append(jc)
        return self._send(c, label)

    def go_pose(self, x, y, z, q, label, pos_tol=0.025, ori_tol=0.1):
        tg = Pose()
        tg.position = Point(x=float(x), y=float(y), z=float(z))
        tg.orientation.x, tg.orientation.y = float(q[0]), float(q[1])
        tg.orientation.z, tg.orientation.w = float(q[2]), float(q[3])

        pc = PositionConstraint()
        pc.header.frame_id = PLANNING_FRAME
        pc.link_name       = TCP_LINK
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
        oc.link_name       = TCP_LINK
        oc.orientation     = tg.orientation
        oc.absolute_x_axis_tolerance = ori_tol
        oc.absolute_y_axis_tolerance = ori_tol
        oc.absolute_z_axis_tolerance = ori_tol
        oc.weight = 1.0

        c = Constraints()
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        return self._send(c, label)

    # ── Navigation ────────────────────────────────────────────────────────────

    def _publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.pose.pose.position.x =  -2.0
        msg.pose.pose.position.y =   7.0
        msg.pose.pose.orientation.z = -0.7071
        msg.pose.pose.orientation.w =  0.7071
        msg.pose.covariance[0]  = 0.25
        msg.pose.covariance[7]  = 0.25
        msg.pose.covariance[35] = 0.04
        for _ in range(10):
            msg.header.stamp = self.get_clock().now().to_msg()
            self._amcl.publish(msg)
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.05)

    def _activate_nav2(self):
        for node_name in ('controller_server', 'velocity_smoother', 'behavior_server'):
            cli = self.create_client(ChangeState, f'/{node_name}/change_state')
            if not cli.wait_for_service(timeout_sec=3.0):
                continue
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_ACTIVATE
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)

    def navigate(self):
        self.get_logger().info(f'=== Navigating to table ({NAV_X}, {NAV_Y}) ===')
        self._activate_nav2()
        self._publish_initial_pose()
        time.sleep(1.0)

        if not self._nav.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('Nav2 not available')
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(NAV_X)
        goal.pose.pose.position.y = float(NAV_Y)
        half = NAV_YAW / 2.0
        goal.pose.pose.orientation.z = math.sin(half)
        goal.pose.pose.orientation.w = math.cos(half)

        for attempt in range(15):
            fut = self._nav.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
            if fut.done() and fut.result() is not None and fut.result().accepted:
                break
            self.get_logger().warn(f'Nav2 goal retry {attempt + 1}/15…')
            time.sleep(2.0)
        else:
            self.get_logger().error('Nav2 goal rejected after retries')
            return False

        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=120.0)
        ok = res.result().status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info(f'Navigation {"complete" if ok else "FAILED"}')
        return ok

    # ── Main sequence ─────────────────────────────────────────────────────────

    def run(self):
        self.get_logger().info(
            '\n╔═══════════════════════════════════════════════╗\n'
            '║  SCENARIO 2 — Place Only                      ║\n'
            '║  Robot starts with box gripped, places it     ║\n'
            '║  on the table to prove gripper hold works.    ║\n'
            '╚═══════════════════════════════════════════════╝')

        # Wait for move_group
        self.get_logger().info('Waiting for /move_action…')
        self._move.wait_for_server(timeout_sec=60.0)

        # ── Step 1: Arm to ready, gripper open ──────────────────────────────
        self.get_logger().info('=== Step 1: Ready pose + open gripper ===')
        self.set_gripper(GRIPPER_OPEN, 'open')
        if not self.go_joints(READY_J, 'ready'):
            self.get_logger().error('Could not reach ready pose — aborting')
            return

        # ── Step 2: Teleport box to gripper_tcp ─────────────────────────────
        self.get_logger().info('=== Step 2: Teleport box to gripper ===')
        if not self.teleport_box_to_gripper():
            return
        time.sleep(0.5)

        # ── Step 3: Weld the box immediately ────────────────────────────────
        self.get_logger().info('=== Step 3: Weld box to gripper_tcp ===')
        self.set_weld(True)

        # ── Step 4: Close gripper (adaptive — stops on contact) ──────────────
        self.get_logger().info('=== Step 4: Close fingers ===')
        self.close_until_contact()

        # ── Step 5: Travel pose (arm folded for driving) ─────────────────────
        self.get_logger().info('=== Step 5: Travel pose (box follows via weld) ===')
        if not self.go_joints(TRAVEL_J, 'travel'):
            self.get_logger().warn('Travel pose failed — continuing anyway')

        # ── Step 6: Navigate to table ────────────────────────────────────────
        self.get_logger().info('=== Step 6: Navigate to table ===')
        if not self.navigate():
            self.get_logger().error('Navigation failed — aborting place')
            return
        time.sleep(0.5)

        # ── Step 7: Arm to ready, hover above table, lower ───────────────────
        self.get_logger().info('=== Step 7: Place sequence ===')
        if not self.go_joints(READY_J, 'ready'):
            self.get_logger().warn('Ready pose failed — trying hover directly')

        if not self.go_pose(PLACE_X, PLACE_Y, PLACE_Z_BOX + HOVER_ABOVE,
                            TOPDOWN_QUAT, 'place_hover'):
            self.get_logger().error('hover pose failed — aborting place')
            return

        if not self.go_pose(PLACE_X, PLACE_Y, PLACE_Z_BOX + PLACE_ABOVE,
                            TOPDOWN_QUAT, 'place_lower'):
            self.get_logger().error('lower pose failed — aborting place')
            return

        # ── Step 8: Release ───────────────────────────────────────────────────
        self.get_logger().info('=== Step 8: Release — box placed on table ===')
        self.set_weld(False)
        time.sleep(0.3)
        self.set_gripper(GRIPPER_OPEN, 'open')
        time.sleep(1.0)

        # ── Step 9: Retract ───────────────────────────────────────────────────
        self.get_logger().info('=== Step 9: Retract arm ===')
        self.go_pose(PLACE_X, PLACE_Y, PLACE_Z_BOX + HOVER_ABOVE,
                     TOPDOWN_QUAT, 'retreat')
        self.go_joints(READY_J, 'ready')
        self.go_joints(TRAVEL_J, 'travel')

        self.get_logger().info(
            '\n╔═══════════════════════════════════════╗\n'
            '║  SCENARIO 2 COMPLETE                  ║\n'
            '║  Box placed on table.                 ║\n'
            '╚═══════════════════════════════════════╝')


def main(args=None):
    rclpy.init(args=args)
    node = PlaceScenario()
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
