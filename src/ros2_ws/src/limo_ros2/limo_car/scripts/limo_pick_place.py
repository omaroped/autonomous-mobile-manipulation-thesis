#!/usr/bin/python3
"""
Integrated pick-and-place node for LIMO + myCobot 280.

Phase 1 — Navigate: drive toward the blue box using the depth camera.
          Stops when the box is ~0.17m from camera (= ~0.27m from arm base).
Phase 2 — Pick:     execute top-down grasp sequence using J5=0 poses.
Phase 3 — Lift:     raise arm to carry position.

All arm poses use J5=0 for true top-down gripper orientation and
J1=-1.5708 because the arm mount was rotated +90° CCW (screen faces side).

Box is on a 10cm table (top at z=0.10m), box center at z=0.125m.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import threading


# ── Arm joint names (controller order) ───────────────────────────────────────
JOINT_NAMES = [
    'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
    'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6'
]

# ── Top-down arm poses (J1=-π/2, J5=0) ───────────────────────────────────────
# J1=-1.5708 compensates for the 90° CCW mount rotation so arm faces forward.
# J5=0 keeps gripper pointing straight down.
# Constraints: J2+J3+J4 ≈ -π/2 maintains top-down orientation.

HOME          = [-1.5708,  0.0,   0.0,  0.0,   0.0, 0.0]  # arm vertical, safe

# Hover above the box with gripper open (down_reach_5 height: ~0.208m)
PRE_GRASP     = [-1.5708, -1.8,   0.4, -0.17,  0.0, 0.0]

# Lower fingers to box-center height (down_reach_7 height: ~0.168m, reach ~0.257m)
GRASP         = [-1.5708, -2.2,   0.8, -0.17,  0.0, 0.0]

# Lift after gripping (same as pre-grasp height but gripper closed)
LIFT          = [-1.5708, -1.8,   0.4, -0.17,  0.0, 0.0]

# Carry — arm tucked but not colliding with base
CARRY         = [-1.5708, -1.0,   0.0,  0.0,   0.0, 0.0]

# Gripper: 6 joints in controller order — explicitly commanded so all fingers move.
# Multipliers: left2×1, left1×-1, right3×-1, right2×-1, right1×1 (mirror of left side)
# cmd=0.15 → open; cmd=-0.50 → closed (capped to keep all joints within their limits)
GRIPPER_OPEN  = [ 0.15,  0.15, -0.15, -0.15, -0.15,  0.15]
GRIPPER_CLOSE = [-0.50, -0.50,  0.50,  0.50,  0.50, -0.50]

# ── Navigation constants ──────────────────────────────────────────────────────
TARGET_DISTANCE = 0.20   # metres from camera to box (= ~0.30m from arm base)
DIST_TOLERANCE  = 0.025  # ±2.5cm acceptable
ALIGN_TOLERANCE = 0.10   # max heading error before arm fires (box must be near centre)
APPROACH_SPEED  = 0.25   # m/s max
KP_LINEAR       = 0.6
KP_ANGULAR      = 1.0    # reduced from 1.5 — prevents over-steer oscillation at close range

# HSV range for the vivid blue Gazebo box
BLUE_HSV_LO = np.array([110, 150,  50])
BLUE_HSV_HI = np.array([130, 255, 255])


class PickPlaceNode(Node):

    def __init__(self):
        super().__init__('limo_pick_place')

        self.bridge = CvBridge()
        self._lock = threading.Lock()

        # ── Publishers ────────────────────────────────────────────────────────
        self.cmd_vel_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_pub  = self.create_publisher(
            Float64MultiArray, '/mycobot_gripper_controller/commands', 10)

        # ── Action client ─────────────────────────────────────────────────────
        self.arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/mycobot_arm_controller/follow_joint_trajectory')

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(Image, '/rgb/image_raw',
                                 self._rgb_cb, 10)
        self.create_subscription(Image, '/depth_camera/depth/image_raw',
                                 self._depth_cb, 10)
        self.create_subscription(JointState, '/joint_states',
                                 self._joint_cb, 10)

        # ── State ─────────────────────────────────────────────────────────────
        self._latest_depth = None
        self._joint_positions = [0.0] * 6
        self._joint_ready = False

        # Phase: 'navigate' → 'pre_grasp' → 'grasp' → 'grip' → 'lift' → 'done'
        self._phase = 'navigate'
        self._phase_start = time.time()
        self._nav_reached = False

        # 10 Hz control loop
        self.create_timer(0.1, self._control_loop)

        self.get_logger().info('PickPlace node started — Phase: NAVIGATE')

    # ── Sensor callbacks ──────────────────────────────────────────────────────

    def _depth_cb(self, msg):
        with self._lock:
            self._latest_depth = msg

    def _joint_cb(self, msg):
        for i, name in enumerate(JOINT_NAMES):
            if name in msg.name:
                idx = msg.name.index(name)
                self._joint_positions[i] = msg.position[idx]
        self._joint_ready = True

    def _rgb_cb(self, msg):
        if self._phase != 'navigate':
            return

        with self._lock:
            depth_msg = self._latest_depth

        if depth_msg is None:
            return

        try:
            bgr   = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, '32FC1')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        h, w = bgr.shape[:2]
        cx = w / 2.0

        hsv  = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, BLUE_HSV_LO, BLUE_HSV_HI)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()
        box_found = False

        if contours:
            candidates = []
            for c in contours:
                if cv2.contourArea(c) < 30:
                    continue
                M = cv2.moments(c)
                if M['m00'] == 0:
                    continue
                bx = int(M['m10'] / M['m00'])
                by = int(M['m01'] / M['m00'])
                depths = []
                for dy in range(-2, 3):
                    for dx in range(-2, 3):
                        ny, nx = by + dy, bx + dx
                        if 0 <= ny < h and 0 <= nx < w:
                            d = depth[ny, nx]
                            if np.isfinite(d) and d > 0.05:
                                depths.append(d)
                if depths:
                    candidates.append((np.median(depths), c, bx, by))

            if candidates:
                candidates.sort(key=lambda x: x[0])
                dist, _, bx, by = candidates[0]
                box_found = True

                dist_err    = dist - TARGET_DISTANCE
                heading_err = (cx - bx) / cx

                if abs(dist_err) > DIST_TOLERANCE:
                    # Drive forward and steer simultaneously
                    twist.linear.x  = float(np.clip(dist_err * KP_LINEAR,
                                                    0.06, APPROACH_SPEED))
                    twist.angular.z = float(heading_err * KP_ANGULAR)
                    self._nav_reached = False
                    self.get_logger().info(
                        f'[NAV] dist={dist:.3f}m  err={dist_err:+.3f}m  '
                        f'steer={twist.angular.z:.2f}', throttle_duration_sec=1.0)
                elif abs(heading_err) > ALIGN_TOLERANCE:
                    # In range but off-centre — stop forward motion and align
                    twist.linear.x  = 0.0
                    twist.angular.z = float(heading_err * KP_ANGULAR)
                    self.get_logger().info(
                        f'[NAV] Aligning... dist={dist:.3f}m  '
                        f'heading_err={heading_err:.3f}', throttle_duration_sec=1.0)
                else:
                    if not self._nav_reached:
                        self.get_logger().info(
                            f'[NAV] Aligned and at {dist:.3f}m — starting pick')
                        self._nav_reached = True
                        self._phase = 'pre_grasp'
                        self._phase_start = time.time()

        if not box_found:
            self.get_logger().info('Scanning — no blue box visible.',
                                   throttle_duration_sec=2.0)

        self.cmd_vel_pub.publish(twist)

    # ── Main control loop (pick phases) ───────────────────────────────────────

    def _control_loop(self):
        if self._phase == 'navigate':
            return  # handled in rgb callback

        # Stop base during pick
        self.cmd_vel_pub.publish(Twist())

        now = time.time()
        elapsed = now - self._phase_start

        if self._phase == 'pre_grasp' and elapsed < 0.5:
            return  # wait one tick before sending goal

        if self._phase == 'pre_grasp':
            self.get_logger().info('[PICK] Opening gripper and moving to pre-grasp')
            self._set_gripper(GRIPPER_OPEN)
            self._send_arm_goal(PRE_GRASP, duration_sec=2.5)
            self._phase = 'wait_pre_grasp'
            self._phase_start = time.time()

        elif self._phase == 'wait_pre_grasp' and elapsed > 3.0:
            self.get_logger().info('[PICK] Lowering to grasp position')
            self._send_arm_goal(GRASP, duration_sec=3.0)
            self._phase = 'wait_grasp'
            self._phase_start = time.time()

        elif self._phase == 'wait_grasp' and elapsed > 3.5:
            self.get_logger().info('[PICK] Closing gripper')
            self._set_gripper(GRIPPER_CLOSE)
            self._phase = 'wait_grip'
            self._phase_start = time.time()

        elif self._phase == 'wait_grip' and elapsed > 1.0:
            self.get_logger().info('[PICK] Lifting object')
            self._send_arm_goal(LIFT, duration_sec=1.5)
            self._phase = 'wait_lift'
            self._phase_start = time.time()

        elif self._phase == 'wait_lift' and elapsed > 2.0:
            self.get_logger().info('[PICK] Moving to carry position')
            self._send_arm_goal(CARRY, duration_sec=1.5)
            self._phase = 'wait_carry'
            self._phase_start = time.time()

        elif self._phase == 'wait_carry' and elapsed > 2.0:
            self.get_logger().info('[PICK] Pick complete! Arm in carry position.')
            self._phase = 'done'

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _set_gripper(self, values):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in values]
        self.gripper_pub.publish(msg)

    def _send_arm_goal(self, positions, duration_sec=2.0):
        if not self.arm_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Arm action server not available!')
            return

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = positions
        secs = int(duration_sec)
        nsecs = int((duration_sec - secs) * 1e9)
        pt.time_from_start = Duration(sec=secs, nanosec=nsecs)
        traj.points = [pt]
        goal.trajectory = traj
        self.arm_client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
