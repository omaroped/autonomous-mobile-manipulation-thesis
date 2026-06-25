#!/usr/bin/python3
"""
smart_grasp.py — GazeboGraspFix-style auto-weld for ROS 2 / Gazebo Classic.

Two detection methods run in parallel — whichever fires first triggers the weld:

  METHOD 1 — Bumper contact sensor:
    gripper_left1 OR gripper_right1 bumper reports contact with 'target_box'.
    Works only if Gazebo resolves the collision name 'col' correctly in SDF.

  METHOD 2 — Joint position error (reliable fallback):
    When the box physically blocks the finger, the actual joint position cannot
    reach the commanded position. When the error (cmd - actual) exceeds
    STUCK_RAD for STUCK_FRAMES consecutive joint_states messages → weld fires.
    This does NOT depend on contact sensor collision names.

Weld: Bool True on /grasp_attach → consumed by grasp_attacher.py.
Release: Bool False when gripper is commanded open.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray
from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import JointState

TARGET_SUBSTR = 'target_box'
CLOSE_THRESH  = -0.05       # gripper cmd[0] below this → closing
MIN_HOLD      = 1           # bumper frames before weld (at 50 Hz = 20 ms)

GRIPPER_JOINT = 'gripper_controller'
STUCK_RAD     = 0.008       # |cmd - actual| > this while closing → blocked by object
STUCK_FRAMES  = 2           # consecutive blocked frames before weld fires
CLOSE_MIN_RAD = -0.06       # joint must be past this before error check activates


class SmartGrasp(Node):
    def __init__(self):
        super().__init__('smart_grasp')

        # bumper state
        self._contact_left   = False
        self._contact_right  = False
        self._hold_count     = 0

        # shared state
        self._gripper_closing = False
        self._weld_on        = False
        self._cmd_pos        = 0.15   # commanded gripper_controller angle

        # joint-error state
        self._actual_pos     = 0.15
        self._stuck_count    = 0

        self._weld_pub = self.create_publisher(Bool, '/grasp_attach', 10)

        self.create_subscription(
            ContactsState, 'gripper_left1_bumper',  self._left_cb,    10)
        self.create_subscription(
            ContactsState, 'gripper_right1_bumper', self._right_cb,   10)
        self.create_subscription(
            Float64MultiArray, '/mycobot_gripper_controller/commands',
            self._gripper_cb, 10)
        self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 10)

        self.get_logger().info(
            'SmartGrasp ready — METHOD1: bumper contact | METHOD2: joint position error')

    # ── helpers ───────────────────────────────────────────────────────────────

    def _has_box(self, msg: ContactsState) -> bool:
        for s in msg.states:
            if TARGET_SUBSTR in s.collision1_name or TARGET_SUBSTR in s.collision2_name:
                return True
        return False

    def _set_weld(self, on: bool, reason: str = ''):
        msg = Bool()
        msg.data = on
        self._weld_pub.publish(msg)
        self._weld_on = on
        state = 'ON' if on else 'OFF'
        self.get_logger().info(f'[SmartGrasp] weld {state}  {reason}')

    # ── gripper command callback ───────────────────────────────────────────────

    def _gripper_cb(self, msg: Float64MultiArray):
        if not msg.data:
            return
        was_closing = self._gripper_closing
        self._gripper_closing = msg.data[0] < CLOSE_THRESH
        self._cmd_pos = msg.data[0]

        if was_closing and not self._gripper_closing:
            # gripper opened → release weld and reset all counters
            self._hold_count  = 0
            self._stuck_count = 0
            if self._weld_on:
                self._set_weld(False, '(gripper opened)')

    # ── METHOD 1: bumper contact sensor ───────────────────────────────────────

    def _left_cb(self, msg: ContactsState):
        self._contact_left = self._has_box(msg)
        self._update_bumper()

    def _right_cb(self, msg: ContactsState):
        self._contact_right = self._has_box(msg)
        self._update_bumper()

    def _update_bumper(self):
        any_contact = self._contact_left or self._contact_right
        if self._gripper_closing and any_contact:
            self._hold_count += 1
            if self._hold_count >= MIN_HOLD and not self._weld_on:
                self._set_weld(True, f'(bumper: left={self._contact_left} right={self._contact_right})')
        elif not any_contact:
            self._hold_count = 0

    # ── METHOD 2: joint position error ────────────────────────────────────────

    def _joint_cb(self, msg: JointState):
        if GRIPPER_JOINT not in msg.name:
            return
        idx = msg.name.index(GRIPPER_JOINT)
        self._actual_pos = msg.position[idx]

        if self._weld_on or not self._gripper_closing:
            self._stuck_count = 0
            return
        if self._actual_pos > CLOSE_MIN_RAD:
            # not far enough into the close range yet
            self._stuck_count = 0
            return

        # negative error means command is more-closed than actual → blocked
        error = self._cmd_pos - self._actual_pos
        if error < -STUCK_RAD:
            self._stuck_count += 1
            if self._stuck_count >= STUCK_FRAMES:
                self._set_weld(True,
                    f'(joint blocked: actual={self._actual_pos:.3f} '
                    f'cmd={self._cmd_pos:.3f} err={error:.3f} rad)')
        else:
            self._stuck_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = SmartGrasp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
