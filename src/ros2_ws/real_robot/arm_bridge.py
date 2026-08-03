#!/usr/bin/env python3
"""arm_bridge — mirror ROS 2 joint state to the REAL myCobot 280 over serial.

Adapted from Elephant Robotics' sync_plan.py pattern (decision 2026-07-21):
a lightweight mirror node instead of a full ros2_control SystemInterface.

  /joint_states (arm joints)                → mc.send_angles(...)
  /mycobot_gripper_controller/commands      → mc.set_gripper_value(...)

DELIBERATE CHOICES
------------------
* Port is EXPLICIT and defaults to /dev/ttyACM0. Elephant's own script probes
  ttyUSB* first, which on THIS robot's wiring would grab the LIDAR's port
  (CP2102 = lidar, CH340/ttyACM0 = arm — verified empirically 2026-07-21,
  after guessing it backwards once).
* pymycobot pinned 3.3.4 — 4.x changed the API surface.
* Joint names match the SIM's controllers (mycobot_controllers.yaml), so the
  same MoveIt config drives both. Sim order = joint2_to_joint1 .. joint6output_to_joint6.
* Gripper: the sim's reference joint (gripper_controller) runs +0.15 rad OPEN
  to -0.20 rad CLOSED; the real gripper takes 0..100 (0 = closed). Linear map.
* Rate-limited to `rate_hz` and skipped when the change is below `min_step_deg`,
  so serial bandwidth isn't flooded by 50 Hz joint_states.

Run:  python3 arm_bridge.py
      python3 arm_bridge.py --ros-args -p port:=/dev/ttyACM0 -p speed:=40
"""
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from pymycobot.mycobot import MyCobot

ARM_JOINTS = [
    'joint2_to_joint1',
    'joint3_to_joint2',
    'joint4_to_joint3',
    'joint5_to_joint4',
    'joint6_to_joint5',
    'joint6output_to_joint6',
]
GRIPPER_OPEN_RAD  = 0.15    # sim reference joint, fully open
GRIPPER_CLOSE_RAD = -0.20   # sim reference joint, fully closed


class ArmBridge(Node):
    def __init__(self):
        super().__init__('arm_bridge')
        self.declare_parameter('port',  '/dev/ttyACM0')  # NEVER auto-detect (lidar!)
        self.declare_parameter('baud',  115200)
        self.declare_parameter('speed', 40)              # 0..100 arm speed
        self.declare_parameter('rate_hz', 10.0)          # max serial command rate
        self.declare_parameter('min_step_deg', 1.0)      # skip smaller changes

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        self.mc = MyCobot(port, baud)
        time.sleep(0.5)
        angles = self.mc.get_angles()
        if not angles or angles == -1:
            raise RuntimeError(
                f'no response from arm on {port} — is it powered, and is this '
                f'really the arm port? (lidar is the CP2102 on ttyUSB0)')
        self.get_logger().info(f'arm on {port}: current angles {angles}')

        self._speed        = int(self.get_parameter('speed').value)
        self._min_interval = 1.0 / float(self.get_parameter('rate_hz').value)
        self._min_step     = float(self.get_parameter('min_step_deg').value)
        self._last_sent    = None
        self._last_t       = 0.0
        self._last_grip    = None

        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        self.create_subscription(Float64MultiArray,
                                 '/mycobot_gripper_controller/commands',
                                 self._grip_cb, 10)
        self.get_logger().info('bridge up — mirroring /joint_states to the arm')

    # ── arm ──────────────────────────────────────────────────────────────────
    def _js_cb(self, msg: JointState):
        try:
            deg = [math.degrees(msg.position[msg.name.index(j)])
                   for j in ARM_JOINTS]
        except ValueError:
            return  # message without the arm joints (e.g. wheels only)

        now = time.time()
        if now - self._last_t < self._min_interval:
            return
        if self._last_sent is not None and \
           max(abs(a - b) for a, b in zip(deg, self._last_sent)) < self._min_step:
            return
        self.mc.send_angles(deg, self._speed)
        self._last_sent, self._last_t = deg, now

    # ── gripper ──────────────────────────────────────────────────────────────
    def _grip_cb(self, msg: Float64MultiArray):
        if not msg.data:
            return
        ref = float(msg.data[0])          # sim reference joint angle (rad)
        span = GRIPPER_OPEN_RAD - GRIPPER_CLOSE_RAD
        val = int(round(100.0 * (ref - GRIPPER_CLOSE_RAD) / span))
        val = max(0, min(100, val))
        if val == self._last_grip:
            return
        self.mc.set_gripper_value(val, self._speed)
        self._last_grip = val
        self.get_logger().info(f'gripper → {val}/100 (ref {ref:+.3f} rad)')


def main():
    rclpy.init()
    node = ArmBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
