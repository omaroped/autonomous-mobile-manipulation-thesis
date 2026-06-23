#!/usr/bin/python3
"""validate_odom.py — PROVE (don't assume) whether the robot's reported odometry
matches its TRUE position in Gazebo.

It prints three things once a second:
  REPORTED : where the robot THINKS it is        (from /odom)
  TRUE     : where the robot ACTUALLY is in sim   (from /model_states, ground truth)
  ERROR    : the difference  (this is the 'drift'; should stay ~0 if odom == ground truth)

Run it, then teleop-rotate the robot in place. If ERROR stays near zero through a full
spin, the odometry is perfect and rotation 'drift' is NOT coming from the wheels/odom.
If ERROR grows, we've measured real drift.

Shebang is /usr/bin/python3 to bypass the pyenv shim (which lacks rclpy).
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates

ROBOT_MODEL = 'mbot'   # the -entity name used at spawn time


def yaw_of(q):
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


class OdomValidator(Node):
    def __init__(self):
        super().__init__('odom_validator')
        self.odom = None   # (x, y, yaw_deg)
        self.true = None
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(ModelStates, '/model_states', self._on_truth, 10)
        self.create_timer(1.0, self._report)
        self.get_logger().info('Validating odom vs ground truth. Rotate the robot now...')

    def _on_odom(self, m):
        p = m.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _on_truth(self, m):
        if ROBOT_MODEL in m.name:
            p = m.pose[m.name.index(ROBOT_MODEL)]
            self.true = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _report(self):
        if self.odom is None:
            self.get_logger().warn('No /odom yet.')
            return
        if self.true is None:
            self.get_logger().warn("No /model_states yet (is libgazebo_ros_state loaded, "
                                   f"and is the model named '{ROBOT_MODEL}'?).")
            return
        ox, oy, oyaw = self.odom
        tx, ty, tyaw = self.true
        dx, dy = ox - tx, oy - ty
        dist = math.hypot(dx, dy)
        dyaw = (oyaw - tyaw + 180) % 360 - 180
        print(f'REPORTED x={ox:+.3f} y={oy:+.3f} yaw={oyaw:+7.2f}deg | '
              f'TRUE x={tx:+.3f} y={ty:+.3f} yaw={tyaw:+7.2f}deg | '
              f'ERROR pos={dist*100:5.1f}cm yaw={dyaw:+6.2f}deg')


def main():
    rclpy.init()
    node = OdomValidator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
