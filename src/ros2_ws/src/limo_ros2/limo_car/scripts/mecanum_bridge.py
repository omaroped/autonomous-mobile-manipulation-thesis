#!/usr/bin/python3
"""mecanum_bridge — adapt the project-standard /cmd_vel and /odom to the
mecanum_drive_controller's interface.

  /cmd_vel (Twist) ──stamp with sim time──▶ /mecanum_drive_controller/reference (TwistStamped)
  /mecanum_drive_controller/odometry (Odometry) ─────────────────────────────▶ /odom

Why this exists:
  * The controller's external command input is ~/reference, a TwistStamped (there is no
    reference_unstamped topic in this build), so /cmd_vel must be wrapped + restamped.
  * The stamp MUST be sim time (use_sim_time=true) or the controller's reference_timeout
    rejects it as stale.
  * Shebang is /usr/bin/python3 (NOT env python3) to bypass the pyenv shim, which lacks rclpy.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry

CMD_IN   = '/cmd_vel'
REF_OUT  = '/mecanum_drive_controller/reference'
ODOM_IN  = '/mecanum_drive_controller/odometry'
ODOM_OUT = '/odom'
BASE_FRAME = 'base_footprint'


class MecanumBridge(Node):
    def __init__(self):
        super().__init__('mecanum_bridge')
        self._ref_pub = self.create_publisher(TwistStamped, REF_OUT, 10)
        self.create_subscription(Twist, CMD_IN, self._on_cmd, 10)
        self._odom_pub = self.create_publisher(Odometry, ODOM_OUT, 10)
        self.create_subscription(Odometry, ODOM_IN, lambda m: self._odom_pub.publish(m), 10)
        self.get_logger().info(
            f'mecanum_bridge up: {CMD_IN}(Twist)→{REF_OUT}(TwistStamped), {ODOM_IN}→{ODOM_OUT}')

    def _on_cmd(self, msg):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()   # sim time (use_sim_time=true)
        out.header.frame_id = BASE_FRAME
        out.twist = msg
        self._ref_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MecanumBridge()
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
