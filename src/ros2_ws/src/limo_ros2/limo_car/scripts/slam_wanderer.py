#!/usr/bin/python3
"""slam_wanderer.py — autonomous reactive wanderer for SLAM mapping.

Drives the robot around automatically so the operator doesn't have to
manually teleop during a SLAM mapping session. Logic:
  - Default: drive forward at cruise speed.
  - Obstacle ahead  (<STOP_DIST): stop linear, rotate away from obstacle.
  - Side bias: slight yaw correction to avoid hugging one wall.
  - Random turn timer: occasionally changes direction so the robot doesn't
    loop the same circuit forever and covers the whole space.

Shebang is /usr/bin/python3 to bypass the pyenv shim (which lacks rclpy).
"""
import math
import random
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

CRUISE_SPEED   = 0.20   # m/s forward
TURN_SPEED     = 0.70   # rad/s in-place rotation
STOP_DIST      = 0.55   # m — start turning if obstacle this close ahead
SIDE_DIST      = 0.35   # m — side clearance threshold for bias correction
RANDOM_TURN_S  = 8.0    # seconds between random direction changes


class SlamWanderer(Node):
    def __init__(self):
        super().__init__('slam_wanderer')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos)
        self.create_timer(0.10, self._control_loop)
        self.create_timer(RANDOM_TURN_S, self._random_turn_timer)

        self.front = float('inf')
        self.left  = float('inf')
        self.right = float('inf')
        self.forced_turn_dir   = 0.0   # set by random timer
        self.forced_turn_ticks = 0     # how many 100ms ticks to hold it
        self.get_logger().info('Wanderer started — robot will explore autonomously.')

    def _sector_min(self, ranges, angle_min, angle_inc, lo_deg, hi_deg):
        lo_idx = max(0, int((math.radians(lo_deg) - angle_min) / angle_inc))
        hi_idx = min(len(ranges)-1, int((math.radians(hi_deg) - angle_min) / angle_inc))
        vals = [r for r in ranges[lo_idx:hi_idx+1]
                if not math.isnan(r) and not math.isinf(r) and r > 0.10]
        return min(vals) if vals else float('inf')

    def _on_scan(self, msg):
        a0 = msg.angle_min
        da = msg.angle_increment
        r  = msg.ranges
        # Forward ±30°, Left 30°→90°, Right −90°→−30°
        self.front = self._sector_min(r, a0, da, -30,  30)
        self.left  = self._sector_min(r, a0, da,  30,  90)
        self.right = self._sector_min(r, a0, da, -90, -30)

    def _random_turn_timer(self):
        # Every RANDOM_TURN_S seconds inject a short random turn so the robot
        # doesn't keep looping the same path.
        direction = random.choice([-1.0, 1.0])
        ticks     = random.randint(8, 20)   # 0.8 – 2.0 s
        self.forced_turn_dir   = direction
        self.forced_turn_ticks = ticks
        self.get_logger().debug(f'Random turn: {"LEFT" if direction>0 else "RIGHT"} for {ticks*0.1:.1f}s')

    def _control_loop(self):
        cmd = Twist()

        if self.forced_turn_ticks > 0 and self.front > STOP_DIST:
            # Random exploration turn (only when path is clear)
            cmd.linear.x  = CRUISE_SPEED * 0.5
            cmd.angular.z = TURN_SPEED * 0.5 * self.forced_turn_dir
            self.forced_turn_ticks -= 1

        elif self.front < STOP_DIST:
            # Obstacle ahead — rotate in place toward the clearer side
            cmd.linear.x  = 0.0
            cmd.angular.z = TURN_SPEED if self.left >= self.right else -TURN_SPEED

        else:
            # Clear ahead — drive forward with gentle side-bias correction
            cmd.linear.x = CRUISE_SPEED
            if self.left < SIDE_DIST:
                cmd.angular.z = -0.25   # too close to left wall, nudge right
            elif self.right < SIDE_DIST:
                cmd.angular.z =  0.25   # too close to right wall, nudge left
            else:
                cmd.angular.z = 0.0

        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = SlamWanderer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())   # stop the robot
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
