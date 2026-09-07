#!/usr/bin/env python3
"""odom_drive_probe.py — measure how far the base ACTUALLY drives vs what odometry claims.

WHY THIS EXISTS. nav_pick_orchestrator._drive_forward() docks the robot by driving a
measured distance ON ODOMETRY and hard-stopping. In simulation that was safe because the
diff_drive plugin ran with odometry_source=WORLD -- odometry WAS ground truth, so the
commanded distance and the real distance were the same number by construction.

On the physical LIMO, odometry is wheel encoders. Nothing has ever measured its error.
That matters concretely: at the pickup dock the cube's near face clears the bumper by
+18 mm (STOP_DISTANCE 0.22 - half a 25 mm cube - bumper_x 0.189). If encoder error over
the approach exceeds 18 mm, the robot drives into the table no matter what the constant
says.

This drives with the SAME velocity law as the dock (K_FWD/MIN_FWD/MAX_FWD/STOP_MARGIN
copied from _drive_forward), so the measured error transfers directly rather than
describing some other motion.

Usage (robot on a clear floor, tape a start line at the bumper):
    ros2 run limo_car odom_drive_probe --ros-args -p distance:=0.30
Then tape-measure how far the bumper actually moved and compare with the printed value.
"""
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

# Copied verbatim from nav_pick_orchestrator._drive_forward / visual_docking so this
# probe measures the dock's real behaviour, not an idealised straight line.
K_FWD, MIN_FWD, MAX_FWD = 0.8, 0.06, 0.16
STOP_MARGIN = 0.005


class OdomDriveProbe(Node):

    def __init__(self):
        super().__init__('odom_drive_probe')
        self.declare_parameter('distance', 0.30)
        self.declare_parameter('countdown', 3)
        self._odom = None
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self._cmd = self.create_publisher(Twist, '/cmd_vel', 10)

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self._odom = (p.x, p.y)

    def _dist_since(self, ox, oy):
        if self._odom is None:
            return 0.0
        return ((self._odom[0] - ox) ** 2 + (self._odom[1] - oy) ** 2) ** 0.5

    def run(self):
        target = float(self.get_parameter('distance').value)
        drive = max(0.0, target - STOP_MARGIN)

        # Wait for odometry before moving -- driving blind is how you lose the datum.
        deadline = time.time() + 10.0
        while self._odom is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._odom is None:
            self.get_logger().error('no /odom after 10 s — is limo_base running?')
            return 1

        for n in range(int(self.get_parameter('countdown').value), 0, -1):
            self.get_logger().info(f'moving in {n}… (Ctrl-C aborts)')
            time.sleep(1.0)

        ox, oy = self._odom
        self.get_logger().info(f'target {target:.3f} m  (driving {drive:.3f} after '
                               f'{STOP_MARGIN*1000:.0f} mm stop margin)')

        t_end = time.time() + 40.0
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)
            disp = self._dist_since(ox, oy)
            remaining = drive - disp
            if remaining <= 0.0:
                break
            v = max(MIN_FWD, min(MAX_FWD, K_FWD * remaining))
            t = Twist()
            t.linear.x = v
            self._cmd.publish(t)

        # Hard stop, then let the base settle before reading the final displacement:
        # the Twist takes effect a control cycle later, which is part of the overshoot
        # this probe is meant to quantify.
        self._cmd.publish(Twist())
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)
        final = self._dist_since(ox, oy)

        print('\n' + '=' * 58)
        print(f'  commanded target      {target:.3f} m')
        print(f'  odometry says driven  {final:.3f} m')
        print(f'  odometry overshoot    {(final - drive)*1000:+.0f} mm past its own stop point')
        print('=' * 58)
        print('  NOW TAPE-MEASURE how far the bumper actually moved.')
        print('  odometry error = tape - odometry.  Repeat 3x.')
        print(f'  Dock clearance budget is +18 mm. If |error| exceeds that,')
        print(f'  the dock is not safe at STOP_DISTANCE = 0.22.\n')
        return 0


def main():
    rclpy.init()
    node = OdomDriveProbe()
    try:
        rc = node.run()
    except KeyboardInterrupt:
        node._cmd.publish(Twist())      # never leave the base coasting
        rc = 130
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
