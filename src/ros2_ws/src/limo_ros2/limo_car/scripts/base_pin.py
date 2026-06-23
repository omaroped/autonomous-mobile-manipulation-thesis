#!/usr/bin/python3
"""base_pin — hold the robot base FIXED in Gazebo during the isolated arm test.

The LIMO base rolls on free wheels, so the arm's reaction force makes it slowly
creep (the box appears to drift) and, on any hard contact, the whole light robot
launches into the sky. For the ISOLATED arm test we PIN the base: read its settled
world pose once, then hold it there via /set_entity_state at 50 Hz (twist zeroed).
This is test-only and legitimate — a real robot is parked/braked while it grasps.

Requires libgazebo_ros_state.so to be loaded in gzserver (added in
ackermann_gazebo.launch.py). Only used by arm_grasp_test.launch.py, never by the
real pipeline, so it never prevents the robot from driving.
"""
import time
import math

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState, SetEntityState
from gazebo_msgs.msg import EntityState
from std_msgs.msg import Bool

MODEL = 'mbot'


class BasePin(Node):
    def __init__(self):
        super().__init__('base_pin')
        self.get_cli = self.create_client(GetEntityState, '/get_entity_state')
        self.set_cli = self.create_client(SetEntityState, '/set_entity_state')
        self._pose = None
        # Optional: pin at a KNOWN x/y (m) instead of wherever the base drifted to on spawn.
        # Used by the isolated test so the box lands at the reachable distance; left NaN
        # (capture-as-is) for the driving pipeline.
        self.declare_parameter('override_x', float('nan'))
        self.declare_parameter('override_y', float('nan'))
        
        self.pinned = True
        self.create_subscription(Bool, '/pin_base', self.pin_cb, 10)

    def pin_cb(self, msg):
        self.pinned = msg.data
        self.get_logger().info(f'pin_base changed to: {self.pinned}')

    def capture(self):
        if not self.get_cli.wait_for_service(timeout_sec=20.0):
            self.get_logger().error('/get_entity_state unavailable — is libgazebo_ros_state.so loaded '
                                    'in gzserver? (see ackermann_gazebo.launch.py)')
            return False
        req = GetEntityState.Request()
        req.name = MODEL
        req.reference_frame = 'world'
        fut = self.get_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        if res is None or not res.success:
            self.get_logger().error(f'could not read pose of model "{MODEL}"')
            return False
        self._pose = res.state.pose
        ox = self.get_parameter('override_x').value
        oy = self.get_parameter('override_y').value
        if not math.isnan(ox):
            self._pose.position.x = ox
        if not math.isnan(oy):
            self._pose.position.y = oy
        p = self._pose.position
        self.get_logger().info(f'pinning base "{MODEL}" at world ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')
        return True

    def hold(self):
        if not self.pinned:
            return
        if self._pose is None:
            return
        st = EntityState()
        st.name = MODEL
        st.pose = self._pose          # twist left at zero -> no velocity
        st.reference_frame = 'world'
        req = SetEntityState.Request()
        req.state = st
        self.set_cli.call_async(req)


def main():
    rclpy.init()
    node = BasePin()
    time.sleep(3.0)                    # let the robot settle on its wheels first
    if not node.capture():
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return
    if not node.set_cli.wait_for_service(timeout_sec=10.0):
        node.get_logger().error('/set_entity_state unavailable')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return
    node.create_timer(0.02, node.hold)   # 50 Hz hold
    node.get_logger().info('base pinned — holding pose so the arm cannot push it')
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
