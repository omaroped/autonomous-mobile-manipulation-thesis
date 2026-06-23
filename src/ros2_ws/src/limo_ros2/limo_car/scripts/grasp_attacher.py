#!/usr/bin/python3
"""grasp_attacher — the physical 'weld' for the isolated grasp test.

Gazebo's state plugin (/get_entity_state, /set_entity_state) only understands whole
MODELS in the WORLD frame — it cannot reference a link. So the weld works in world
coordinates: the base is pinned (known world pose B), and TF gives base_footprint ->
gripper_tcp, so the gripper's world position = B (+) TF. On /grasp_attach=True we
record the box's offset from the gripper and then, at 25 Hz, set the box's world pose
to follow the gripper (position-follow, keeping the box upright). It lifts with the
arm and is released on False. Honest: only attach at the real grasp moment.

Needs the libgazebo_ros_state.so WORLD plugin (in final_map.world) and a NON-static
box. Test-only.
"""
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import tf2_ros
from gazebo_msgs.srv import GetEntityState, SetEntityState
from gazebo_msgs.msg import EntityState

BOX = 'grasp_test_box'
BASE_MODEL = 'mbot'
BASE_FRAME = 'base_footprint'
TCP_FRAME = 'gripper_tcp'


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


class GraspAttacher(Node):
    def __init__(self):
        super().__init__('grasp_attacher')
        self.get_cli = self.create_client(GetEntityState, '/get_entity_state')
        self.set_cli = self.create_client(SetEntityState, '/set_entity_state')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._B_pos = None
        self._R_B = None
        self._want = False
        self._attached = False
        self._off = None          # box_pos - gripper_pos at attach (world)
        self._box_quat = None      # box orientation to keep while carried
        self._box_name = None      # dynamically detected box name (target_box or grasp_test_box)
        self.create_subscription(Bool, '/grasp_attach', self._cb, 10)

    def _cb(self, msg: Bool):
        self._want = bool(msg.data)

    # ── world-pose helpers (synchronous; only called from the main loop) ──────
    def _get_world(self, name):
        if not self.get_cli.service_is_ready():
            return None
        req = GetEntityState.Request()
        req.name = name
        req.reference_frame = ''
        fut = self.get_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        r = fut.result()
        if r is None or not r.success:
            return None
        return r.state.pose

    def capture_base(self):
        # No longer needed — gripper_world_pos uses TF directly.
        # Kept as a no-op so the startup call in main() still works.
        self.get_logger().info('grasp_attacher: using live TF for gripper world pos (odom = Gazebo world)')
        return True

    def gripper_world_pos(self):
        """Return the gripper TCP position in Gazebo world coordinates.

        With odometry_source=1 (WORLD), the 'odom' frame IS the Gazebo world
        frame — so a direct TF lookup from odom to gripper_tcp gives the correct
        world position regardless of where the robot has driven to.

        The old approach (capture_base at startup + static _B_pos) was broken:
        once the robot drove to the table, _B_pos was stale and the computed
        gripper position was wrong, causing the box to float away.
        """
        try:
            tf = self.tf_buffer.lookup_transform('odom', TCP_FRAME, rclpy.time.Time())
            t = tf.transform.translation
            return np.array([t.x, t.y, t.z])
        except Exception as e:
            self.get_logger().warn(f'TF odom→{TCP_FRAME} unavailable: {e}',
                                   throttle_duration_sec=1.0)
            return None

    def set_box_world(self, pos):
        if self._box_name is None:
            return
        st = EntityState()
        st.name = self._box_name
        st.reference_frame = ''
        st.pose.position.x, st.pose.position.y, st.pose.position.z = float(pos[0]), float(pos[1]), float(pos[2])
        if self._box_quat is not None:
            st.pose.orientation = self._box_quat
        else:
            st.pose.orientation.w = 1.0
        req = SetEntityState.Request()
        req.state = st
        self.set_cli.call_async(req)

    # ── reconcile once per loop tick ──────────────────────────────────────────
    def tick(self):
        if self._want and not self._attached:
            if self._box_name is None:
                for b_name in ['target_box', 'grasp_test_box']:
                    p = self._get_world(b_name)
                    if p is not None:
                        self._box_name = b_name
                        self.get_logger().info(f'Detected box model in world: {b_name}')
                        break
            
            box = self._get_world(self._box_name) if self._box_name else None
            g = self.gripper_world_pos()
            if box is None or g is None:
                self.get_logger().warn(f'attach pending — box/gripper pose not ready yet (box name: {self._box_name})',
                                       throttle_duration_sec=1.0)
                return
            bpos = np.array([box.position.x, box.position.y, box.position.z])
            self._off = bpos - g
            self._box_quat = box.orientation
            self._attached = True
            self.get_logger().info(f'WELD ON — box follows gripper (offset {self._off.round(3)})')
        elif not self._want and self._attached:
            self._attached = False
            self.get_logger().info('WELD OFF — box released')

        if self._attached and self._off is not None:
            g = self.gripper_world_pos()
            if g is not None:
                self.set_box_world(g + self._off)


def main():
    rclpy.init()
    node = GraspAttacher()
    node.get_cli.wait_for_service(timeout_sec=20.0)
    node.set_cli.wait_for_service(timeout_sec=20.0)
    # capture the pinned base pose (retry until TF + state are ready)
    import time
    t0 = time.time()
    while rclpy.ok() and not node.capture_base() and time.time() - t0 < 20:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.get_logger().info('grasp_attacher ready (waiting for /grasp_attach)')
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.04)
            node.tick()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
