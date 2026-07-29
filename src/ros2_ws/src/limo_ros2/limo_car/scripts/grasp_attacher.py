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

# Max gripper_tcp -> box-centre distance that still counts as "between the fingers".
# The box is 3.5 cm wide / 4 cm tall and the TCP sits ~6 cm above the box centre at
# the grasp pose, so a genuine grasp is well under 10 cm. Anything beyond that is not
# being held — refuse to weld it. Without this guard, a single-box world always welds
# that box no matter where it is (a box on the FLOOR was welded and flown, 2026-07-28).
MAX_ATTACH_DIST = 0.10


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
        self._box_name = None      # dynamically detected box name (stack_box_0/1/2, target_box, or grasp_test_box)
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

    def gripper_world_transform(self):
        """Return (pos, R): gripper_tcp world position and 3×3 rotation matrix.

        Storing the attach offset in gripper frame (via R^T) and rotating it
        back on every tick (via R) means the box follows both translations AND
        rotations of the gripper — not just translations.
        """
        try:
            tf = self.tf_buffer.lookup_transform('odom', TCP_FRAME, rclpy.time.Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            return (np.array([t.x, t.y, t.z]),
                    quat_to_R(q.x, q.y, q.z, q.w))
        except Exception as e:
            self.get_logger().warn(f'TF odom→{TCP_FRAME} unavailable: {e}',
                                   throttle_duration_sec=1.0)
            return None, None

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
            g_pos, R_g = self.gripper_world_transform()
            if g_pos is None:
                self.get_logger().warn('attach pending — gripper TF not ready yet',
                                       throttle_duration_sec=1.0)
                return

            # Pick whichever candidate box is CURRENTLY closest to the gripper — not a
            # name cached from the first attach ever seen. With 3 boxes coexisting
            # (self-centering stack), a cached name means every later pick welds to
            # the same first-found box regardless of which one was actually touched —
            # confirmed bug: gripper closed on the left box, the right box rose instead.
            best_name, best_box, best_dist = None, None, float('inf')
            for b_name in ['stack_box_0', 'stack_box_1', 'stack_box_2', 'target_box', 'grasp_test_box']:
                p = self._get_world(b_name)
                if p is None:
                    continue
                bpos = np.array([p.position.x, p.position.y, p.position.z])
                d = float(np.linalg.norm(bpos - g_pos))
                if d < best_dist:
                    best_name, best_box, best_dist = b_name, p, d

            if best_name is None:
                self.get_logger().warn('attach pending — no box pose available yet',
                                       throttle_duration_sec=1.0)
                return

            # Distance guard. "Nearest box" is meaningless on its own when only ONE
            # box exists — it wins by default no matter how far away it is. Observed
            # 2026-07-28: a box that had fallen onto the FLOOR was welded to the
            # gripper even though the fingers closed on empty air at table height,
            # and it then flew up with the arm. A box that is not physically between
            # the fingers must never be welded.
            if best_dist > MAX_ATTACH_DIST:
                self.get_logger().warn(
                    f'attach REFUSED — nearest box {best_name} is {best_dist:.3f} m from the '
                    f'gripper (limit {MAX_ATTACH_DIST:.3f} m). Nothing is between the fingers; '
                    f'reporting grasp failure rather than welding a distant box.',
                    throttle_duration_sec=1.0)
                return

            self._box_name = best_name
            bpos = np.array([best_box.position.x, best_box.position.y, best_box.position.z])
            # store offset in gripper frame so it rotates correctly on each tick
            self._off = R_g.T @ (bpos - g_pos)
            self._box_quat = best_box.orientation
            self._attached = True
            self.get_logger().info(
                f'WELD ON — attaching {self._box_name} (dist={best_dist:.3f} m from gripper, '
                f'offset {self._off.round(3)} gripper-frame)')
        elif not self._want and self._attached:
            self._attached = False
            self._box_name = None   # reset so the NEXT attach re-evaluates nearest box
            self.get_logger().info('WELD OFF — box released')

        if self._attached and self._off is not None:
            g_pos, R_g = self.gripper_world_transform()
            if g_pos is not None:
                # rotate gripper-frame offset back to world frame on every tick
                self.set_box_world(g_pos + R_g @ self._off)


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
            rclpy.spin_once(node, timeout_sec=0.02)
            node.tick()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
