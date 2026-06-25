#!/usr/bin/python3
"""
ik_place_check.py — Gate check before trusting "arm absorbs lateral offset".

Tests whether the top-down IK succeeds across the expected residual-lateral × dock-range
× stack-level space of the place scenario, using the tag-yaw-computed orientation:

    place_quat = topdown ⊗ yaw(tag_yaw)   # still vertical, spun to match box/table

This must succeed before the full nav-pick-place pipeline trusts the assumption that
a ±5 cm lateral base error is corrected purely by the arm reaching the latched drop
point — without any turn-drive-turn lateral correction of the base.

Run BEFORE the full pipeline:
    ros2 launch limo_car nav_pick.launch.py
    ros2 launch limo_cobot_moveit_config moveit.launch.py
    ros2 run limo_car ik_place_check.py

Prints a ✓/✗ grid and exits with code 0 (all pass) or 1 (any fail).
"""

import sys
import math
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from geometry_msgs.msg import PoseStamped

# ── Sweep parameters — match the plan's expected operating envelope ────────────
# x: base_link forward distance to table centre (arm reach axis)
X_RANGE = [0.20, 0.22, 0.24, 0.26, 0.28]
# y: lateral residual after heading-only base alignment (the arm absorbs this)
Y_RANGE = [-0.07, -0.05, -0.03, 0.0, 0.03, 0.05, 0.07]
# z_tcp: TCP height at descend for each stack level L
# Formula: PLACE_BOX_Z(0.08) + L*BOX_HEIGHT(0.04) + GRASP_ABOVE(0.06)
# L=0 → 0.14, L=1 → 0.18, L=2 → 0.22
Z_LEVELS = {0: 0.14, 1: 0.18, 2: 0.22}
# Hover height (before descend) per level: + PLACE_HOVER(0.12)
Z_HOVER_DELTA = 0.06    # PLACE_HOVER - GRASP_ABOVE = 0.12 - 0.06

# Tag yaw values to test (radians). 0 = tag facing straight ahead.
# After Phase A alignment, residual yaw should be <5° — test a wider ±30° range.
TAG_YAW_RANGE = [-math.pi/6, -math.pi/12, 0.0, math.pi/12, math.pi/6]


def topdown_yaw_quat(tag_yaw: float):
    """Compute  topdown ⊗ yaw(tag_yaw).

    topdown = (-0.7071, 0, 0, 0.7071)  (qx, qy, qz, qw) — gripper points straight down.
    yaw_q   = (0, 0, sin(tag_yaw/2), cos(tag_yaw/2))     — rotation about Z.

    Hamilton product q1 ⊗ q2 with:
      q1 = topdown:  x1=-0.7071, y1=0, z1=0, w1=0.7071
      q2 = yaw:      x2=0,       y2=0, z2=sz, w2=cz

    Result (still vertical, spun by tag_yaw):
      w =  0.7071 * cz
      x = -0.7071 * cz
      y =  0.7071 * sz
      z =  0.7071 * sz
    """
    sz = math.sin(tag_yaw / 2.0)
    cz = math.cos(tag_yaw / 2.0)
    qw =  0.7071 * cz
    qx = -0.7071 * cz
    qy =  0.7071 * sz
    qz =  0.7071 * sz
    return qx, qy, qz, qw


class IKPlaceCheck(Node):

    def __init__(self):
        super().__init__('ik_place_check')
        self._ik = self.create_client(GetPositionIK, '/compute_ik')

    def _query(self, x, y, z, qx, qy, qz, qw) -> bool:
        req = GetPositionIK.Request()
        r   = PositionIKRequest()
        r.group_name    = 'arm'
        r.ik_link_name  = 'gripper_tcp'
        ps = PoseStamped()
        ps.header.frame_id = 'base_link'
        ps.header.stamp    = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        ps.pose.orientation.x = float(qx)
        ps.pose.orientation.y = float(qy)
        ps.pose.orientation.z = float(qz)
        ps.pose.orientation.w = float(qw)
        r.pose_stamped      = ps
        r.avoid_collisions  = True
        req.ik_request      = r
        fut = self._ik.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.result() is not None:
            return fut.result().error_code.val == 1
        return False

    def run(self):
        self.get_logger().info('=== IK Place Envelope Pre-Check ===')
        if not self._ik.wait_for_service(timeout_sec=30.0):
            self.get_logger().error('/compute_ik not available — is MoveIt running?')
            return False

        total = passed = 0
        failures = []

        for level, z in Z_LEVELS.items():
            self.get_logger().info(f'\n── Stack level {level}  z_tcp={z:.2f} m ──')
            # Header row
            header = f"{'':10}" + ''.join(f'  y={y:+.2f}' for y in Y_RANGE)
            self.get_logger().info(header)

            for tag_yaw in TAG_YAW_RANGE:
                qx, qy, qz, qw = topdown_yaw_quat(tag_yaw)
                row_label = f'yaw={math.degrees(tag_yaw):+5.0f}°'
                cells = []
                for x in X_RANGE:
                    row_cells = []
                    for y in Y_RANGE:
                        ok = self._query(x, y, z, qx, qy, qz, qw)
                        total += 1
                        if ok:
                            passed += 1
                            row_cells.append('  ✓    ')
                        else:
                            row_cells.append('  ✗    ')
                            failures.append((level, tag_yaw, x, y, z))
                    cells.append(row_cells)
                # Print one row per x value
                for xi, x in enumerate(X_RANGE):
                    prefix = f'{row_label} x={x:.2f}: ' if xi == 0 else f'{"":10} x={x:.2f}: '
                    self.get_logger().info(prefix + ''.join(cells[xi]))

        self.get_logger().info(
            f'\n=== RESULT: {passed}/{total} IK solutions found '
            f'({"PASS" if not failures else "FAIL"}) ===')
        if failures:
            self.get_logger().warn(f'{len(failures)} failing poses:')
            for lvl, yaw, x, y, z in failures:
                self.get_logger().warn(
                    f'  L={lvl} yaw={math.degrees(yaw):+.0f}° '
                    f'x={x:.2f} y={y:+.2f} z={z:.2f}')
            self.get_logger().warn(
                'Mitigation: tighten dock_range so x stays where IK succeeds, '
                'or rely more on the Nav2 pre-dock pose to reduce lateral offset.')
        else:
            self.get_logger().info(
                'All poses reachable — "arm absorbs lateral" assumption holds. '
                'Pipeline can proceed.')
        return not bool(failures)


def main(args=None):
    rclpy.init(args=args)
    node = IKPlaceCheck()
    try:
        ok = node.run()
    except KeyboardInterrupt:
        ok = False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
