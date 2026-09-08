#!/usr/bin/env python3
"""base_drift_probe — measure how far the LIMO base actually moves, from Gazebo
ground truth, and separate the two kinds of motion that matter for sim-to-real.

WHY THIS EXISTS
---------------
The pipeline holds the base still during arm motion by TELEPORTING it: base_pin.py
calls /set_entity_state at 50 Hz with the twist zeroed. That is a simulation-only
device. A real LIMO PRO stands on free-rolling wheels with no brake, so the arm's
reaction force does move it, and nothing teleports it back.

Two numbers follow from that, and neither is currently measured:

  DRIFT WHILE PINNED   -- how much the sim's pin is actually suppressing. Run the
                          pipeline normally; whatever this reads is the reaction
                          motion the simulation is hiding from you.

  DRIFT WHILE UNPINNED -- what the real robot would do instead. Run with
                          base_pin_enabled:=false; this is the honest number.

The difference between them is a sim-to-real gap that is quantifiable rather than
argued about, which is exactly the kind of result the thesis needs. It also bounds
the grasp error budget: base motion during the descent moves the box relative to
the gripper just as surely as a bad IK target does.

It also records drift DURING DRIVING (Nav2 legs), which is a different question --
there the wheels are commanded, and what matters is how far odometry disagrees with
ground truth over the leg.

USAGE
-----
Start the pipeline in one terminal, then in another:

    ros2 run limo_car base_drift_probe --ros-args -p out_csv:=/tmp/drift_pinned.csv

Then repeat the whole thing with base_pin_enabled:=false and a second csv:

    ros2 launch limo_car nav_pick.launch.py base_pin_enabled:=false
    ros2 run limo_car base_drift_probe --ros-args -p out_csv:=/tmp/drift_free.csv

Compare with:  ros2 run limo_car base_drift_probe --ros-args -p compare:='["/tmp/drift_pinned.csv","/tmp/drift_free.csv"]'

OUTPUT
------
One CSV row per sample: wall time, sim time, ground-truth base x/y/yaw, odom x/y/yaw,
and the running divergence between them. Ground truth comes from /get_entity_state
(sim only); odom from /odom, which is what the robot believes.
"""
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetEntityState

MODEL = 'mbot'


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class BaseDriftProbe(Node):
    def __init__(self):
        super().__init__('base_drift_probe')
        self.declare_parameter('out_csv', '/tmp/base_drift.csv')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('model', MODEL)

        self._odom = None
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self._gt = self.create_client(GetEntityState, '/get_entity_state')

        self._path = self.get_parameter('out_csv').value
        self._rows = []
        self._t0 = None
        self._gt0 = None
        self._od0 = None

        if not self._gt.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                '/get_entity_state unavailable — is gzserver running with '
                'libgazebo_ros_state.so? Ground truth is sim-only.')
            raise SystemExit(1)

        hz = float(self.get_parameter('rate_hz').value)
        self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(
            f'recording base drift at {hz:.0f} Hz -> {self._path}  (Ctrl-C to stop)')

    def _odom_cb(self, msg):
        self._odom = msg

    def _ground_truth(self):
        req = GetEntityState.Request()
        req.name = self.get_parameter('model').value
        req.reference_frame = 'world'
        fut = self._gt.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=0.5)
        if not fut.done() or fut.result() is None or not fut.result().success:
            return None
        p = fut.result().state.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)

    def _tick(self):
        gt = self._ground_truth()
        if gt is None or self._odom is None:
            return
        o = self._odom.pose.pose
        od = (o.position.x, o.position.y, yaw_of(o.orientation))

        now = time.time()
        if self._t0 is None:
            self._t0, self._gt0, self._od0 = now, gt, od

        # Displacement of each source since the probe started. Their DIFFERENCE is
        # the odometry error: how far what the robot believes has diverged from what
        # actually happened. That is the quantity that survives to real hardware --
        # ground truth does not exist there, but the same divergence does.
        gd = math.dist(gt[:2], self._gt0[:2])
        odd = math.dist(od[:2], self._od0[:2])
        self._rows.append({
            't':          round(now - self._t0, 3),
            'sim_t':      round(self.get_clock().now().nanoseconds * 1e-9, 3),
            'gt_x':       round(gt[0], 5),
            'gt_y':       round(gt[1], 5),
            'gt_yaw':     round(gt[2], 5),
            'odom_x':     round(od[0], 5),
            'odom_y':     round(od[1], 5),
            'odom_yaw':   round(od[2], 5),
            'gt_moved':   round(gd, 5),
            'odom_moved': round(odd, 5),
            'divergence': round(abs(gd - odd), 5),
        })

    def flush(self):
        if not self._rows:
            self.get_logger().warn('no samples recorded')
            return
        os.makedirs(os.path.dirname(self._path) or '.', exist_ok=True)
        with open(self._path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(self._rows[0].keys()))
            w.writeheader()
            w.writerows(self._rows)

        gt_tot = self._rows[-1]['gt_moved']
        od_tot = self._rows[-1]['odom_moved']
        worst = max(r['divergence'] for r in self._rows)
        # Largest jump between consecutive ground-truth samples: a teleport (the pin
        # correcting the base) shows up here as a spike that smooth rolling cannot
        # produce.
        jumps = [math.dist((a['gt_x'], a['gt_y']), (b['gt_x'], b['gt_y']))
                 for a, b in zip(self._rows, self._rows[1:])]
        self.get_logger().info(
            f'\n  samples            : {len(self._rows)}'
            f'\n  ground truth moved : {gt_tot*1000:.1f} mm'
            f'\n  odometry believes  : {od_tot*1000:.1f} mm'
            f'\n  worst divergence   : {worst*1000:.1f} mm'
            f'\n  largest single-sample jump: {max(jumps)*1000 if jumps else 0:.1f} mm'
            f'\n  wrote {self._path}')


def main(args=None):
    rclpy.init(args=args)
    node = BaseDriftProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.flush()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
