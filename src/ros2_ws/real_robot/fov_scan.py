#!/usr/bin/env python3
"""Measure which angles the robot blocks with its own body.

Run WITH measure_fov.yaml loaded (full 360 deg, range_min 0.02) so self-hits are
visible instead of filtered, and with the robot in an OPEN area so real walls do
not look like self-hits.

Three-way classification per beam:
  BLOCKED  median range < BLOCKED_M          -- seeing the robot
  NO_DATA  too few valid returns             -- ALSO blocked, just too close to measure
  CLEAR    everything else

The first version of this script had two bugs that made its output partly
invalid (2026-08-03):
  (a) NO_DATA beams were counted as CLEAR. A fully-blocked beam often returns
      nothing at all, so one entire side (-100 to -153 deg) was wrongly reported
      clear even though the physical obstruction is a symmetric cover plate.
  (b) "widest clear span" took min/max of all clear beams rather than the widest
      CONTIGUOUS run, so it printed -180.0 to +163.0 -- a span straight across
      the blocked sector -- and suggested angle_min: -175.0. Nonsense.
Both are fixed here.

Usage:
    ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
        params_file:=$HOME/limo_ros2_ws/src/ydlidar_ros2_driver/params/measure_fov.yaml
    python3 fov_scan.py
"""
import math
import statistics

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

N_SCANS        = 40
BLOCKED_M      = 0.25
MIN_VALID_FRAC = 0.5


class FOV(Node):
    def __init__(self):
        super().__init__('fov_scan')
        self.rows, self.msg = [], None
        self.create_subscription(LaserScan, '/scan', self.cb,
                                 qos_profile_sensor_data)

    def cb(self, m):
        if len(self.rows) >= N_SCANS:
            return
        self.msg = m
        self.rows.append(list(m.ranges))
        if len(self.rows) % 10 == 0:
            self.get_logger().info(f'{len(self.rows)}/{N_SCANS}')


def main():
    rclpy.init()
    n = FOV()
    while rclpy.ok() and len(n.rows) < N_SCANS:
        rclpy.spin_once(n, timeout_sec=1.0)
    m = n.msg
    if m is None:
        print('no /scan received')
        return
    nb, ns = len(m.ranges), len(n.rows)

    def ang(i):
        return math.degrees(m.angle_min + i * m.angle_increment)

    state = []
    for i in range(nb):
        vals = [r[i] for r in n.rows
                if i < len(r) and r[i] > 0.0 and math.isfinite(r[i])]
        if len(vals) / ns < MIN_VALID_FRAC:
            state.append('NO_DATA')
        else:
            state.append('BLOCKED' if statistics.median(vals) < BLOCKED_M
                         else 'CLEAR')

    def runs_of(label):
        out, run = [], None
        for i in range(nb):
            if state[i] == label:
                run = [i, i] if run is None else [run[0], i]
            elif run:
                out.append(run)
                run = None
        if run:
            out.append(run)
        return out

    print(f'\nscans={ns} beams={nb}  '
          f'{math.degrees(m.angle_min):.1f}..{math.degrees(m.angle_max):.1f} deg')
    for lab in ('BLOCKED', 'NO_DATA'):
        rr = runs_of(lab)
        print(f'\n{lab} arcs:' if rr else f'\n{lab} arcs: none')
        for a, b in rr:
            print(f'  {ang(a):+7.1f} to {ang(b):+7.1f} deg  ({b - a + 1:4d} beams)')

    best = max(runs_of('CLEAR'), key=lambda r: r[1] - r[0], default=None)
    if best:
        lo, hi = ang(best[0]), ang(best[1])
        print(f'\nWidest CONTIGUOUS clear arc: {lo:+.1f} to {hi:+.1f} deg '
              f'(width {hi - lo:.1f} deg)')
        sym = min(abs(lo), abs(hi)) - 5
        print(f'Symmetric, 5 deg margin  ->  angle_min: {-sym:.1f}  '
              f'angle_max: {sym:.1f}')
    rclpy.shutdown()


main()
