#!/usr/bin/python3
"""
calibrate_contact_angle.py — Measure BOX_CONTACT_ANGLE for the aperture detector.

WHAT IT DOES
  For N_TRIALS: open the gripper → step-close slowly toward -0.20 rad →
  detect stall (measured position stops following command) → record stall angle.
  Prints the median (→ BOX_CONTACT_ANGLE), a MAD-derived spread
  (→ APERTURE_TOL suggestion), a separation check against the floor, and
  writes a per-trial CSV.

  DO NOT calibrate by logging where the old WELD_FALLBACK_ANGLE trigger fired —
  that calibrates the new detector with the flawed one (circular).

SETUP
  1. Launch the sim and let the robot dock at the pick table with the box
     in place, TCP positioned as it would be just before close_until_contact()
     runs (i.e. arm at the grasp pose).
  2. Kill or pause the orchestrator BEFORE it calls close_until_contact().
  3. Run this script: ros2 run limo_car calibrate_contact_angle
  4. In sim: if a trial nudges the box, reset it with SetEntityState before
     the next trial (the script does not reset it automatically).

SIGN CONVENTION
  Closing DECREASES the joint value: open ≈ +0.15, fully closed = -0.20.
  Matching GRIPPER_OPEN/GRIPPER_GRASP in nav_pick_orchestrator.py.
"""

import csv
import datetime
import statistics
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# ── match nav_pick_orchestrator.py constants exactly ────────────────────────
GRIPPER_OPEN  = [ 0.15,  0.15, -0.15, -0.15, -0.15,  0.15]
GRIPPER_GRASP = [-0.20, -0.20,  0.20,  0.20,  0.20, -0.20]
GRIPPER_JOINT = 'gripper_controller'
GRIPPER_TOPIC = '/mycobot_gripper_controller/commands'

# ── calibration tunables ─────────────────────────────────────────────────────
PROBE_ANGLE  = GRIPPER_GRASP[0]   # -0.20: always probe to full close
STEP         = 0.005              # rad per step — slow for precision
STEP_PERIOD  = 0.30               # seconds to settle per step
LAG_THRESH   = 0.02               # rad: actual lags commanded by this → stall onset
STALL_STEPS  = 3                  # consecutive lagging steps to confirm stall
N_TRIALS     = 5
TOL_FLOOR    = 0.015              # rad lower bound for suggested APERTURE_TOL
# ─────────────────────────────────────────────────────────────────────────────


class Calibrator(Node):
    def __init__(self):
        super().__init__('contact_angle_calibrator')
        self._pos = None
        self._pub = self.create_publisher(Float64MultiArray, GRIPPER_TOPIC, 10)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        self.get_logger().info('Calibrator ready — waiting for /joint_states …')

    def _joint_cb(self, msg: JointState):
        try:
            self._pos = msg.position[msg.name.index(GRIPPER_JOINT)]
        except ValueError:
            pass

    def position(self) -> float:
        while self._pos is None:
            rclpy.spin_once(self, timeout_sec=0.05)
        rclpy.spin_once(self, timeout_sec=0.02)
        return self._pos

    def command(self, angle: float):
        t = (GRIPPER_OPEN[0] - angle) / (GRIPPER_OPEN[0] - GRIPPER_GRASP[0])
        t = max(0.0, min(1.0, t))
        msg = Float64MultiArray()
        msg.data = [float(o + t * (c - o))
                    for o, c in zip(GRIPPER_OPEN, GRIPPER_GRASP)]
        self._pub.publish(msg)

    def spin_for(self, seconds: float):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def run_trial(self, trial: int):
        """One slow close. Returns stall angle or None if swept to full close."""
        self.command(GRIPPER_OPEN[0])
        self.spin_for(1.5)

        cmd      = GRIPPER_OPEN[0]
        lag_run  = 0
        while cmd > PROBE_ANGLE:
            cmd = max(cmd - STEP, PROBE_ANGLE)
            self.command(cmd)
            self.spin_for(STEP_PERIOD)
            meas    = self.position()
            lagging = (meas - cmd) > LAG_THRESH
            lag_run = lag_run + 1 if lagging else 0
            if lag_run >= STALL_STEPS:
                self.get_logger().info(
                    f'  trial {trial}: stall at {meas:+.4f} rad (cmd {cmd:+.4f})')
                return meas

        self.get_logger().warn(
            f'  trial {trial}: NO STALL — swept to {PROBE_ANGLE:+.3f}. '
            f'Box missing or misplaced?')
        return None


def main(args=None):
    rclpy.init(args=args)
    cal = Calibrator()
    stalls = []

    cal.get_logger().info(
        f'Starting {N_TRIALS} trials '
        f'(step={STEP} rad / {STEP_PERIOD}s, lag>{LAG_THRESH} ×{STALL_STEPS})')

    for k in range(1, N_TRIALS + 1):
        angle = cal.run_trial(k)
        if angle is not None:
            stalls.append(angle)
        cal.command(GRIPPER_OPEN[0])
        cal.spin_for(1.0)

    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = f'calibration_{stamp}.csv'
    with open(csv_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['trial', 'stall_angle_rad'])
        for i, a in enumerate(stalls, 1):
            w.writerow([i, f'{a:.5f}'])

    if len(stalls) < 3:
        cal.get_logger().error(
            f'Only {len(stalls)} valid trials — fix setup and re-run. Log: {csv_path}')
        cal.destroy_node()
        rclpy.shutdown()
        return

    med = statistics.median(stalls)
    mad = statistics.median(abs(a - med) for a in stalls)
    tol = max(3.0 * 1.4826 * mad, TOL_FLOOR)
    sep = med - PROBE_ANGLE                   # distance from contact to full close
    ok  = sep > (tol + 0.02)                  # separation invariant check

    print('\n================ RESULTS ================')
    print(f'  BOX_CONTACT_ANGLE = {med:+.4f}   # median of {len(stalls)} trials')
    print(f'  APERTURE_TOL      =  {tol:.4f}   # max(3·σ_MAD, {TOL_FLOOR})')
    print(f'  separation to floor: {sep:.4f} rad → '
          f'{"PASS" if ok else "FAIL — air close would land inside tolerance band"}')
    print(f'  per-trial CSV: {csv_path}')
    print('\nPaste BOX_CONTACT_ANGLE and APERTURE_TOL into nav_pick_orchestrator.py.')

    cal.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
