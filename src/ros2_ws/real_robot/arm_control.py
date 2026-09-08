#!/usr/bin/env python3
"""arm_control.py — safe Cartesian control of the real myCobot 280.

Consolidates what was measured on the physical robot 2026-08-31 (see
limo_car/config/hardware_calibration.yaml and docs/experiment_log.md
real_hardware_12..21) into one place, so the pipeline stops re-deriving it.

WHY THIS EXISTS RATHER THAN send_coords() DIRECTLY. pymycobot's IK has no
collision model: it does not know the depth camera, the chassis or the table
exist. On 2026-08-31 it solved a perfectly ordinary grasp target by folding
joint 2 to -121 deg, driving the upper link onto the camera. A guard that checks
AFTER the move has already completed cannot prevent that -- J2 reached -113 deg
before the check ran. So every motion here is INTERPOLATED and J2 is tested
between steps, which is what makes it stoppable partway.

Frames. base_link is the robot's frame: +x forward, +y LEFT, origin at the
wheelbase midpoint. The arm speaks its own frame in mm, rotated +90 deg yaw and
offset, so:
        arm_y = -(base_x + BASE_TO_ARM_X) * 1000
        arm_x =   base_y * 1000
Heights are given as metres ABOVE THE FLOOR, because that is what can actually
be measured with a tape:
        arm_z = (floor_height - FINGER_Z_OFFSET) * 1000
FINGER_Z_OFFSET exists because get_coords() reports the FLANGE, not the
fingertips; the gripper hangs 52.5 mm below it (measured, not derived).
"""
import time

from pymycobot.mycobot import MyCobot

# ── Calibration, all MEASURED on the physical robot 2026-08-31 ───────────────
BASE_TO_ARM_X   = 0.030    # base_link -> arm base, metres forward
FINGER_Z_OFFSET = 0.0525   # fingertips sit this far below the reported flange
Z_UNDERSHOOT    = 0.008    # arm lands low by ~this at ~0.22 m extension
GRASP_RX, GRASP_RY, GRASP_RZ = -180.0, 0.0, -38.0
# rx/ry point the gripper straight DOWN. rz sets the wrist angle about the vertical.
#
# -45 was a first approximation: on a square cube it clearly beat -90 and 0, which
# both present the jaws diagonally. But -45 was never checked for exact alignment,
# and it left the finger faces rotated slightly CLOCKWISE (viewed from above) with
# respect to the cube's faces -- so the fingers met the cube near its corners rather
# than flat, which is why grasps closed at 41-48 and only settled to 38 during the
# lift. Verified visually at -38 on 2026-09-04: finger faces parallel to the cube
# faces. Positive rz is counter-clockwise from above.
J2_MIN_DEG      = -85.0    # below this the upper link approaches the camera
POSE_TOL        = 0.003   # m; accept a pose within this of the target on every axis
# 3 mm, tightened from 6 mm on 2026-09-03. At 6 mm the loop accepted a pose 4.5 mm
# short of commanded, which is harmless when closing on a 24.7 mm cube sitting on a
# table -- the fingers straddle it anyway -- but is not harmless when PLACING one cube
# on top of another, where the landing footprint is the cube itself.
HOVER_TOL       = 0.015   # m; the approach pose ABOVE the target only has to be roughly
                          # right -- the descent is the step that is corrected precisely.

# Grasp ABOVE the object's centre by this much.
#
# The grasp point is the midpoint between the fingers, but the fingertips extend
# BELOW it. Targeting the cube's centre therefore drives the tips toward the
# supporting surface: with a 24.7 mm cube on a 130 mm table the centre sits at
# 142 mm and leaves only ~12 mm of room underneath. When the tips reach the table
# first the arm presses down, which loads the joints, takes up their backlash and
# introduces exactly the positional variability this calibration is trying to
# remove -- reported on the robot 2026-09-04 as the fingers "pushing hard".
#
# Grasping 6 mm high keeps the fingers on the cube's face (6 mm below its top edge
# for this cube) while lifting the tips clear of the surface. The correct long-term
# fix is to measure the fingertip length below the grasp point and derive the
# clearance from it; this is a conservative interim that does not require it.
GAIN_BAND       = 0.015   # m; above this a height correction is commanded 1:1, so a
                          # large descent cannot overshoot into the object
GRASP_RAISE     = 0.012
# 12 mm asked, ~7 mm delivered. The arm undershoots commanded height by roughly
# 5 mm at grasp extension and settle_height does not close that gap: it exhausts
# its iterations because the arm moves less than each commanded correction near
# this part of the workspace. Asking for centre + 6 mm therefore landed exactly on
# the centre (measured 2026-09-04: asked 14.7 cm, reached 14.2 cm). The raise is
# specified as what must be ASKED, with the shortfall folded in, rather than
# pretending the loop converges.

# Feedforward compensation for mechanical backlash in the joints, applied FORWARD.
#
# NOT correctable by the position loop. That loop closes on get_coords(), which
# derives from the joint ENCODERS -- the play happens downstream of them, so the arm
# reports arriving on target while physically sitting ~2 mm short of it. Observed
# 2026-09-03: the gripper can be pushed ~2 mm by hand at the hover pose, and it
# consistently settles toward the robot. Feedback cannot see what the encoders cannot
# see, so this is a fixed offset rather than a gain.
LATERAL_COMP    = 0.010   # m; shift targets this far to the robot's LEFT (+y)
# CALIBRATED, CAUSE NOT ISOLATED. The gripper consistently settled to the RIGHT of
# the cube, and +10 mm centres it (verified 2026-09-04 by parking the open gripper
# over the target and inspecting). Which of two causes is responsible is not
# established:
#   * perception reporting the target slightly right of its true position, or
#   * a lateral offset between the finger line and the point get_coords() reports,
#     the gripper being mounted with a 45 deg wrist rotation.
# The drive trials showed the arm reaching commanded y within 1-4 mm, which argues
# against a servo-level bias and for one of the two above. Recorded as a single
# empirical constant rather than split between layers on a guess; isolating it needs
# a target of known position measured independently of the camera.

BACKLASH_X_COMP = 0.000   # m; feedforward reach correction -- MEASURED AS ZERO
# Set to 0 after a direct measurement on 2026-09-03: commanded base_link x=0.200, the
# arm reported 0.1955, and a tape from the chassis front (base_link 0.161) to the gap
# between the fingertips read 34 mm -- exactly what the model predicts. The arm sits
# where it says it sits; there is no hidden horizontal offset and no reach shortfall.
#
# Two earlier readings suggested otherwise and both were artefacts:
#   * a cube "40 mm" from the chassis front was measured while held 18.8 cm IN THE AIR,
#     so the tape ran diagonally and read long;
#   * a 6 mm discrepancy on 2026-08-31 was referenced off the wheel centre, which is
#     hard to locate -- the chassis front is a hard edge and disagrees with it.
# Kept as a named constant rather than deleted so a future measurement has somewhere
# to go, and so the reasoning above is not re-derived from scratch.

# Max forward reach at GRASP height, measured by sweeping send_coords targets until
# the arm stopped tracking. Height-dependent: reaching down as well as out gives only
# 0.228. scene.yaml's arm_reach 0.24 is a Gazebo value.
MAX_REACH_X     = 0.243

# Gripper. Readback needs an explicit type argument in this pymycobot version or it
# raises MyCobotDataException; type 4 works on this arm.
GRIP_OPEN, GRIP_CLOSE = 100, 0
GRIP_THRESHOLD  = 15       # readback above this = holding something
GRIP_TYPE       = 4
# Reference values, measured: closed on air settles to 0; closed on the 24.7 mm cube
# stalls at 37-38 square-on, ~61 gripped corner-to-corner. The value measures OBJECT
# WIDTH, so it distinguishes a good grasp from a bad one, not merely present/absent.


class ArmError(RuntimeError):
    pass


class ArmController:

    def __init__(self, port='/dev/ttyACM0', baud=115200, speed=20, verbose=True):
        self.mc = MyCobot(port, baud)
        self.speed = speed
        self.verbose = verbose
        time.sleep(2)
        if not self.mc.get_angles():
            raise ArmError('arm did not reply to get_angles() — powered on? cable?')
        self.mc.set_gripper_mode(0)
        self.mc.init_eletric_gripper()
        time.sleep(1)

    # ── plumbing ────────────────────────────────────────────────────────────
    def _log(self, msg):
        if self.verbose:
            print(msg, flush=True)

    def to_arm(self, base_x, base_y, floor_h):
        """base_link metres -> arm frame mm. Height is above the FLOOR.

        base_x carries BACKLASH_X_COMP so the arm physically lands where asked
        rather than where its encoders believe it is.
        """
        return ((base_y + LATERAL_COMP) * 1000.0,
                -(base_x + BACKLASH_X_COMP + BASE_TO_ARM_X) * 1000.0,
                (floor_h - FINGER_Z_OFFSET) * 1000.0)

    def state(self):
        c, a = self.mc.get_coords(), self.mc.get_angles()
        if not c or not a:
            raise ArmError('no reply from arm')
        # LATERAL_COMP is removed here so that state() reports the target frame the
        # caller asked in. Leaving it in would make the pose-verification loop see a
        # 10 mm error it created itself and drive the arm a further 10 mm each pass.
        return {
            'base_x': -c[1] / 1000.0 - BASE_TO_ARM_X,
            'base_y': c[0] / 1000.0 - LATERAL_COMP,
            'floor_h': c[2] / 1000.0 + FINGER_Z_OFFSET,
            'j2': a[1], 'coords': c, 'angles': a,
        }

    # ── gripper ─────────────────────────────────────────────────────────────
    def grip_value(self):
        for t in (GRIP_TYPE, 3, 1):
            try:
                v = self.mc.get_gripper_value(t)
                if v is not None:
                    return v
            except Exception:
                pass
        return None

    def _grip_settled(self, timeout=14):
        """Wait until the reading stops changing.

        Necessary, not defensive: an EMPTY gripper also reads 36-37 at t+4 s
        while still travelling, so a fixed-delay sample cannot tell 'holding the
        cube' from 'still closing on nothing'.
        """
        last, stable = None, 0
        for _ in range(timeout):
            time.sleep(1)
            v = self.grip_value()
            if v is not None and v == last:
                stable += 1
                if stable >= 3:
                    return v
            else:
                stable = 0
            last = v
        return self.grip_value()

    def open_gripper(self):
        self.mc.set_gripper_value(GRIP_OPEN, 30)
        time.sleep(3)
        return self.grip_value()

    def close_gripper(self):
        self.mc.set_gripper_value(GRIP_CLOSE, 20)
        return self._grip_settled()

    # ── guarded motion ──────────────────────────────────────────────────────
    def move_to(self, base_x, base_y, floor_h, steps=4, settle=3.0,
                compensate=True, rz=GRASP_RZ, tol=None):
        """Interpolated Cartesian move with J2 checked BETWEEN steps.

        Returns the final state. Raises ArmError if J2 crosses J2_MIN_DEG, having
        stopped where it was rather than completing the motion.
        """
        if base_x > MAX_REACH_X:
            raise ArmError(f'{base_x:.3f} m exceeds measured reach {MAX_REACH_X:.3f} m')

        tx, ty, tz = self.to_arm(base_x, base_y, floor_h)
        if compensate:
            tz += Z_UNDERSHOOT * 1000.0

        c = self.mc.get_coords()
        for i in range(1, steps + 1):
            f = i / float(steps)
            self.mc.send_coords([c[0] + (tx - c[0]) * f,
                                 c[1] + (ty - c[1]) * f,
                                 c[2] + (tz - c[2]) * f,
                                 GRASP_RX, GRASP_RY, rz], self.speed, 0)
            time.sleep(settle)
            s = self.state()
            if s['j2'] < J2_MIN_DEG:
                raise ArmError(
                    f'J2 {s["j2"]:.1f} deg past the {J2_MIN_DEG} limit at step '
                    f'{i}/{steps} — stopped before reaching the target (camera)')

        # Tolerance follows the PURPOSE of the move. A hover 6 cm above the target
        # only has to be roughly right -- settle_height corrects the height during the
        # descent, which is the only point at which it matters. Applying the grasp
        # tolerance to the hover rejected an entirely serviceable pose on 2026-09-04
        # over a 6 mm height residual that the very next step would have removed.
        tol = POSE_TOL if tol is None else tol

        # CLOSE THE LOOP ON ALL THREE AXES, not just height.
        #
        # The IK settles where it settles: on 2026-09-03 a target 3 cm right of
        # centre came back 10 cm LEFT -- 13 cm of lateral error -- and the code
        # accepted it because only J2 was being checked. The gripper then closed on
        # empty air. Commanding a pose is not the same as reaching it, and on a
        # 24.7 mm cube a 13 mm error is already a miss.
        for _ in range(3):
            s = self.state()
            ex = base_x - s['base_x']
            ey = base_y - s['base_y']
            ez = floor_h - s['floor_h']
            if max(abs(ex), abs(ey), abs(ez)) < tol:
                return s
            c = self.mc.get_coords()
            self.mc.send_coords([c[0] + ey * 1000.0,      # base +y  -> arm +x
                                 c[1] - ex * 1000.0,      # base +x  -> arm -y
                                 c[2] + ez * 1000.0,
                                 GRASP_RX, GRASP_RY, rz], 15, 0)
            time.sleep(5)
            if self.state()['j2'] < J2_MIN_DEG:
                raise ArmError('J2 guard tripped while correcting position')
        s = self.state()
        err = max(abs(base_x - s['base_x']), abs(base_y - s['base_y']),
                  abs(floor_h - s['floor_h']))
        if err > tol * 2:
            raise ArmError(
                f'could not reach target: asked x={base_x:.3f} y={base_y:.3f} '
                f'h={floor_h:.3f}, reached x={s["base_x"]:.3f} y={s["base_y"]:.3f} '
                f'h={s["floor_h"]:.3f} — residual {err*1000:.0f} mm. Likely outside '
                f'the arm\'s workspace at this height.')
        return s

    def settle_height(self, floor_h, base_x=None, base_y=None,
                      tries=6, tol=0.004, gain=1.4):
        """Correct residual height error in place, and SAY SO if it cannot.

        The arm undershoots commanded height by an amount that varies with
        extension, so a single commanded move never lands on a 24.7 mm cube's
        centre. This closes the loop on the measured height.

        Two things learned on 2026-09-03, both from the same symptom -- the arm
        settling ~10 mm below target on three consecutive runs:

        * Three iterations was not enough, and the correction was applied at unity
          gain into a system that undershoots, so each step fell short of its own
          correction. `gain` over-drives it; more tries give it room to converge.
        * The old version returned SILENTLY on failure, so a persistent 10 mm error
          looked like success. It now warns. That matters beyond neatness: the
          fingers extend below their own centre, so descending 10 mm low eats the
          clearance between the fingertips and the table, and the arm presses down.
        """
        # HOLD THE INTENDED x/y, DO NOT RE-COMMAND THE MEASURED ONE.
        #
        # This loop previously read the arm's current coordinates and re-sent c[0]
        # and c[1] unchanged while adjusting z. That makes each iteration adopt
        # wherever the arm happened to settle as the new lateral/forward goal: the
        # arm settles slightly short, the next iteration treats the short position
        # as the target, settles short of THAT, and the error compounds. Observed
        # 2026-09-04 as three visible steps walking the gripper back toward the
        # robot after it had arrived correctly over the cube, leaving it ~15 mm
        # short and gripping the near edge. Holding the commanded x/y fixed removes
        # the ratchet; only z is being corrected, so only z should move.
        hold_x = None if base_y is None else (base_y + LATERAL_COMP) * 1000.0
        hold_y = (None if base_x is None else
                  -(base_x + BACKLASH_X_COMP + BASE_TO_ARM_X) * 1000.0)

        for _ in range(tries):
            s = self.state()
            err = floor_h - s['floor_h']
            if abs(err) < tol:
                return s
            # The gain compensates for the arm moving less than commanded, which is
            # true of a small trim and false of a large move. Applying it to the
            # initial descent from the hover -- a ~60 mm error -- commanded 84 mm and
            # drove the fingers into the object before later iterations pulled them
            # back up (observed on the robot 2026-09-04). Gain is therefore applied
            # only within GAIN_BAND, where the undershoot actually dominates.
            step = err * (gain if abs(err) < GAIN_BAND else 1.0)
            c = self.mc.get_coords()
            self.mc.send_coords([c[0] if hold_x is None else hold_x,
                                 c[1] if hold_y is None else hold_y,
                                 c[2] + step * 1000.0,
                                 GRASP_RX, GRASP_RY, GRASP_RZ], 15, 0)
            time.sleep(4)
            if self.state()['j2'] < J2_MIN_DEG:
                raise ArmError('J2 guard tripped while settling height')
        s = self.state()
        resid = (floor_h - s['floor_h']) * 1000.0
        if abs(resid) > tol * 1000.0:
            self._log(f'  WARNING: height did not converge — asked '
                      f'{floor_h*100:.1f} cm, reached {s["floor_h"]*100:.1f} cm '
                      f'({resid:+.0f} mm). Fingertip clearance to the table is '
                      f'{abs(resid):.0f} mm less than intended.')
        return s

    # ── task-level ──────────────────────────────────────────────────────────
    def pick_at(self, base_x, base_y, cube_centre_h, hover=0.06, lift=0.06):
        """Hover above, descend vertically, close, lift, verify.

        Hover-then-descend is not cosmetic: approaching the target diagonally
        makes the fingers arrive sideways and shove the cube off its mark
        (observed 2026-08-31). The descent must be vertical.
        """
        grasp_h = cube_centre_h + GRASP_RAISE
        self._log(f'pick at base_link x={base_x:.3f} y={base_y:.3f}, '
                  f'cube centre {cube_centre_h*100:.1f} cm, '
                  f'grasping at {grasp_h*100:.1f} cm '
                  f'(+{GRASP_RAISE*1000:.0f} mm for fingertip clearance)')
        self.open_gripper()
        s = self.move_to(base_x, base_y, grasp_h + hover, tol=HOVER_TOL)
        self._log(f'  hover   x={s["base_x"]:.3f} y={s["base_y"]:.3f} '
                  f'h={s["floor_h"]*100:.1f}cm J2={s["j2"]:.1f}')

        s = self.settle_height(grasp_h, base_x=base_x, base_y=base_y)
        self._log(f'  descend x={s["base_x"]:.3f} y={s["base_y"]:.3f} '
                  f'h={s["floor_h"]*100:.1f}cm J2={s["j2"]:.1f}')

        v_close = self.close_gripper()
        c = self.mc.get_coords()
        self.mc.send_coords([c[0], c[1], c[2] + lift * 1000.0,
                             GRASP_RX, GRASP_RY, GRASP_RZ], 15, 0)
        time.sleep(6)
        v_lift = self.grip_value()

        # The POST-LIFT value decides. At close time the fingers can stall early
        # on a corner and read wildly high -- 65 was observed for a 24.7 mm cube
        # that settled to 37 once lifted.
        held = v_lift is not None and v_lift > GRIP_THRESHOLD
        self._log(f'  gripper close={v_close} lift={v_lift} -> '
                  f'{"HELD" if held else "NOT HELD"}')
        return held, v_lift

    def place_at(self, base_x, base_y, surface_h, cube_edge=0.0247,
                 hover=0.06, approach_gap=0.002):
        """Carry to a target and release, cube resting on the surface."""
        centre = surface_h + cube_edge / 2.0 + approach_gap
        s = self.move_to(base_x, base_y, centre + hover, tol=HOVER_TOL)
        self._log(f'  hover   x={s["base_x"]:.3f} h={s["floor_h"]*100:.1f}cm')
        s = self.settle_height(centre, base_x=base_x, base_y=base_y)
        self._log(f'  at drop x={s["base_x"]:.3f} h={s["floor_h"]*100:.1f}cm')
        self.open_gripper()
        c = self.mc.get_coords()
        self.mc.send_coords([c[0], c[1], c[2] + hover * 1000.0,
                             GRASP_RX, GRASP_RY, GRASP_RZ], 15, 0)
        time.sleep(5)
        return self.state()

    def park(self):
        """Return to zeros and confirm every joint is inside its limit.

        The servos release when the robot is powered off, so the arm falls to
        wherever gravity leaves it. If a joint lands outside its limit the arm
        answers get_angles() normally but SILENTLY DISCARDS every motion command
        on the next session -- which looks exactly like a dead arm and cost two
        hours on 2026-08-31. Parking folded and low reduces the drop.
        """
        self.mc.send_angles([0, 0, 0, 0, 0, 0], 25)
        time.sleep(10)
        a = self.mc.get_angles()
        lim = [(-168, 168), (-135, 135), (-150, 150),
               (-145, 145), (-165, 165), (-180, 180)]
        bad = [i + 1 for i, (v, (lo, hi)) in enumerate(zip(a, lim))
               if not (lo < v < hi)]
        self._log(f'parked: {a}' + ('  ALL INSIDE LIMITS' if not bad
                                    else f'  OUT OF LIMITS: J{bad}'))
        return a, bad


def recover_joint_limits(port='/dev/ttyACM0', baud=115200):
    """Free an arm that has fallen outside its limits and won't move.

    Standalone because it must run BEFORE ArmController is usable. Commands a
    target with every joint clamped inside range: an array containing even one
    illegal value is rejected whole, which is why the arm appears dead.
    """
    mc = MyCobot(port, baud)
    time.sleep(2)
    a = mc.get_angles()
    if not a:
        raise ArmError('no reply — arm powered on? cable seated?')
    lim = [(-160, 160), (-130, 130), (-145, 145),
           (-135, 135), (-160, 160), (-175, 175)]   # inside the true limits
    fixed = [max(lo, min(hi, v)) for v, (lo, hi) in zip(a, lim)]
    if fixed == a:
        print(f'all joints already legal: {a}')
        return a
    print(f'out of range {a} -> commanding {fixed}')
    mc.send_angles(fixed, 20)
    time.sleep(8)
    print(f'now: {mc.get_angles()}')
    return mc.get_angles()


if __name__ == '__main__':
    arm = ArmController()
    s = arm.state()
    print(f'base_link x={s["base_x"]:.3f} y={s["base_y"]:.3f} '
          f'height={s["floor_h"]*100:.1f}cm J2={s["j2"]:.1f} '
          f'gripper={arm.grip_value()}')
