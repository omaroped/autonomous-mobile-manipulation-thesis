#!/usr/bin/env python3
"""arm_keyboard_jog.py — direct single-joint (+ gripper) keyboard control of the
real myCobot 280.

Talks straight to pymycobot over serial. NO ROS, NO MoveIt, NO planner — deliberately,
after the 2026-08-10 incident where a MoveGroup goal constraining only ONE joint let
OMPL swing the other five however it wanted (up to ~170 deg on one joint) on the REAL
arm. This tool can only ever move ONE joint (or the gripper), by a small fixed step,
per keypress. Nothing else can move as a side effect.

Needs exclusive use of the serial port — stop arm_bridge.py first if it's running,
they cannot both hold /dev/ttyACM0 at once.

Run directly on the robot (needs /dev/ttyACM0):
    python3 arm_keyboard_jog.py

Keys:
    Left/Right   select joint/gripper (also 1-6 = joints, 7 = gripper, directly)
    Up/Down      jog the selection by the step size (also + / -)
    [  / ]       halve / double the step size (starts at 3 deg / 3 for gripper)
    space        stop (mc.stop())
    a            print current angles + gripper value
    q            quit
"""
import sys
import time
import termios
import tty

from pymycobot.mycobot import MyCobot

PORT       = '/dev/ttyACM0'   # NEVER auto-detect -- CP2102/ttyUSB0 is the lidar
BAUD       = 115200
SPEED      = 20                # 0..100, slow and deliberate for manual jogging
STEP_DEG   = 3.0
GRIPPER_INDEX = 6
NAMES = [
    'joint2_to_joint1 (base)',
    'joint3_to_joint2',
    'joint4_to_joint3',
    'joint5_to_joint4',
    'joint6_to_joint5',
    'joint6output_to_joint6 (wrist)',
    'gripper (0=closed, 100=open)',
]
# From pymycobot's own robot_limit.json (MyCobot class) -- the REAL firmware-
# enforced range per joint, not the sim URDF's (looser, ~-180/180) limits.
# send_angle() raises MyCobotDataException if you exceed these; clamp first
# so a jog near a limit degrades to "stop at the limit", not a crash.
ANGLE_MIN = [-170, -135, -150, -145, -170, -180]
ANGLE_MAX = [ 170,  140,  150,  135,  170,  180]


def get_angles_safe(mc, fallback):
    """mc.get_angles() occasionally returns None on a dropped serial read (same
    hiccup arm_bridge.py retries for at startup). Retry briefly, then fall back
    to the last known-good angles rather than crash the session."""
    for _ in range(3):
        angles = mc.get_angles()
        if angles and angles != -1:
            return angles
        time.sleep(0.1)
    print('  (no response reading angles — using last known values)')
    return fallback


GRIPPER_TYPE = 1   # adaptive gripper (this arm's hardware). get_gripper_value's
                    # default (None) is documented as "1" but the library's own
                    # validation rejects None outright -- must pass 1 explicitly.


def get_gripper_safe(mc, fallback):
    for _ in range(3):
        try:
            val = mc.get_gripper_value(GRIPPER_TYPE)
        except Exception as e:
            print(f'  get_gripper_value error: {e}')
            val = None
        if val is not None and val != -1:
            return val
        time.sleep(0.1)
    print('  (no response reading gripper — using last known value)')
    return fallback


def read_key():
    """Single keypress, with arrow keys decoded from their escape sequence
    (ESC [ A/B/C/D) into the strings 'UP'/'DOWN'/'RIGHT'/'LEFT'."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            rest = sys.stdin.read(2)
            return {'[A': 'UP', '[B': 'DOWN', '[C': 'RIGHT', '[D': 'LEFT'}.get(rest, '')
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def jog_joint(mc, angles, selected, step, sign):
    angles = get_angles_safe(mc, angles)
    old_val = angles[selected]
    new = max(ANGLE_MIN[selected], min(ANGLE_MAX[selected], old_val + sign * step))
    try:
        mc.send_angle(selected + 1, new, SPEED)
    except Exception as e:
        print(f'  send_angle rejected: {e}')
        return angles
    angles[selected] = new   # assume it arrives; next safe-read corrects it if not
    hit_limit = ' (LIMIT)' if new in (ANGLE_MIN[selected], ANGLE_MAX[selected]) else ''
    print(f'{NAMES[selected]}: {old_val:+.1f} -> {new:+.1f} deg{hit_limit}')
    return angles


def jog_gripper(mc, grip_val, step, sign):
    grip_val = get_gripper_safe(mc, grip_val)
    new = max(0, min(100, int(round(grip_val + sign * step))))
    try:
        mc.set_gripper_value(new, SPEED)
    except Exception as e:
        print(f'  set_gripper_value rejected: {e}')
        return grip_val
    hit_limit = ' (LIMIT)' if new in (0, 100) else ''
    print(f'{NAMES[GRIPPER_INDEX]}: {grip_val} -> {new}{hit_limit}')
    return new


def main():
    print(f'connecting to {PORT}...')
    mc = MyCobot(PORT, BAUD)
    angles = mc.get_angles()
    if not angles or angles == -1:
        print('no response from the arm -- is arm_bridge.py still holding the port?')
        sys.exit(1)
    try:
        grip_val = mc.get_gripper_value(GRIPPER_TYPE)
    except Exception as e:
        print(f'  get_gripper_value error: {e}')
        grip_val = None
    if grip_val is None or grip_val == -1:
        grip_val = 50   # unknown -- assume mid-range rather than crash on first jog
    print(f'connected. current angles (deg): {[round(a, 1) for a in angles]}, '
          f'gripper: {grip_val}')

    selected = 0   # 0-5 = joints, 6 = gripper
    step = STEP_DEG
    print(f'\nselected: {NAMES[selected]}   step: {step}')
    print('Left/Right select | Up/Down jog | [ ] step size | space STOP | a status | q quit\n')

    while True:
        k = read_key()

        if k == 'q':
            print('quit — arm left where it is')
            break

        elif k in '1234567':
            selected = int(k) - 1
            print(f'selected: {NAMES[selected]}')

        elif k == 'RIGHT':
            selected = (selected + 1) % 7
            print(f'selected: {NAMES[selected]}')

        elif k == 'LEFT':
            selected = (selected - 1) % 7
            print(f'selected: {NAMES[selected]}')

        elif k in ('+', '=', 'UP'):
            if selected == GRIPPER_INDEX:
                grip_val = jog_gripper(mc, grip_val, step, +1.0)
            else:
                angles = jog_joint(mc, angles, selected, step, +1.0)

        elif k in ('-', '_', 'DOWN'):
            if selected == GRIPPER_INDEX:
                grip_val = jog_gripper(mc, grip_val, step, -1.0)
            else:
                angles = jog_joint(mc, angles, selected, step, -1.0)

        elif k == '[':
            step = max(0.5, step / 2)
            print(f'step size: {step}')

        elif k == ']':
            step = min(20.0, step * 2)
            print(f'step size: {step}')

        elif k == ' ':
            mc.stop()
            print('STOP sent')

        elif k == 'a':
            angles = get_angles_safe(mc, angles)
            grip_val = get_gripper_safe(mc, grip_val)
            print(f'current angles (deg): {[round(a, 1) for a in angles]}, '
                  f'gripper: {grip_val}')

        elif k == '\x03':   # Ctrl-C
            print('quit — arm left where it is')
            break


if __name__ == '__main__':
    main()
