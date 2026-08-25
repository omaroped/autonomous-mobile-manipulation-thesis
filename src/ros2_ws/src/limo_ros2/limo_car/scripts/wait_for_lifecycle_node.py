#!/usr/bin/env python3
"""wait_for_lifecycle_node.py — block until a lifecycle node's services exist.

Replaces a fixed-time TimerAction before starting the Nav2 lifecycle manager.
The old approach (`TimerAction(period=6.0, ...)`) guessed that map_server would
be ready in 6 s. On a loaded machine (Gazebo + RViz + camera all starting at
once) that guess sometimes fails: the lifecycle manager's change_state call
times out before map_server has even registered its service, Nav2 bring-up
aborts silently, and the symptom is "no costmap in RViz, robot won't move" —
intermittent, because it depends on how busy the machine happens to be at that
moment. This waits for the actual event (the service existing) instead of a
clock, so it works regardless of machine speed.

Usage: wait_for_lifecycle_node.py <node_name> [<node_name> ...]
Exits 0 once every named node's `get_state` service is available, or exits 1
after WAIT_TIMEOUT_SEC with a clear message if one never appears.
"""
import sys

import rclpy
from rclpy.utilities import remove_ros_args
from lifecycle_msgs.srv import GetState

WAIT_TIMEOUT_SEC = 60.0
POLL_SEC         = 1.0


def main():
    # Strip ROS arguments before reading node names. launch_ros appends
    # `--ros-args -r __node:=...` to every Node's command line, so a bare
    # sys.argv[1:] picked those up as node names and tried to create a client for
    # '/--ros-args/get_state' -- an invalid service name, which raised
    # InvalidServiceNameException and killed this process on EVERY launch since it
    # was added. The failure was near-invisible: the launch file starts the Nav2
    # lifecycle manager from this process's OnProcessExit, so a crash-on-startup
    # fired that handler INSTANTLY and the manager began bringup with no wait at
    # all -- reintroducing the exact race this script exists to remove, and
    # surfacing downstream as "Unable to start transition 3 from current state
    # active" on controller_server / velocity_smoother / behavior_server.
    # (velocity_smoother was still launched when that was observed; it was removed on
    # 2026-08-26 -- see nav2_limo.launch.py. The failure above is unchanged, it just
    # names one node that no longer exists.)
    names = remove_ros_args(sys.argv)[1:]
    if not names:
        print('wait_for_lifecycle_node: no node names given', file=sys.stderr)
        return 1

    rclpy.init()
    node = rclpy.create_node('wait_for_lifecycle_node')
    clients = {n: node.create_client(GetState, f'/{n}/get_state') for n in names}

    remaining = set(names)
    waited = 0.0
    while remaining:
        for n in list(remaining):
            if clients[n].wait_for_service(timeout_sec=POLL_SEC):
                node.get_logger().info(f'{n}: get_state service is up')
                remaining.discard(n)
        if remaining:
            waited += POLL_SEC
            if waited >= WAIT_TIMEOUT_SEC:
                node.get_logger().error(
                    f'timed out after {WAIT_TIMEOUT_SEC:.0f} s waiting for: '
                    f'{sorted(remaining)} — check that these nodes actually started '
                    f'(colcon build clean? crashed on launch?)')
                node.destroy_node()
                rclpy.shutdown()
                return 1
            if int(waited) % 5 == 0:
                node.get_logger().info(
                    f'still waiting on {sorted(remaining)} ({waited:.0f}s)…')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
