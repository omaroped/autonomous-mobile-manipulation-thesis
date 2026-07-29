#!/usr/bin/env bash
# kill_sim.sh — verified clean teardown of the thesis simulation stack.
#
# WHY THIS EXISTS: orphaned nodes from a previous run produce symptoms that are
# indistinguishable from algorithmic bugs, and they silently invalidate experiments.
# Real cases from this project:
#   - two base_pin processes  -> each pins the base to a different remembered pose,
#                                fighting at 50 Hz -> "the robot oscillates for no reason"
#   - orphaned grasp_attacher -> still welding a box to a stale gripper pose at 25 Hz
#                                -> "a box is floating in mid-air, nobody put it there"
#   - stale gzserver          -> holds the sim so a new launch never starts its own
#                                -> "my edit had no effect" (watching the OLD world)
#
# TWO TRAPS this script avoids:
#   1. `pkill -f "<pattern>"` can kill its own shell, because the pattern string appears
#      in pkill's own command line and therefore matches itself. The [g]-bracket form
#      below cannot match itself.
#   2. Broad patterns like "_server" or "orchestrator" also match the Antigravity IDE
#      language server and unrelated Node apps. Every pattern here is specific.
#
# Usage:  ./kill_sim.sh        (run before every launch)

set -uo pipefail

# Every pattern is anchored to a full path or a unique node name. Bare words are NOT
# safe: an earlier version used "[s]pawner" and killed /usr/libexec/gvfsd-trash, whose
# command line contains "--spawner". Verified against `ps` before every kill below.
PATTERN='[g]zserver|[g]zclient|[r]viz2/rviz2|[m]oveit_ros_move_group/move_group|[r]obot_state_publisher/robot_state_publisher|[l]imo_car/nav_pick_orchestrator|[l]imo_car/grasp_attacher|[l]imo_car/smart_grasp|[l]imo_car/box_pose_estimator|[l]imo_car/tag_dock_estimator|[l]imo_car/base_pin|[l]imo_car/base_joint_state_pub|[n]av2_[a-z_]*/|[c]ontroller_manager/spawner'

pids=$(pgrep -f "$PATTERN" 2>/dev/null | tr '\n' ' ')

if [ -z "${pids// }" ]; then
  echo "Nothing running — already clean."
  exit 0
fi

echo "Found simulation processes:"
ps -o pid,cmd -p ${pids} 2>/dev/null | tail -n +2 | cut -c1-110
echo
echo "Killing: ${pids}"
kill -9 ${pids} 2>/dev/null
sleep 2

# ROS 2 child nodes do not always die with their launch parent — sweep again.
remaining=$(pgrep -f "$PATTERN" 2>/dev/null | tr '\n' ' ')
if [ -n "${remaining// }" ]; then
  echo "Second pass on survivors: ${remaining}"
  kill -9 ${remaining} 2>/dev/null
  sleep 2
fi

final=$(pgrep -f "$PATTERN" 2>/dev/null | tr '\n' ' ')
if [ -z "${final// }" ]; then
  echo "CLEAN — verified nothing left."
  exit 0
else
  echo "STILL ALIVE (investigate before launching):"
  ps -o pid,cmd -p ${final} 2>/dev/null | tail -n +2 | cut -c1-110
  exit 1
fi
