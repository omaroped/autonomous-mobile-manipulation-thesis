#!/usr/bin/env bash
# sync_to_robot.sh — deploy the MINIMAL real-hardware package set to the robot.
#
# WHY THIS EXISTS: on 2026-08-07 a bulk copy of the whole laptop workspace (45+
# packages, 1.3 GB) put dead code on the robot next to the real code, and the dead
# code got run by mistake (limo_cobot_bridge/bridge_node.py — June-2026, depends on
# a service that no longer exists — mistaken for the real arm_bridge.py). It also
# duplicated limo_base/limo_description/limo_msgs, which already exist in the
# robot's OWN working driver workspace (~/limo_ros2_ws) — whichever workspace gets
# sourced last would have silently shadowed the real hardware drivers.
#
# This script copies exactly three packages, and trims the one that's mostly
# irrelevant-vendor-arm bloat:
#   - limo_car                  (real_arm.xacro, launch files, controllers.yaml)
#   - limo_cobot_moveit_config  (SRDF, kinematics, initial_positions.yaml)
#   - mycobot_description       (TRIMMED — only mycobot_280_m5/ + adaptive_gripper/,
#                                 41 MB instead of 1.1 GB; real_arm.xacro references
#                                 exactly these two subdirectories and nothing else)
#
# Target layout on the robot — standard underlay/overlay, NO duplicate package names:
#   ~/limo_ros2_ws  — underlay, the real hardware drivers, untouched by this script
#   ~/thesis_ws     — overlay, exactly what this script deploys
#
# Usage:  ./sync_to_robot.sh [robot-ssh-host]
#         defaults to the Tailscale address; pass a different host/IP to override.

set -euo pipefail

ROBOT="${1:-agilex@100.106.125.32}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/src" && pwd)"
STAGING="$(mktemp -d)"
trap 'rm -rf "$STAGING"' EXIT

echo "=== staging the trimmed package set ==="

# limo_car and limo_cobot_moveit_config — copied whole, already small.
rsync -a --exclude="__pycache__" --exclude="*.pyc" \
  "$REPO_ROOT/limo_ros2/limo_car/" "$STAGING/limo_car/"
rsync -a "$REPO_ROOT/limo_cobot_moveit_config/" "$STAGING/limo_cobot_moveit_config/"

# mycobot_description — TRIMMED. real_arm.xacro only ever references
# urdf/mycobot_280_m5/ and urdf/adaptive_gripper/ (verified: grep -o "urdf/[a-z0-9_/]*"
# gazebo/real_arm.xacro). Everything else in that package is a different robot
# entirely (mecharm, myarm_*, mybuddy, mypalletizer_*, ...) or a CAD source file
# never read at runtime (mycobot_step.STEP, 57 MB on its own).
MCD_SRC="$REPO_ROOT/mycobot_ros2/mycobot_description"
MCD_DST="$STAGING/mycobot_description"
mkdir -p "$MCD_DST/urdf"
cp "$MCD_SRC/package.xml" "$MCD_SRC/setup.py" "$MCD_DST/"
[ -f "$MCD_SRC/setup.cfg" ] && cp "$MCD_SRC/setup.cfg" "$MCD_DST/"
cp -r "$MCD_SRC/resource" "$MCD_DST/"
cp -r "$MCD_SRC/mycobot_description" "$MCD_DST/mycobot_description"
rsync -a --exclude="*.STEP" "$MCD_SRC/urdf/mycobot_280_m5" "$MCD_DST/urdf/"
rsync -a "$MCD_SRC/urdf/adaptive_gripper" "$MCD_DST/urdf/"

echo "staged: $(du -sh "$STAGING" | cut -f1)"
echo

echo "=== deploying to $ROBOT:~/thesis_ws/src ==="
ssh "$ROBOT" 'mkdir -p ~/thesis_ws/src'
rsync -az --delete "$STAGING/" "$ROBOT:~/thesis_ws/src/"

echo
echo "=== building on the robot (underlay: ~/limo_ros2_ws) ==="
ssh "$ROBOT" '
  source /opt/ros/humble/setup.bash
  source ~/limo_ros2_ws/install/setup.bash
  cd ~/thesis_ws && colcon build
'

echo
echo "DONE. On the robot: source ~/limo_ros2_ws/install/setup.bash && source ~/thesis_ws/install/setup.bash"
