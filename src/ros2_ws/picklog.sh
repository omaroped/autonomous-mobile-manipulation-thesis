#!/usr/bin/env bash
# picklog.sh — pull the shareable summary out of a full run log.
#
# WHY: a full nav_pick run log is ~90% perception heartbeat. Measured on
# /tmp/run3.log: 4845 lines from box_pose_estimator and 1642 from
# tag_dock_estimator, against 69 from nav_pick_orchestrator — and most of that
# is the sim idling AFTER the task finished (that log spanned 83 minutes for a
# 3-minute task). The task itself is small; it is just buried.
#
# Usage:  ./picklog.sh /tmp/f56.log            # summary to stdout
#         ./picklog.sh /tmp/f56.log > share.txt
set -euo pipefail
LOG="${1:-/tmp/f56.log}"
[ -f "$LOG" ] || { echo "no such log: $LOG" >&2; exit 1; }

echo "=== $LOG — $(wc -l < "$LOG") lines total ==="
echo

echo "--- orchestrator (the actual task) ---"
grep -E "nav_pick_orchestrator" "$LOG" | sed 's/.*nav_pick_orchestrator\]: //' || true
echo
echo "--- grasp plugin ---"
grep -oE "GazeboGraspFix.*" "$LOG" | sort | uniq -c || true
echo
echo "--- errors / warnings elsewhere ---"
grep -E "\[(ERROR|WARN)\]" "$LOG" \
  | grep -vE "nav_pick_orchestrator|box_pose_estimator|tag_dock_estimator" \
  | sed 's/\[\([a-z_0-9-]*\)\].*\[\(ERROR\|WARN\)\].*\]: /[\2] \1: /' \
  | sort -u | head -30 || true
