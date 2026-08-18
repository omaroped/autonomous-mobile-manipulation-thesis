#!/usr/bin/env python3
"""Reachability lookup — a fast oracle over docs/references/data/reach_map.csv.

NOT a trajectory cache: the sweep only recorded (x, y, z, orientation) -> whether a
MoveIt plan to that pose succeeded, not the resulting joint solution. So this cannot
skip planning outright. What it CAN do: tell grasp_and_retract() which hover clearance
is known-reachable near a given (x, y) *before* invoking OMPL, so the existing
try-each-clearance fallback attempts the likely-good one first instead of a fixed
blind order, and report how far a target sits from the characterized envelope instead
of only finding out after 3 live planner failures.
"""
import csv
import math

REACH_MAP_PATH = '/home/omar/Desktop/Thesisorg/docs/references/data/reach_map.csv'

# Sweep grid spacing is ~0.02 m (x/z) and ~0.04 m (y), so a genuine in-grid neighbour
# is never farther than this. Anything farther means "outside the characterized
# envelope" -- trusting a distant sample's success flag there is worse than having no
# opinion at all (verified: the 2026-08-14 phantom-blob target's nearest sample was
# 0.211 m away and happened to read success=True, which would have been actively
# misleading -- that target was never in the swept region to begin with).
MAX_TRUST_DIST = 0.05

_samples = None  # lazy-loaded: list of (x, y, z, success)


def _load():
    global _samples
    if _samples is not None:
        return _samples
    _samples = []
    try:
        with open(REACH_MAP_PATH) as f:
            for row in csv.DictReader(f):
                _samples.append((
                    float(row['x']), float(row['y']), float(row['z']),
                    bool(int(row['success'])),
                ))
    except FileNotFoundError:
        _samples = []
    return _samples


def nearest(x, y, z):
    """Closest sweep sample to (x, y, z), or None if the map is unavailable.

    Returns (dist_m, success). Brute-force over ~2300 rows -- fast enough to call
    synchronously; not worth a KD-tree at this size.
    """
    samples = _load()
    if not samples:
        return None
    bx, by, bz, bsucc = min(
        samples, key=lambda s: (s[0] - x) ** 2 + (s[1] - y) ** 2 + (s[2] - z) ** 2)
    dist = math.sqrt((bx - x) ** 2 + (by - y) ** 2 + (bz - z) ** 2)
    return dist, bsucc


def rank_clearances(tx, ty, tz, clearances):
    """Order candidate hover clearances by predicted likelihood of success.

    For each candidate z = tz + clearance, look up the nearest characterized sample.
    Known-success-and-close sorts first; unknown/known-failure/too-far sorts last.
    Ties break on the original clearance order, so behaviour is unchanged when the
    map has nothing trustworthy to say about a given point.
    """
    scored = []
    for i, c in enumerate(clearances):
        hit = nearest(tx, ty, tz + c)
        if hit is None or hit[0] > MAX_TRUST_DIST:
            scored.append((1, 1.0, i, c))           # no map data, or too far to trust
        else:
            dist, success = hit
            scored.append((0 if success else 1, dist, i, c))
    scored.sort()
    return [c for *_, c in scored]
