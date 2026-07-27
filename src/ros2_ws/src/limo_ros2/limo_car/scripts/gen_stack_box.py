#!/usr/bin/python3
"""
gen_stack_box.py — procedural mesh generator for a self-centering stack box.

Body: BODY x BODY x BODY_H rectangular box (matches the original target_box footprint,
      35x35 mm, so it still fits the gripper's 31.8 mm grasp gap / 43.4 mm open gap).
Top:    a convex dome bump of radius DOME_R, height DOME_H, centred on the top face.
Bottom: a concave dish cavity of radius CAV_R (> DOME_R), depth CAV_H (> DOME_H),
        centred on the bottom face. The cavity is deliberately larger than the dome
        below it (tolerance band) so a box seats onto the one below it even with a
        few mm of placement error — self-centering "ball-and-socket" stacking.

Run once (no ROS needed):
    python3 gen_stack_box.py

Writes:
    ../meshes/stack_box.stl   binary STL, metres, centred at the origin.
"""

import os
import struct

import numpy as np

# ── Geometry parameters (all in metres) ──────────────────────────────────────
BODY   = 0.035   # box footprint (square) — matches gripper grasp geometry
BODY_H = 0.040   # full box height
DOME_R = 0.010   # dome base radius  (top,    convex)
DOME_H = 0.005   # dome cap height
CAV_R  = 0.011   # cavity radius     (bottom, concave) = DOME_R + 1 mm tolerance
CAV_H  = 0.006   # cavity depth                        = DOME_H + 1 mm clearance

GRID_N = 24      # grid resolution for the domed/cavity top & bottom faces

OUT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', 'meshes', 'stack_box.stl')


def _dome_offset(x, y, radius, height):
    """Spherical-cap-like bump: height*sqrt(1-(r/radius)^2) inside r<radius, else 0."""
    r = np.sqrt(x ** 2 + y ** 2)
    inside = r < radius
    z = np.zeros_like(r)
    z[inside] = height * np.sqrt(1.0 - (r[inside] / radius) ** 2)
    return z


def _make_face_grid(z_base, radius, height, n=GRID_N):
    """Grid of (x, y, z) points over the BODY x BODY footprint, z = z_base + bump."""
    half = BODY / 2.0
    xs = np.linspace(-half, half, n + 1)
    ys = np.linspace(-half, half, n + 1)
    X, Y = np.meshgrid(xs, ys, indexing='ij')
    Z = z_base + _dome_offset(X, Y, radius, height)
    return X, Y, Z


def _grid_to_triangles(X, Y, Z, flip=False):
    """Triangulate a regular (n+1)x(n+1) grid, CCW winding (flip reverses it)."""
    n = X.shape[0] - 1
    tris = []
    for i in range(n):
        for j in range(n):
            p00 = np.array([X[i, j],       Y[i, j],       Z[i, j]])
            p10 = np.array([X[i + 1, j],   Y[i + 1, j],   Z[i + 1, j]])
            p01 = np.array([X[i, j + 1],   Y[i, j + 1],   Z[i, j + 1]])
            p11 = np.array([X[i + 1, j + 1], Y[i + 1, j + 1], Z[i + 1, j + 1]])
            if not flip:
                tris.append((p00, p10, p11))
                tris.append((p00, p11, p01))
            else:
                tris.append((p00, p11, p10))
                tris.append((p00, p01, p11))
    return tris


def _boundary_points(X, Y, Z):
    """Ordered perimeter points of an (n+1)x(n+1) grid, walking CCW (viewed from
    +z) starting at (i=0, j=0). Each corner appears once. Shape (4n, 3)."""
    n = X.shape[0] - 1
    pts = []
    for i in range(n):                # south edge: j=0,  i=0..n-1
        pts.append([X[i, 0], Y[i, 0], Z[i, 0]])
    for j in range(n):                # east edge:  i=n,  j=0..n-1
        pts.append([X[n, j], Y[n, j], Z[n, j]])
    for i in range(n, 0, -1):         # north edge: j=n,  i=n..1
        pts.append([X[i, n], Y[i, n], Z[i, n]])
    for j in range(n, 0, -1):         # west edge:  i=0,  j=n..1
        pts.append([X[0, j], Y[0, j], Z[0, j]])
    return np.array(pts)


def _side_wall_triangles(Xt, Yt, Zt, Xb, Yb, Zb):
    """Vertical walls connecting the (flat) grid perimeter of the top face to the
    matching perimeter of the bottom face — one quad per grid boundary segment, so
    every wall edge exactly matches a top/bottom grid boundary edge (watertight).
    A single flat quad per side would NOT match the grid's finer subdivisions and
    leaves the mesh non-watertight along all four sides."""
    top_pts = _boundary_points(Xt, Yt, Zt)
    bot_pts = _boundary_points(Xb, Yb, Zb)
    m = len(top_pts)
    tris = []
    for k in range(m):
        k2 = (k + 1) % m
        top0, top1 = top_pts[k], top_pts[k2]
        bot0, bot1 = bot_pts[k], bot_pts[k2]
        tris.append((bot0, bot1, top1))
        tris.append((bot0, top1, top0))
    return tris


def _write_stl(triangles, path):
    with open(path, 'wb') as f:
        f.write(b'\0' * 80)
        f.write(struct.pack('<I', len(triangles)))
        for a, b, c in triangles:
            nrm = np.cross(b - a, c - a)
            ln = np.linalg.norm(nrm)
            nrm = nrm / ln if ln > 1e-12 else np.array([0.0, 0.0, 1.0])
            f.write(struct.pack('<3f', float(nrm[0]), float(nrm[1]), float(nrm[2])))
            for v in (a, b, c):
                f.write(struct.pack('<3f', float(v[0]), float(v[1]), float(v[2])))
            f.write(struct.pack('<H', 0))


def main():
    half_h = BODY_H / 2.0

    # Top face: convex dome bump — outward normal +z (up)
    Xt, Yt, Zt = _make_face_grid(half_h, DOME_R, DOME_H)
    top_tris = _grid_to_triangles(Xt, Yt, Zt, flip=False)

    # Bottom face: concave cavity dish, carved UP into the box from z=-half_h —
    # outward normal -z (down)
    Xb, Yb, Zb = _make_face_grid(-half_h, CAV_R, CAV_H)
    bot_tris = _grid_to_triangles(Xb, Yb, Zb, flip=True)

    tris = top_tris + bot_tris + _side_wall_triangles(Xt, Yt, Zt, Xb, Yb, Zb)

    os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
    _write_stl(tris, OUT_PATH)

    print(f'Wrote {OUT_PATH}: {len(tris)} triangles')
    print(f'Body {BODY * 1000:.1f}x{BODY * 1000:.1f}x{BODY_H * 1000:.1f} mm  '
          f'dome R={DOME_R * 1000:.1f} H={DOME_H * 1000:.1f} mm  '
          f'cavity R={CAV_R * 1000:.1f} H={CAV_H * 1000:.1f} mm  '
          f'(tolerance: {(CAV_R - DOME_R) * 1000:.1f} mm radial, '
          f'{(CAV_H - DOME_H) * 1000:.1f} mm depth)')


if __name__ == '__main__':
    main()
