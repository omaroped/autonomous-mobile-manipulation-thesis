#!/usr/bin/env python3
"""Build simplified and visual base meshes scaled to meters.
Reads limo_base.stl (~750k tris, in mm),
1. Scales the visual mesh by 0.0012 (mm -> meters * 1.2) and writes limo_base_meters.stl.
2. Computes the convex hull, scales it by 0.0012, and writes limo_base_collision_meters.stl.
"""
import struct
import numpy as np
from scipy.spatial import ConvexHull

SRC = 'limo_base.stl'
OUT_VIS = 'limo_base_meters.stl'
OUT_COL = 'limo_base_collision_meters.stl'

# Read binary STL
data = open(SRC, 'rb').read()
n = struct.unpack('<I', data[80:84])[0]
dt = np.dtype([('normal', '<f4', 3), ('v1', '<f4', 3),
               ('v2', '<f4', 3), ('v3', '<f4', 3), ('attr', '<u2')])
tri = np.frombuffer(data[84:84 + n * 50], dtype=dt).copy()

# 1. Scale and write visual mesh in meters (scale factor 0.0012)
tri['v1'] *= 0.0012
tri['v2'] *= 0.0012
tri['v3'] *= 0.0012

with open(OUT_VIS, 'wb') as f:
    f.write(b'\0' * 80)
    f.write(struct.pack('<I', n))
    f.write(tri.tobytes())
print(f'Wrote scaled visual mesh: {OUT_VIS} ({n} tris)')

# 2. Compute convex hull on unscaled vertices, then scale hull to meters
verts = np.vstack([tri['v1'], tri['v2'], tri['v3']]).astype(np.float64)
# Note: verts are already scaled to meters here because we got them from tri
verts = np.unique(verts, axis=0)

hull = ConvexHull(verts)
pts = hull.points
faces = hull.simplices

# Write binary STL of the hull (already in meters)
with open(OUT_COL, 'wb') as f:
    f.write(b'\0' * 80)
    f.write(struct.pack('<I', len(faces)))
    for s in faces:
        a, b, c = pts[s[0]], pts[s[1]], pts[s[2]]
        nrm = np.cross(b - a, c - a)
        ln = np.linalg.norm(nrm)
        nrm = nrm / ln if ln else nrm
        f.write(struct.pack('<3f', *nrm))
        for v in (a, b, c):
            f.write(struct.pack('<3f', *v))
        f.write(struct.pack('<H', 0))
print(f'Wrote scaled collision mesh: {OUT_COL} ({len(faces)} faces)')

# Print scaled bounding box
mn, mx = verts.min(0), verts.max(0)
print('Scaled mesh bbox min=%s max=%s' % (np.round(mn, 4), np.round(mx, 4)))
print('=> in base_link, chassis z spans [%.3f, %.3f] (arm mounts ~+0.025)'
      % (mn[2] - 0.15, mx[2] - 0.15))
