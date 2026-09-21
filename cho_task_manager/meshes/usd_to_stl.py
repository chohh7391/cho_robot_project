"""Flatten a lab-asset USD's VISUAL meshes into one binary STL for rviz.

Two things about these particular assets, both measured rather than assumed:

* the collision prims are a convex decomposition -- 60-odd unit cubes -- and
  drawing them would show a pile of boxes, so only ``/visual/`` is taken;
* ``metersPerUnit`` is 0.01 but the xform scale already converts to METRES.
  Applying both gives a 0.65 mm beaker; applying only the xform gives 65 mm,
  and the flask agrees (139 mm across, 132 mm tall). Two assets, both sane
  only one way, so metersPerUnit is deliberately NOT applied.

The result is normalised: centred on its bounding box and scaled to a unit box
on every axis, so the marker's ``scale`` is the object's size in metres exactly
as it is for a CUBE or a CYLINDER. One meaning of ``size`` for every shape.
"""
import struct
import sys

import numpy as np
from pxr import Usd, UsdGeom


def visual_triangles(path):
    """Return every triangle of the stage's visual meshes, in metres."""
    stage = Usd.Stage.Open(path)
    cache = UsdGeom.XformCache()
    tris = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh) or '/visual' not in str(prim.GetPath()):
            continue
        mesh = UsdGeom.Mesh(prim)
        pts = np.asarray(mesh.GetPointsAttr().Get() or [], dtype=float)
        if pts.size == 0:
            continue
        counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get() or [], dtype=int)
        idx = np.asarray(mesh.GetFaceVertexIndicesAttr().Get() or [], dtype=int)
        m = cache.GetLocalToWorldTransform(prim)
        M = np.array([[m[r][c] for c in range(4)] for r in range(4)], dtype=float)
        world = (np.hstack([pts, np.ones((len(pts), 1))]) @ M)[:, :3]
        cursor = 0
        for n in counts:
            face = idx[cursor:cursor + n]
            cursor += n
            for k in range(1, n - 1):
                tris.append(world[[face[0], face[k], face[k + 1]]])
    return np.asarray(tris)


def write_stl(tris, out):
    """Write the triangles to *out* as a binary STL."""
    with open(out, 'wb') as handle:
        handle.write(b'\0' * 80)
        handle.write(struct.pack('<I', len(tris)))
        for tri in tris:
            n = np.cross(tri[1] - tri[0], tri[2] - tri[0])
            length = np.linalg.norm(n)
            handle.write(struct.pack('<3f', *(n / length if length else np.zeros(3))))
            for vertex in tri:
                handle.write(struct.pack('<3f', *vertex))
            handle.write(b'\0\0')


src, dst = sys.argv[1], sys.argv[2]
tris = visual_triangles(src)
flat = tris.reshape(-1, 3)
natural = flat.max(0) - flat.min(0)
tris = (tris - (flat.max(0) + flat.min(0)) / 2.0) / natural
write_stl(tris, dst)
print(f'{src.rsplit("/", 1)[-1]:12s} -> {dst.rsplit("/", 1)[-1]:12s} '
      f'{len(tris):5d} triangles   as authored {np.round(natural * 1000, 1)} mm')
