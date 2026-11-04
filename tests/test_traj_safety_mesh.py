"""Phase C: STL sampling helpers."""

from __future__ import annotations

import struct
from pathlib import Path

import numpy as np

from sensors.traj_safety.mesh import load_stl_vertices, sample_points, transform_points


def _write_binary_stl(path: Path, triangles: list[tuple]) -> None:
    # triangles: list of 3 xyz triples
    buf = bytearray(80)  # header
    buf += struct.pack("<I", len(triangles))
    for tri in triangles:
        buf += struct.pack("<3f", 0.0, 0.0, 1.0)  # normal
        for v in tri:
            buf += struct.pack("<3f", float(v[0]), float(v[1]), float(v[2]))
        buf += struct.pack("<H", 0)
    path.write_bytes(bytes(buf))


def test_load_binary_stl_and_sample(tmp_path: Path) -> None:
    stl = tmp_path / "box.stl"
    _write_binary_stl(
        stl,
        [
            ((0, 0, 0), (1, 0, 0), (0, 1, 0)),
            ((1, 0, 0), (1, 1, 0), (0, 1, 0)),
        ],
    )
    verts = load_stl_vertices(stl)
    assert len(verts) >= 3
    pts = sample_points(verts, max_points=8)
    assert 3 <= len(pts) <= 8


def test_transform_points() -> None:
    T = np.eye(4)
    T[0, 3] = 1.0
    out = transform_points([(0.0, 0.0, 0.0)], T)
    assert abs(out[0][0] - 1.0) < 1e-9
