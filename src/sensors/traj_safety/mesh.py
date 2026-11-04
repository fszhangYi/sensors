"""Minimal STL vertex sampling + pose transform (Phase C, no trimesh required)."""

from __future__ import annotations

import struct
from pathlib import Path
from typing import Sequence

import numpy as np


def load_stl_vertices(path: str | Path) -> np.ndarray:
    """Load unique-ish vertices from binary or ASCII STL → (N,3) float64 meters."""
    p = Path(path)
    data = p.read_bytes()
    if len(data) < 84:
        raise ValueError(f"STL too small: {p}")
    # Heuristic: binary STL has an 80-byte header + uint32 tri count.
    n_tri = struct.unpack_from("<I", data, 80)[0]
    expect = 84 + n_tri * 50
    if expect == len(data) and n_tri > 0:
        verts = []
        off = 84
        for _ in range(n_tri):
            # normal(3f) + 3*vertex(3f) + attr(u16)
            v1 = struct.unpack_from("<3f", data, off + 12)
            v2 = struct.unpack_from("<3f", data, off + 24)
            v3 = struct.unpack_from("<3f", data, off + 36)
            verts.extend((v1, v2, v3))
            off += 50
        arr = np.asarray(verts, dtype=float)
    else:
        # ASCII
        text = data.decode("utf-8", errors="ignore")
        verts = []
        for line in text.splitlines():
            line = line.strip()
            if line.lower().startswith("vertex"):
                parts = line.split()
                if len(parts) >= 4:
                    verts.append((float(parts[1]), float(parts[2]), float(parts[3])))
        if not verts:
            raise ValueError(f"no vertices in STL: {p}")
        arr = np.asarray(verts, dtype=float)
    # Dedup roughly
    if len(arr) > 1:
        arr = np.unique(np.round(arr, 6), axis=0)
    return arr


def sample_points(
    vertices: np.ndarray,
    *,
    max_points: int = 28,
) -> list[tuple[float, float, float]]:
    """Stride-sample vertices (embody default ~28 pts/link)."""
    v = np.asarray(vertices, dtype=float).reshape(-1, 3)
    if len(v) == 0:
        return []
    max_points = max(4, min(64, int(max_points)))
    if len(v) <= max_points:
        return [(float(x), float(y), float(z)) for x, y, z in v]
    idx = np.linspace(0, len(v) - 1, num=max_points, dtype=int)
    out = []
    for i in idx:
        x, y, z = v[int(i)]
        out.append((float(x), float(y), float(z)))
    return out


def transform_points(
    points: Sequence[tuple[float, float, float]],
    T: np.ndarray,
) -> list[tuple[float, float, float]]:
    """Apply 4x4 homogeneous transform to points."""
    R = np.asarray(T, dtype=float)
    out = []
    for x, y, z in points:
        p = R @ np.array([x, y, z, 1.0], dtype=float)
        out.append((float(p[0]), float(p[1]), float(p[2])))
    return out


def cloud_radius(points: Sequence[tuple[float, float, float]]) -> float:
    if not points:
        return 0.015
    arr = np.asarray(points, dtype=float)
    c = arr.mean(axis=0)
    d = np.linalg.norm(arr - c, axis=1)
    r = float(d.max()) if len(d) else 0.015
    return max(0.015, r)


def load_and_sample_stl(
    path: str | Path,
    *,
    max_points: int = 28,
) -> tuple[list[tuple[float, float, float]], float]:
    verts = load_stl_vertices(path)
    pts = sample_points(verts, max_points=max_points)
    return pts, cloud_radius(pts)
