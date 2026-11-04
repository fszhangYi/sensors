"""Build sphere/hull proxies from FK link poses + safety profile."""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from .profile import CapsuleSpec, LinkSphereSpec, MeshSpec, SafetyProfile


def _xform_point(T: np.ndarray, offset: Sequence[float]) -> tuple[float, float, float]:
    p = np.asarray(T, dtype=float) @ np.array(
        [float(offset[0]), float(offset[1]), float(offset[2]), 1.0], dtype=float
    )
    return (float(p[0]), float(p[1]), float(p[2]))


def spheres_from_link_poses(
    poses: Sequence[np.ndarray],
    *,
    links: Sequence[LinkSphereSpec],
    capsules: Sequence[CapsuleSpec] = (),
) -> list[dict[str, Any]]:
    """Map one pose chain → sphere entries ``{name, pos, radius}`` for analysis.

    ``poses`` length should be 7 (base + J1..J6). Capsule samples are midpoints
    along the segment between two link origins (not full GJK).
    """
    n = len(poses)
    out: list[dict[str, Any]] = []
    for spec in links:
        idx = int(spec.link_index)
        if idx < 0 or idx >= n:
            continue
        pos = _xform_point(poses[idx], spec.offset_m)
        out.append({"name": spec.name, "pos": pos, "radius": float(spec.radius_m)})

    for cap in capsules:
        a, b = int(cap.from_link_index), int(cap.to_link_index)
        if a < 0 or b < 0 or a >= n or b >= n:
            continue
        pa = np.asarray(poses[a], dtype=float)[:3, 3]
        pb = np.asarray(poses[b], dtype=float)[:3, 3]
        samples = max(1, int(cap.samples))
        for k in range(samples):
            t = (k + 1) / (samples + 1)
            pos = (1.0 - t) * pa + t * pb
            out.append(
                {
                    "name": f"{cap.name}@{k}",
                    "pos": (float(pos[0]), float(pos[1]), float(pos[2])),
                    "radius": float(cap.radius_m),
                }
            )
    return out


@lru_cache(maxsize=32)
def _cached_stl_sample(stl_resolved: str, max_points: int) -> tuple[tuple[tuple[float, float, float], ...], float]:
    from .mesh import load_and_sample_stl

    pts, radius = load_and_sample_stl(stl_resolved, max_points=max_points)
    return tuple(pts), float(radius)


def _resolve_stl(spec: MeshSpec, mesh_root: str | None) -> Path | None:
    raw = Path(spec.stl)
    if raw.is_file():
        return raw.resolve()
    if mesh_root:
        cand = Path(mesh_root).expanduser() / spec.stl
        if cand.is_file():
            return cand.resolve()
    # Env override for station machines that keep sensors-arm checkout nearby
    import os

    env_root = (os.environ.get("SENSORS_TRAJ_SAFETY_MESH_ROOT") or "").strip()
    if env_root:
        cand = Path(env_root).expanduser() / spec.stl
        if cand.is_file():
            return cand.resolve()
    return None


def clouds_from_link_poses(
    poses: Sequence[np.ndarray],
    *,
    meshes: Sequence[MeshSpec],
    mesh_root: str | None = None,
) -> list[dict[str, Any]]:
    """STL samples in link frames → world clouds for hull analysis."""
    from .mesh import transform_points

    n = len(poses)
    out: list[dict[str, Any]] = []
    for spec in meshes:
        idx = int(spec.link_index)
        if idx < 0 or idx >= n:
            continue
        stl = _resolve_stl(spec, mesh_root)
        if stl is None:
            continue
        local_pts, radius = _cached_stl_sample(str(stl), int(spec.max_points))
        world = transform_points(local_pts, poses[idx])
        out.append(
            {
                "name": spec.name,
                "points": world,
                "radius": radius,
            }
        )
    return out


def frames_from_joint_series(
    joints_series: Sequence[Sequence[float]],
    profile: SafetyProfile,
    *,
    stride: int = 1,
) -> list[list[dict[str, Any]]]:
    """joints (rad) → per-frame sphere lists or hull clouds."""
    from sensors.cs66_kinematics import fk_link_poses_from_joints_rad

    stride = max(1, int(stride))
    frames: list[list[dict[str, Any]]] = []
    kin_id = profile.kinematics_profile_id
    use_mesh = bool(profile.meshes) and str(profile.safety.method).lower() == "hull"
    for i, q in enumerate(joints_series):
        if i % stride != 0:
            continue
        poses = fk_link_poses_from_joints_rad(q, profile_id=kin_id)
        if use_mesh:
            clouds = clouds_from_link_poses(
                poses, meshes=profile.meshes, mesh_root=profile.mesh_root
            )
            if clouds:
                frames.append(clouds)
                continue
        frames.append(
            spheres_from_link_poses(
                poses, links=profile.links, capsules=profile.capsules
            )
        )
    return frames
