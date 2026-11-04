"""Load independent safety YAML profiles (do not mutate kinematics YAML)."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

import yaml

from .config import SafetyConfig

_PROFILES_DIR = Path(__file__).resolve().parent / "profiles"


@dataclass(frozen=True)
class LinkSphereSpec:
    name: str
    link_index: int
    radius_m: float = 0.05
    offset_m: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class CapsuleSpec:
    name: str
    from_link_index: int
    to_link_index: int
    radius_m: float = 0.05
    samples: int = 3


@dataclass(frozen=True)
class MeshSpec:
    """Local-frame STL attached to a FK link index (Phase C)."""

    name: str
    link_index: int
    stl: str
    max_points: int = 28
    prefer_collision: bool = True


@dataclass(frozen=True)
class SafetyProfile:
    id: str
    kinematics_profile_id: str
    safety: SafetyConfig
    links: tuple[LinkSphereSpec, ...] = field(default_factory=tuple)
    capsules: tuple[CapsuleSpec, ...] = field(default_factory=tuple)
    meshes: tuple[MeshSpec, ...] = field(default_factory=tuple)
    mesh_root: str | None = None
    raw: Mapping[str, Any] = field(default_factory=dict)


def profiles_dir() -> Path:
    return _PROFILES_DIR


def list_safety_profile_ids() -> list[str]:
    if not _PROFILES_DIR.is_dir():
        return []
    return sorted(p.stem for p in _PROFILES_DIR.glob("*.yaml"))


def _offset(v: Any) -> tuple[float, float, float]:
    if isinstance(v, (list, tuple)) and len(v) >= 3:
        return (float(v[0]), float(v[1]), float(v[2]))
    return (0.0, 0.0, 0.0)


def load_safety_profile(
    profile_id: str | Path | None = None,
    *,
    path: str | Path | None = None,
) -> SafetyProfile:
    """Load by id (under ``traj_safety/profiles``) or explicit YAML path."""
    if path is not None:
        p = Path(path).expanduser().resolve()
    else:
        pid = str(profile_id or "elite_cs66").strip()
        p = _PROFILES_DIR / f"{pid}.yaml"
    if not p.is_file():
        known = list_safety_profile_ids()
        raise FileNotFoundError(f"safety profile not found: {p} (known: {known})")
    data = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise ValueError(f"safety profile must be a mapping: {p}")

    sid = str(data.get("id") or p.stem).strip()
    kin_id = str(data.get("kinematics_profile_id") or sid).strip()
    safety = SafetyConfig.from_mapping(
        {
            "method": data.get("method", "sphere"),
            "table_z": data.get("table_z", 0.0),
            "table_clearance_m": data.get("table_clearance_m", 0.008),
            "self_min_dist_m": data.get("self_min_dist_m", 0.02),
            "skip_adjacent": data.get("skip_adjacent", 1),
            "near_miss_margin_m": data.get("near_miss_margin_m", 0.015),
            "max_events": data.get("max_events", 40),
            "up_axis": data.get("up_axis", "z"),
        }
    )

    links: list[LinkSphereSpec] = []
    for row in data.get("links") or []:
        if not isinstance(row, dict):
            continue
        links.append(
            LinkSphereSpec(
                name=str(row.get("name") or f"link{row.get('link_index')}"),
                link_index=int(row.get("link_index")),
                radius_m=float(row.get("radius_m", 0.05)),
                offset_m=_offset(row.get("offset_m")),
            )
        )

    capsules: list[CapsuleSpec] = []
    for row in data.get("capsules") or []:
        if not isinstance(row, dict):
            continue
        capsules.append(
            CapsuleSpec(
                name=str(row.get("name") or "capsule"),
                from_link_index=int(row.get("from_link_index")),
                to_link_index=int(row.get("to_link_index")),
                radius_m=float(row.get("radius_m", 0.05)),
                samples=max(1, int(row.get("samples", 3))),
            )
        )

    meshes: list[MeshSpec] = []
    for row in data.get("meshes") or []:
        if not isinstance(row, dict):
            continue
        stl = str(row.get("stl") or row.get("path") or "").strip()
        if not stl:
            continue
        meshes.append(
            MeshSpec(
                name=str(row.get("name") or Path(stl).stem),
                link_index=int(row.get("link_index")),
                stl=stl,
                max_points=max(4, min(64, int(row.get("max_points", 28)))),
                prefer_collision=bool(row.get("prefer_collision", True)),
            )
        )

    if not links and not capsules and not meshes:
        links = [
            LinkSphereSpec(name=f"L{i}", link_index=i, radius_m=0.05) for i in range(7)
        ]

    mesh_root = data.get("mesh_root")
    mesh_root_s = str(mesh_root).strip() if mesh_root else None

    # If meshes are configured, prefer hull analysis unless explicitly set.
    if meshes and "method" not in data:
        safety = SafetyConfig.from_mapping({**safety.to_dict(), "method": "hull"})

    return SafetyProfile(
        id=sid,
        kinematics_profile_id=kin_id,
        safety=safety,
        links=tuple(links),
        capsules=tuple(capsules),
        meshes=tuple(meshes),
        mesh_root=mesh_root_s,
        raw=data,
    )
