"""High-level: joint series + safety profile → graded geometric result."""

from __future__ import annotations

from typing import Any, Mapping, Sequence

from .analyze import analyze_traj_safety
from .bodies import frames_from_joint_series
from .profile import SafetyProfile, load_safety_profile
from .types import SCHEMA_RESULT


def analyze_joint_trajectory(
    joints_series: Sequence[Sequence[float]],
    *,
    profile: SafetyProfile | str | None = "elite_cs66",
    safety_profile_path: str | None = None,
    stride: int = 1,
) -> dict[str, Any]:
    """Run thick geometric safety on a joint trajectory (radians).

    ``profile`` may be a ``SafetyProfile``, profile id, or None (default elite_cs66).
    """
    if isinstance(profile, SafetyProfile):
        prof = profile
    else:
        prof = load_safety_profile(profile, path=safety_profile_path)

    if not joints_series:
        return {
            "schema": SCHEMA_RESULT,
            "ok": False,
            "grade": "missing",
            "error": "empty joint series",
            "n_table": 0,
            "n_self": 0,
            "n_near_miss": 0,
            "table_hits": [],
            "self_hits": [],
            "near_miss": [],
            "method": prof.safety.method,
            "profile_id": prof.id,
            "kinematics_profile_id": prof.kinematics_profile_id,
            "n_frames": 0,
        }

    frames = frames_from_joint_series(joints_series, prof, stride=stride)
    cfg = prof.safety
    # Auto-select analyzer from frame payload (mesh → hull clouds; else spheres).
    sample0 = (frames[0][0] if frames and frames[0] else None) or {}
    if isinstance(sample0, dict) and sample0.get("points") is not None:
        if cfg.method != "hull":
            from .config import SafetyConfig

            cfg = SafetyConfig.from_mapping({**cfg.to_dict(), "method": "hull"})
    elif cfg.method == "hull":
        from .config import SafetyConfig

        cfg = SafetyConfig.from_mapping({**cfg.to_dict(), "method": "sphere"})
    result = analyze_traj_safety(frames, config=cfg)
    result["profile_id"] = prof.id
    result["kinematics_profile_id"] = prof.kinematics_profile_id
    result["n_frames"] = len(frames)
    result["stride"] = max(1, int(stride))
    result["n_joint_samples"] = len(joints_series)
    result["backend"] = "mesh_hull" if cfg.method == "hull" else "sphere_capsule"
    return result


def analyze_joint_trajectory_dict(
    payload: Mapping[str, Any],
) -> dict[str, Any]:
    """Convenience for JSON-shaped input (embody / CLI)."""
    series = payload.get("joints_rad") or payload.get("q_series") or payload.get("frames")
    joints: list[list[float]] = []
    if isinstance(series, list):
        for row in series:
            if isinstance(row, dict) and "joints" in row:
                q = row.get("joints")
            elif isinstance(row, dict) and "q" in row:
                q = row.get("q")
            else:
                q = row
            if isinstance(q, (list, tuple)) and q:
                joints.append([float(x) for x in q[:6]])
    return analyze_joint_trajectory(
        joints,
        profile=payload.get("profile_id") or "elite_cs66",
        safety_profile_path=payload.get("safety_profile_path"),
        stride=int(payload.get("stride") or 1),
    )
