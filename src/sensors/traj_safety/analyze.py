"""Top-level analyze + grade helpers."""

from __future__ import annotations

from typing import Any, Mapping, Sequence

from .config import SafetyConfig
from .hull import analyze_hull_collisions
from .sphere import analyze_sphere_collisions
from .types import SCHEMA_RESULT


def grade_from_counts(*, n_table: int, n_self: int, n_near_miss: int) -> str:
    """good | warn | bad — thick M2 geometric grade."""
    if int(n_table) > 0 or int(n_self) > 0:
        return "bad"
    if int(n_near_miss) > 0:
        return "warn"
    return "good"


def analyze_traj_safety(
    frames: Sequence[Any],
    *,
    config: SafetyConfig | Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Dispatch sphere/hull analysis and attach schema + grade.

    For ``method=sphere``, each frame is link centers.
    For ``method=hull``, each frame is link point clouds.
    """
    cfg = (
        config
        if isinstance(config, SafetyConfig)
        else SafetyConfig.from_mapping(config)  # type: ignore[arg-type]
    )
    if cfg.method == "hull":
        raw = analyze_hull_collisions(frames, cfg)
    else:
        raw = analyze_sphere_collisions(frames, cfg)
    grade = grade_from_counts(
        n_table=int(raw.get("n_table") or 0),
        n_self=int(raw.get("n_self") or 0),
        n_near_miss=int(raw.get("n_near_miss") or 0),
    )
    return {
        "schema": SCHEMA_RESULT,
        "ok": True,
        "grade": grade,
        "config": cfg.to_dict(),
        **raw,
    }
