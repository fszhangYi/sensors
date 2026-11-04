"""Safety thresholds aligned with embody robots.json collision blocks."""

from __future__ import annotations

from dataclasses import asdict, dataclass, fields
from typing import Any, Mapping


@dataclass(frozen=True)
class SafetyConfig:
    method: str = "sphere"  # sphere | hull
    table_z: float = 0.0
    table_clearance_m: float = 0.008
    self_min_dist_m: float = 0.02
    skip_adjacent: int = 1
    near_miss_margin_m: float = 0.015
    max_events: int = 40
    up_axis: str = "z"  # desk plane normal axis
    # hull: pad = 0.15 * 0.25 * (r_a + r_b)  (match embody analyzeHullCollisions)
    hull_radius_pad_scale: float = 0.15
    # sphere: if True and entries carry radius, use surface clearance (dist - ra - rb)
    use_radii: bool = True

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_mapping(cls, raw: Mapping[str, Any] | None) -> "SafetyConfig":
        if not raw:
            return cls()
        known = {f.name for f in fields(cls)}
        kwargs: dict[str, Any] = {}
        for k, v in raw.items():
            key = str(k)
            if key not in known:
                continue
            if key in {
                "table_z",
                "table_clearance_m",
                "self_min_dist_m",
                "near_miss_margin_m",
                "hull_radius_pad_scale",
            }:
                kwargs[key] = float(v)
            elif key in {"skip_adjacent", "max_events"}:
                kwargs[key] = int(v)
            elif key in {"method", "up_axis"}:
                kwargs[key] = str(v)
            elif key == "use_radii":
                kwargs[key] = bool(v)
            else:
                kwargs[key] = v
        method = str(kwargs.get("method", "sphere")).strip().lower()
        if method not in {"sphere", "hull"}:
            method = "sphere"
        kwargs["method"] = method
        up = str(kwargs.get("up_axis", "z")).strip().lower()
        if up not in {"x", "y", "z"}:
            up = "z"
        kwargs["up_axis"] = up
        kwargs["skip_adjacent"] = max(0, int(kwargs.get("skip_adjacent", 1)))
        kwargs["max_events"] = max(1, int(kwargs.get("max_events", 40)))
        return cls(**kwargs)
