"""Point-cloud table / self collision — port of embody analyzeHullCollisions."""

from __future__ import annotations

from typing import Any, Mapping, Sequence

from .config import SafetyConfig
from .types import as_xyz, axis_value, dist3


def _cap(events: list[dict[str, Any]], n: int) -> list[dict[str, Any]]:
    return events[: max(1, int(n))]


def _min_dist_clouds(
    a: Sequence[tuple[float, float, float]],
    b: Sequence[tuple[float, float, float]],
) -> float:
    best = float("inf")
    for pa in a:
        for pb in b:
            d = dist3(pa, pb)
            if d < best:
                best = d
    return best if best < float("inf") else 0.0


def _parse_cloud(item: Mapping[str, Any]) -> tuple[str, list[tuple[float, float, float]], float] | None:
    name = str(item.get("name") or item.get("link") or "").strip() or "?"
    pts_raw = item.get("points") or []
    points: list[tuple[float, float, float]] = []
    if isinstance(pts_raw, Sequence):
        for p in pts_raw:
            xyz = as_xyz(p)
            if xyz is not None:
                points.append(xyz)
    if not points:
        return None
    try:
        radius = float(item.get("radius") if item.get("radius") is not None else 0.015)
    except (TypeError, ValueError):
        radius = 0.015
    radius = max(0.0, radius)
    return name, points, radius


def analyze_hull_collisions(
    hull_series: Sequence[Sequence[Mapping[str, Any]] | Mapping[str, Any]],
    config: SafetyConfig | Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Analyze per-frame link point clouds.

    ``hull_series[i]`` is a list of ``{name, points, radius}`` or
    ``{"clouds": [...]}``.
    """
    cfg = (
        config
        if isinstance(config, SafetyConfig)
        else SafetyConfig.from_mapping(config)  # type: ignore[arg-type]
    )
    table_hits: list[dict[str, Any]] = []
    self_hits: list[dict[str, Any]] = []
    near_miss: list[dict[str, Any]] = []
    table_band = float(cfg.table_z) + float(cfg.table_clearance_m)
    skip = int(cfg.skip_adjacent)
    pad_scale = float(cfg.hull_radius_pad_scale)

    for i, frame in enumerate(hull_series):
        clouds_raw: Sequence[Mapping[str, Any]]
        if isinstance(frame, Mapping) and "clouds" in frame:
            clouds_raw = frame.get("clouds") or []  # type: ignore[assignment]
        else:
            clouds_raw = frame  # type: ignore[assignment]
        clouds: list[tuple[str, list[tuple[float, float, float]], float]] = []
        for item in clouds_raw:
            if not isinstance(item, Mapping):
                continue
            parsed = _parse_cloud(item)
            if parsed is not None:
                clouds.append(parsed)
        if not clouds:
            continue

        for a, (na, pts_a, ra) in enumerate(clouds):
            for p in pts_a:
                z = axis_value(p, cfg.up_axis)
                if z < table_band:
                    table_hits.append(
                        {
                            "i": i,
                            "link": na,
                            "z": z,
                            "axis": cfg.up_axis,
                            "method": "hull",
                        }
                    )
                    break
            for b in range(a + 1 + skip, len(clouds)):
                nb, pts_b, rb = clouds[b]
                d = _min_dist_clouds(pts_a, pts_b)
                pad = 0.25 * (ra + rb)
                hit_limit = float(cfg.self_min_dist_m) + pad * pad_scale
                near_limit = hit_limit + float(cfg.near_miss_margin_m)
                if d < hit_limit:
                    self_hits.append(
                        {
                            "i": i,
                            "a": na,
                            "b": nb,
                            "dist_mm": round(d * 1000.0, 3),
                            "method": "hull",
                        }
                    )
                elif d <= near_limit:
                    near_miss.append(
                        {
                            "i": i,
                            "a": na,
                            "b": nb,
                            "dist_mm": round(d * 1000.0, 3),
                            "clearance_m": round(d - hit_limit, 6),
                            "method": "hull",
                        }
                    )

    return {
        "table_hits": _cap(table_hits, cfg.max_events),
        "self_hits": _cap(self_hits, cfg.max_events),
        "near_miss": _cap(near_miss, cfg.max_events),
        "n_table": len(table_hits),
        "n_self": len(self_hits),
        "n_near_miss": len(near_miss),
        "method": "hull",
    }
