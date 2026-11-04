"""Link-center (sphere) table / self collision — port of embody analyzeCollisionsLegacy."""

from __future__ import annotations

from typing import Any, Mapping, Sequence

from .config import SafetyConfig
from .types import as_xyz, axis_value, dist3


def _cap(events: list[dict[str, Any]], n: int) -> list[dict[str, Any]]:
    return events[: max(1, int(n))]


def analyze_sphere_collisions(
    link_series: Sequence[Sequence[Mapping[str, Any]] | Mapping[str, Any]],
    config: SafetyConfig | Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Analyze per-frame link centers.

    ``link_series[i]`` is either a list of ``{name, pos}`` or
    ``{"links": [...]}``. ``pos`` is meters ``[x,y,z]`` or ``{x,y,z}``.
    """
    cfg = (
        config
        if isinstance(config, SafetyConfig)
        else SafetyConfig.from_mapping(config)  # type: ignore[arg-type]
    )
    table_hits: list[dict[str, Any]] = []
    self_hits: list[dict[str, Any]] = []
    near_miss: list[dict[str, Any]] = []
    hit_limit = float(cfg.self_min_dist_m)
    near_limit = hit_limit + float(cfg.near_miss_margin_m)
    table_band = float(cfg.table_z) + float(cfg.table_clearance_m)
    skip = int(cfg.skip_adjacent)

    for i, frame in enumerate(link_series):
        links_raw: Sequence[Mapping[str, Any]]
        if isinstance(frame, Mapping) and "links" in frame:
            links_raw = frame.get("links") or []  # type: ignore[assignment]
        else:
            links_raw = frame  # type: ignore[assignment]
        links: list[tuple[str, tuple[float, float, float], float]] = []
        for item in links_raw:
            if not isinstance(item, Mapping):
                continue
            name = str(item.get("name") or item.get("link") or "").strip() or "?"
            pos = as_xyz(item.get("pos") or item.get("position"))
            if pos is None:
                continue
            try:
                radius = float(item.get("radius") if item.get("radius") is not None else 0.0)
            except (TypeError, ValueError):
                radius = 0.0
            links.append((name, pos, max(0.0, radius)))
        if not links:
            continue

        for a, (na, pa, ra) in enumerate(links):
            z = axis_value(pa, cfg.up_axis)
            # If radii enabled, sphere touches table when center is within clearance+radius.
            z_eff = z - (ra if cfg.use_radii else 0.0)
            if z_eff < table_band:
                table_hits.append(
                    {
                        "i": i,
                        "link": na,
                        "z": z,
                        "axis": cfg.up_axis,
                        "method": "sphere",
                    }
                )
            for b in range(a + 1 + skip, len(links)):
                nb, pb, rb = links[b]
                d = dist3(pa, pb)
                use_surf = bool(cfg.use_radii) and (ra > 0.0 or rb > 0.0)
                if use_surf:
                    clearance = d - ra - rb
                    if clearance < 0.0:
                        self_hits.append(
                            {
                                "i": i,
                                "a": na,
                                "b": nb,
                                "dist_mm": round(d * 1000.0, 3),
                                "clearance_m": round(clearance, 6),
                                "method": "sphere",
                            }
                        )
                    elif clearance <= float(cfg.near_miss_margin_m):
                        near_miss.append(
                            {
                                "i": i,
                                "a": na,
                                "b": nb,
                                "dist_mm": round(d * 1000.0, 3),
                                "clearance_m": round(clearance, 6),
                                "method": "sphere",
                            }
                        )
                elif d < hit_limit:
                    self_hits.append(
                        {
                            "i": i,
                            "a": na,
                            "b": nb,
                            "dist_mm": round(d * 1000.0, 3),
                            "method": "sphere",
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
                            "method": "sphere",
                        }
                    )

    return {
        "table_hits": _cap(table_hits, cfg.max_events),
        "self_hits": _cap(self_hits, cfg.max_events),
        "near_miss": _cap(near_miss, cfg.max_events),
        "n_table": len(table_hits),
        "n_self": len(self_hits),
        "n_near_miss": len(near_miss),
        "method": "sphere",
    }
