"""Schema constants and small typed helpers for traj_safety."""

from __future__ import annotations

from typing import Any, Mapping, Sequence

SCHEMA_INPUT = "sensors_traj_safety_input_v1"
SCHEMA_RESULT = "sensors_traj_safety_v1"


def as_xyz(v: Any) -> tuple[float, float, float] | None:
    if not isinstance(v, (list, tuple)) or len(v) < 3:
        if isinstance(v, Mapping):
            try:
                return (float(v["x"]), float(v["y"]), float(v["z"]))
            except (KeyError, TypeError, ValueError):
                return None
        return None
    try:
        x, y, z = float(v[0]), float(v[1]), float(v[2])
    except (TypeError, ValueError):
        return None
    if x != x or y != y or z != z:
        return None
    return (x, y, z)


def axis_value(pos: Sequence[float], up_axis: str) -> float:
    ax = (up_axis or "z").strip().lower()
    if ax == "x":
        return float(pos[0])
    if ax == "y":
        return float(pos[1])
    return float(pos[2])


def dist3(a: Sequence[float], b: Sequence[float]) -> float:
    return float(
        ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5
    )
