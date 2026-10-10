from __future__ import annotations

"""YAML-backed multi-arm kinematics (sibling of ``sensors.kinematics``).

Does **not** modify the original ``sensors.kinematics`` package.
Robot geometry / limits / teach poses live in ``profiles/*.yaml``.

FK needs NumPy; IK needs SciPy (lazy import).
"""

from typing import TYPE_CHECKING, Any, Optional, Sequence

import numpy as np

from .config import (
    build_kinematic_config,
    clear_kinematic_config_cache,
    get_kinematic_config,
)
from .fk import fk_flange, fk_position_mm, fk_tool, joint_q_rad
from .yaml_profile import (
    get_profile,
    list_profile_ids,
    resolve_profile_id,
    yaml_migration_checklist,
)

if TYPE_CHECKING:
    from .ik import ik_flange as ik_flange

__all__ = [
    "fk_flange",
    "fk_tool",
    "fk_position_mm",
    "fk_flange_from_joints_rad",
    "make_hik_fk_fn",
    "ik_flange",
    "joint_q_rad",
    "get_kinematic_config",
    "build_kinematic_config",
    "clear_kinematic_config_cache",
    "get_profile",
    "list_profile_ids",
    "resolve_profile_id",
    "yaml_migration_checklist",
]


def __getattr__(name: str) -> Any:
    if name == "ik_flange":
        from .ik import ik_flange as _ik_flange

        return _ik_flange
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def fk_flange_from_joints_rad(
    joints_rad: Sequence[float],
    *,
    profile_id: Optional[str] = None,
    **kwargs,
) -> np.ndarray:
    """FK flange pose from joints already in radians."""
    j = [float(x) for x in list(joints_rad)[:6]]
    if len(j) < 6:
        j = j + [0.0] * (6 - len(j))
    jdeg = np.rad2deg(np.asarray(j, dtype=float))
    kin_cfg = kwargs.pop("kin_cfg", None)
    if kin_cfg is None:
        kin_cfg = get_kinematic_config(profile_id)
    mode = kwargs.pop("joint_angle_mode", None)
    if mode is None:
        mode = get_profile(profile_id).default_joint_angle_mode
    return fk_flange(jdeg, joint_angle_mode=mode, kin_cfg=kin_cfg, **kwargs)


def make_hik_fk_fn(profile_id: Optional[str] = None):
    """Return ``fk(joints_rad) -> 4x4`` bound to a YAML profile."""

    pid = resolve_profile_id(profile_id)
    cfg = get_kinematic_config(pid)

    def _fk(joints: list[float]) -> np.ndarray:
        return fk_flange_from_joints_rad(joints, kin_cfg=cfg, profile_id=pid)

    return _fk
