from __future__ import annotations

"""Elite EC kinematics (ported from autodl-tmp/demo_test).

Provides FK/IK for sensors-dcs hik_dataset cartesian fields and arm tooling.
Default DH table matches the teach/calibration constants used by demo_test.
"""

from typing import Sequence

import numpy as np

from .fk import fk_flange, fk_position_mm, fk_tool, joint_q_rad
from .ik import ik_flange

__all__ = [
    "fk_flange",
    "fk_tool",
    "fk_position_mm",
    "fk_flange_from_joints_rad",
    "make_hik_fk_fn",
    "ik_flange",
    "joint_q_rad",
]


def fk_flange_from_joints_rad(
    joints_rad: Sequence[float],
    **kwargs,
) -> np.ndarray:
    """FK flange pose from Elite ``machinePos`` joints already in radians.

    Sensors / DCS store arm joints as radians (degrees from EC monitor → rad).
    demo_test ``machine_deg`` expects degrees, so we convert once here.
    """
    j = [float(x) for x in list(joints_rad)[:6]]
    if len(j) < 6:
        j = j + [0.0] * (6 - len(j))
    jdeg = np.rad2deg(np.asarray(j, dtype=float))
    return fk_flange(jdeg, joint_angle_mode="machine_deg", **kwargs)


def make_hik_fk_fn():
    """Return ``fk(joints_rad) -> 4x4`` for ``export.hik_dataset.build_steps``.

    Matches hik_gello usage: flange FK only; TCP offset is applied in
    ``build_steps`` via ``tcp_xyz``.
    """

    def _fk(joints: list[float]) -> np.ndarray:
        return fk_flange_from_joints_rad(joints)

    return _fk
