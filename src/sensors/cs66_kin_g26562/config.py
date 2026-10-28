from __future__ import annotations

"""运动学配置构建与缓存（YAML profile 驱动）。"""

from typing import Dict, Optional

import numpy as np

from .constants import _AXIS_VEC
from .dh import dh_table_from_link_lengths_m
from .model import DhRow, KinematicConfig
from .teach import teach_base_rotation_from_install_deg
from .yaml_profile import (
    ArmYamlProfile,
    clear_profile_cache,
    get_profile,
    resolve_profile_id,
)


def cs_to_rotation(base_dir_x: int, base_dir_z: int) -> np.ndarray:
    """根据控制器坐标系轴向枚举值计算正交旋转矩阵。"""
    x = np.asarray(_AXIS_VEC.get(base_dir_x, _AXIS_VEC[6]), dtype=float)
    z = np.asarray(_AXIS_VEC.get(base_dir_z, _AXIS_VEC[1]), dtype=float)
    x /= np.linalg.norm(x)
    z /= np.linalg.norm(z)
    if abs(np.dot(x, z)) > 0.98:
        return np.eye(3)
    y = np.cross(z, x)
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, y, z])


def _empty_dh_table() -> tuple[DhRow, ...]:
    return tuple((0.0, 0.0, 0.0, 0.0) for _ in range(6))


def build_kinematic_config_from_profile(profile: ArmYamlProfile) -> KinematicConfig:
    """Build ``KinematicConfig`` from a validated YAML profile."""
    if not profile.supported:
        raise RuntimeError(
            f"kinematics profile {profile.id!r} has supported=false; "
            "fill YAML geometry/limits/teach_poses then set supported: true"
        )
    if profile.manipulator_type != 1:
        raise ValueError(
            f"Unsupported ManipulatorType={profile.manipulator_type} (only type 1 is supported)."
        )

    R = cs_to_rotation(profile.base_cs_x, profile.base_cs_z) @ teach_base_rotation_from_install_deg(
        rot_z1_deg=profile.install_rot_z1_deg,
        rot_xb_q1_deg=profile.install_rot_xb_q1_deg,
    )
    T_base = np.eye(4)
    T_base[:3, :3] = R

    geometry_kind = "standard_dh"
    mdh_alpha = mdh_a = mdh_d = None
    link_m = np.array(profile.link_length_mm, dtype=float) / 1000.0
    dh_table: tuple[DhRow, ...]

    if profile.dh_convention == "explicit_dh_table":
        if profile.dh_table is None:
            raise ValueError(f"{profile.id}: dh_convention=explicit_dh_table but dh_table missing")
        dh_table = profile.dh_table
        # Best-effort link lengths from d/a columns for debugging.
        link_m = np.array([row[1] if i in (0, 3, 4, 5) else row[2] for i, row in enumerate(dh_table)])
    elif profile.dh_convention == "mdh":
        if profile.mdh_alpha_rad is None or profile.mdh_a_m is None or profile.mdh_d_m is None:
            raise ValueError(f"{profile.id}: dh_convention=mdh but mdh block incomplete")
        geometry_kind = "elite_mdh"
        mdh_alpha = np.array(profile.mdh_alpha_rad, dtype=float)
        mdh_a = np.array(profile.mdh_a_m, dtype=float)
        mdh_d = np.array(profile.mdh_d_m, dtype=float)
        dh_table = _empty_dh_table()
        link_m = mdh_d.copy()  # diagnostic only
    else:
        # link_lengths_standard — fixed offset-wrist alpha pattern in dh.py
        if all(abs(x) < 1e-15 for x in profile.link_length_mm):
            raise ValueError(f"{profile.id}: link_length_mm is all zeros")
        dh_table = dh_table_from_link_lengths_m(link_m)

    return KinematicConfig(
        joint_offset_deg=np.array(profile.joint_offset_deg, dtype=float),
        joint_dir=np.array(profile.joint_direction, dtype=float),
        manipulator_type=int(profile.manipulator_type),
        T_base=T_base,
        dh_table=dh_table,
        link_len_m=np.asarray(link_m, dtype=float).copy(),
        soft_min_deg=np.array(profile.soft_min_deg, dtype=float),
        soft_max_deg=np.array(profile.soft_max_deg, dtype=float),
        profile_id=profile.id,
        geometry_kind=geometry_kind,  # type: ignore[arg-type]
        mdh_alpha_rad=mdh_alpha,
        mdh_a_m=mdh_a,
        mdh_d_m=mdh_d,
        default_tcp_xyz_m=np.array(profile.default_tcp_xyz_m, dtype=float),
    )


def build_kinematic_config(profile_id: Optional[str] = None) -> KinematicConfig:
    """构建完整的 `KinematicConfig`（不走缓存）。"""
    return build_kinematic_config_from_profile(get_profile(profile_id))


_CFG_CACHE: Dict[str, KinematicConfig] = {}


def get_kinematic_config(profile_id: Optional[str] = None) -> KinematicConfig:
    """按 profile_id 缓存的运动学配置；``None`` → 默认/环境变量选定的型号。"""
    pid = resolve_profile_id(profile_id)
    cfg = _CFG_CACHE.get(pid)
    if cfg is None:
        cfg = build_kinematic_config(pid)
        _CFG_CACHE[pid] = cfg
    return cfg


def clear_kinematic_config_cache() -> None:
    """清空配置与 YAML profile 缓存。"""
    _CFG_CACHE.clear()
    clear_profile_cache()
