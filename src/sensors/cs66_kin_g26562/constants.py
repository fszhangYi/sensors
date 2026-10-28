from __future__ import annotations

"""兼容层：几何/限位数据来自 YAML profile，不再硬编码机械臂型号参数。

算法无关的共享常量（PI2、轴向枚举）仍留在本模块。
型号相关量通过 ``get_profile()`` / 下列属性代理访问。
"""

import numpy as np

from .yaml_profile import get_profile, resolve_profile_id

# π/2 — used by dh_table_from_link_lengths_m (also redefined in dh.py).
PI2 = np.pi / 2.0

# Controller axis enum → unit vectors (shared convention, not per-arm YAML).
_AXIS_VEC = {
    1: np.array([0.0, 0.0, 1.0]),
    2: np.array([0.0, 0.0, -1.0]),
    3: np.array([-1.0, 0.0, 0.0]),
    4: np.array([1.0, 0.0, 0.0]),
    5: np.array([0.0, 1.0, 0.0]),
    6: np.array([1.0, 0.0, 0.0]),
}


def _p():
    return get_profile()


def __getattr__(name: str):
    """Lazy proxies so ``from .constants import LINK_LENGTH_MM`` follows active profile."""
    p = _p()
    mapping = {
        "LINK_LENGTH_MM": p.link_length_mm,
        "JOINT_OFFSET_DEG": p.joint_offset_deg,
        "JOINT_DIRECTION": p.joint_direction,
        "MANIPULATOR_TYPE": p.manipulator_type,
        "BASE_CS_X": p.base_cs_x,
        "BASE_CS_Z": p.base_cs_z,
        "TEACH_INSTALL_NOTE": p.install_note,
        "TEACH_INSTALL_ROT_Z1_DEG": p.install_rot_z1_deg,
        "TEACH_INSTALL_ROT_XB_Q1_DEG": p.install_rot_xb_q1_deg,
        "TEACH_PENDANT_POSES": p.teach_poses,
        "TEACH_JOINT_VEL_MAX_DEG_S": p.joint_vel_max_deg_s,
        "TEACH_JOINT_VEL_MIN_DEG_S": p.joint_vel_min_deg_s,
        "TEACH_JOINT_ACCEL_MAX_DEG_S2": p.joint_accel_max_deg_s2,
        "TEACH_CART_VEL_MAX_MM_S": p.cart_vel_max_mm_s,
        "TEACH_CART_VEL_MIN_MM_S": p.cart_vel_min_mm_s,
        "TEACH_CART_ACCEL_MAX_MM_S2": p.cart_accel_max_mm_s2,
        "TEACH_ORI_VEL_MAX_DEG_S": p.ori_vel_max_deg_s,
        "TEACH_ORI_VEL_MIN_DEG_S": p.ori_vel_min_deg_s,
        "TEACH_ORI_ACCEL_MAX_DEG_S2": p.ori_accel_max_deg_s2,
        "TEACH_JOINT_SOFT_MIN_DEG": p.soft_min_deg,
        "TEACH_JOINT_SOFT_MAX_DEG": p.soft_max_deg,
        "ACTIVE_PROFILE_ID": resolve_profile_id(),
    }
    if name in mapping:
        return mapping[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
