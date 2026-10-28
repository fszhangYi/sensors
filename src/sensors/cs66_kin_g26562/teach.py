from __future__ import annotations

"""示教器语义相关工具函数（限位/校验点来自 YAML profile）。"""

from typing import List, Literal, Optional, Sequence, Tuple, Union

import numpy as np

from .yaml_profile import get_profile


def teach_joint_soft_limits_deg(
    profile_id: Optional[str] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """返回示教器软限位（单位 deg）。"""
    p = get_profile(profile_id)
    lo = np.array(p.soft_min_deg, dtype=float)
    hi = np.array(p.soft_max_deg, dtype=float)
    return lo, hi


def clip_joints_to_teach_soft_limits(
    joint_deg: Union[np.ndarray, Sequence[float]],
    profile_id: Optional[str] = None,
) -> np.ndarray:
    """将关节角裁剪到软限位范围（单位 deg）。"""
    lo, hi = teach_joint_soft_limits_deg(profile_id)
    return np.clip(np.asarray(joint_deg, dtype=float).reshape(6), lo, hi)


def joints_within_teach_soft_limits(
    joint_deg: Union[np.ndarray, Sequence[float]],
    profile_id: Optional[str] = None,
    *,
    lo_deg: Optional[np.ndarray] = None,
    hi_deg: Optional[np.ndarray] = None,
) -> bool:
    """判断关节角是否位于软限位内（带微小容差）。"""
    q = np.asarray(joint_deg, dtype=float).reshape(6)
    if lo_deg is None or hi_deg is None:
        lo, hi = teach_joint_soft_limits_deg(profile_id)
    else:
        lo = np.asarray(lo_deg, dtype=float).reshape(6)
        hi = np.asarray(hi_deg, dtype=float).reshape(6)
    return bool(np.all(q >= lo - 1e-9) and np.all(q <= hi + 1e-9))


def teach_poses_joint_xyz_mm(
    profile_id: Optional[str] = None,
) -> List[Tuple[np.ndarray, np.ndarray]]:
    """导出示教标定数据为 ndarray 形式。"""
    p = get_profile(profile_id)
    return [
        (np.array(pose.joint_deg, dtype=float), np.array(pose.position_mm, dtype=float))
        for pose in p.teach_poses
    ]


def teach_flange_T_m_from_pendant(
    position_mm: Union[np.ndarray, Sequence[float]],
    rxyz_deg: Union[np.ndarray, Sequence[float]],
    *,
    euler_seq: Literal["XYZ", "ZYX", "xyz", "zyx"] = "xyz",
    degrees: bool = True,
) -> np.ndarray:
    """示教器报告的法兰位姿 → 4×4 **米**（基座→法兰）。

    Elite CS 示教器 RPY[°] 与 SciPy intrinsic ``xyz`` 对齐（G26562 / 既有 CS66
    teach_poses 交叉验证）；勿默认用大写 ``XYZ``（外旋）。
    """
    p = np.asarray(position_mm, dtype=float).reshape(3)
    r = np.asarray(rxyz_deg, dtype=float).reshape(3)
    from scipy.spatial.transform import Rotation

    R = Rotation.from_euler(euler_seq, r, degrees=degrees).as_matrix()
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = p / 1000.0
    return T


def teach_base_rotation_from_install_deg(
    rot_z1_deg: Optional[float] = None,
    rot_xb_q1_deg: Optional[float] = None,
    profile_id: Optional[str] = None,
) -> np.ndarray:
    """根据安装节角度构造基座旋转矩阵 ``Rz(V) @ Rx(Q1)``。"""
    if rot_z1_deg is None or rot_xb_q1_deg is None:
        p = get_profile(profile_id)
        if rot_z1_deg is None:
            rot_z1_deg = p.install_rot_z1_deg
        if rot_xb_q1_deg is None:
            rot_xb_q1_deg = p.install_rot_xb_q1_deg
    z1 = float(rot_z1_deg)
    x1 = float(rot_xb_q1_deg)
    if abs(z1) < 1e-12 and abs(x1) < 1e-12:
        return np.eye(3, dtype=float)
    rz = np.deg2rad(z1)
    rx = np.deg2rad(x1)
    c1, s1 = np.cos(rz), np.sin(rz)
    R_z = np.array([[c1, -s1, 0.0], [s1, c1, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    c2, s2 = np.cos(rx), np.sin(rx)
    R_x = np.array([[1.0, 0.0, 0.0], [0.0, c2, -s2], [0.0, s2, c2]], dtype=float)
    return R_z @ R_x
