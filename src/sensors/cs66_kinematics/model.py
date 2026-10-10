from __future__ import annotations

"""核心数据模型与类型定义（cs66_kinematics 副本；支持 YAML profile 扩展字段）。"""

from dataclasses import dataclass
from typing import Literal, NamedTuple, Optional, Tuple

import numpy as np

JointAngleMode = Literal["machine_deg", "pendant_deg", "xml_delta_dir", "raw_deg"]

# standard DH 单行定义：(theta_offset, d, a, alpha)
DhRow = Tuple[float, float, float, float]

GeometryKind = Literal["standard_dh", "elite_mdh"]


@dataclass(frozen=True)
class TeachPendantPose:
    """示教器标定表中的单行记录。"""

    index: int
    joint_deg: Tuple[float, float, float, float, float, float]
    position_mm: Tuple[float, float, float]
    orientation_rxyz_deg: Tuple[float, float, float]


@dataclass
class KinematicConfig:
    """运动学求解所需的最小配置集。"""

    joint_offset_deg: np.ndarray
    joint_dir: np.ndarray
    manipulator_type: int
    T_base: np.ndarray
    dh_table: Tuple[DhRow, ...]
    link_len_m: np.ndarray
    # YAML profile extensions
    soft_min_deg: Optional[np.ndarray] = None
    soft_max_deg: Optional[np.ndarray] = None
    profile_id: Optional[str] = None
    geometry_kind: GeometryKind = "standard_dh"
    mdh_alpha_rad: Optional[np.ndarray] = None
    mdh_a_m: Optional[np.ndarray] = None
    mdh_d_m: Optional[np.ndarray] = None
    default_tcp_xyz_m: Optional[np.ndarray] = None


class IkResult(NamedTuple):
    """逆运动学返回明细（return_details=True 时使用）。"""

    joint_deg: np.ndarray
    success: bool
    message: str
    nfev: int
    residual_norm: float
