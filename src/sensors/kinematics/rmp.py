from __future__ import annotations

"""RMP/DLL 兼容层（纯 Python 替代实现）。

说明：
- 该模块提供与历史接口近似的调用方式，便于替换老工程中的 DLL 依赖；
- 核心仍基于本项目 FK/IK 能力，不实现环境凸包与动力学高级特性。
"""

import os
from typing import List, Optional, Sequence, Tuple, Union

import numpy as np

from .config import build_kinematic_config
from .constants import TEACH_JOINT_ACCEL_MAX_DEG_S2, TEACH_JOINT_VEL_MAX_DEG_S
from .dh import fk_link_poses_accumulated, fk_standard_dh
from .ik import ik_flange
from .model import IkResult, KinematicConfig
from .robot_xml import kinematic_config_and_limits_from_robot_xml


def trap_time_sync(dq: float, vmax: float, amax: float) -> float:
    """计算单轴位移在对称梯形速度曲线下的最短运动时间。

    参数：
        dq:    关节位移（弧度），绝对值表示运动幅度。
        vmax:  最大速度（弧度/秒），决定梯形曲线的恒速段速率。
        amax:  最大加速度（弧度/秒²），决定加/减速段时间。

    返回：
        float: 完成该位移所需的最短时间（秒）。若位移为零返回 0.0。

    算法：
        - 若位移小于 `vmax^2 / amax`，则运动为三角速度曲线（无恒速段），
          时间 = 2 * sqrt(dq / amax)。
        - 否则为完整梯形曲线，时间 = dq / vmax + vmax / amax。
    
    注意：
        该函数假设加/减速度能力相同，适用于对称梯形规划。
        实际多轴同步时，系统取各轴时间的最大值作为整段运动的总时长。
    """
    adq = abs(float(dq))
    if adq < 1e-12:
        return 0.0
    if vmax <= 1e-12 or amax <= 1e-12:
        # 防止除零或无效参数，返回保守时间 1 秒
        return 1.0
    if adq < vmax * vmax / amax:
        # 三角速度曲线（无恒速段）
        return 2.0 * float(np.sqrt(adq / amax))
    # 梯形速度曲线（包含匀速段）
    return adq / vmax + vmax / amax


class RobotRmpKinematics:
    """机器人运动学兼容类。

    兼容语义：
    - 输入关节统一为弧度（rad）；
    - 位姿缓冲统一为 16 元素行主序的 4x4 矩阵（展平）；
    - IK 输出缓冲形状对齐历史 DLL 的 8x6 约定（目前只填充第一组解）。
    
    主要方法映射：
        - rmp_set_robot(): 加载机器人配置（XML 或内置默认值）
        - rmp_set_tool_transform(): 设置工具中心点（TCP）相对法兰的变换
        - rmp_forward_kin(): 正向运动学，返回 TCP 位姿的 16 元素数组
        - rmp_forward_kin_all(): 正向运动学，同时返回每级连杆位姿链
        - rmp_inverse_kin(): 逆运动学，返回兼容格式的解缓冲区
        - p2p_plan(): 关节空间点到点轨迹规划（梯形速度时间标定 + 平滑插值）
    """

    def __init__(
        self,
        robot_xml_path: Optional[str] = None,
        *,
        joint_vel_max_deg_s: Optional[Union[np.ndarray, Sequence[float]]] = None,
        joint_acc_max_deg_s2: Optional[Union[np.ndarray, Sequence[float]]] = None,
    ) -> None:
        """初始化运动学对象。

        参数：
            robot_xml_path:       可选的 URDF/机器人描述文件路径（XML 格式），
                                  若提供且文件存在，则从中解析运动学参数及关节限位。
            joint_vel_max_deg_s:  可选，六轴最大速度（度/秒），覆盖默认值。
            joint_acc_max_deg_s2: 可选，六轴最大加速度（度/秒²），覆盖默认值。
        """
        # 法兰盘到工具（TCP）的变换矩阵，初始为单位矩阵（即无偏移）
        self.T_flange_tool: np.ndarray = np.eye(4, dtype=float)
        # 以下属性将在 rmp_set_robot() 中赋值
        self._joint_vel_max_deg_s: np.ndarray
        self._joint_acc_max_deg_s2: np.ndarray
        self._cfg: KinematicConfig
        # 调用装载配置方法
        self.rmp_set_robot(
            robot_xml_path,
            joint_vel_max_deg_s=joint_vel_max_deg_s,
            joint_acc_max_deg_s2=joint_acc_max_deg_s2,
        )

    @property
    def kinematic_config(self) -> KinematicConfig:
        """当前生效的运动学配置（只读视图）。"""
        return self._cfg

    def rmp_set_robot(
        self,
        robot_xml_path: Optional[str],
        *,
        joint_vel_max_deg_s: Optional[Union[np.ndarray, Sequence[float]]] = None,
        joint_acc_max_deg_s2: Optional[Union[np.ndarray, Sequence[float]]] = None,
    ) -> int:
        """装载机器人运动学配置与限制参数。

        参数：
            robot_xml_path:       机器人描述文件路径（XML），若为 None 或文件不存在，
                                  则回退使用内置常量。
            joint_vel_max_deg_s:  自定义六轴最大速度（度/秒），优先级最高。
            joint_acc_max_deg_s2: 自定义六轴最大加速度（度/秒²），优先级最高。

        返回：
            int: 0 表示成功（本实现始终返回 0）。

        行为：
            - 优先从 XML 加载，解析得到 KinematicConfig 以及速度/加速度限位。
            - 若未提供 XML 或解析失败，则使用 constants 中的默认配置。
            - 如果用户显式传入 joint_vel_max_deg_s / joint_acc_max_deg_s2，则覆盖上述值。
        """
        if robot_xml_path and os.path.isfile(robot_xml_path):
            # 从 XML 文件解析配置及运动限位
            cfg, v_xml, a_xml = kinematic_config_and_limits_from_robot_xml(robot_xml_path)
            self._cfg = cfg
            self._joint_vel_max_deg_s = np.asarray(v_xml, dtype=float).reshape(6)
            self._joint_acc_max_deg_s2 = np.asarray(a_xml, dtype=float).reshape(6)
        else:
            # 使用内置默认运动学参数及限位
            self._cfg = build_kinematic_config()
            self._joint_vel_max_deg_s = np.asarray(TEACH_JOINT_VEL_MAX_DEG_S, dtype=float).reshape(6)
            self._joint_acc_max_deg_s2 = np.asarray(TEACH_JOINT_ACCEL_MAX_DEG_S2, dtype=float).reshape(6)

        # 若用户显式提供了关节速度/加速度限位，则覆盖已设置的数值
        if joint_vel_max_deg_s is not None:
            self._joint_vel_max_deg_s = np.asarray(joint_vel_max_deg_s, dtype=float).reshape(6)
        if joint_acc_max_deg_s2 is not None:
            self._joint_acc_max_deg_s2 = np.asarray(joint_acc_max_deg_s2, dtype=float).reshape(6)

        # 校验机械臂类型：当前实现仅支持类型 1（标准的六轴偏置腕结构）
        if self._cfg.manipulator_type != 1:
            raise ValueError(f"RobotRmpKinematics 仅支持 ManipulatorType=1，当前为 {self._cfg.manipulator_type}")
        return 0

    def rmp_set_env(self, *_args: object, **_kwargs: object) -> int:
        """环境设置占位函数（本实现不建模环境障碍）。

        历史 DLL 接口中用于设置碰撞形状、工作空间凸包等高级特性。
        纯 Python 替代版本中不做实际处理，仅保留占位以维持接口兼容。
        """
        return 0

    def rmp_set_tool_transform(self, T_flange_tool: Union[np.ndarray, Sequence[float]]) -> int:
        """设置法兰盘到工具（TCP）的固定齐次变换。

        参数：
            T_flange_tool: 4x4 齐次变换矩阵，以行主序数组或嵌套列表形式提供。

        返回：
            int: 0 表示成功。

        说明：
            该变换将作用于后续所有正向/逆运动学计算：
                T_tcp = T_flange * T_flange_tool
        """
        self.T_flange_tool = np.asarray(T_flange_tool, dtype=float).reshape(4, 4).copy()
        return 0

    def rmp_forward_kin(self, joint_rad: Union[np.ndarray, Sequence[float]]) -> np.ndarray:
        """正向运动学：计算机器人 TCP（工具中心点）在世界坐标系下的位姿。

        参数：
            joint_rad: 六个关节角度（弧度），形状 (6,)。

        返回：
            np.ndarray: 形状 (16,) 的 float32 数组，按行主序（C-order）存储的 4x4 齐次矩阵。
        
        算法步骤：
            1. 调用 DH 正运动学得到法兰位姿（基坐标系下）。
            2. 右乘工具变换 T_flange_tool，得到 TCP 位姿。
            3. 展平为 16 元素向量，类型转换为 float32（与历史 DLL 返回类型一致）。
        """
        q = np.asarray(joint_rad, dtype=float).reshape(6)
        # 正向运动学：基座 → 法兰盘
        T = fk_standard_dh(q, self._cfg.T_base, self._cfg.dh_table)
        # 应用工具变换
        T = T @ self.T_flange_tool
        # 转换为 float32 并按行主序展平为 16 个元素
        return T.astype(np.float32).reshape(16, order="C")

    def rmp_forward_kin_all(
        self,
        joint_rad: Union[np.ndarray, Sequence[float]],
    ) -> Tuple[np.ndarray, List[np.ndarray]]:
        """正向运动学（增强版）：返回 TCP 位姿以及所有连杆（含 TCP）的位姿列表。

        参数：
            joint_rad: 六个关节角度（弧度），形状 (6,)。

        返回：
            tuple: (pose16, mats)
                - pose16: TCP 位姿展平的 16 元素 float32 数组（与 rmp_forward_kin 输出一致）
                - mats:   列表，元素为 4x4 齐次矩阵。顺序为：
                          [T_base, T_after_J1, ..., T_after_J6, T_tcp]
                          共计 8 个矩阵（基座 + 6 个连杆 + TCP）。
        
        用途：
            可视化、碰撞检测、中间连杆位姿分析等场景。
        """
        q = np.asarray(joint_rad, dtype=float).reshape(6)
        # 累积各连杆（包括基座）的位姿，共 7 个：T_base, T_J1, ..., T_J6
        Ts = fk_link_poses_accumulated(q, self._cfg.T_base, self._cfg.dh_table)
        # 计算 TCP 位姿
        T_tcp = Ts[-1] @ self.T_flange_tool
        # 展平 TCP 位姿为 16 元素数组
        pose16 = T_tcp.astype(np.float32).reshape(16, order="C")
        # 构建完整列表：基座 + 6 个关节坐标系 + TCP
        joint_mats = Ts + [T_tcp]
        return pose16, joint_mats

    def rmp_inverse_kin(
        self,
        tool_pose16: Union[np.ndarray, Sequence[float]],
        seed_rad: Union[np.ndarray, Sequence[float]],
        *,
        max_nfev: int = 800,
        enforce_teach_soft_limits: bool = False,
    ) -> Tuple[int, Optional[np.ndarray]]:
        """逆运动学兼容接口，返回历史 DLL 约定的缓冲区格式。

        参数：
            tool_pose16: 目标 TCP 位姿，展平的 16 元素数组（行主序 4x4 矩阵）。
            seed_rad:    初始猜测关节角度（弧度），形状 (6,)。
            max_nfev:    最大函数评估次数，传给 ik_flange。
            enforce_teach_soft_limits: 是否强制执行关节软限位。

        返回：
            tuple: (解数量, 8x6 缓冲数组)
                - 解数量：通常为 1（找到解）或 0（未找到）。
                - 缓冲：形状 (8, 6) 的 float64 数组，第一行为解（弧度），其余行为零。
                      历史 DLL 中支持最多 8 个 IK 解，此处只输出一个。
        
        说明：
            1. 先从 TCP 目标位姿反推出法兰盘目标位姿：T_flange = T_tcp * inv(T_flange_tool)
            2. 调用数值 IK（ik_flange）求解关节角度（度）。
            3. 若求解成功，将弧度制解填入缓冲区索引 0 行。
            4. 异常时返回 (0, None)，与历史接口行为一致（不抛出异常）。
        """
        T_tcp = np.asarray(tool_pose16, dtype=float).reshape(4, 4)
        # 检查矩阵是否有效（无 NaN/Inf）
        R = T_tcp[:3, :3]
        if not np.isfinite(R).all():
            return 0, None
        # 将初始猜测从弧度转为度（IK 接口要求度）
        seed_deg = np.rad2deg(np.asarray(seed_rad, dtype=float).reshape(6))
        try:
            # 从 TCP 目标反推法兰目标：T_fl = T_tcp * inv(T_tool)
            T_fl = T_tcp @ np.linalg.inv(self.T_flange_tool)
            q_deg = ik_flange(
                T_fl,
                seed_deg,
                kin_cfg=self._cfg,
                max_nfev=max_nfev,
                enforce_teach_soft_limits=enforce_teach_soft_limits,
            )
            # 如果返回的是 IkResult 对象，提取 joint_deg
            if isinstance(q_deg, IkResult):
                q_deg = q_deg.joint_deg
            # 构建输出缓冲区：8x6 零矩阵，第一行放解（弧度）
            buf = np.zeros((8, 6), dtype=float)
            buf[0, :] = np.deg2rad(np.asarray(q_deg, dtype=float).reshape(6))
            return 1, buf
        except (RuntimeError, np.linalg.LinAlgError):
            # 保持与旧接口的一致性：失败时不抛异常，只返回 (0, None)
            return 0, None

    def p2p_plan(
        self,
        start_rad: Union[np.ndarray, Sequence[float]],
        target_rad: Union[np.ndarray, Sequence[float]],
        dt: float,
        *,
        max_points: int = 6000,
    ) -> np.ndarray:
        """生成关节空间点到点轨迹（梯形速度时间标定 + 三次平滑插值）。

        参数：
            start_rad:   起始关节角度（弧度），形状 (6,)。
            target_rad:  目标关节角度（弧度），形状 (6,)。
            dt:          离散化时间步长（秒），决定输出轨迹的时间分辨率。
            max_points:  最大输出点数，用于限制内存占用。

        返回：
            np.ndarray: 形状 (N, 6) 的轨迹点，单位为弧度。
                        N 由总时长 T 和 dt 决定，但不超过 max_points。

        算法细节：
            1. 对各轴分别调用 trap_time_sync 计算所需时间，取最大值作为同步总时长 T。
            2. 使用平滑步长函数 s(t) = 3*(t/T)^2 - 2*(t/T)^3，满足 s(0)=0，s(1)=1，
               且一阶导数在端点处为零（起止速度为零）。
            3. 轨迹点 = start + s(t) * (target - start)。
        """
        start = np.asarray(start_rad, dtype=float).reshape(6)
        target = np.asarray(target_rad, dtype=float).reshape(6)
        dq = target - start

        # 各轴最大速度、加速度（弧度/秒，弧度/秒²）
        vmax = np.deg2rad(self._joint_vel_max_deg_s)
        amax = np.deg2rad(self._joint_acc_max_deg_s2)

        # 计算各轴所需运动时间
        t_need = [trap_time_sync(float(dq[i]), float(vmax[i]), float(amax[i])) for i in range(6)]
        # 同步总时长 = 各轴最大时间，同时确保至少包含两个采样点
        T = max(float(np.max(t_need)) if t_need else 0.0, float(dt) * 2.0, float(dt))

        # 计算离散点数，限制最大点数
        n = min(int(max_points), int(np.ceil(T / float(dt))) + 1)
        t = np.linspace(0.0, T, n)

        # 三次平滑步长函数
        s = 3.0 * (t / T) ** 2 - 2.0 * (t / T) ** 3
        # 线性插值生成轨迹
        return start[np.newaxis, :] + s[:, np.newaxis] * dq[np.newaxis, :]

    # -------------------- 历史别名（兼容旧代码调用）--------------------
    def forward_kin(self, joint_rad: Union[np.ndarray, Sequence[float]]) -> np.ndarray:
        """历史别名：等价于 `rmp_forward_kin`。"""
        return self.rmp_forward_kin(joint_rad)

    def forward_kin_all(self, joint_rad: Union[np.ndarray, Sequence[float]]) -> Tuple[np.ndarray, List[np.ndarray]]:
        """历史别名：等价于 `rmp_forward_kin_all`。"""
        return self.rmp_forward_kin_all(joint_rad)

    def inverse_kin(
        self,
        pose: Union[np.ndarray, Sequence[float]],
        seed: Union[np.ndarray, Sequence[float]],
        *,
        max_nfev: int = 800,
        enforce_teach_soft_limits: bool = False,
    ) -> Tuple[bool, Optional[List[float]]]:
        """历史别名：返回 `(是否成功, 最近解列表（弧度）或 None)`。

        参数：
            pose: 目标 TCP 位姿（16 元素展平数组）。
            seed: 初始猜测关节角度（弧度）。
            max_nfev: 最大函数评估次数。
            enforce_teach_soft_limits: 是否强制执行关节软限位。

        返回：
            tuple(ok, q_list):
                - ok: True 表示求解成功。
                - q_list: 解列表（弧度），长度为 6；失败时为 None。
        """
        n, buf = self.rmp_inverse_kin(
            pose,
            seed,
            max_nfev=max_nfev,
            enforce_teach_soft_limits=enforce_teach_soft_limits,
        )
        if n <= 0 or buf is None:
            return False, None
        q = buf[0].tolist()
        return True, q