from __future__ import annotations

"""robot.xml 解析器。

目标：从厂商模板 XML 中提取本项目真正需要的最小字段集合，
并转换成统一的 `KinematicConfig` + 速度/加速度上限数组。
"""

import xml.etree.ElementTree as ET
from typing import Tuple

import numpy as np

from .config import cs_to_rotation
from .constants import BASE_CS_X, BASE_CS_Z
from .dh import dh_table_from_link_lengths_m
from .model import KinematicConfig
from .teach import teach_base_rotation_from_install_deg


def kinematic_config_and_limits_from_robot_xml(
    xml_path: str,
) -> Tuple[KinematicConfig, np.ndarray, np.ndarray]:
    """解析机器人 XML 配置文件（厂商格式），提取运动学参数与动态限制。

    该函数读取特定格式的 XML 文件（通常由机器人厂商提供或 URDF 转换而来），
    从中抽取出本项目运动学求解器所需的：
        - 连杆长度（6 个，mm → 自动转换为米）
        - 关节零点偏移（度）
        - 关节方向系数（±1）
        - 关节最大速度（度/秒）
        - 关节最大加速度（度/秒²）
        - 机械臂类型（目前仅支持类型 1）
        - 基坐标系轴向枚举（X/Z 轴方向）

    同时结合安装标定旋转（`teach_base_rotation_from_install_deg`）计算最终的
    基坐标系变换矩阵 `T_base`。

    参数：
        xml_path: XML 文件的路径（字符串）。

    返回：
        tuple:
            - cfg: `KinematicConfig` 对象，可直接用于正/逆运动学求解。
            - vel_max: shape (6,) 的 numpy 数组，关节最大速度（度/秒）。
            - acc_max: shape (6,) 的 numpy 数组，关节最大加速度（度/秒²）。

    异常：
        ValueError: 如果 XML 中缺少 `<Manipulator>` 根元素。

    注意：
        - 本函数不解析碰撞几何凸包、视觉传感器等扩展信息，仅关注运动学核心。
        - 若 XML 中未提供某些字段，则对应数组元素保持为默认值（零、1 等），
          调用者需自行确保完整性或后续覆盖。
    """
    # 解析 XML 文件树
    tree = ET.parse(xml_path)
    root = tree.getroot()

    # 查找 <Manipulator> 标签（所有运动学/动力学参数应集中于此）
    manipulator = root.find("Manipulator")
    if manipulator is None:
        raise ValueError(f"No <Manipulator> in {xml_path}")

    # 初始化各数组（长度 6，对应 J1~J6）
    link_mm = np.zeros(6, dtype=float)      # 连杆长度（毫米）
    zero_ofs = np.zeros(6, dtype=float)     # 关节零点偏移（度）
    axis_dir = np.ones(6, dtype=float)      # 关节方向系数（默认为 +1）
    vel_max = np.zeros(6, dtype=float)      # 关节最大速度（度/秒）
    acc_max = np.zeros(6, dtype=float)      # 关节最大加速度（度/秒²）

    # 默认机械臂类型（1 = 标准六轴偏置腕，本项目仅支持此类型）
    mtype = 1
    # 默认基坐标系轴向枚举（来自常量定义，通常为 X=6, Z=1）
    base_x = BASE_CS_X
    base_z = BASE_CS_Z

    # 遍历 <Manipulator> 下所有 <Item> 元素，根据 keyname 提取值
    for item in manipulator.findall("Item"):
        key = item.get("keyname")
        val = item.get("value")
        if key is None or val is None:
            continue   # 跳过无效条目

        if key == "ManipulatorType":
            mtype = int(val)
        elif key == "BaseCsX":
            base_x = int(val)
        elif key == "BaseCsZ":
            base_z = int(val)
        elif key.startswith("LinkLength_"):
            # 格式: "LinkLength_1" ~ "LinkLength_6"
            idx = int(key.split("_")[1])
            if 1 <= idx <= 6:
                link_mm[idx - 1] = float(val)
        elif key.startswith("JointOffset_"):
            # 关节零点偏移（度）
            idx = int(key.split("_")[1])
            if 1 <= idx <= 6:
                zero_ofs[idx - 1] = float(val)
        elif key.startswith("JointDirection_"):
            # 关节方向系数，应为 ±1（XML 中通常为整数 1 或 -1）
            idx = int(key.split("_")[1])
            if 1 <= idx <= 6:
                axis_dir[idx - 1] = float(int(val))
        elif key.startswith("JointVelocityMax_"):
            # 关节最大速度（度/秒），用于轨迹规划约束
            idx = int(key.split("_")[1])
            if 1 <= idx <= 6:
                vel_max[idx - 1] = float(val)
        elif key.startswith("JointAccelerationMax_"):
            # 关节最大加速度（度/秒²）
            idx = int(key.split("_")[1])
            if 1 <= idx <= 6:
                acc_max[idx - 1] = float(val)
        # 其他字段（如凸包尺寸、工具坐标等）本函数忽略，留给高层处理

    # 将连杆长度从毫米转换为米（DH 参数要求国际单位）
    link_m = link_mm / 1000.0

    # 构建基坐标系旋转矩阵：
    #   1) `cs_to_rotation` 根据 XML 定义的轴向枚举（BaseCsX, BaseCsZ）计算出
    #      从“控制器定义坐标系”到“机器人本体系”的旋转。
    #   2) 右乘 `teach_base_rotation_from_install_deg()` 得到安装标定后的最终旋转变换。
    R = cs_to_rotation(base_x, base_z) @ teach_base_rotation_from_install_deg()

    # 组合成 4x4 齐次变换矩阵（平移部分为 0，即基座原点与安装面重合）
    T_base = np.eye(4, dtype=float)
    T_base[:3, :3] = R

    # 组装运动学配置对象
    cfg = KinematicConfig(
        joint_offset_deg=zero_ofs,                # 关节零点偏移（度）
        joint_dir=axis_dir,                       # 关节方向系数
        manipulator_type=mtype,                   # 机械臂类型
        T_base=T_base,                            # 基坐标系变换矩阵
        dh_table=dh_table_from_link_lengths_m(link_m),  # 标准 DH 参数表（根据连杆长度自动生成）
        link_len_m=link_m.copy(),                 # 连杆长度（米，深拷贝避免外部修改）
    )
    return cfg, vel_max, acc_max