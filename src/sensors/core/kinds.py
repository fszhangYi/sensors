from __future__ import annotations

from enum import Enum


class SensorKind(str, Enum):
    """Logical kinds aligned with MegaCollect lifecycle."""

    BUS = "bus"
    ARM = "arm"
    ARM_READ = "arm_read"  # Elite EC monitor only — never command
    ARM_WRITE = "arm_write"  # Elite EC TT stream — gated by arm/disarm
    ARM_CS_READ = "arm_cs_read"  # Elite CS Primary/RTSI read — never command
    ARM_CS_WRITE = "arm_cs_write"  # Elite CS EliteDriver servoj — gated by arm/disarm
    GELLO = "gello"
    GRIPPER = "gripper"  # DH AG95 Modbus
    GRIPPER_HUILING = "gripper_huiling"  # 慧灵 Z-EFG-C65 Modbus
    REALSENSE = "realsense"
    PIPELINE = "pipeline"
    FT = "ft"
    TACTILE = "tactile"

    @classmethod
    def parse(cls, value: str) -> "SensorKind":
        return cls(str(value).strip().lower())
