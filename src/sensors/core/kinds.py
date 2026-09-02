from __future__ import annotations

from enum import Enum


class SensorKind(str, Enum):
    """Logical kinds aligned with MegaCollect lifecycle."""

    BUS = "bus"
    ARM = "arm"
    ARM_READ = "arm_read"  # Elite EC monitor only — never command
    GELLO = "gello"
    GRIPPER = "gripper"
    REALSENSE = "realsense"
    PIPELINE = "pipeline"
    FT = "ft"
    TACTILE = "tactile"

    @classmethod
    def parse(cls, value: str) -> "SensorKind":
        return cls(str(value).strip().lower())
