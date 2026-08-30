from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Flag, auto
from typing import Any, Mapping

from sensors.core.health import HealthReport
from sensors.core.kinds import SensorKind


class SensorCapability(Flag):
    NONE = 0
    PROBE = auto()
    SAMPLE = auto()
    CONTROL = auto()
    PREVIEW = auto()
    STREAM = auto()


@dataclass
class SensorContext:
    dry_run: bool = False
    extras: dict[str, Any] = field(default_factory=dict)


class Sensor(ABC):
    """Uniform lifecycle for every device."""

    kind: SensorKind
    capabilities: SensorCapability = SensorCapability.PROBE

    def __init__(self, sensor_id: str, config: Mapping[str, Any], ctx: SensorContext | None = None):
        self.id = sensor_id
        self.config = dict(config)
        self.ctx = ctx or SensorContext()
        self._opened = False

    @property
    def opened(self) -> bool:
        return self._opened

    @property
    def name(self) -> str:
        return str(self.config.get("name") or self.id)

    def configure(self, **kwargs: Any) -> None:
        self.config.update(kwargs)

    @abstractmethod
    def probe(self) -> HealthReport:
        """Read-only health check — must not move hardware or grab exclusive streams."""

    def open(self) -> None:
        self._opened = True

    def close(self) -> None:
        self._opened = False

    def read(self) -> Mapping[str, Any]:
        if not self._opened:
            raise RuntimeError(f"{self.id}: call open() before read()")
        return {}

    def write(self, command: Mapping[str, Any]) -> Mapping[str, Any]:
        """Optional actuator command. Override on CONTROL-capable drivers."""
        if not self._opened:
            raise RuntimeError(f"{self.id}: call open() before write()")
        raise NotImplementedError(f"{self.id} ({self.kind.value}) does not support write()")

    def __enter__(self) -> "Sensor":
        self.open()
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()
