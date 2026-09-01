from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Flag, auto
from typing import Any, Mapping

from sensors.core.health import HealthReport
from sensors.core.kinds import SensorKind
from sensors.core.rate_probe import rate_probe_unsupported


class SensorCapability(Flag):
    NONE = 0
    PROBE = auto()
    SAMPLE = auto()
    CONTROL = auto()
    PREVIEW = auto()
    STREAM = auto()
    RATE_PROBE = auto()


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
        self._initialized = False

    @property
    def opened(self) -> bool:
        return self._opened

    @property
    def initialized(self) -> bool:
        return self._initialized

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
        self._initialized = False

    def initialize(self, **options: Any) -> dict[str, Any]:
        """Optional post-``open()`` hardware setup (registers, streams, calibration).

        Default: no-op success. Override on drivers that require an explicit init
        step before ``read()`` / ``write()`` / rate probes (e.g. Modbus gripper).
        """
        if not self._opened:
            raise RuntimeError(f"{self.id}: call open() before initialize()")
        self._initialized = True
        return {
            "ok": True,
            "initialized": True,
            "skipped": True,
            "kind": self.kind.value,
            "dry_run": bool(self.ctx.dry_run),
        }

    def read(self) -> Mapping[str, Any]:
        if not self._opened:
            raise RuntimeError(f"{self.id}: call open() before read()")
        return {}

    def write(self, command: Mapping[str, Any]) -> Mapping[str, Any]:
        """Optional actuator command. Override on CONTROL-capable drivers."""
        if not self._opened:
            raise RuntimeError(f"{self.id}: call open() before write()")
        raise NotImplementedError(f"{self.id} ({self.kind.value}) does not support write()")

    def probe_max_read_hz(self, duration_s: float = 5.0) -> dict[str, Any]:
        """Measure max sustained read throughput using driver-native I/O.

        Default: ``unsupported``. Drivers with ``RATE_PROBE`` must override and
        must not call the public ``read()`` loop for timing.
        """
        _ = duration_s
        if not self._opened:
            raise RuntimeError(f"{self.id}: call open() before probe_max_read_hz()")
        if self.capabilities & SensorCapability.RATE_PROBE:
            raise NotImplementedError(
                f"{self.id} ({self.kind.value}) declares RATE_PROBE but probe_max_read_hz() is not implemented"
            )
        return rate_probe_unsupported(
            self.kind.value,
            reason=f"rate probe not supported for kind={self.kind.value}",
        )

    def probe_max_sync_write_hz(
        self,
        *,
        duration_s: float = 10.0,
        baseline_s: float = 1.5,
        value_min: float = 0.0,
        value_max: float = 1.0,
        hz_min: float = 0.2,
        hz_max: float = 200.0,
    ) -> dict[str, Any]:
        """Optional 1:1 write→read sync probe for CONTROL actuators."""
        _ = duration_s, baseline_s, value_min, value_max, hz_min, hz_max
        if not self._opened:
            raise RuntimeError(f"{self.id}: call open() before probe_max_sync_write_hz()")
        return rate_probe_unsupported(
            self.kind.value,
            reason=f"sync write-rate probe not supported for kind={self.kind.value}",
        )

    def __enter__(self) -> "Sensor":
        self.open()
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()
