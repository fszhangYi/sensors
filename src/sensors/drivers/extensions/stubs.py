from __future__ import annotations

from typing import Any, Mapping

from sensors.core.base import Sensor, SensorCapability, SensorContext
from sensors.core.health import HealthReport, HealthStatus
from sensors.core.kinds import SensorKind
from sensors.core.registry import register_sensor


@register_sensor(SensorKind.FT)
class ForceTorqueSensor(Sensor):
    """Wrist F/T — NOT on MegaCollect main path; extension stub."""

    kind = SensorKind.FT
    capabilities = SensorCapability.PROBE

    def __init__(self, sensor_id: str, config: Mapping[str, Any], ctx: SensorContext | None = None):
        super().__init__(sensor_id, config, ctx)
        self.endpoint = str(config.get("endpoint") or "—")

    def probe(self) -> HealthReport:
        return HealthReport(
            sensor_id=self.id,
            kind=self.kind.value,
            status=HealthStatus.UNSUPPORTED,
            message="F/T not wired in MegaCollect main collect loop",
            metrics={"endpoint": self.endpoint},
            hints=[
                "Do not confuse with DH gripper target_force",
                "Implement read() when hardware + driver are available",
            ],
        )


@register_sensor(SensorKind.TACTILE)
class TactileSensor(Sensor):
    """Tactile array — extension stub."""

    kind = SensorKind.TACTILE
    capabilities = SensorCapability.PROBE

    def probe(self) -> HealthReport:
        return HealthReport(
            sensor_id=self.id,
            kind=self.kind.value,
            status=HealthStatus.UNSUPPORTED,
            message="Tactile not on MegaCollect main path",
            hints=["Reserved for future array / gel sensors"],
        )
