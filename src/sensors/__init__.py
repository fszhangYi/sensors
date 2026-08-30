"""hik-sensors: unified MegaCollect-aligned sensor stack."""

from sensors.core.base import Sensor, SensorCapability, SensorContext
from sensors.core.health import CheckResult, HealthReport, HealthStatus
from sensors.core.kinds import SensorKind
from sensors.core.registry import get_sensor_class, list_registered_kinds, register_sensor
from sensors.runtime.manager import SensorManager

__version__ = "0.1.0"

__all__ = [
    "Sensor",
    "SensorCapability",
    "SensorContext",
    "SensorKind",
    "HealthStatus",
    "HealthReport",
    "CheckResult",
    "register_sensor",
    "get_sensor_class",
    "list_registered_kinds",
    "SensorManager",
    "__version__",
]
