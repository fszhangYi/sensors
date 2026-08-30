from __future__ import annotations

from typing import Callable, Type

from sensors.core.base import Sensor
from sensors.core.kinds import SensorKind

_REGISTRY: dict[str, Type[Sensor]] = {}


def register_sensor(kind: str | SensorKind) -> Callable[[Type[Sensor]], Type[Sensor]]:
    key = kind.value if isinstance(kind, SensorKind) else str(kind).lower()

    def decorator(cls: Type[Sensor]) -> Type[Sensor]:
        if not issubclass(cls, Sensor):
            raise TypeError(f"{cls} must subclass Sensor")
        _REGISTRY[key] = cls
        try:
            cls.kind = SensorKind.parse(key)
        except ValueError:
            pass
        return cls

    return decorator


def get_sensor_class(kind: str | SensorKind) -> Type[Sensor]:
    key = kind.value if isinstance(kind, SensorKind) else str(kind).lower()
    if key not in _REGISTRY:
        raise KeyError(f"No sensor registered for kind={key!r}. Known: {sorted(_REGISTRY)}")
    return _REGISTRY[key]


def list_registered_kinds() -> list[str]:
    return sorted(_REGISTRY)


def clear_registry() -> None:
    _REGISTRY.clear()
