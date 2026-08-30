from __future__ import annotations

from pathlib import Path
from typing import Any, Iterable, Iterator

from sensors.core.base import Sensor, SensorContext
from sensors.core.config import BundleConfig, device_to_sensor_config, load_bundle_config
from sensors.core.health import HealthReport
from sensors.core.registry import get_sensor_class
from sensors.drivers import load_all_drivers


class SensorManager:
    """Config-driven factory + lifecycle for a site sensor bundle."""

    def __init__(self, bundle: BundleConfig, *, dry_run: bool | None = None):
        load_all_drivers()
        self.bundle = bundle
        self.ctx = SensorContext(dry_run=bundle.dry_run if dry_run is None else dry_run)
        self._sensors: dict[str, Sensor] = {}
        self._build()

    @classmethod
    def from_yaml(cls, path: str | Path, *, dry_run: bool | None = None) -> "SensorManager":
        return cls(load_bundle_config(path), dry_run=dry_run)

    def _build(self) -> None:
        self._sensors.clear()
        for dev in self.bundle.devices:
            if not dev.enabled:
                continue
            cls = get_sensor_class(dev.kind)
            cfg = device_to_sensor_config(dev, self.bundle.defaults.get(dev.kind, {}))
            self._sensors[dev.id] = cls(dev.id, cfg, self.ctx)

    def __iter__(self) -> Iterator[Sensor]:
        return iter(self._sensors.values())

    def __len__(self) -> int:
        return len(self._sensors)

    def get(self, sensor_id: str) -> Sensor:
        return self._sensors[sensor_id]

    def ids(self) -> list[str]:
        return list(self._sensors)

    def by_kind(self, kind: str) -> list[Sensor]:
        k = kind.lower()
        return [s for s in self._sensors.values() if s.kind.value == k]

    def probe_all(self, only: Iterable[str] | None = None) -> list[HealthReport]:
        wanted = set(only) if only else None
        reports: list[HealthReport] = []
        for sid, sensor in self._sensors.items():
            if wanted is not None and sid not in wanted:
                continue
            reports.append(sensor.probe())
        return reports

    def open_all(self) -> None:
        for s in self._sensors.values():
            s.open()

    def close_all(self) -> None:
        for s in self._sensors.values():
            try:
                s.close()
            except Exception:  # noqa: BLE001
                pass

    def read(self, sensor_id: str) -> dict[str, Any]:
        """Open → read → close a single device (caller owns longer sessions via get())."""
        sensor = self.get(sensor_id)
        sensor.open()
        try:
            sample = dict(sensor.read())
        finally:
            sensor.close()
        return sample

    def summary(self) -> dict[str, Any]:
        reports = self.probe_all()
        return {
            "site": self.bundle.site,
            "dry_run": self.ctx.dry_run,
            "count": len(reports),
            "devices": [r.to_dict() for r in reports],
        }
