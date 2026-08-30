from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class HealthStatus(str, Enum):
    OK = "ok"
    WARN = "warn"
    ERROR = "error"
    OFFLINE = "offline"
    UNKNOWN = "unknown"
    UNSUPPORTED = "unsupported"


@dataclass
class CheckResult:
    name: str
    ok: bool
    detail: str = ""
    critical: bool = False

    def to_dict(self) -> dict[str, Any]:
        return {"name": self.name, "ok": self.ok, "detail": self.detail, "critical": self.critical}


@dataclass
class HealthReport:
    sensor_id: str
    kind: str
    status: HealthStatus
    message: str = ""
    checks: list[CheckResult] = field(default_factory=list)
    metrics: dict[str, Any] = field(default_factory=dict)
    hints: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.sensor_id,
            "kind": self.kind,
            "status": self.status.value,
            "message": self.message,
            "checks": [c.to_dict() for c in self.checks],
            "metrics": self.metrics,
            "hints": self.hints,
        }

    @staticmethod
    def aggregate_status(checks: list[CheckResult]) -> HealthStatus:
        if not checks:
            return HealthStatus.UNKNOWN
        if any((not c.ok) and c.critical for c in checks):
            return HealthStatus.ERROR
        if any(not c.ok for c in checks):
            return HealthStatus.WARN
        return HealthStatus.OK
