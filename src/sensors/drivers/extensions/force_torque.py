"""Wrist six-axis force/torque (F/T).

Canonical data shape follows HIK LMM (`HIK_LMM_FORCE_TORQUE` in hik_lmm_lib.h):

    force[3]   # Fx, Fy, Fz  (N)
    torque[3]  # Tx, Ty, Tz  (N·m)

This is **not** finger tactile (Paxini / `HIK_LMM_FINGER_TACTILE`) and **not**
DH AG95 `target_force` (gripper command). Those belong on tactile / gripper.

Reference confusion to avoid (`dh_ag95_force.py`):
- that script drives a gripper with Paxini `get_force()` contact scalar
- it must not be wired into this F/T driver
"""

from __future__ import annotations

import math
import time
from typing import Any, Mapping, Sequence

from sensors.core.base import Sensor, SensorCapability, SensorContext
from sensors.core.health import CheckResult, HealthReport, HealthStatus
from sensors.core.kinds import SensorKind
from sensors.core.registry import register_sensor

FORCE_LABELS = ("Fx", "Fy", "Fz")
TORQUE_LABELS = ("Tx", "Ty", "Tz")
WRENCH_LABELS = FORCE_LABELS + TORQUE_LABELS


def _as6(values: Sequence[Any] | None, default: float = 0.0) -> list[float]:
    out: list[float] = []
    src = list(values or [])
    for i in range(6):
        try:
            out.append(float(src[i]))
        except (IndexError, TypeError, ValueError):
            out.append(default)
    return out


@register_sensor(SensorKind.FT)
class ForceTorqueSensor(Sensor):
    """Wrist F/T: force[3] + torque[3], optional tare bias."""

    kind = SensorKind.FT
    capabilities = SensorCapability.PROBE | SensorCapability.SAMPLE | SensorCapability.CONTROL

    def __init__(self, sensor_id: str, config: Mapping[str, Any], ctx: SensorContext | None = None):
        super().__init__(sensor_id, config, ctx)
        self.endpoint = str(config.get("endpoint") or "—")
        self.backend = str(config.get("backend") or "stub").strip().lower()  # stub | (future)
        self._bias = _as6(config.get("bias"))
        self._tick = 0
        self._last_raw = [0.0] * 6

    def _synthetic_raw(self) -> list[float]:
        t = self._tick * 0.18
        return [
            0.8 * math.sin(t * 0.9),
            0.6 * math.cos(t * 1.1),
            2.0 + 0.5 * math.sin(t * 0.55),
            0.05 * math.sin(t * 1.3 + 0.2),
            0.04 * math.cos(t * 1.05),
            0.03 * math.sin(t * 0.7),
        ]

    def _apply_bias(self, raw: Sequence[float]) -> list[float]:
        r = _as6(raw)
        return [r[i] - self._bias[i] for i in range(6)]

    def probe(self) -> HealthReport:
        checks: list[CheckResult] = []
        metrics: dict[str, Any] = {
            "endpoint": self.endpoint,
            "backend": self.backend,
            "bias": list(self._bias),
            "force_labels": list(FORCE_LABELS),
            "torque_labels": list(TORQUE_LABELS),
            "schema": "HIK_LMM_FORCE_TORQUE",
            "note": "Wrist F/T ≠ Paxini tactile ≠ DH gripper target_force",
        }

        if self.ctx.dry_run:
            checks.append(CheckResult("dry_run", True, "synthetic force[3]+torque[3]"))
            return HealthReport(
                sensor_id=self.id,
                kind=self.kind.value,
                status=HealthStatus.OK,
                message="dry-run wrist F/T ready (Fx..Tz)",
                checks=checks,
                metrics=metrics,
                hints=[
                    "Live path not on MegaCollect main loop yet",
                    "Do not use dh_ag95_force / Paxini here",
                ],
            )

        # Live: no MegaCollect main-path driver yet — honest offline/warn.
        checks.append(
            CheckResult(
                "backend",
                False,
                f"backend={self.backend} (live F/T driver not wired)",
                critical=True,
            )
        )
        return HealthReport(
            sensor_id=self.id,
            kind=self.kind.value,
            status=HealthStatus.UNSUPPORTED,
            message="Wrist F/T not on MegaCollect main path; use dry_run for preview",
            checks=checks,
            metrics=metrics,
            hints=[
                "Confirm hardware F/T install + calibration",
                "Distinguish from gripper target_force and finger tactile",
            ],
        )

    def open(self) -> None:
        self._tick = 0
        if not self.ctx.dry_run:
            raise RuntimeError(
                f"{self.id}: live wrist F/T backend not implemented "
                f"(backend={self.backend}); use dry_run=True"
            )
        super().open()

    def read(self) -> Mapping[str, Any]:
        if not self._opened:
            raise RuntimeError(f"{self.id}: call open() before read()")
        self._tick += 1
        raw = self._synthetic_raw() if self.ctx.dry_run else list(self._last_raw)
        self._last_raw = list(raw)
        wrench = self._apply_bias(raw)
        force = wrench[:3]
        torque = wrench[3:]
        return {
            "ts": time.time(),
            "dry_run": bool(self.ctx.dry_run),
            # HIK_LMM_FORCE_TORQUE shaped fields
            "force": force,
            "torque": torque,
            # convenience
            "wrench": wrench,
            "wrench_raw": list(raw),
            "wrench_labels": list(WRENCH_LABELS),
            "force_labels": list(FORCE_LABELS),
            "torque_labels": list(TORQUE_LABELS),
            "bias": list(self._bias),
            "force_norm": math.sqrt(sum(v * v for v in force)),
            "torque_norm": math.sqrt(sum(v * v for v in torque)),
            "endpoint": self.endpoint,
            "backend": self.backend,
        }

    def write(self, command: Mapping[str, Any]) -> Mapping[str, Any]:
        """CONTROL: tare / set bias. Does not move the arm."""
        if not self._opened:
            raise RuntimeError(f"{self.id}: call open() before write()")
        if command.get("tare") or command.get("zero"):
            # Bias so current raw reads as ~0 (HIK-style zeroing / 偏置)
            self._bias = list(self._last_raw)
            return {"ok": True, "action": "tare", "bias": list(self._bias)}
        if "bias" in command:
            self._bias = _as6(command.get("bias"))
            return {"ok": True, "action": "set_bias", "bias": list(self._bias)}
        raise ValueError(f"{self.id}: unsupported F/T command keys {list(command)}")
