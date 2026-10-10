from __future__ import annotations

"""Elite CS arm — **read-only** joint state via RTSI / Primary.

Distinct from EC ``arm_read`` (``elirobots`` / monitor 8056).

Safety:
  - May connect RTSI (30004) or probe Primary (30001).
  - Must NEVER call ``writeServoj`` / motion APIs.
  - ``write()`` always raises.
"""

import math
import socket
import time
from typing import Any, Mapping

from sensors.core.base import Sensor, SensorCapability, SensorContext
from sensors.core.health import CheckResult, HealthReport, HealthStatus
from sensors.core.kinds import SensorKind
from sensors.core.registry import register_sensor
from sensors.drivers.arm._cs_sdk import (
    close_cs_rtsi,
    import_elite_cs_sdk,
    open_cs_rtsi,
    read_joints_rad_from_rtsi,
)

_DEFAULT_PRIMARY_PORT = 30001


@register_sensor(SensorKind.ARM_CS_READ)
class EliteCsArmReadSensor(Sensor):
    """Read-only Elite CS joints (RTSI preferred; dry_run without SDK)."""

    kind = SensorKind.ARM_CS_READ
    capabilities = (
        SensorCapability.PROBE
        | SensorCapability.SAMPLE
        | SensorCapability.RATE_PROBE
    )

    def __init__(self, sensor_id: str, config: Mapping[str, Any], ctx: SensorContext | None = None):
        super().__init__(sensor_id, config, ctx)
        self.robot_ip = str(
            config.get("robot_ip") or config.get("endpoint") or config.get("ip") or "10.111.34.201"
        )
        # CS Primary (not EC 54321). Kept as robot_tcp_port for UI/probe compatibility.
        self.robot_tcp_port = int(
            config.get("primary_port") or config.get("robot_tcp_port") or _DEFAULT_PRIMARY_PORT
        )
        self.num_joints = int(config.get("num_joints", 6))
        self.connect_timeout_s = float(config.get("connect_timeout_s", 0.4))
        self.rtsi_output_recipe = str(config.get("rtsi_output_recipe") or "")
        self.rtsi_input_recipe = str(config.get("rtsi_input_recipe") or "")
        self.rtsi_frequency = float(config.get("rtsi_frequency", 125.0))
        self._rtsi: Any = None

    @property
    def endpoint(self) -> str:
        return f"{self.robot_ip}:{self.robot_tcp_port}"

    @staticmethod
    def _tcp_open(host: str, port: int, timeout: float = 0.4) -> bool:
        try:
            with socket.create_connection((host, port), timeout=timeout):
                return True
        except OSError:
            return False

    def probe(self) -> HealthReport:
        checks: list[CheckResult] = []
        metrics: dict[str, Any] = {
            "robot_ip": self.robot_ip,
            "robot_tcp_port": self.robot_tcp_port,
            "primary_port": self.robot_tcp_port,
            "num_joints": self.num_joints,
            "backend": "elite_cs",
            "mode": "read_only",
        }
        primary_ok = self._tcp_open(
            self.robot_ip, self.robot_tcp_port, timeout=self.connect_timeout_s
        )
        checks.append(
            CheckResult(
                "cs_primary",
                primary_ok,
                f"{self.endpoint} {'open' if primary_ok else 'closed'}",
            )
        )
        try:
            import_elite_cs_sdk()
            metrics["elite_cs_sdk"] = True
        except ImportError:
            metrics["elite_cs_sdk"] = False
            checks.append(
                CheckResult(
                    "elite_cs_sdk",
                    False,
                    "elite_cs_sdk missing (CS series; not elirobots)",
                )
            )
        if self.ctx.dry_run:
            status = HealthStatus.OK
            message = f"arm_cs_read dry_run ({self.robot_ip})"
        elif primary_ok:
            status = HealthStatus.OK if metrics.get("elite_cs_sdk") else HealthStatus.WARN
            message = f"arm_cs_read @ {self.robot_ip}"
        else:
            status = HealthStatus.OFFLINE
            message = f"arm_cs_read unreachable {self.endpoint}"
        return HealthReport(
            sensor_id=self.id,
            kind=self.kind.value,
            status=status,
            message=message,
            checks=checks,
            metrics=metrics,
            hints=[
                "CS series only — use arm_read/arm_write for Elite EC",
                "Primary port default 30001; RTSI 30004 via rtsi_output_recipe",
                "Install elite_cs_sdk from Elite_Robots_CS_SDK_Python",
            ],
        )

    def open(self) -> None:
        if self.ctx.dry_run:
            self._opened = True
            self._rtsi = None
            return
        if not str(self.rtsi_output_recipe).strip():
            raise RuntimeError(
                f"{self.id}: arm_cs_read live mode requires params.rtsi_output_recipe "
                "(RTSI output recipe file path). Or set dry_run=true."
            )
        self._rtsi = open_cs_rtsi(
            robot_ip=self.robot_ip,
            output_recipe_file=self.rtsi_output_recipe,
            input_recipe_file=self.rtsi_input_recipe,
            frequency=self.rtsi_frequency,
        )
        self._opened = True

    def close(self) -> None:
        close_cs_rtsi(self._rtsi)
        self._rtsi = None
        self._opened = False
        self._initialized = False

    def read(self) -> Mapping[str, Any]:
        super().read()
        ts = time.time()
        if self.ctx.dry_run or self._rtsi is None:
            return {
                "joints_rad": None,
                "joints_deg": None,
                "num_joints": self.num_joints,
                "robot_ip": self.robot_ip,
                "endpoint": self.endpoint,
                "backend": "elite_cs",
                "mode": "read_only",
                "dry_run": True,
                "ts": ts,
            }
        joints_rad = read_joints_rad_from_rtsi(self._rtsi, n=self.num_joints)
        return {
            "joints_rad": joints_rad,
            "joints_deg": [math.degrees(x) for x in joints_rad],
            "num_joints": self.num_joints,
            "robot_ip": self.robot_ip,
            "endpoint": self.endpoint,
            "backend": "elite_cs",
            "mode": "read_only",
            "dry_run": False,
            "ts": ts,
        }

    def write(self, command: Mapping[str, Any]) -> Mapping[str, Any]:
        raise RuntimeError(
            f"{self.id}: arm_cs_read is read-only; write/command is forbidden "
            f"(refused keys={list(command.keys()) if command else []})"
        )

    def probe_max_read_hz(self, duration_s: float = 5.0) -> dict[str, Any]:
        from sensors.core.rate_probe import clamp_duration, rate_probe_fail, rate_probe_ok

        if not self._opened:
            raise RuntimeError(f"{self.id}: call open() before probe_max_read_hz()")
        duration_s = clamp_duration(duration_s)
        times: list[float] = []
        errors = 0
        last_error: str | None = None
        t0 = time.perf_counter()
        deadline = t0 + duration_s
        dry = bool(self.ctx.dry_run or self._rtsi is None)
        while time.perf_counter() < deadline:
            try:
                if not dry:
                    read_joints_rad_from_rtsi(self._rtsi, n=self.num_joints)
                times.append(time.perf_counter())
            except Exception as e:  # noqa: BLE001
                errors += 1
                last_error = str(e)
                if errors >= 8 and len(times) < 3:
                    break
        if len(times) < 3:
            return rate_probe_fail(
                error=last_error or "arm_cs_read probe produced too few samples",
                method="elite_cs_rtsi_actual_q",
                samples=len(times),
                errors=errors,
                duration_s=time.perf_counter() - t0,
                dry_run=dry,
                last_error=last_error,
            )
        return rate_probe_ok(
            times,
            method="elite_cs_rtsi_actual_q",
            errors=errors,
            t0=t0,
            dry_run=dry,
            last_error=last_error,
        )
