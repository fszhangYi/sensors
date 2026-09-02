from __future__ import annotations

"""Elite EC arm — **read-only** joint state via network monitor.

Safety contract (DCS / collection):
  - May connect and start the Elite **monitor** thread to receive state.
  - Must NEVER call motion / servo / IO / trajectory APIs that command the arm.
  - ``write()`` always raises.

Allowed (state path only): ``EC(..., auto_connect=True)``, ``monitor_thread_run``,
``monitor_thread_stop``, reading ``monitor_info.machinePos``.

Forbidden examples (non-exhaustive): ``stop``, ``robot_servo_on``, ``set_servo_status``,
``TT_init``, ``TT_add_joint``, ``move_joint``, ``wait_stop``, ``set_digital_io``,
``freedriveMode``, any gripper command.
"""

import math
import socket
import time
from typing import Any, Mapping

from sensors.core.base import Sensor, SensorCapability, SensorContext
from sensors.core.health import CheckResult, HealthReport, HealthStatus
from sensors.core.kinds import SensorKind
from sensors.core.registry import register_sensor


@register_sensor(SensorKind.ARM_READ)
class EliteArmReadSensor(Sensor):
    """Read-only Elite follower joints over Ethernet (``elite.EC`` monitor)."""

    kind = SensorKind.ARM_READ
    capabilities = (
        SensorCapability.PROBE
        | SensorCapability.SAMPLE
        | SensorCapability.RATE_PROBE
        # intentionally no CONTROL
    )

    def __init__(self, sensor_id: str, config: Mapping[str, Any], ctx: SensorContext | None = None):
        super().__init__(sensor_id, config, ctx)
        self.robot_ip = str(
            config.get("robot_ip") or config.get("endpoint") or config.get("ip") or "10.111.34.200"
        )
        self.robot_tcp_port = int(config.get("robot_tcp_port", 54321))
        self.num_joints = int(config.get("num_joints", 6))
        self.monitor_wait_s = float(config.get("monitor_wait_s", 10.0))
        self.connect_timeout_s = float(config.get("connect_timeout_s", 0.4))
        self._robot: Any = None

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
            "num_joints": self.num_joints,
            "mode": "read_only",
        }

        robot_ok = self._tcp_open(self.robot_ip, self.robot_tcp_port, timeout=self.connect_timeout_s)
        checks.append(
            CheckResult(
                "robot_tcp",
                robot_ok,
                f"{self.endpoint} {'open' if robot_ok else 'closed'}",
            )
        )

        try:
            import elite  # noqa: F401

            metrics["elite_sdk"] = True
        except ImportError:
            metrics["elite_sdk"] = False
            checks.append(
                CheckResult(
                    "elite_sdk",
                    False,
                    "elite module missing from desktop bundle (rebuild with elirobots)",
                )
            )

        if self.ctx.dry_run:
            status = HealthStatus.OK
            message = f"arm_read dry_run ({self.robot_ip})"
        elif robot_ok:
            status = HealthStatus.OK if metrics.get("elite_sdk") else HealthStatus.WARN
            message = f"arm_read monitor-only @ {self.robot_ip}"
        else:
            status = HealthStatus.OFFLINE
            message = f"arm_read unreachable {self.endpoint}"

        return HealthReport(
            sensor_id=self.id,
            kind=self.kind.value,
            status=status,
            message=message,
            checks=checks,
            metrics=metrics,
            hints=[
                "READ-ONLY: never commands the arm (no servo/TT/move/IO)",
                "Requires Elite EC SDK on PYTHONPATH: from elite import EC",
                f"Set params.robot_ip or endpoint to controller IP (now {self.robot_ip})",
            ],
        )

    def open(self) -> None:
        if self.ctx.dry_run:
            self._opened = True
            self._robot = None
            return
        try:
            from elite import EC
        except ImportError as e:
            raise RuntimeError(
                "elite SDK required for arm_read open() when dry_run=false. "
                "Desktop bundle should include elirobots (import elite); rebuild sensors-dcs desktop, "
                "or keep dry_run=true in config."
            ) from e

        robot = EC(ip=self.robot_ip, auto_connect=True)
        # Start state monitor only — do not servo-on / TT_init / stop PLAY.
        if not hasattr(robot, "monitor_thread_run"):
            raise RuntimeError("elite.EC missing monitor_thread_run (cannot read safely)")
        robot.monitor_thread_run()

        deadline = time.perf_counter() + max(0.5, self.monitor_wait_s)
        while time.perf_counter() < deadline:
            pos = getattr(getattr(robot, "monitor_info", None), "machinePos", None)
            if pos is not None and len(pos) >= self.num_joints and pos[0] is not None:
                break
            time.sleep(0.05)
        else:
            try:
                robot.monitor_thread_stop()
            except Exception:  # noqa: BLE001
                pass
            raise TimeoutError(
                f"{self.id}: monitor_info.machinePos not ready within {self.monitor_wait_s}s "
                f"(ip={self.robot_ip})"
            )

        self._robot = robot
        self._opened = True

    def close(self) -> None:
        robot = self._robot
        self._robot = None
        if robot is not None:
            try:
                robot.monitor_thread_stop()
            except Exception:  # noqa: BLE001
                pass
            # Prefer leaving controller state alone — do not servo_off / stop.
        self._opened = False
        self._initialized = False

    def _read_joints_rad(self) -> list[float]:
        robot = self._robot
        if robot is None:
            raise RuntimeError(f"{self.id}: not open")
        pos = getattr(getattr(robot, "monitor_info", None), "machinePos", None)
        if pos is None or len(pos) < self.num_joints:
            raise RuntimeError(f"{self.id}: monitor_info.machinePos unavailable")
        out: list[float] = []
        for i in range(self.num_joints):
            val = pos[i]
            if val is None:
                raise RuntimeError(f"{self.id}: machinePos[{i}] is None")
            out.append(math.radians(float(val)))
        return out

    def read(self) -> Mapping[str, Any]:
        super().read()
        ts = time.time()
        if self.ctx.dry_run or self._robot is None:
            return {
                "joints_rad": None,
                "joints_deg": None,
                "num_joints": self.num_joints,
                "robot_ip": self.robot_ip,
                "endpoint": self.endpoint,
                "backend": "elite_ec",
                "mode": "read_only",
                "dry_run": True,
                "ts": ts,
            }
        joints_rad = self._read_joints_rad()
        joints_deg = [math.degrees(x) for x in joints_rad]
        return {
            "joints_rad": joints_rad,
            "joints_deg": joints_deg,
            "num_joints": self.num_joints,
            "robot_ip": self.robot_ip,
            "endpoint": self.endpoint,
            "backend": "elite_ec",
            "mode": "read_only",
            "dry_run": False,
            "ts": ts,
        }

    def write(self, command: Mapping[str, Any]) -> Mapping[str, Any]:
        raise RuntimeError(
            f"{self.id}: arm_read is read-only; write/command is forbidden "
            f"(refused keys={list(command.keys()) if command else []})"
        )

    def _probe_joint_state_once(self) -> None:
        if self.ctx.dry_run or self._robot is None:
            return
        self._read_joints_rad()

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
        dry = bool(self.ctx.dry_run or self._robot is None)
        while time.perf_counter() < deadline:
            try:
                self._probe_joint_state_once()
                times.append(time.perf_counter())
            except Exception as e:  # noqa: BLE001
                errors += 1
                last_error = str(e)
                if errors >= 8 and len(times) < 3:
                    break
        if len(times) < 3:
            return rate_probe_fail(
                error=last_error or "arm_read probe produced too few samples",
                method="elite_monitor_machinePos",
                samples=len(times),
                errors=errors,
                duration_s=time.perf_counter() - t0,
                dry_run=dry,
                last_error=last_error,
            )
        return rate_probe_ok(
            times,
            method="elite_monitor_machinePos",
            errors=errors,
            t0=t0,
            dry_run=dry,
            last_error=last_error,
        )
