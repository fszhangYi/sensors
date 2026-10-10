from __future__ import annotations

"""Elite CS arm — **gated write** via ``EliteDriver.writeServoj``.

Distinct from EC ``arm_write`` (``elirobots`` / ``TT_add_joint`` / ports 54321+8056).

Safety contract (same UI shape as EC arm_write):
  - ``open()`` connects EliteDriver (+ optional RTSI). Does NOT enable streaming alone.
  - Motion requires explicit ``arm`` / ``initialize`` → ``armed``.
  - ``write(joints_rad|jog)`` rejects unless armed (except stop/disarm/arm).
  - Soft teach limits + per-command max |Δq| before ``writeServoj``.
  - Joint units to SDK are **radians** (EC TT path uses degrees).

Reference: Elite_Robots_CS_SDK_Python EliteDriver.writeServoj.
"""

import math
import socket
import threading
import time
from typing import Any, Mapping, Sequence

from sensors.core.base import Sensor, SensorCapability, SensorContext
from sensors.core.config import coerce_bool
from sensors.core.health import CheckResult, HealthReport, HealthStatus
from sensors.core.kinds import SensorKind
from sensors.core.registry import register_sensor
from sensors.drivers.arm._cs_sdk import (
    close_cs_driver,
    close_cs_rtsi,
    import_elite_cs_sdk,
    open_cs_driver,
    open_cs_rtsi,
    read_joints_rad_from_rtsi,
)

_DEFAULT_MAX_DELTA_DEG = 2.0
_DEFAULT_PRIMARY_PORT = 30001


def _finite_list(values: Sequence[Any], *, n: int) -> list[float]:
    if len(values) < n:
        raise ValueError(f"need {n} values, got {len(values)}")
    out: list[float] = []
    for i, v in enumerate(values[:n]):
        x = float(v)
        if not math.isfinite(x):
            raise ValueError(f"value[{i}] not finite: {v!r}")
        out.append(x)
    return out


@register_sensor(SensorKind.ARM_CS_WRITE)
class EliteCsArmWriteSensor(Sensor):
    """Elite CS follower joints with arm/disarm gate and servoj streaming."""

    kind = SensorKind.ARM_CS_WRITE
    capabilities = (
        SensorCapability.PROBE
        | SensorCapability.SAMPLE
        | SensorCapability.CONTROL
        | SensorCapability.RATE_PROBE
    )

    def __init__(self, sensor_id: str, config: Mapping[str, Any], ctx: SensorContext | None = None):
        super().__init__(sensor_id, config, ctx)
        self.robot_ip = str(
            config.get("robot_ip") or config.get("endpoint") or config.get("ip") or "10.111.34.201"
        )
        self.robot_tcp_port = int(
            config.get("primary_port") or config.get("robot_tcp_port") or _DEFAULT_PRIMARY_PORT
        )
        self.num_joints = int(config.get("num_joints", 6))
        self.connect_timeout_s = float(config.get("connect_timeout_s", 0.4))
        self.headless_mode = coerce_bool(config.get("headless_mode"), True)
        self.local_ip = str(config.get("local_ip") or "")
        self.script_file_path = str(config.get("script_file_path") or "")
        self.servoj_time = float(config.get("servoj_time", 0.008))
        self.servoj_lookahead_time = float(config.get("servoj_lookahead_time", 0.1))
        self.servoj_gain = int(config.get("servoj_gain", 300))
        self.stopj_acc = float(config.get("stopj_acc", 8.0))
        self.servoj_timeout_ms = int(config.get("servoj_timeout_ms", 100))
        self.rtsi_output_recipe = str(config.get("rtsi_output_recipe") or "")
        self.rtsi_input_recipe = str(config.get("rtsi_input_recipe") or "")
        self.rtsi_frequency = float(config.get("rtsi_frequency", 125.0))
        max_delta_deg = float(config.get("max_delta_deg", _DEFAULT_MAX_DELTA_DEG))
        self.max_delta_rad = abs(math.radians(max_delta_deg))
        self.enforce_soft_limits = coerce_bool(config.get("enforce_soft_limits"), True)
        self._driver: Any = None
        self._rtsi: Any = None
        self._armed = False
        self._io_lock = threading.RLock()
        self._last_cmd_rad: list[float] | None = None

    @property
    def endpoint(self) -> str:
        return f"{self.robot_ip}:{self.robot_tcp_port}"

    @property
    def armed(self) -> bool:
        return self._armed

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
            "mode": "write_gated",
            "max_delta_deg": math.degrees(self.max_delta_rad),
            "armed": self._armed,
            "headless_mode": self.headless_mode,
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
                    "elite_cs_sdk missing (CS series; not elirobots/EC)",
                )
            )
        if self.ctx.dry_run:
            status = HealthStatus.OK
            message = f"arm_cs_write dry_run ({self.robot_ip})"
        elif primary_ok:
            status = HealthStatus.OK if metrics.get("elite_cs_sdk") else HealthStatus.WARN
            message = f"arm_cs_write gated @ {self.robot_ip}"
        else:
            status = HealthStatus.OFFLINE
            message = f"arm_cs_write unreachable {self.endpoint}"
        return HealthReport(
            sensor_id=self.id,
            kind=self.kind.value,
            status=status,
            message=message,
            checks=checks,
            metrics=metrics,
            hints=[
                "open() = EliteDriver (+ optional RTSI); call arm before writeServoj",
                "UI: Arm → ± jog / abs ramp; Disarm/Estop to release",
                f"max_delta_deg={math.degrees(self.max_delta_rad):.3g}",
                "Do not use arm_write (EC TT) on CS66 — wrong SDK/ports",
            ],
        )

    def open(self) -> None:
        if self._opened:
            return
        if self.ctx.dry_run:
            self._opened = True
            self._driver = None
            self._rtsi = None
            self._armed = False
            return
        try:
            self._driver = open_cs_driver(
                robot_ip=self.robot_ip,
                headless_mode=self.headless_mode,
                local_ip=self.local_ip,
                script_file_path=self.script_file_path,
                servoj_time=self.servoj_time,
                servoj_lookahead_time=self.servoj_lookahead_time,
                servoj_gain=self.servoj_gain,
                stopj_acc=self.stopj_acc,
            )
            if str(self.rtsi_output_recipe).strip():
                self._rtsi = open_cs_rtsi(
                    robot_ip=self.robot_ip,
                    output_recipe_file=self.rtsi_output_recipe,
                    input_recipe_file=self.rtsi_input_recipe,
                    frequency=self.rtsi_frequency,
                )
        except ImportError as e:
            raise RuntimeError(
                "elite_cs_sdk required for arm_cs_write open() when dry_run=false. "
                "Install Elite_Robots_CS_SDK_Python (not elirobots)."
            ) from e
        self._opened = True
        self._armed = False
        self._initialized = False

    def close(self) -> None:
        with self._io_lock:
            try:
                if self._armed:
                    self._disarm_unlocked(stop=True)
            except Exception:  # noqa: BLE001
                pass
            close_cs_rtsi(self._rtsi)
            self._rtsi = None
            close_cs_driver(self._driver)
            self._driver = None
            self._armed = False
            self._initialized = False
            self._opened = False

    def initialize(self, **options: Any) -> dict[str, Any]:
        return self.arm(**options)

    def arm(self, **options: Any) -> dict[str, Any]:
        del options  # CS path has no TT_init knobs; kept for API parity with EC.
        if not self._opened:
            raise RuntimeError(f"{self.id}: call open() before arm()")
        with self._io_lock:
            if self.ctx.dry_run:
                self._armed = True
                self._initialized = True
                if self._last_cmd_rad is None:
                    self._last_cmd_rad = [0.0] * self.num_joints
                return {
                    "ok": True,
                    "armed": True,
                    "initialized": True,
                    "dry_run": True,
                    "kind": self.kind.value,
                    "backend": "elite_cs",
                }
            if self._driver is None:
                return {"ok": False, "error": "CS driver not connected", "armed": False}
            if hasattr(self._driver, "isRobotConnected") and not self._driver.isRobotConnected():
                if hasattr(self._driver, "sendExternalControlScript"):
                    try:
                        self._driver.sendExternalControlScript()
                    except Exception as e:  # noqa: BLE001
                        return {"ok": False, "error": f"CS reconnect script: {e}", "armed": False}
            self._armed = True
            self._initialized = True
            try:
                self._last_cmd_rad = self._read_joints_rad_unlocked()
            except Exception:  # noqa: BLE001
                pass
            return {
                "ok": True,
                "armed": True,
                "initialized": True,
                "dry_run": False,
                "kind": self.kind.value,
                "backend": "elite_cs",
            }

    def disarm(self, *, stop: bool = True) -> dict[str, Any]:
        if not self._opened:
            raise RuntimeError(f"{self.id}: call open() before disarm()")
        with self._io_lock:
            return self._disarm_unlocked(stop=stop)

    def _disarm_unlocked(self, *, stop: bool) -> dict[str, Any]:
        err: str | None = None
        if not self.ctx.dry_run and self._driver is not None and stop:
            try:
                if hasattr(self._driver, "writeIdle"):
                    self._driver.writeIdle(int(self.servoj_timeout_ms))
            except Exception as e:  # noqa: BLE001
                err = str(e)
        self._armed = False
        self._initialized = False
        out: dict[str, Any] = {
            "ok": err is None,
            "armed": False,
            "stopped": bool(stop),
            "backend": "elite_cs",
        }
        if err:
            out["error"] = err
        return out

    def _read_joints_rad_unlocked(self) -> list[float]:
        if self.ctx.dry_run or self._driver is None:
            if self._last_cmd_rad is not None:
                return list(self._last_cmd_rad)
            return [0.0] * self.num_joints
        if self._rtsi is not None:
            return read_joints_rad_from_rtsi(self._rtsi, n=self.num_joints)
        if self._last_cmd_rad is not None:
            return list(self._last_cmd_rad)
        raise RuntimeError(
            f"{self.id}: no CS joint feedback "
            "(set params.rtsi_output_recipe for RTSI, or command once to seed last_cmd)"
        )

    def _soft_limits_rad(self) -> tuple[list[float], list[float]] | None:
        if not self.enforce_soft_limits:
            return None
        try:
            from sensors.kinematics.teach import teach_joint_soft_limits_deg

            lo_deg, hi_deg = teach_joint_soft_limits_deg()
            lo = [math.radians(float(x)) for x in lo_deg[: self.num_joints]]
            hi = [math.radians(float(x)) for x in hi_deg[: self.num_joints]]
            while len(lo) < self.num_joints:
                lo.append(-math.pi)
                hi.append(math.pi)
            return lo, hi
        except Exception:  # noqa: BLE001
            return None

    def _apply_guards(
        self,
        target: list[float],
        *,
        reference: list[float] | None,
    ) -> tuple[list[float] | None, str | None]:
        n = self.num_joints
        try:
            tgt = _finite_list(target, n=n)
        except ValueError as e:
            return None, str(e)

        ref = reference
        if ref is None:
            ref = self._last_cmd_rad
        if ref is not None:
            try:
                ref_l = _finite_list(ref, n=n)
            except ValueError as e:
                return None, f"reference: {e}"
            for i in range(n):
                d = abs(tgt[i] - ref_l[i])
                if d > self.max_delta_rad + 1e-12:
                    return None, (
                        f"joint[{i}] |Δq|={math.degrees(d):.3f}° exceeds "
                        f"max_delta={math.degrees(self.max_delta_rad):.3f}°"
                    )

        limits = self._soft_limits_rad()
        if limits is not None:
            lo, hi = limits
            for i in range(n):
                if tgt[i] < lo[i] - 1e-9 or tgt[i] > hi[i] + 1e-9:
                    return None, (
                        f"joint[{i}]={math.degrees(tgt[i]):.2f}° outside soft limits "
                        f"[{math.degrees(lo[i]):.1f}, {math.degrees(hi[i]):.1f}]°"
                    )
        return tgt, None

    def _servoj_unlocked(self, joints_rad: list[float]) -> dict[str, Any]:
        deg = [math.degrees(x) for x in joints_rad]
        t0 = time.perf_counter()
        if self.ctx.dry_run or self._driver is None:
            self._last_cmd_rad = list(joints_rad)
            return {
                "ok": True,
                "joints_rad": list(joints_rad),
                "joints_deg": deg,
                "dry_run": True,
                "backend": "elite_cs",
                "write_ms": (time.perf_counter() - t0) * 1000.0,
            }
        if not hasattr(self._driver, "writeServoj"):
            return {"ok": False, "error": "EliteDriver missing writeServoj", "backend": "elite_cs"}
        ok = bool(
            self._driver.writeServoj(
                list(joints_rad),
                int(self.servoj_timeout_ms),
                False,  # cartesian=False → joint space
                False,  # queue_mode
            )
        )
        if not ok:
            return {
                "ok": False,
                "error": "writeServoj returned false",
                "backend": "elite_cs",
                "write_ms": (time.perf_counter() - t0) * 1000.0,
            }
        self._last_cmd_rad = list(joints_rad)
        return {
            "ok": True,
            "joints_rad": list(joints_rad),
            "joints_deg": deg,
            "dry_run": False,
            "backend": "elite_cs",
            "write_ms": (time.perf_counter() - t0) * 1000.0,
        }

    def read(self) -> Mapping[str, Any]:
        super().read()
        ts = time.time()
        with self._io_lock:
            dry = bool(self.ctx.dry_run or self._driver is None)
            try:
                joints_rad = self._read_joints_rad_unlocked()
                err = None
            except Exception as e:  # noqa: BLE001
                joints_rad = list(self._last_cmd_rad) if self._last_cmd_rad else None
                err = str(e)
            return {
                "joints_rad": joints_rad,
                "joints_deg": (
                    [math.degrees(x) for x in joints_rad] if joints_rad is not None else None
                ),
                "command_joints_rad": list(self._last_cmd_rad) if self._last_cmd_rad else None,
                "num_joints": self.num_joints,
                "robot_ip": self.robot_ip,
                "endpoint": self.endpoint,
                "backend": "elite_cs",
                "mode": "write_gated",
                "armed": self._armed,
                "max_delta_rad": self.max_delta_rad,
                "max_delta_deg": math.degrees(self.max_delta_rad),
                "dry_run": dry,
                "error": err,
                "ts": ts,
            }

    def write(self, command: Mapping[str, Any]) -> Mapping[str, Any]:
        if not self._opened:
            raise RuntimeError(f"{self.id}: call open() before write()")
        if not command:
            return {"ok": False, "error": "empty command"}

        if command.get("arm") or command.get("initialize"):
            return self.arm()
        if command.get("disarm"):
            return self.disarm(stop=bool(command.get("stop", True)))
        if command.get("stop") and not command.get("joints_rad") and command.get("jog_joint") is None:
            with self._io_lock:
                return self._disarm_unlocked(stop=True)

        with self._io_lock:
            if not self._armed:
                return {"ok": False, "error": "arm not armed; click Arm first", "armed": False}

            if "joints_rad" in command and command.get("joints_rad") is not None:
                target_raw = list(command["joints_rad"])
                ref = command.get("reference_joints_rad")
                if ref is not None:
                    try:
                        ref = _finite_list(list(ref), n=self.num_joints)
                    except ValueError as e:
                        return {"ok": False, "error": f"reference_joints_rad: {e}", "armed": True}
                elif coerce_bool(command.get("relative_to_last"), False):
                    ref = self._last_cmd_rad
                else:
                    try:
                        ref = self._read_joints_rad_unlocked()
                    except Exception:  # noqa: BLE001
                        ref = self._last_cmd_rad
                guarded, err = self._apply_guards(target_raw, reference=ref)
                if err or guarded is None:
                    return {"ok": False, "error": err or "guard failed", "armed": True}
                result = self._servoj_unlocked(guarded)
                result["armed"] = True
                return result

            if command.get("jog_joint") is not None:
                idx = int(command["jog_joint"])
                if idx < 0 or idx >= self.num_joints:
                    return {"ok": False, "error": f"jog_joint out of range 0..{self.num_joints-1}"}
                if "delta_rad" not in command:
                    return {"ok": False, "error": "delta_rad required for jog"}
                delta = float(command["delta_rad"])
                if not math.isfinite(delta):
                    return {"ok": False, "error": "delta_rad not finite"}
                if abs(delta) > self.max_delta_rad + 1e-12:
                    return {
                        "ok": False,
                        "error": (
                            f"|delta|={math.degrees(abs(delta)):.3f}° exceeds "
                            f"max_delta={math.degrees(self.max_delta_rad):.3f}°"
                        ),
                    }
                try:
                    base = self._read_joints_rad_unlocked()
                except Exception:  # noqa: BLE001
                    if self._last_cmd_rad is None:
                        return {"ok": False, "error": "no joint reference for jog"}
                    base = list(self._last_cmd_rad)
                target = list(base)
                target[idx] = float(base[idx]) + delta
                guarded, err = self._apply_guards(target, reference=base)
                if err or guarded is None:
                    return {"ok": False, "error": err or "guard failed", "armed": True}
                result = self._servoj_unlocked(guarded)
                result["armed"] = True
                result["jog_joint"] = idx
                result["delta_rad"] = delta
                return result

        return {
            "ok": False,
            "error": (
                "unsupported command; use arm/initialize, disarm/stop, "
                "joints_rad, or jog_joint+delta_rad"
            ),
            "backend": "elite_cs",
        }

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
        dry = bool(self.ctx.dry_run or self._driver is None)
        while time.perf_counter() < deadline:
            try:
                with self._io_lock:
                    self._read_joints_rad_unlocked()
                times.append(time.perf_counter())
            except Exception as e:  # noqa: BLE001
                errors += 1
                last_error = str(e)
                if errors >= 8 and len(times) < 3:
                    break
        if len(times) < 3:
            return rate_probe_fail(
                error=last_error or "arm_cs_write probe produced too few samples",
                method="elite_cs_servoj_read",
                samples=len(times),
                errors=errors,
                duration_s=time.perf_counter() - t0,
                dry_run=dry,
                last_error=last_error,
            )
        return rate_probe_ok(
            times,
            method="elite_cs_servoj_read",
            errors=errors,
            t0=t0,
            dry_run=dry,
            last_error=last_error,
        )
