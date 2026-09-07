from __future__ import annotations

"""Elite EC arm — **gated write** via TT joint stream (hik_gello EliteRobot path).

Safety contract (DCS):
  - ``open()`` connects + starts monitor only (same as arm_read). Does NOT servo_on / TT_init.
  - Motion requires explicit ``arm`` / ``initialize`` → ``armed``.
  - ``write(joints_rad|jog)`` rejects unless armed (except stop/disarm/arm).
  - Soft teach limits + per-command max |Δq| applied before ``TT_add_joint``.
  - Gripper is NOT commanded here (use gripper_write).

Reference: ``hww/hik_gello/gello/robots/elite_robot.py`` ``command_joint_state``.
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
from sensors.drivers.arm._elite_monitor import close_ec_monitor, open_ec_with_monitor

_DEFAULT_MAX_DELTA_DEG = 2.0


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


@register_sensor(SensorKind.ARM_WRITE)
class EliteArmWriteSensor(Sensor):
    """Elite follower joints with explicit arm/disarm gate and TT streaming."""

    kind = SensorKind.ARM_WRITE
    capabilities = (
        SensorCapability.PROBE
        | SensorCapability.SAMPLE
        | SensorCapability.CONTROL
        | SensorCapability.RATE_PROBE
    )

    def __init__(self, sensor_id: str, config: Mapping[str, Any], ctx: SensorContext | None = None):
        super().__init__(sensor_id, config, ctx)
        self.robot_ip = str(
            config.get("robot_ip") or config.get("endpoint") or config.get("ip") or "10.111.34.200"
        )
        self.robot_tcp_port = int(config.get("robot_tcp_port", 54321))
        self.num_joints = int(config.get("num_joints", 6))
        self.monitor_wait_s = float(config.get("monitor_wait_s", 10.0))
        self.monitor_retries = int(config.get("monitor_retries", 3))
        self.monitor_retry_backoff_s = float(config.get("monitor_retry_backoff_s", 1.0))
        self.post_close_cooldown_s = float(config.get("post_close_cooldown_s", 0.3))
        self.connect_timeout_s = float(config.get("connect_timeout_s", 0.4))
        self.tt_t = float(config.get("tt_t", 2.0))
        self.tt_response_enable = int(config.get("tt_response_enable", 0))
        max_delta_deg = float(config.get("max_delta_deg", _DEFAULT_MAX_DELTA_DEG))
        self.max_delta_rad = abs(math.radians(max_delta_deg))
        self.enforce_soft_limits = coerce_bool(config.get("enforce_soft_limits"), True)
        self._robot: Any = None
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
            "num_joints": self.num_joints,
            "mode": "write_gated",
            "max_delta_deg": math.degrees(self.max_delta_rad),
            "armed": self._armed,
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
                    "elite module missing (pip/desktop: elirobots)",
                )
            )
        if self.ctx.dry_run:
            status = HealthStatus.OK
            message = f"arm_write dry_run ({self.robot_ip})"
        elif robot_ok:
            status = HealthStatus.OK if metrics.get("elite_sdk") else HealthStatus.WARN
            message = f"arm_write gated @ {self.robot_ip}"
        else:
            status = HealthStatus.OFFLINE
            message = f"arm_write unreachable {self.endpoint}"
        return HealthReport(
            sensor_id=self.id,
            kind=self.kind.value,
            status=status,
            message=message,
            checks=checks,
            metrics=metrics,
            hints=[
                "open() = monitor only; call arm/initialize before TT_add_joint",
                "UI: Arm → ± jog with delta slider; Disarm/Estop to release",
                f"max_delta_deg={math.degrees(self.max_delta_rad):.3g}",
                "8056 reconnect: monitor_retries / see docs/elite-monitor-reconnect.md",
            ],
        )

    def open(self) -> None:
        if self._opened:
            return
        if self.ctx.dry_run:
            self._opened = True
            self._robot = None
            self._armed = False
            return
        try:
            robot = open_ec_with_monitor(
                robot_ip=self.robot_ip,
                num_joints=self.num_joints,
                monitor_wait_s=self.monitor_wait_s,
                monitor_retries=self.monitor_retries,
                monitor_retry_backoff_s=self.monitor_retry_backoff_s,
                post_close_cooldown_s=self.post_close_cooldown_s,
                sensor_id=self.id,
            )
        except ImportError as e:
            raise RuntimeError(
                "elite SDK required for arm_write open() when dry_run=false"
            ) from e

        self._robot = robot
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
            robot = self._robot
            self._robot = None
            self._armed = False
            self._initialized = False
            if robot is not None and not self.ctx.dry_run:
                close_ec_monitor(
                    robot,
                    post_close_cooldown_s=self.post_close_cooldown_s,
                )
            self._opened = False

    def initialize(self, **options: Any) -> dict[str, Any]:
        """Alias for arm() — explicit servo + TT_init gate."""
        return self.arm(**options)

    def arm(self, **options: Any) -> dict[str, Any]:
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
                }
            robot = self._robot
            if robot is None:
                return {"ok": False, "error": "robot not connected", "armed": False}
            try:
                # Match elite_robot.__init__ motion prep (only when explicitly armed).
                state = getattr(getattr(robot, "state", None), "name", None) or str(
                    getattr(robot, "state", "")
                )
                if "PLAY" in str(state).upper() and hasattr(robot, "stop"):
                    robot.stop()
                if hasattr(robot, "robot_servo_on"):
                    robot.robot_servo_on()
                if hasattr(robot, "set_servo_status"):
                    robot.set_servo_status(1)
                if hasattr(robot, "TT_init"):
                    t = float(options.get("tt_t", self.tt_t))
                    resp = int(options.get("response_enable", self.tt_response_enable))
                    robot.TT_init(t=t, response_enable=resp)
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
                    "tt_t": float(options.get("tt_t", self.tt_t)),
                }
            except Exception as e:  # noqa: BLE001
                self._armed = False
                self._initialized = False
                return {"ok": False, "error": str(e), "armed": False}

    def disarm(self, *, stop: bool = True) -> dict[str, Any]:
        if not self._opened:
            raise RuntimeError(f"{self.id}: call open() before disarm()")
        with self._io_lock:
            return self._disarm_unlocked(stop=stop)

    def _disarm_unlocked(self, *, stop: bool) -> dict[str, Any]:
        err: str | None = None
        if not self.ctx.dry_run and self._robot is not None and stop:
            try:
                if hasattr(self._robot, "stop"):
                    self._robot.stop()
            except Exception as e:  # noqa: BLE001
                err = str(e)
        self._armed = False
        # Keep _initialized False so UI mirrors gripper "need init" clarity.
        self._initialized = False
        out: dict[str, Any] = {"ok": err is None, "armed": False, "stopped": bool(stop)}
        if err:
            out["error"] = err
        return out

    def _read_joints_rad_unlocked(self) -> list[float]:
        if self.ctx.dry_run or self._robot is None:
            if self._last_cmd_rad is not None:
                return list(self._last_cmd_rad)
            return [0.0] * self.num_joints
        pos = getattr(getattr(self._robot, "monitor_info", None), "machinePos", None)
        if pos is None or len(pos) < self.num_joints:
            raise RuntimeError(f"{self.id}: monitor_info.machinePos unavailable")
        out: list[float] = []
        for i in range(self.num_joints):
            val = pos[i]
            if val is None:
                raise RuntimeError(f"{self.id}: machinePos[{i}] is None")
            out.append(math.radians(float(val)))
        return out

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

    def _tt_add_unlocked(self, joints_rad: list[float]) -> dict[str, Any]:
        deg = [math.degrees(x) for x in joints_rad]
        t0 = time.perf_counter()
        if self.ctx.dry_run or self._robot is None:
            self._last_cmd_rad = list(joints_rad)
            return {
                "ok": True,
                "joints_rad": list(joints_rad),
                "joints_deg": deg,
                "dry_run": True,
                "write_ms": (time.perf_counter() - t0) * 1000.0,
            }
        if not hasattr(self._robot, "TT_add_joint"):
            return {"ok": False, "error": "elite.EC missing TT_add_joint"}
        self._robot.TT_add_joint(deg)
        self._last_cmd_rad = list(joints_rad)
        return {
            "ok": True,
            "joints_rad": list(joints_rad),
            "joints_deg": deg,
            "dry_run": False,
            "write_ms": (time.perf_counter() - t0) * 1000.0,
        }

    def read(self) -> Mapping[str, Any]:
        super().read()
        ts = time.time()
        with self._io_lock:
            dry = bool(self.ctx.dry_run or self._robot is None)
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
                "backend": "elite_ec",
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
            return self.arm(
                tt_t=command.get("tt_t", self.tt_t),
                response_enable=command.get("response_enable", self.tt_response_enable),
            )
        if command.get("disarm"):
            return self.disarm(stop=bool(command.get("stop", True)))
        if command.get("stop") and not command.get("joints_rad") and command.get("jog_joint") is None:
            with self._io_lock:
                return self._disarm_unlocked(stop=True)

        with self._io_lock:
            if not self._armed:
                return {"ok": False, "error": "arm not armed; click Arm first", "armed": False}

            # Absolute joints
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
                result = self._tt_add_unlocked(guarded)
                result["armed"] = True
                return result

            # Jog one joint by delta_rad
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
                result = self._tt_add_unlocked(guarded)
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
        }
