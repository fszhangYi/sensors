from __future__ import annotations

"""Elite CS arm — **gated write** via ``EliteDriver.writeServoj``.

Distinct from EC ``arm_write`` (``elirobots`` / ``TT_add_joint`` / ports 54321+8056).

Safety contract (same UI shape as EC arm_write):
  - ``open()`` connects EliteDriver (+ required RTSI when ``require_rtsi``).
  - Motion requires explicit ``arm`` / ``initialize`` → ``armed``.
  - ``write(joints_rad|jog)`` rejects unless armed (except stop/disarm/arm).
  - Soft teach limits + per-command max |Δq| before ``writeServoj``.
  - Joint units to SDK are **radians** (EC TT path uses degrees).
  - Live mode refuses motion without **measured** RTSI joints (no home synth).

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
    ensure_cs_motion_ready,
    import_elite_cs_sdk,
    open_cs_driver,
    open_cs_rtsi,
    read_joints_rad_from_rtsi,
)

_DEFAULT_MAX_DELTA_DEG = 2.0
_DEFAULT_PRIMARY_PORT = 30001
# external_control.script ignores steps with |Δq|/SERVOJ_TIME > JOINT_IGNORE_SPEED (30).
# Controller / speed slider is usually much lower — keep a conservative default.
_DEFAULT_STREAM_MAX_RAD_S = 0.5
# After each command the robot uses timeout_ms as the *next* reverse read deadline;
# 100ms is only safe for dense streaming — UI jog needs a longer hold.
_DEFAULT_SERVOJ_TIMEOUT_MS = 2000
_DEFAULT_IDLE_TIMEOUT_MS = 30000
_DEFAULT_KEEPALIVE_S = 2.0
# Skip keepalive writeIdle while dense servoj_once is active (teleop / abs-ramp).
_DEFAULT_KEEPALIVE_IDLE_GRACE_S = 0.4
# If |last_cmd − RTSI| stays under this, stream/jog from last_cmd (no rewind jerk).
# Larger gap → re-origin on measured RTSI (speed-limit safety).
_DEFAULT_CMD_CONTINUITY_RAD = 0.10  # ~5.7°


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
        self.servoj_timeout_ms = int(config.get("servoj_timeout_ms", _DEFAULT_SERVOJ_TIMEOUT_MS))
        self.idle_timeout_ms = int(config.get("idle_timeout_ms", _DEFAULT_IDLE_TIMEOUT_MS))
        self.keepalive_s = float(config.get("keepalive_s", _DEFAULT_KEEPALIVE_S))
        self.keepalive_idle_grace_s = float(
            config.get("keepalive_idle_grace_s", _DEFAULT_KEEPALIVE_IDLE_GRACE_S)
        )
        self.stream_max_rad_s = float(config.get("stream_max_rad_s", _DEFAULT_STREAM_MAX_RAD_S))
        self.cmd_continuity_rad = float(
            config.get("cmd_continuity_rad", _DEFAULT_CMD_CONTINUITY_RAD)
        )
        self.rtsi_output_recipe = str(config.get("rtsi_output_recipe") or "")
        self.rtsi_input_recipe = str(config.get("rtsi_input_recipe") or "")
        self.rtsi_frequency = float(config.get("rtsi_frequency", 125.0))
        # Live CS must have RTSI measured joints before Arm/jog (default on).
        self.require_rtsi = coerce_bool(config.get("require_rtsi"), True)
        self.kinematics = str(config.get("kinematics") or config.get("kinematics_pkg") or "")
        self.kinematics_profile = str(
            config.get("kinematics_profile") or config.get("cs66_profile") or ""
        )
        # Match Elite examples: powerOn + brakeRelease before ExternalControl.
        self.ensure_power = coerce_bool(config.get("ensure_power"), True)
        self.external_control_timeout_s = float(config.get("external_control_timeout_s", 15.0))
        max_delta_deg = float(config.get("max_delta_deg", _DEFAULT_MAX_DELTA_DEG))
        self.max_delta_rad = abs(math.radians(max_delta_deg))
        self.enforce_soft_limits = coerce_bool(config.get("enforce_soft_limits"), True)
        self._driver: Any = None
        self._rtsi: Any = None
        self._armed = False
        self._io_lock = threading.RLock()
        self._last_cmd_rad: list[float] | None = None
        # True after at least one writeServoj in this armed session (skip re-seed).
        self._stream_latched = False
        # Monotonic time of last successful dense writeServoj (servoj_once).
        self._last_servoj_mono: float = 0.0
        self._keep_stop = threading.Event()
        self._keep_thread: threading.Thread | None = None
        self._last_keepalive_error: str | None = None

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
            "require_rtsi": self.require_rtsi,
            "rtsi_configured": bool(str(self.rtsi_output_recipe).strip()),
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
        if self.require_rtsi and not str(self.rtsi_output_recipe).strip():
            checks.append(
                CheckResult(
                    "rtsi_recipe",
                    False,
                    "require_rtsi=true but rtsi_output_recipe unset",
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
                "open() = EliteDriver + RTSI; call arm before writeServoj",
                "UI: Arm → ± jog / abs ramp; Disarm/Estop to release",
                f"max_delta_deg={math.degrees(self.max_delta_rad):.3g}",
                "Live CS requires measured RTSI joints (no YAML-home synth)",
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
        if self.require_rtsi and not str(self.rtsi_output_recipe).strip():
            raise RuntimeError(
                f"{self.id}: live arm_cs_write requires params.rtsi_output_recipe "
                "(RTSI measured joints). Set require_rtsi=false only for bring-up."
            )
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
            elif self.require_rtsi:
                raise RuntimeError(f"{self.id}: require_rtsi=true but RTSI open skipped")
        except ImportError as e:
            raise RuntimeError(
                "elite_cs_sdk required for arm_cs_write open() when dry_run=false. "
                "Install Elite_Robots_CS_SDK_Python (not elirobots)."
            ) from e
        except Exception as e:  # noqa: BLE001
            msg = str(e)
            low = msg.lower()
            # Windows WSAEACCES (10013) / reverse bind — often firewall or leftover EliteDriver.
            if (
                "访问权限不允许" in msg
                or "socket disconnected" in low
                or "wsaeacces" in low
                or "10013" in msg
            ):
                raise RuntimeError(
                    f"{self.id}: CS EliteDriver socket failed ({self.robot_ip}, "
                    f"local_ip={self.local_ip or '(auto)'}): {msg}. "
                    "Usually PC reverse ports 50001/50003/50004: close other sensors-dcs/"
                    "sensors-arm/SDK sessions; wait ~60s for TIME_WAIT; allow inbound TCP "
                    "50001,50003,50004 (Windows firewall — python.exe if not using "
                    "sensors-dcs.exe); confirm teach pendant Remote/Auto + ExternalControl; "
                    "local_ip must be the NIC the arm can reach (same subnet as robot_ip)."
                ) from e
            if "primary" in low or "30001" in msg or "socket connect" in low:
                raise RuntimeError(
                    f"{self.id}: CS Primary connect failed ({self.robot_ip}:30001): {msg}. "
                    "RTSI 30004 can be up while Primary is down. On the teach pendant: "
                    "power on controller, Remote/Auto, enable remote control; "
                    "close any other EliteDriver/SDK session; then retry. "
                    f"PC local_ip={self.local_ip or '(auto)'}."
                ) from e
            raise
        self._opened = True
        self._armed = False
        self._initialized = False

    def close(self) -> None:
        self._stop_keepalive()
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
            ready = ensure_cs_motion_ready(
                self._driver,
                headless_mode=self.headless_mode,
                power_on=self.ensure_power,
                timeout_s=self.external_control_timeout_s,
            )
            if not ready.get("ok"):
                self._armed = False
                self._initialized = False
                return {
                    "ok": False,
                    "error": str(ready.get("error") or "CS external control not ready"),
                    "armed": False,
                    "backend": "elite_cs",
                    "ready": ready,
                }
            try:
                seed, source = self._read_joints_with_source_unlocked()
            except Exception as e:  # noqa: BLE001
                self._armed = False
                self._initialized = False
                return {
                    "ok": False,
                    "error": (
                        f"CS Arm refused: no measured joints to seed "
                        f"(_last_cmd / RTSI). {e}"
                    ),
                    "armed": False,
                    "backend": "elite_cs",
                    "ready": ready,
                }
            if self.require_rtsi and source != "rtsi":
                self._armed = False
                self._initialized = False
                return {
                    "ok": False,
                    "error": (
                        "CS Arm refused: require_rtsi=true but joints_source="
                        f"{source!r} (need RTSI actual_joint_positions)"
                    ),
                    "armed": False,
                    "backend": "elite_cs",
                    "joints_source": source,
                    "ready": ready,
                }
            self._last_cmd_rad = list(seed)
            self._stream_latched = False
            self._last_servoj_mono = 0.0
            self._armed = True
            self._initialized = True
            self._last_keepalive_error = None
            self._write_idle_unlocked(self.idle_timeout_ms)
            self._start_keepalive()
            return {
                "ok": True,
                "armed": True,
                "initialized": True,
                "dry_run": False,
                "kind": self.kind.value,
                "backend": "elite_cs",
                "joints_source": source,
                "joints_rad": list(seed),
                "ready": ready,
            }

    def disarm(self, *, stop: bool = True) -> dict[str, Any]:
        if not self._opened:
            raise RuntimeError(f"{self.id}: call open() before disarm()")
        self._stop_keepalive()
        with self._io_lock:
            return self._disarm_unlocked(stop=stop)

    def _force_disarm(self, reason: str) -> None:
        """Drop armed state after reverse-channel / keepalive failure."""
        self._last_keepalive_error = str(reason)
        self._stop_keepalive()
        with self._io_lock:
            self._armed = False
            self._initialized = False
            self._stream_latched = False
            self._last_servoj_mono = 0.0
            try:
                if self._driver is not None and hasattr(self._driver, "stopControl"):
                    self._driver.stopControl(200)
            except Exception:  # noqa: BLE001
                pass

    def _dense_servoj_active(self) -> bool:
        """True while recent dense servoj_once should suppress keepalive writeIdle."""
        if not self._stream_latched:
            return False
        if self._last_servoj_mono <= 0.0:
            return False
        grace = max(0.05, float(self.keepalive_idle_grace_s))
        return (time.monotonic() - self._last_servoj_mono) < grace

    def _start_keepalive(self) -> None:
        """While armed, periodically ``writeIdle`` so ExternalControl does not exit.

        During dense teleop / abs-ramp (``servoj_once`` stream) skip idle so the
        controller keeps a continuous setpoint; resume idle after grace expires.
        """
        self._stop_keepalive()
        if self.ctx.dry_run:
            return
        self._keep_stop = threading.Event()
        interval = max(
            0.5, min(float(self.keepalive_s), max(0.5, self.idle_timeout_ms / 1000.0 / 3.0))
        )

        def _loop() -> None:
            while not self._keep_stop.wait(interval):
                if not self._armed:
                    continue
                if self._dense_servoj_active():
                    continue
                if not self._io_lock.acquire(blocking=False):
                    continue
                try:
                    if not self._armed or self._driver is None:
                        continue
                    # Re-check under lock: a servoj may have just latched.
                    if self._dense_servoj_active():
                        continue
                    connected = bool(
                        hasattr(self._driver, "isRobotConnected")
                        and self._driver.isRobotConnected()
                    )
                    if not connected:
                        ready = ensure_cs_motion_ready(
                            self._driver,
                            headless_mode=self.headless_mode,
                            power_on=False,
                            timeout_s=min(8.0, self.external_control_timeout_s),
                        )
                        if not ready.get("ok"):
                            self._io_lock.release()
                            self._force_disarm(
                                str(ready.get("error") or "keepalive reconnect failed")
                            )
                            return
                    if not self._write_idle_unlocked(self.idle_timeout_ms):
                        self._io_lock.release()
                        self._force_disarm("writeIdle returned false during keepalive")
                        return
                    self._last_keepalive_error = None
                except Exception as e:  # noqa: BLE001
                    try:
                        self._io_lock.release()
                    except RuntimeError:
                        pass
                    self._force_disarm(f"keepalive error: {e}")
                    return
                else:
                    self._io_lock.release()

        self._keep_thread = threading.Thread(
            target=_loop,
            name=f"cs-keepalive-{self.id}",
            daemon=True,
        )
        self._keep_thread.start()

    def _stop_keepalive(self) -> None:
        self._keep_stop.set()
        th = self._keep_thread
        self._keep_thread = None
        if th is not None and th.is_alive() and th is not threading.current_thread():
            th.join(timeout=1.0)

    def _disarm_unlocked(self, *, stop: bool) -> dict[str, Any]:
        err: str | None = None
        if not self.ctx.dry_run and self._driver is not None and stop:
            try:
                if hasattr(self._driver, "stopControl"):
                    self._driver.stopControl(500)
                elif hasattr(self._driver, "writeIdle"):
                    self._driver.writeIdle(100)
            except Exception as e:  # noqa: BLE001
                err = str(e)
        self._armed = False
        self._initialized = False
        self._stream_latched = False
        self._last_servoj_mono = 0.0
        out: dict[str, Any] = {
            "ok": err is None,
            "armed": False,
            "stopped": bool(stop),
            "backend": "elite_cs",
        }
        if err:
            out["error"] = err
        if self._last_keepalive_error:
            out["keepalive_error"] = self._last_keepalive_error
        return out

    def _read_joints_with_source_unlocked(self) -> tuple[list[float], str]:
        """Return (joints_rad, source) where source is rtsi|command|dry."""
        if self.ctx.dry_run or self._driver is None:
            if self._last_cmd_rad is not None:
                return list(self._last_cmd_rad), "command"
            return [0.0] * self.num_joints, "dry"
        if self._rtsi is not None:
            return read_joints_rad_from_rtsi(self._rtsi, n=self.num_joints), "rtsi"
        if self._last_cmd_rad is not None:
            return list(self._last_cmd_rad), "command"
        raise RuntimeError(
            f"{self.id}: no CS joint feedback "
            "(set params.rtsi_output_recipe for RTSI, or command once to seed last_cmd)"
        )

    def _read_joints_rad_unlocked(self) -> list[float]:
        joints, _src = self._read_joints_with_source_unlocked()
        return joints

    def _soft_limits_rad(self) -> tuple[list[float], list[float]] | None:
        if not self.enforce_soft_limits:
            return None
        lo_deg: list[float] | None = None
        hi_deg: list[float] | None = None
        pkg = (self.kinematics or "").strip()
        prof = (self.kinematics_profile or "").strip() or None
        try:
            if pkg in (
                "cs66_kin_g26562",
                "cs66_g26562",
                "sensors.cs66_kin_g26562",
            ):
                from sensors.cs66_kin_g26562.teach import teach_joint_soft_limits_deg

                lo_a, hi_a = teach_joint_soft_limits_deg(prof)
                lo_deg = [float(x) for x in lo_a]
                hi_deg = [float(x) for x in hi_a]
            elif pkg in (
                "cs66_kinematics",
                "cs66",
                "sensors.cs66_kinematics",
            ):
                from sensors.cs66_kinematics.teach import teach_joint_soft_limits_deg

                lo_a, hi_a = teach_joint_soft_limits_deg(prof)
                lo_deg = [float(x) for x in lo_a]
                hi_deg = [float(x) for x in hi_a]
            else:
                from sensors.kinematics.teach import teach_joint_soft_limits_deg

                lo_a, hi_a = teach_joint_soft_limits_deg()
                lo_deg = [float(x) for x in lo_a]
                hi_deg = [float(x) for x in hi_a]
        except Exception:  # noqa: BLE001
            return None
        if lo_deg is None or hi_deg is None:
            return None
        lo = [math.radians(float(x)) for x in lo_deg[: self.num_joints]]
        hi = [math.radians(float(x)) for x in hi_deg[: self.num_joints]]
        while len(lo) < self.num_joints:
            lo.append(-math.pi)
            hi.append(math.pi)
        return lo, hi

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

    def _write_idle_unlocked(self, timeout_ms: int | None = None) -> bool:
        """Extend reverse-socket wait so ExternalControl does not exit on EOF/timeout."""
        if self.ctx.dry_run or self._driver is None:
            return True
        if not hasattr(self._driver, "writeIdle"):
            return False
        ms = int(timeout_ms if timeout_ms is not None else self.idle_timeout_ms)
        try:
            return bool(self._driver.writeIdle(max(100, ms)))
        except Exception:  # noqa: BLE001
            return False

    def _rtsi_speed_scale_unlocked(self) -> float:
        """Read teach speed_scaling (0..1) when RTSI recipe provides it."""
        if self._rtsi is None:
            return 1.0
        for attr in ("getSpeedScaling", "get_speed_scaling"):
            fn = getattr(self._rtsi, attr, None)
            if callable(fn):
                try:
                    v = float(fn())
                    if math.isfinite(v) and v > 0.0:
                        return max(0.05, min(1.0, v if v <= 1.5 else v / 100.0))
                except Exception:  # noqa: BLE001
                    pass
        # Recipe field via generic getter if present.
        for name in ("speed_scaling", "target_speed_fraction"):
            getter = getattr(self._rtsi, "getRecipeValue", None) or getattr(
                self._rtsi, "getOutput", None
            )
            if not callable(getter):
                continue
            try:
                v = float(getter(name))
                if math.isfinite(v) and v > 0.0:
                    return max(0.05, min(1.0, v if v <= 1.5 else v / 100.0))
            except Exception:  # noqa: BLE001
                continue
        return 1.0

    def _max_abs_delta(self, a: Sequence[float], b: Sequence[float]) -> float:
        n = min(len(a), len(b), self.num_joints)
        if n <= 0:
            return float("inf")
        return max(abs(float(a[i]) - float(b[i])) for i in range(n))

    def _pick_motion_base_unlocked(
        self,
    ) -> tuple[list[float], str, list[float] | None, str | None]:
        """Choose jog/stream origin: continuous last_cmd when close to RTSI.

        Returns ``(base, base_src, measured_or_none, measured_src_or_none)``.
        ``base_src`` is ``command`` (continuity) or ``rtsi`` / ``dry``.
        """
        measured: list[float] | None = None
        measured_src: str | None = None
        try:
            measured, measured_src = self._read_joints_with_source_unlocked()
        except Exception:  # noqa: BLE001
            measured, measured_src = None, None

        if self.require_rtsi and (measured is None or measured_src != "rtsi"):
            if measured is None and self._last_cmd_rad is None:
                raise RuntimeError("no RTSI joints and no last_cmd")
            # Still allow continuity only if we already have a command frame;
            # caller must have verified RTSI at Arm time.
            if self._last_cmd_rad is not None and measured is not None:
                gap = self._max_abs_delta(self._last_cmd_rad, measured)
                if gap <= max(1e-4, abs(float(self.cmd_continuity_rad))):
                    return list(self._last_cmd_rad), "command", measured, measured_src
            if measured is not None and measured_src == "rtsi":
                return list(measured), "rtsi", measured, measured_src
            if self._last_cmd_rad is not None:
                return list(self._last_cmd_rad), "command", measured, measured_src
            raise RuntimeError(f"no RTSI joints for motion (got {measured_src!r})")

        if (
            self._last_cmd_rad is not None
            and measured is not None
            and self._max_abs_delta(self._last_cmd_rad, measured)
            <= max(1e-4, abs(float(self.cmd_continuity_rad)))
        ):
            return list(self._last_cmd_rad), "command", measured, measured_src

        if measured is not None:
            return list(measured), str(measured_src or "rtsi"), measured, measured_src
        if self._last_cmd_rad is not None:
            return list(self._last_cmd_rad), "command", None, None
        raise RuntimeError("no joint reference for motion")

    def _servoj_once(
        self,
        joints_rad: list[float],
        *,
        idle_after: bool = False,
    ) -> dict[str, Any]:
        """Single ``writeServoj`` for dense abs-ramp (no mini-stream / no per-point sleep).

        First call after Arm re-seeds the servo setpoint once; later points only
        send the new target so a 20–125 Hz ramp stays smooth.
        """
        deg = [math.degrees(x) for x in joints_rad]
        t0 = time.perf_counter()
        if self.ctx.dry_run or self._driver is None:
            with self._io_lock:
                self._last_cmd_rad = list(joints_rad)
                self._stream_latched = True
                self._last_servoj_mono = time.monotonic()
            return {
                "ok": True,
                "joints_rad": list(joints_rad),
                "joints_deg": deg,
                "dry_run": True,
                "backend": "elite_cs",
                "servoj_once": True,
                "write_ms": (time.perf_counter() - t0) * 1000.0,
            }
        if not hasattr(self._driver, "writeServoj"):
            return {"ok": False, "error": "EliteDriver missing writeServoj", "backend": "elite_cs"}

        timeout_ms = max(int(self.servoj_timeout_ms), 200)
        with self._io_lock:
            if hasattr(self._driver, "isRobotConnected") and not self._driver.isRobotConnected():
                ready = ensure_cs_motion_ready(
                    self._driver,
                    headless_mode=self.headless_mode,
                    power_on=self.ensure_power,
                    timeout_s=min(10.0, self.external_control_timeout_s),
                )
                if not ready.get("ok"):
                    self._armed = False
                    return {
                        "ok": False,
                        "error": (
                            "writeServoj skipped: reverse channel not connected. "
                            + str(ready.get("error") or "")
                        ).strip(),
                        "backend": "elite_cs",
                        "armed": False,
                        "ready": ready,
                        "servoj_once": True,
                        "write_ms": (time.perf_counter() - t0) * 1000.0,
                    }
            if not self._armed:
                return {
                    "ok": False,
                    "error": "disarmed during servoj_once",
                    "backend": "elite_cs",
                    "armed": False,
                    "servoj_once": True,
                    "write_ms": (time.perf_counter() - t0) * 1000.0,
                }
            # Latch servo to continuous command frame once per armed session.
            if not self._stream_latched:
                try:
                    start, _src, _m, _ms = self._pick_motion_base_unlocked()
                except Exception as e:  # noqa: BLE001
                    return {
                        "ok": False,
                        "error": f"no start joints for servoj_once: {e}",
                        "backend": "elite_cs",
                        "armed": self._armed,
                        "servoj_once": True,
                        "write_ms": (time.perf_counter() - t0) * 1000.0,
                    }
                if not bool(self._driver.writeServoj(list(start), int(timeout_ms), False)):
                    self._armed = False
                    self._initialized = False
                    return {
                        "ok": False,
                        "error": "writeServoj false while latching servo setpoint",
                        "backend": "elite_cs",
                        "armed": False,
                        "servoj_once": True,
                        "write_ms": (time.perf_counter() - t0) * 1000.0,
                    }
                self._last_cmd_rad = list(start)
                self._stream_latched = True

            ok = bool(self._driver.writeServoj(list(joints_rad), int(timeout_ms), False))
            if not ok:
                connected = bool(
                    hasattr(self._driver, "isRobotConnected")
                    and self._driver.isRobotConnected()
                )
                self._armed = False
                self._initialized = False
                self._stream_latched = False
                self._last_servoj_mono = 0.0
                return {
                    "ok": False,
                    "error": (
                        "writeServoj returned false"
                        + (
                            " (ExternalControl may have stopped / speed limit)"
                            if connected
                            else " (reverse ports disconnected)"
                        )
                    ),
                    "backend": "elite_cs",
                    "cs_connected": connected,
                    "armed": False,
                    "servoj_once": True,
                    "write_ms": (time.perf_counter() - t0) * 1000.0,
                }
            self._last_cmd_rad = list(joints_rad)
            self._stream_latched = True
            self._last_servoj_mono = time.monotonic()
            if idle_after:
                self._write_idle_unlocked(self.idle_timeout_ms)
        return {
            "ok": True,
            "joints_rad": list(joints_rad),
            "joints_deg": deg,
            "dry_run": False,
            "backend": "elite_cs",
            "servoj_once": True,
            "stream_latched": True,
            "write_ms": (time.perf_counter() - t0) * 1000.0,
        }

    def _servoj_stream(self, joints_rad: list[float]) -> dict[str, Any]:
        """Stream ``writeServoj`` waypoints; acquires ``_io_lock`` per point only."""
        deg = [math.degrees(x) for x in joints_rad]
        t0 = time.perf_counter()
        n = self.num_joints
        if self.ctx.dry_run or self._driver is None:
            with self._io_lock:
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

        with self._io_lock:
            if hasattr(self._driver, "isRobotConnected") and not self._driver.isRobotConnected():
                ready = ensure_cs_motion_ready(
                    self._driver,
                    headless_mode=self.headless_mode,
                    power_on=self.ensure_power,
                    timeout_s=min(10.0, self.external_control_timeout_s),
                )
                if not ready.get("ok"):
                    self._armed = False
                    return {
                        "ok": False,
                        "error": (
                            "writeServoj skipped: reverse channel not connected. "
                            + str(ready.get("error") or "")
                        ).strip(),
                        "backend": "elite_cs",
                        "armed": False,
                        "ready": ready,
                        "write_ms": (time.perf_counter() - t0) * 1000.0,
                    }
            try:
                start, start_src, measured, measured_src = self._pick_motion_base_unlocked()
            except Exception as e:  # noqa: BLE001
                return {
                    "ok": False,
                    "error": f"no measured start joints for stream: {e}",
                    "backend": "elite_cs",
                    "armed": self._armed,
                    "write_ms": (time.perf_counter() - t0) * 1000.0,
                }
            if self.require_rtsi and measured_src != "rtsi" and measured is None:
                return {
                    "ok": False,
                    "error": f"refuse stream without RTSI (got {measured_src!r})",
                    "backend": "elite_cs",
                    "armed": self._armed,
                    "write_ms": (time.perf_counter() - t0) * 1000.0,
                }
            # Continuity from last_cmd avoids RTSI-lag rewind shake; re-origin on
            # measured when gap is large (exceed-speed safety).
            # First move after Arm always re-seeds servo setpoint once.
            if not self._stream_latched or start_src != "command" or self._last_cmd_rad is None:
                need_sync = True
            else:
                need_sync = self._max_abs_delta(start, self._last_cmd_rad) > 1e-5
            scale = self._rtsi_speed_scale_unlocked()

        dt = max(0.008, float(self.servoj_time))
        max_rate = abs(float(self.stream_max_rad_s)) * max(0.05, min(1.0, scale))
        max_rate = min(max_rate, 1.5)
        max_step = max(1e-4, max_rate * dt)
        diffs = [float(joints_rad[i]) - float(start[i]) for i in range(n)]
        max_abs = max((abs(d) for d in diffs), default=0.0)
        n_steps = max(1, int(math.ceil(max_abs / max_step))) if max_abs > 1e-9 else 1
        timeout_ms = max(int(self.servoj_timeout_ms), int(dt * 1000 * 8), 200)

        def _one_servoj(q: list[float], *, step: int, n_steps: int, sent: int) -> dict[str, Any] | None:
            """Send one point; return error dict on failure, else None."""
            with self._io_lock:
                if not self._armed or self._driver is None:
                    return {
                        "ok": False,
                        "error": "disarmed during servoj stream",
                        "backend": "elite_cs",
                        "armed": False,
                        "stream_sent": sent,
                        "write_ms": (time.perf_counter() - t0) * 1000.0,
                    }
                ok = bool(self._driver.writeServoj(q, int(timeout_ms), False))
                if not ok:
                    connected = bool(
                        hasattr(self._driver, "isRobotConnected")
                        and self._driver.isRobotConnected()
                    )
                    self._armed = False
                    self._initialized = False
                    return {
                        "ok": False,
                        "error": (
                            "writeServoj returned false"
                            + f" at stream step {step}/{n_steps}"
                            + (
                                " (ExternalControl may have stopped / speed limit)"
                                if connected
                                else " (reverse ports disconnected)"
                            )
                        ),
                        "backend": "elite_cs",
                        "cs_connected": connected,
                        "armed": False,
                        "stream_steps": n_steps,
                        "stream_sent": sent,
                        "write_ms": (time.perf_counter() - t0) * 1000.0,
                    }
                self._last_cmd_rad = list(q)
                self._stream_latched = True
            return None

        # Re-seed servo setpoint only on first latch / RTSI re-origin.
        # Continuous jogs from last_cmd skip this — avoids back-then-forward shake.
        if need_sync:
            err0 = _one_servoj(list(start), step=0, n_steps=n_steps, sent=0)
            if err0 is not None:
                return err0
            time.sleep(dt)

        sent = 0
        for step in range(1, n_steps + 1):
            alpha = step / float(n_steps)
            q = [float(start[i]) + diffs[i] * alpha for i in range(n)]
            err = _one_servoj(q, step=step, n_steps=n_steps, sent=sent)
            if err is not None:
                return err
            sent = step
            if step < n_steps:
                time.sleep(dt)

        with self._io_lock:
            # Hold reverse socket open; do not stopj — keeps servo continuity for next jog.
            self._write_idle_unlocked(self.idle_timeout_ms)
        return {
            "ok": True,
            "joints_rad": list(joints_rad),
            "joints_deg": deg,
            "dry_run": False,
            "backend": "elite_cs",
            "stream_steps": n_steps,
            "stream_dt_s": dt,
            "stream_start_src": start_src,
            "stream_synced": bool(need_sync),
            "write_ms": (time.perf_counter() - t0) * 1000.0,
        }

    def read(self) -> Mapping[str, Any]:
        super().read()
        ts = time.time()
        with self._io_lock:
            dry = bool(self.ctx.dry_run or self._driver is None)
            try:
                joints_rad, source = self._read_joints_with_source_unlocked()
                err = None
            except Exception as e:  # noqa: BLE001
                joints_rad = list(self._last_cmd_rad) if self._last_cmd_rad else None
                source = "command" if joints_rad is not None else "none"
                err = str(e)
            measured = source == "rtsi" or (dry and source in ("dry", "command"))
            return {
                "joints_rad": joints_rad,
                "joints_deg": (
                    [math.degrees(x) for x in joints_rad] if joints_rad is not None else None
                ),
                "command_joints_rad": list(self._last_cmd_rad) if self._last_cmd_rad else None,
                "joints_source": source,
                "measured": bool(measured and joints_rad is not None),
                "num_joints": self.num_joints,
                "robot_ip": self.robot_ip,
                "endpoint": self.endpoint,
                "backend": "elite_cs",
                "mode": "write_gated",
                "armed": self._armed,
                "max_delta_rad": self.max_delta_rad,
                "max_delta_deg": math.degrees(self.max_delta_rad),
                "dry_run": dry,
                "keepalive_error": self._last_keepalive_error,
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
            self._stop_keepalive()
            with self._io_lock:
                return self._disarm_unlocked(stop=True)

        # Guards under lock; streaming releases lock between points.
        with self._io_lock:
            if not self._armed:
                return {
                    "ok": False,
                    "error": "arm not armed; click Arm first"
                    + (
                        f" (keepalive: {self._last_keepalive_error})"
                        if self._last_keepalive_error
                        else ""
                    ),
                    "armed": False,
                }

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
                        ref, ref_src = self._read_joints_with_source_unlocked()
                        if self.require_rtsi and ref_src != "rtsi":
                            ref = self._last_cmd_rad
                    except Exception:  # noqa: BLE001
                        ref = self._last_cmd_rad
                guarded, err = self._apply_guards(target_raw, reference=ref)
                if err or guarded is None:
                    return {"ok": False, "error": err or "guard failed", "armed": True}
                target = guarded
                jog_meta: dict[str, Any] | None = None
            elif command.get("jog_joint") is not None:
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
                    base, base_src, _meas, meas_src = self._pick_motion_base_unlocked()
                    if self.require_rtsi and meas_src != "rtsi" and self._last_cmd_rad is None:
                        return {
                            "ok": False,
                            "error": f"no RTSI joints for jog (got {meas_src!r})",
                            "armed": True,
                        }
                except Exception as e:  # noqa: BLE001
                    if self._last_cmd_rad is None:
                        return {"ok": False, "error": f"no joint reference for jog: {e}"}
                    base = list(self._last_cmd_rad)
                    base_src = "command"
                target_l = list(base)
                target_l[idx] = float(base[idx]) + delta
                guarded, err = self._apply_guards(target_l, reference=base)
                if err or guarded is None:
                    return {"ok": False, "error": err or "guard failed", "armed": True}
                target = guarded
                jog_meta = {
                    "jog_joint": idx,
                    "delta_rad": delta,
                    "jog_base_src": base_src,
                }
            else:
                return {
                    "ok": False,
                    "error": (
                        "unsupported command; use arm/initialize, disarm/stop, "
                        "joints_rad, or jog_joint+delta_rad"
                    ),
                    "backend": "elite_cs",
                }

        # Abs-ramp / dense UI: one writeServoj per call (path already timed).
        # Sparse jog / large jumps: rate-limited multi-step stream.
        once = coerce_bool(command.get("servoj_once"), False) or coerce_bool(
            command.get("dense"), False
        )
        if once and jog_meta is None:
            result = self._servoj_once(
                target,
                idle_after=coerce_bool(command.get("idle_after"), False),
            )
        else:
            result = self._servoj_stream(target)
        # Keep latch semantics: command ok/fail must not clear armed in the result
        # (UI/agent snap armed from this field; anding with ok made jog buttons flicker).
        result["armed"] = bool(self._armed)
        if jog_meta:
            result.update(jog_meta)
        return result

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
