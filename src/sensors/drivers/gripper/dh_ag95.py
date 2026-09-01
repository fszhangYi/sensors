"""DH AG95 gripper over Modbus RTU (hik_gello compatible)."""

from __future__ import annotations

import random
import time
from pathlib import Path
from typing import Any, Mapping

from sensors.core.base import Sensor, SensorCapability, SensorContext
from sensors.core.config import coerce_bool
from sensors.core.health import CheckResult, HealthReport, HealthStatus
from sensors.core.kinds import SensorKind
from sensors.core.registry import register_sensor

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None  # type: ignore[assignment]

NORM_SCALE = 0.000637
REG_INIT = 0x0100
REG_FORCE = 0x0101
REG_SPEED = 0x0104
REG_POSITION = 0x0103
REG_INIT_STATE = 0x0200
REG_POSITION_FB = 0x0202
REG_FAULT = 0x0201

INIT_MAGIC = 0xA5
DEFAULT_FORCE = 50
DEFAULT_SPEED = 50


def _scalar(value: Any, default: float | int) -> float | int:
    if isinstance(value, (list, tuple)) and value:
        return value[0]
    if value is None:
        return default
    return value


def _crc16_modbus(data: bytes | list[int]) -> int:
    if isinstance(data, list):
        data = bytes(data)
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def norm_to_raw(norm: float) -> int:
    n = float(norm)
    return int(min(1.0, max(1.0 - n / 0.637, 0.0)) * 1000)


def raw_to_norm(raw: int) -> float:
    return (1000 - int(raw)) * NORM_SCALE


@register_sensor(SensorKind.GRIPPER)
class DhAg95Sensor(Sensor):
    kind = SensorKind.GRIPPER
    capabilities = (
        SensorCapability.PROBE
        | SensorCapability.SAMPLE
        | SensorCapability.CONTROL
        | SensorCapability.RATE_PROBE
    )

    def __init__(self, sensor_id: str, config: Mapping[str, Any], ctx: SensorContext | None = None):
        super().__init__(sensor_id, config, ctx)
        self.port = str(config.get("port") or config.get("endpoint") or "")
        self.baudrate = int(config.get("baudrate", 115200))
        self.slave_id = int(config.get("slave_id", 1))
        self.timeout_ms = int(config.get("timeout_ms", 50))
        self.default_force = int(_scalar(config.get("default_force") or config.get("force"), DEFAULT_FORCE))
        self.default_speed = int(_scalar(config.get("default_speed") or config.get("speed"), DEFAULT_SPEED))
        self.init_on_open = coerce_bool(config.get("init_on_open"), False)
        self.read_scale = float(_scalar(config.get("read_scale"), 1.0))
        self.read_offset = float(_scalar(config.get("read_offset"), 0.0))
        self.position_raw_min = config.get("position_raw_min")
        self.position_raw_max = config.get("position_raw_max")
        self._ser = None
        self._initialized = False
        self._tick = 0

    def probe(self) -> HealthReport:
        checks: list[CheckResult] = []
        if self.ctx.dry_run:
            checks.append(CheckResult("dry_run", True, "simulated"))
            return HealthReport(
                sensor_id=self.id,
                kind=self.kind.value,
                status=HealthStatus.OK,
                message="dry-run",
                checks=checks,
            )
        if serial is None:
            checks.append(CheckResult("pyserial", False, "pip install pyserial", critical=True))
            return HealthReport(
                sensor_id=self.id,
                kind=self.kind.value,
                status=HealthStatus.ERROR,
                message="pyserial missing",
                checks=checks,
            )
        port_ok = bool(self.port) and Path(self.port).exists()
        checks.append(CheckResult("port", port_ok, self.port or "(empty)", critical=True))
        status = HealthStatus.OK if port_ok else HealthStatus.OFFLINE
        return HealthReport(
            sensor_id=self.id,
            kind=self.kind.value,
            status=status,
            message="ready" if port_ok else "port missing",
            checks=checks,
        )

    def open(self) -> None:
        if self._opened:
            return
        if self.ctx.dry_run:
            self._opened = True
            return
        if serial is None:
            raise RuntimeError("pyserial required")
        self._ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=self.timeout_ms / 1000.0,
        )
        self._opened = True
        self._initialized = False
        if self.init_on_open:
            self.initialize()

    def initialize(self, **options: Any) -> dict[str, Any]:
        if not self._opened:
            raise RuntimeError(f"{self.id}: call open() before initialize()")
        timeout_s = float(options.get("timeout_s", 8.0))
        if self.ctx.dry_run:
            self._initialized = True
            return {
                "ok": True,
                "initialized": True,
                "skipped": False,
                "kind": self.kind.value,
                "dry_run": True,
            }
        ok = self._ensure_initialized(timeout_s=timeout_s)
        if ok:
            self._apply_force_speed()
        self._initialized = ok
        return {
            "ok": ok,
            "initialized": ok,
            "skipped": False,
            "kind": self.kind.value,
            "dry_run": False,
        }

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:  # noqa: BLE001
                pass
        self._ser = None
        self._opened = False
        self._initialized = False

    def _modbus_request(self, payload: list[int], *, expect_len: int | None = None) -> bytes | None:
        """Send one Modbus RTU frame and read the expected reply length.

        Critical: never ``read(256)`` with a long timeout. FC03/FC06 replies are
        7–8 bytes; waiting for 256 bytes burns the full serial timeout (~200ms)
        every transaction and caps throughput around ~5 Hz. hik_gello reads the
        exact expected length so a good reply returns as soon as it arrives.
        """
        if self._ser is None:
            return None
        func = int(payload[0]) if payload else 0
        if expect_len is None:
            if func == 0x06:
                expect_len = 8
            elif func == 0x03:
                qty = (int(payload[3]) << 8) | int(payload[4]) if len(payload) >= 5 else 1
                # addr + func + byte_count + 2*qty data + 2 CRC
                expect_len = 5 + 2 * max(1, qty)
            else:
                expect_len = 8
        expect_len = max(1, int(expect_len))

        frame = [self.slave_id] + list(payload)
        crc = _crc16_modbus(frame)
        frame.extend([crc & 0xFF, (crc >> 8) & 0xFF])
        self._ser.reset_input_buffer()
        self._ser.write(bytes(frame))
        self._ser.flush()
        resp = self._ser.read(expect_len)
        return resp if resp else None

    def _write_register(self, reg: int, value: int) -> bool:
        payload = [0x06, (reg >> 8) & 0xFF, reg & 0xFF, (value >> 8) & 0xFF, value & 0xFF]
        resp = self._modbus_request(payload, expect_len=8)
        return resp is not None and len(resp) >= 8

    def _read_register(self, reg: int) -> int | None:
        payload = [0x03, (reg >> 8) & 0xFF, reg & 0xFF, 0x00, 0x01]
        resp = self._modbus_request(payload, expect_len=7)
        if not resp or len(resp) < 7:
            return None
        return (resp[3] << 8) | resp[4]

    def _ensure_initialized(self, *, timeout_s: float = 8.0) -> bool:
        if self.ctx.dry_run:
            self._initialized = True
            return True
        if self._initialized:
            state = self._read_register(REG_INIT_STATE)
            if state == 1:
                return True
        if not self._write_register(REG_INIT, INIT_MAGIC):
            return False
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            state = self._read_register(REG_INIT_STATE)
            if state == 1:
                self._initialized = True
                return True
            time.sleep(0.05)
        return False

    def _require_initialized(self) -> bool:
        """Return True only when gripper init_state==1 (never auto-init)."""
        if self.ctx.dry_run:
            return True
        if self._initialized:
            return True
        state = self._read_register(REG_INIT_STATE)
        if state == 1:
            self._initialized = True
            return True
        return False

    def _apply_force_speed(self) -> None:
        self._write_register(REG_FORCE, self.default_force)
        self._write_register(REG_SPEED, self.default_speed)

    def _position_norm_from_raw(self, raw: int | None) -> float | None:
        if raw is None:
            return None
        if self.position_raw_min is not None and self.position_raw_max is not None:
            lo = int(self.position_raw_min)
            hi = int(self.position_raw_max)
            span = max(hi - lo, 1)
            open_norm = 0.0
            close_norm = 0.637
            t = (int(raw) - lo) / span
            return open_norm + (1.0 - t) * (close_norm - open_norm)
        return raw_to_norm(raw)

    def read(self) -> Mapping[str, Any]:
        if not self._opened:
            raise RuntimeError(f"{self.id}: call open() before read()")
        self._tick += 1
        t0 = time.perf_counter()
        if self.ctx.dry_run:
            norm = 0.25 * (1.0 + __import__("math").sin(time.time()))
            return {
                "ts": time.time(),
                "tick": self._tick,
                "dry_run": True,
                "position_norm": norm,
                "position_raw": int(norm_to_raw(norm)),
                "init_state": 1,
                "read_ms": (time.perf_counter() - t0) * 1000.0,
            }
        # Fast path: position every tick; init/fault polled sparsely (shared Modbus bus).
        # Rate probes call read() in a tight loop — keep extras rare so measured Hz
        # reflects position-register throughput (hik_gello GetCurrentPosition style).
        if self._tick == 1 or self._tick % 50 == 0 or not hasattr(self, "_cached_init"):
            self._cached_init = self._read_register(REG_INIT_STATE)
            self._cached_fault = self._read_register(REG_FAULT)
        init_state = getattr(self, "_cached_init", None)
        fault = getattr(self, "_cached_fault", None)
        raw = self._read_register(REG_POSITION_FB)
        norm = self._position_norm_from_raw(raw)
        if norm is not None:
            norm = norm * self.read_scale + self.read_offset
        return {
            "ts": time.time(),
            "tick": self._tick,
            "position_raw": raw,
            "position_norm": norm,
            "init_state": init_state,
            "fault": fault,
            "position_raw_min": self.position_raw_min,
            "position_raw_max": self.position_raw_max,
            "read_ms": (time.perf_counter() - t0) * 1000.0,
        }

    def write(self, command: Mapping[str, Any]) -> Mapping[str, Any]:
        if not self._opened:
            raise RuntimeError(f"{self.id}: call open() before write()")
        if command.get("calibrate"):
            return self.calibrate()
        if command.get("initialize"):
            return self.initialize()

        pos = command.get("position_norm")
        if pos is None and "position_raw" in command:
            raw = int(command["position_raw"])
            if not self._require_initialized():
                return {"ok": False, "error": "gripper not initialized; click Initialize first"}
            t0 = time.perf_counter()
            if self.ctx.dry_run:
                return {
                    "ok": True,
                    "position_raw": raw,
                    "write_ms": (time.perf_counter() - t0) * 1000.0,
                    "dry_run": True,
                }
            ok = self._write_register(REG_POSITION, raw)
            return {
                "ok": ok,
                "position_raw": raw,
                "write_ms": (time.perf_counter() - t0) * 1000.0,
            }

        if pos is None:
            return {"ok": False, "error": "position_norm required"}

        if not self._require_initialized():
            return {"ok": False, "error": "gripper not initialized; click Initialize first"}
        t0 = time.perf_counter()
        raw = norm_to_raw(float(pos))
        if self.ctx.dry_run:
            return {
                "ok": True,
                "position_norm": float(pos),
                "position_raw": raw,
                "write_ms": (time.perf_counter() - t0) * 1000.0,
                "wait_ms": int(command.get("wait_ms", 0) or 0),
                "dry_run": True,
            }
        ok = self._write_register(REG_POSITION, raw)
        # Optional settle only when caller explicitly asks — never a hidden 100/400ms gate.
        wait_ms = int(command.get("wait_ms", 0) or 0)
        if ok and wait_ms > 0 and not self.ctx.dry_run:
            time.sleep(wait_ms / 1000.0)
        return {
            "ok": ok,
            "position_norm": float(pos),
            "position_raw": raw,
            "write_ms": (time.perf_counter() - t0) * 1000.0,
            "wait_ms": wait_ms,
        }

    def calibrate(self, *, settle_s: float = 2.0, poll_s: float = 0.05) -> dict[str, Any]:
        """Fully close then open; record position_raw min/max for mapping."""
        if self.ctx.dry_run:
            self.position_raw_min = 12
            self.position_raw_max = 987
            return {
                "ok": True,
                "dry_run": True,
                "position_raw_min": self.position_raw_min,
                "position_raw_max": self.position_raw_max,
                "read_scale": self.read_scale,
                "read_offset": self.read_offset,
            }

        if not self._ensure_initialized():
            return {"ok": False, "error": "initialization failed"}
        self._apply_force_speed()

        def _sweep(target_raw: int, track_min: bool) -> int | None:
            self._write_register(REG_POSITION, target_raw)
            time.sleep(settle_s * 0.25)
            deadline = time.time() + settle_s
            extreme: int | None = None
            while time.time() < deadline:
                pos = self._read_register(REG_POSITION_FB)
                if pos is not None:
                    extreme = pos if extreme is None else (min(extreme, pos) if track_min else max(extreme, pos))
                time.sleep(poll_s)
            return extreme

        raw_close = _sweep(0, track_min=True)
        raw_open = _sweep(1000, track_min=False)
        if raw_close is None or raw_open is None:
            return {"ok": False, "error": "failed to read position during calibration"}

        self.position_raw_min = min(raw_close, raw_open)
        self.position_raw_max = max(raw_close, raw_open)

        return {
            "ok": True,
            "initialized": True,
            "position_raw_min": self.position_raw_min,
            "position_raw_max": self.position_raw_max,
            "position_raw_close": raw_close,
            "position_raw_open": raw_open,
            "position_norm_min": self._position_norm_from_raw(int(self.position_raw_max)),
            "position_norm_max": self._position_norm_from_raw(int(self.position_raw_min)),
            "read_scale": self.read_scale,
            "read_offset": self.read_offset,
        }

    def _probe_read_position_once(self) -> None:
        """One Modbus position feedback read (no init/fault polling)."""
        if self.ctx.dry_run:
            return
        if self._read_register(REG_POSITION_FB) is None:
            raise RuntimeError(f"{self.id}: position feedback read failed")

    def probe_max_read_hz(self, duration_s: float = 10.0) -> dict[str, Any]:
        """Tight Modbus REG_POSITION_FB loop — not sample read()."""
        from sensors.core.rate_probe import clamp_duration, rate_probe_fail, rate_probe_ok

        if not self._opened:
            raise RuntimeError(f"{self.id}: call open() before probe_max_read_hz()")
        duration_s = clamp_duration(duration_s)
        times: list[float] = []
        errors = 0
        last_error: str | None = None
        t0 = time.perf_counter()
        deadline = t0 + duration_s
        dry = bool(self.ctx.dry_run)
        while time.perf_counter() < deadline:
            try:
                self._probe_read_position_once()
                times.append(time.perf_counter())
            except Exception as e:  # noqa: BLE001
                errors += 1
                last_error = str(e)
                if errors >= 8 and len(times) < 3:
                    break
        if len(times) < 3:
            return rate_probe_fail(
                error=last_error or "gripper read probe produced too few samples",
                method="gripper_modbus_position_fb",
                samples=len(times),
                errors=errors,
                duration_s=time.perf_counter() - t0,
                dry_run=dry,
                last_error=last_error,
            )
        return rate_probe_ok(
            times,
            method="gripper_modbus_position_fb",
            errors=errors,
            t0=t0,
            dry_run=dry,
            last_error=last_error,
        )

    def probe_max_sync_write_hz(
        self,
        *,
        duration_s: float = 10.0,
        baseline_s: float = 1.5,
        value_min: float = 0.0,
        value_max: float = 0.637,
        hz_min: float = 0.2,
        hz_max: float = 200.0,
    ) -> dict[str, Any]:
        """Measure max 1:1 sync rate: random Modbus write then position read."""
        from sensors.core.rate_probe import cap_from_measured, clamp_duration, measure_rate_from_times

        if not self._opened:
            raise RuntimeError(f"{self.id}: call open() before probe_max_sync_write_hz()")
        hz_min = max(0.2, float(hz_min))
        hz_max = max(hz_min, float(hz_max))
        lo = float(value_min)
        hi = float(value_max)
        if hi < lo:
            lo, hi = hi, lo
        sync_duration_s = clamp_duration(duration_s)
        sync_duration_s = max(3.0, sync_duration_s)

        baseline = self.probe_max_read_hz(duration_s=max(0.5, float(baseline_s)))
        baseline_read_hz = baseline.get("measured_hz") if baseline.get("ok") else None

        times: list[float] = []
        write_ok = 0
        write_fail = 0
        read_fail = 0
        t_start = time.perf_counter()
        deadline = t_start + sync_duration_s
        dry = bool(self.ctx.dry_run)
        while time.perf_counter() < deadline:
            target = lo + random.random() * (hi - lo) if hi > lo else lo
            raw = norm_to_raw(target)
            try:
                if dry:
                    ok = True
                else:
                    ok = bool(self._write_register(REG_POSITION, raw))
            except Exception:  # noqa: BLE001
                ok = False
            if not ok:
                write_fail += 1
                if write_fail >= 8 and write_ok < 3:
                    break
                continue
            write_ok += 1
            try:
                self._probe_read_position_once()
                times.append(time.perf_counter())
            except Exception:  # noqa: BLE001
                read_fail += 1
                if read_fail >= 8 and len(times) < 3:
                    break

        measured = measure_rate_from_times(times)
        sync_elapsed = round(time.perf_counter() - t_start, 3)
        if measured is None:
            return {
                "ok": False,
                "error": "sync write-rate probe failed (too few successful write→read cycles)",
                "finish_reason": "too_few_samples",
                "duration_s": sync_elapsed,
                "baseline_read_hz": None if baseline_read_hz is None else round(float(baseline_read_hz), 3),
                "read_samples": 0,
                "write_samples": write_ok,
                "write_fail": write_fail,
                "read_fail": read_fail,
                "max_sync_write_hz": None,
                "method": "gripper_modbus_sync_write_read",
                "dry_run": dry,
            }

        cap = cap_from_measured(measured)
        if baseline_read_hz is not None and baseline_read_hz > 0:
            cap = min(cap, cap_from_measured(float(baseline_read_hz)))
        cap = min(hz_max, max(hz_min, cap))
        finish_reason = "measured"
        if abs(cap - hz_max) < 0.05 and measured * 0.95 >= hz_max - 0.05:
            finish_reason = "ceiling"

        return {
            "ok": True,
            "finish_reason": finish_reason,
            "duration_s": sync_elapsed,
            "sync_duration_s": round(sync_duration_s, 3),
            "write_measured_hz": round(measured, 3),
            "max_sync_write_hz": round(cap, 3),
            "baseline_read_hz": None if baseline_read_hz is None else round(float(baseline_read_hz), 3),
            "read_samples": len(times),
            "write_samples": write_ok,
            "write_fail": write_fail,
            "read_fail": read_fail,
            "method": "gripper_modbus_sync_write_read",
            "dry_run": dry,
        }
