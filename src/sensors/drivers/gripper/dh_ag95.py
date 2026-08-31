"""DH AG95 gripper over Modbus RTU (hik_gello compatible)."""

from __future__ import annotations

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
    capabilities = SensorCapability.PROBE | SensorCapability.SAMPLE | SensorCapability.CONTROL

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
            self._ensure_initialized()

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
            ok = self._ensure_initialized()
            if ok:
                self._apply_force_speed()
            return {"ok": ok, "initialized": ok}

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
