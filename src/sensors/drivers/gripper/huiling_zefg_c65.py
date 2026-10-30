"""慧灵 Z-EFG-C65 电动夹爪（Modbus RTU）— kind ``gripper_huiling``.

协议与 ``慧灵C65/huiling_clean`` 的 ``Reference.py`` / ``sealed_sensor.py`` 一致：
RS485、115200 8N1、从站 ID 默认 1；位置 mm（0 闭合～65 张开），速度 mm/s，电流 A。
浮点寄存器为 IEEE754 大端，占两个保持寄存器。

不依赖厂商 DLL；纯 ``pyserial``（与 DH AG95 驱动同风格）。
"""

from __future__ import annotations

import struct
import threading
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
    from serial.tools import list_ports as serial_list_ports
except ImportError:  # pragma: no cover
    serial = None  # type: ignore[assignment]
    serial_list_ports = None  # type: ignore[assignment]


POSITION_MIN_MM = 0.0
POSITION_MAX_MM = 65.0
SPEED_MIN_MM_S = 1.0
SPEED_MAX_MM_S = 100.0
CURRENT_MIN_A = 0.1
CURRENT_MAX_A = 0.5

REG_INIT = 0x0000
REG_POSITION = 0x0002
REG_SPEED = 0x0004
REG_CURRENT = 0x0006
REG_INIT_STATUS = 0x0040  # + grip @ 0x0041, pos/spd/cur floats @ 0x0042..

INIT_STATUS_TEXT = {
    0: "未初始化",
    5: "初始化完成",
}
GRIP_STATUS_TEXT = {
    0: "到位",
    1: "运动中",
    2: "夹持中",
    3: "掉落",
}

# Manual init complete code (手册 / Reference.py)
INIT_DONE_CODE = 5


def _scalar(value: Any, default: float | int) -> float | int:
    if isinstance(value, (list, tuple)) and value:
        return value[0]
    if value is None:
        return default
    return value


def _crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def _clamp(name: str, value: float, lo: float, hi: float) -> float:
    v = float(value)
    if v < lo or v > hi:
        raise ValueError(f"{name} out of range [{lo}, {hi}]: {v}")
    return v


def _port_present(port: str) -> bool:
    if not port:
        return False
    # Linux by-id / device nodes
    try:
        if Path(port).exists():
            return True
    except OSError:
        pass
    # Windows COMx
    if serial_list_ports is None:
        return port.upper().startswith("COM")
    try:
        names = {p.device.upper() for p in serial_list_ports.comports()}
        return port.upper() in names
    except Exception:  # noqa: BLE001
        return False


@register_sensor(SensorKind.GRIPPER_HUILING)
class HuilingZefgC65Sensor(Sensor):
    """慧灵 Z-EFG-C65 — ``kind=gripper_huiling``."""

    kind = SensorKind.GRIPPER_HUILING
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
        self.timeout_ms = int(config.get("timeout_ms", 300))
        self.default_speed_mm_s = float(
            _scalar(config.get("default_speed_mm_s") or config.get("default_speed"), 50.0)
        )
        self.default_current_a = float(
            _scalar(config.get("default_current_a") or config.get("default_current"), 0.3)
        )
        self.init_on_open = coerce_bool(config.get("init_on_open"), False)
        self.position_min_mm = float(config.get("position_min_mm", POSITION_MIN_MM))
        self.position_max_mm = float(config.get("position_max_mm", POSITION_MAX_MM))
        self._ser = None
        self._io_lock = threading.RLock()
        self._tick = 0
        self._initialized = False
        self._last_cmd_mm: float | None = None
        self._last_cmd_t: float | None = None

    # ---- lifecycle ----------------------------------------------------------

    def probe(self) -> HealthReport:
        checks: list[CheckResult] = []
        metrics: dict[str, Any] = {
            "port": self.port,
            "baudrate": self.baudrate,
            "slave_id": self.slave_id,
            "model": "Z-EFG-C65",
            "vendor": "Huiling",
        }
        if self.ctx.dry_run:
            checks.append(CheckResult("dry_run", True, "simulated"))
            return HealthReport(
                sensor_id=self.id,
                kind=self.kind.value,
                status=HealthStatus.OK,
                message="dry-run",
                checks=checks,
                metrics=metrics,
            )
        if serial is None:
            checks.append(CheckResult("pyserial", False, "pip install pyserial", critical=True))
            return HealthReport(
                sensor_id=self.id,
                kind=self.kind.value,
                status=HealthStatus.ERROR,
                message="pyserial missing",
                checks=checks,
                metrics=metrics,
            )
        port_ok = _port_present(self.port)
        checks.append(CheckResult("port", port_ok, self.port or "(empty)", critical=True))
        status = HealthStatus.OK if port_ok else HealthStatus.OFFLINE
        return HealthReport(
            sensor_id=self.id,
            kind=self.kind.value,
            status=status,
            message="ready" if port_ok else "port missing",
            checks=checks,
            metrics=metrics,
            hints=[
                "RS485 Modbus RTU 115200 8N1, slave_id=1",
                "position 0 mm=闭合, 65 mm=张开; init_code==5 表示已校准",
            ],
        )

    def open(self) -> None:
        if self._opened:
            return
        if self.ctx.dry_run:
            self._opened = True
            self._initialized = False
            return
        if serial is None:
            raise RuntimeError("pyserial required for gripper_huiling")
        if not self.port:
            raise RuntimeError(f"{self.id}: port/endpoint required")
        self._ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=max(0.05, self.timeout_ms / 1000.0),
        )
        self._opened = True
        self._initialized = False
        if self.init_on_open:
            self.initialize()

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:  # noqa: BLE001
                pass
        self._ser = None
        self._opened = False
        self._initialized = False

    def _finish_init_ok(self, *, init_code: int, t0: float) -> dict[str, Any]:
        self._initialized = True
        # Best-effort defaults; do not fail init if speed/current write glitches.
        self._write_float(REG_SPEED, self.default_speed_mm_s)
        self._write_float(REG_CURRENT, self.default_current_a)
        return {
            "ok": True,
            "initialized": True,
            "init_code": int(init_code),
            "kind": self.kind.value,
            "init_ms": (time.perf_counter() - t0) * 1000.0,
        }

    def initialize(self, **options: Any) -> dict[str, Any]:
        """手动找零（REG_INIT=1）。行程内勿放物体。init_code==5 为完成。

        Holds ``_io_lock`` for the whole sequence so gripper_read's poll
        cannot interleave Modbus frames and steal init responses.
        """
        if not self._opened:
            raise RuntimeError(f"{self.id}: call open() before initialize()")
        timeout_s = float(options.get("timeout_s", 30.0))
        if self.ctx.dry_run:
            self._initialized = True
            return {
                "ok": True,
                "initialized": True,
                "init_code": INIT_DONE_CODE,
                "kind": self.kind.value,
                "dry_run": True,
            }
        t0 = time.perf_counter()
        with self._io_lock:
            # Already homed — do not re-trigger REG_INIT (can look like a no-op).
            st0 = self._read_status_block()
            if st0 is not None and int(st0["init_code"]) == INIT_DONE_CODE:
                return self._finish_init_ok(init_code=INIT_DONE_CODE, t0=t0)

            wrote = self._write_u16(REG_INIT, 1)
            if not wrote:
                # Some USB-RS485 dongles drop/mangle FC06 echoes; re-check status.
                st_fail = self._read_status_block()
                if st_fail is not None and int(st_fail["init_code"]) == INIT_DONE_CODE:
                    return self._finish_init_ok(init_code=INIT_DONE_CODE, t0=t0)
                return {
                    "ok": False,
                    "initialized": False,
                    "init_code": None if st_fail is None else int(st_fail["init_code"]),
                    "error": "write REG_INIT failed (no FC06 echo; check COM/RS485)",
                    "kind": self.kind.value,
                    "init_ms": (time.perf_counter() - t0) * 1000.0,
                }
            deadline = time.time() + timeout_s
            last_code: int | None = None
            while time.time() < deadline:
                st = self._read_status_block()
                if st is not None:
                    last_code = int(st["init_code"])
                    if last_code == INIT_DONE_CODE:
                        return self._finish_init_ok(init_code=last_code, t0=t0)
                time.sleep(0.05)
            self._initialized = False
            return {
                "ok": False,
                "initialized": False,
                "init_code": last_code,
                "error": f"init timeout ({timeout_s}s); last init_code={last_code}",
                "kind": self.kind.value,
                "init_ms": (time.perf_counter() - t0) * 1000.0,
            }

    # ---- Modbus helpers -----------------------------------------------------

    def _transact(self, request: bytes, *, min_length: int) -> bytes | None:
        if self._ser is None:
            return None
        with self._io_lock:
            try:
                self._ser.reset_input_buffer()
                self._ser.write(request)
                self._ser.flush()
                deadline = time.monotonic() + max(0.05, self.timeout_ms / 1000.0)
                buf = bytearray()
                expected = min_length
                while time.monotonic() < deadline and len(buf) < expected:
                    waiting = getattr(self._ser, "in_waiting", 0) or 0
                    chunk = self._ser.read(waiting if waiting else 1)
                    if not chunk:
                        continue
                    buf.extend(chunk)
                    if len(buf) >= 2 and buf[1] & 0x80:
                        expected = 5
                    elif len(buf) >= 3 and buf[1] == 0x03:
                        expected = 5 + buf[2]
                if len(buf) < expected:
                    return None
                frame = bytes(buf[:expected])
            except Exception:  # noqa: BLE001
                return None
        if len(frame) < 4:
            return None
        body, crc = frame[:-2], frame[-2:]
        if struct.pack("<H", _crc16_modbus(body)) != crc:
            return None
        if body[1] & 0x80:
            return None
        if body[0] != self.slave_id:
            return None
        return body

    def _write_u16(self, address: int, value: int) -> bool:
        value = int(value) & 0xFFFF
        req = struct.pack(">BBHH", self.slave_id, 0x06, address, value)
        req += struct.pack("<H", _crc16_modbus(req))
        body = self._transact(req, min_length=8)
        # Accept any valid FC06 reply (exact echo is ideal but some adapters
        # alter padding; DH AG95 driver only checks length).
        return body is not None and len(body) >= 6 and body[1] == 0x06

    def _write_registers(self, address: int, payload: bytes) -> bool:
        if len(payload) == 0 or len(payload) % 2:
            return False
        count = len(payload) // 2
        head = struct.pack(
            ">BBHHB",
            self.slave_id,
            0x10,
            address,
            count,
            len(payload),
        )
        req = head + payload
        req += struct.pack("<H", _crc16_modbus(req))
        body = self._transact(req, min_length=8)
        return body is not None and len(body) >= 6 and body[1] == 0x10

    def _write_float(self, address: int, value: float) -> bool:
        return self._write_registers(address, struct.pack(">f", float(value)))

    def _read_registers(self, address: int, count: int) -> bytes | None:
        req = struct.pack(">BBHH", self.slave_id, 0x03, address, count)
        req += struct.pack("<H", _crc16_modbus(req))
        body = self._transact(req, min_length=5 + count * 2)
        if body is None or body[1] != 0x03 or body[2] != count * 2:
            return None
        return body[3:]

    def _read_status_block(self) -> dict[str, Any] | None:
        data = self._read_registers(REG_INIT_STATUS, 8)
        if data is None or len(data) != 16:
            return None
        init_code, grip_code = struct.unpack_from(">HH", data, 0)
        return {
            "init_code": int(init_code),
            "grip_code": int(grip_code),
            "position_mm": float(struct.unpack(">f", data[4:8])[0]),
            "speed_mm_s": float(struct.unpack(">f", data[8:12])[0]),
            "current_a": float(struct.unpack(">f", data[12:16])[0]),
        }

    def _require_initialized(self) -> bool:
        if self.ctx.dry_run:
            return True
        if self._initialized:
            return True
        st = self._read_status_block()
        if st is not None and int(st["init_code"]) == INIT_DONE_CODE:
            self._initialized = True
            return True
        return False

    def _mm_to_norm(self, mm: float) -> float:
        span = max(1e-6, self.position_max_mm - self.position_min_mm)
        return (float(mm) - self.position_min_mm) / span

    def _norm_to_mm(self, norm: float) -> float:
        n = min(1.0, max(0.0, float(norm)))
        return self.position_min_mm + n * (self.position_max_mm - self.position_min_mm)

    # ---- sample / control ---------------------------------------------------

    def read(self) -> Mapping[str, Any]:
        if not self._opened:
            raise RuntimeError(f"{self.id}: call open() before read()")
        self._tick += 1
        t0 = time.perf_counter()
        if self.ctx.dry_run:
            mm = 32.5 if self._last_cmd_mm is None else float(self._last_cmd_mm)
            return {
                "ts": time.time(),
                "tick": self._tick,
                "dry_run": True,
                "position_mm": mm,
                "speed_mm_s": 0.0,
                "current_a": 0.0,
                "position_norm": self._mm_to_norm(mm),
                "init_code": INIT_DONE_CODE,
                "grip_code": 0,
                "init_text": INIT_STATUS_TEXT[INIT_DONE_CODE],
                "grip_text": GRIP_STATUS_TEXT[0],
                "initialized": True,
                "read_ms": (time.perf_counter() - t0) * 1000.0,
            }
        st = self._read_status_block()
        if st is None:
            port_ok = _port_present(self.port)
            return {
                "ts": time.time(),
                "tick": self._tick,
                "ok": False,
                "dry_run": False,
                "error": (
                    f"status read failed on {self.port or '(no port)'}"
                    + ("" if port_ok else " (COM missing — plug in USB-RS485)")
                ),
                "port": self.port,
                "port_present": port_ok,
                "read_ms": (time.perf_counter() - t0) * 1000.0,
            }
        if int(st["init_code"]) == INIT_DONE_CODE:
            self._initialized = True
        mm = float(st["position_mm"])
        return {
            "ts": time.time(),
            "tick": self._tick,
            "ok": True,
            "dry_run": False,
            "position_mm": mm,
            "speed_mm_s": float(st["speed_mm_s"]),
            "current_a": float(st["current_a"]),
            "position_norm": self._mm_to_norm(mm),
            "init_code": int(st["init_code"]),
            "grip_code": int(st["grip_code"]),
            "init_text": INIT_STATUS_TEXT.get(int(st["init_code"]), "初始化中"),
            "grip_text": GRIP_STATUS_TEXT.get(int(st["grip_code"]), f"未知({st['grip_code']})"),
            "initialized": int(st["init_code"]) == INIT_DONE_CODE,
            "read_ms": (time.perf_counter() - t0) * 1000.0,
        }

    def write(self, command: Mapping[str, Any]) -> Mapping[str, Any]:
        if not self._opened:
            raise RuntimeError(f"{self.id}: call open() before write()")
        if command.get("initialize") or command.get("init"):
            return self.initialize(**{k: v for k, v in command.items() if k == "timeout_s"})

        t0 = time.perf_counter()
        # Resolve target opening (mm).
        position_mm: float | None = None
        if command.get("open") is True or str(command.get("action") or "").lower() == "open":
            position_mm = float(command.get("position_mm", self.position_max_mm))
        elif command.get("close") is True or str(command.get("action") or "").lower() == "close":
            position_mm = float(command.get("position_mm", self.position_min_mm))
        elif "position_mm" in command and command.get("position_mm") is not None:
            position_mm = float(command["position_mm"])
        elif "position_norm" in command and command.get("position_norm") is not None:
            position_mm = self._norm_to_mm(float(command["position_norm"]))

        speed = command.get("speed_mm_s")
        if speed is None and "speed" in command:
            speed = command.get("speed")
        current = command.get("current_a")
        if current is None and "current" in command:
            current = command.get("current")

        if position_mm is None and speed is None and current is None:
            return {
                "ok": False,
                "error": "need position_mm|position_norm|open|close and/or speed_mm_s|current_a",
            }

        try:
            if speed is not None:
                speed = _clamp("speed_mm_s", float(speed), SPEED_MIN_MM_S, SPEED_MAX_MM_S)
            if current is not None:
                current = _clamp("current_a", float(current), CURRENT_MIN_A, CURRENT_MAX_A)
            if position_mm is not None:
                position_mm = _clamp(
                    "position_mm",
                    float(position_mm),
                    self.position_min_mm,
                    self.position_max_mm,
                )
        except ValueError as e:
            return {"ok": False, "error": str(e)}

        if self.ctx.dry_run:
            if position_mm is not None:
                self._last_cmd_mm = position_mm
                self._last_cmd_t = time.time()
            return {
                "ok": True,
                "dry_run": True,
                "position_mm": position_mm,
                "position_norm": None if position_mm is None else self._mm_to_norm(position_mm),
                "speed_mm_s": speed,
                "current_a": current,
                "write_ms": (time.perf_counter() - t0) * 1000.0,
            }

        # Exclusive bus section: status check + writes must not interleave with read agent.
        with self._io_lock:
            if not self._require_initialized():
                st = self._read_status_block()
                if st is None:
                    return {
                        "ok": False,
                        "error": (
                            f"gripper status read failed on {self.port or '(no port)'} "
                            "(check USB-RS485 / COM, baud 115200, slave_id; "
                            "then click 初始化 until init_code==5)"
                        ),
                    }
                return {
                    "ok": False,
                    "error": (
                        "gripper not initialized; call initialize first "
                        f"(init_code={st.get('init_code')}, need {INIT_DONE_CODE})"
                    ),
                    "init_code": int(st["init_code"]),
                }

            ok = True
            if speed is not None:
                ok = self._write_float(REG_SPEED, float(speed)) and ok
            if current is not None:
                ok = self._write_float(REG_CURRENT, float(current)) and ok
            if position_mm is not None:
                ok = self._write_float(REG_POSITION, float(position_mm)) and ok
                if ok:
                    self._last_cmd_mm = float(position_mm)
                    self._last_cmd_t = time.time()

            wait_ms = int(command.get("wait_ms", 0) or 0)
            if ok and wait_ms > 0:
                time.sleep(wait_ms / 1000.0)

            out: dict[str, Any] = {
                "ok": ok,
                "position_mm": position_mm,
                "position_norm": None if position_mm is None else self._mm_to_norm(position_mm),
                "speed_mm_s": speed,
                "current_a": current,
                "write_ms": (time.perf_counter() - t0) * 1000.0,
                "wait_ms": wait_ms,
                "kind": self.kind.value,
            }
            if not ok:
                out["error"] = "modbus write failed (timeout / CRC / exception)"
            return out

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
        dry = bool(self.ctx.dry_run)
        while time.perf_counter() < deadline:
            try:
                if dry:
                    pass
                else:
                    st = self._read_status_block()
                    if st is None:
                        raise RuntimeError("status read failed")
                times.append(time.perf_counter())
            except Exception as e:  # noqa: BLE001
                errors += 1
                last_error = str(e)
                if errors >= 8 and len(times) < 3:
                    break
        if len(times) < 3:
            return rate_probe_fail(
                error=last_error or "huiling status probe too few samples",
                method="gripper_huiling_status_block",
                samples=len(times),
                errors=errors,
                duration_s=time.perf_counter() - t0,
                dry_run=dry,
                last_error=last_error,
            )
        return rate_probe_ok(
            times,
            method="gripper_huiling_status_block",
            errors=errors,
            t0=t0,
            dry_run=dry,
            last_error=last_error,
        )
