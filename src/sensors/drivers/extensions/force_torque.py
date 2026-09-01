"""Wrist six-axis force/torque — HIK serial protocol (HIK_LMM_FORCE_TORQUE).

Reimplemented inside hik-sensors from the reference firmware spec:

  Start sampling : ``49 AA 0D 0A``
  Stop sampling  : ``43 AA 0D 0A``
  Data frame     : 28 bytes — ``49 AA`` + 6× float32 LE (Fx,Fy,Fz,Mx,My,Mz) + ``0D 0A``

Not Paxini tactile; not DH gripper target force.
"""

from __future__ import annotations

import math
import struct
import sys
import time
from pathlib import Path
from typing import Any, Mapping

from sensors.core.base import Sensor, SensorCapability, SensorContext
from sensors.core.health import CheckResult, HealthReport, HealthStatus
from sensors.core.kinds import SensorKind
from sensors.core.registry import register_sensor
from sensors.drivers.bus.serial_bus import list_serial_by_id

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None  # type: ignore[assignment]

# --- HIK F/T wire protocol -------------------------------------------------

START_CMD = bytes([0x49, 0xAA, 0x0D, 0x0A])
STOP_CMD = bytes([0x43, 0xAA, 0x0D, 0x0A])
FRAME_SIZE = 28
HEADER = (0x49, 0xAA)
TAIL = (0x0D, 0x0A)
DEFAULT_BAUDRATE = 460800
DEFAULT_TIMEOUT_MS = 500


def parse_wrench_frame(frame_bytes: bytes) -> tuple[list[float], list[float]]:
    """Parse a validated 28-byte HIK wrench frame into force[3], torque[3]."""
    if len(frame_bytes) != FRAME_SIZE:
        raise ValueError(f"frame size {len(frame_bytes)} != {FRAME_SIZE}")
    if frame_bytes[0:2] != bytes(HEADER) or frame_bytes[-2:] != bytes(TAIL):
        raise ValueError("invalid frame header/tail")
    fx, fy, fz, mx, my, mz = struct.unpack("<6f", frame_bytes[2:26])
    force = [float(fx), float(fy), float(fz)]
    torque = [float(mx), float(my), float(mz)]
    return force, torque


def normalize_serial_port(port: str) -> str:
    p = str(port).strip()
    if sys.platform == "win32" and p.upper().startswith("COM") and not p.startswith("\\\\.\\"):
        return f"\\\\.\\{p}"
    return p


def resolve_ft_port(port: str, port_substr: str) -> str:
    """Resolve serial path: explicit ``port`` wins; else Linux by-id substring match."""
    if port:
        return normalize_serial_port(port)
    if port_substr:
        for candidate in list_serial_by_id():
            if port_substr in candidate:
                return candidate
    return ""


def pop_wrench_frame_from_buffer(buf: bytearray, *, latest: bool = False) -> bytes | None:
    """Extract one validated 28-byte frame from a receive buffer.

    When ``latest`` is True (streaming), skip stale queued frames and return the
    most recent valid frame so continuous sampling stays in sync.
    """
    found_at: int | None = None
    i = 0
    while i <= len(buf) - FRAME_SIZE:
        if buf[i] == HEADER[0] and buf[i + 1] == HEADER[1]:
            if (
                buf[i + FRAME_SIZE - 2] == TAIL[0]
                and buf[i + FRAME_SIZE - 1] == TAIL[1]
            ):
                if not latest:
                    frame = bytes(buf[i : i + FRAME_SIZE])
                    del buf[: i + FRAME_SIZE]
                    return frame
                found_at = i
                i += FRAME_SIZE
                continue
        i += 1
    if latest and found_at is not None:
        end = found_at + FRAME_SIZE
        frame = bytes(buf[found_at:end])
        del buf[:end]
        return frame
    if len(buf) > FRAME_SIZE * 8:
        del buf[:-FRAME_SIZE]
    return None


@register_sensor(SensorKind.FT)
class ForceTorqueSensor(Sensor):
    """HIK wrist F/T over pyserial — same lifecycle as gello / gripper drivers."""

    kind = SensorKind.FT
    capabilities = SensorCapability.PROBE | SensorCapability.SAMPLE | SensorCapability.CONTROL

    def __init__(self, sensor_id: str, config: Mapping[str, Any], ctx: SensorContext | None = None):
        super().__init__(sensor_id, config, ctx)
        self.port_substr = str(config.get("port_substr") or "")
        self.port = resolve_ft_port(
            str(config.get("port") or config.get("endpoint") or ""),
            self.port_substr,
        )
        self.baudrate = int(config.get("baudrate", DEFAULT_BAUDRATE))
        self.timeout_ms = int(config.get("timeout_ms", DEFAULT_TIMEOUT_MS))
        self.bias = [float(x) for x in (config.get("bias") or [0.0] * 6)]
        while len(self.bias) < 6:
            self.bias.append(0.0)
        self._ser = None
        self._sampling = False
        self._rx_buf = bytearray()
        self._tick = 0

    def _refresh_port(self) -> None:
        self.port_substr = str(self.config.get("port_substr") or self.port_substr or "")
        self.port = resolve_ft_port(
            str(self.config.get("port") or self.config.get("endpoint") or ""),
            self.port_substr,
        )
        self.baudrate = int(self.config.get("baudrate", self.baudrate))
        self.timeout_ms = int(self.config.get("timeout_ms", self.timeout_ms))

    def probe(self) -> HealthReport:
        self._refresh_port()
        checks: list[CheckResult] = []
        metrics: dict[str, Any] = {
            "port": self.port or "(empty)",
            "baudrate": self.baudrate,
            "timeout_ms": self.timeout_ms,
            "port_substr": self.port_substr or None,
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

        port_ok = bool(self.port) and Path(self.port).exists()
        checks.append(CheckResult("serial_path", port_ok, self.port or "(empty)", critical=True))
        if self.port_substr and not self.config.get("port"):
            by_id = list_serial_by_id()
            hit = any(self.port_substr in p for p in by_id)
            checks.append(CheckResult("by_id_match", hit, f"substr={self.port_substr}"))

        status = HealthStatus.OK if port_ok else HealthStatus.OFFLINE
        return HealthReport(
            sensor_id=self.id,
            kind=self.kind.value,
            status=status,
            message="ready" if port_ok else "port missing",
            checks=checks,
            metrics=metrics,
            hints=[
                "Windows: fill port with COM5 (no port_substr needed)",
                "Linux: port=/dev/serial/by-id/… or port_substr=USB keyword when port empty",
                "HIK F/T: 460800 8N1, frame 28 bytes, start 49 AA 0D 0A",
            ],
        )

    def open(self) -> None:
        if self._opened:
            return
        if self.ctx.dry_run:
            self._opened = True
            return
        if serial is None:
            raise RuntimeError("pyserial required for force/torque open()")
        self._refresh_port()
        if not self.port:
            raise RuntimeError(f"{self.id}: port not configured (set port=COM5 or /dev/serial/by-id/…)")
        self._ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=0.05,
            rtscts=False,
            dsrdtr=False,
            xonxoff=False,
        )
        self._rx_buf = bytearray()
        self._write_cmd(START_CMD, reset=True)
        time.sleep(0.1)
        self._sampling = True
        self._opened = True

    def close(self) -> None:
        if self._ser is not None:
            try:
                if self._sampling:
                    self._write_cmd(STOP_CMD, reset=False)
            except Exception:  # noqa: BLE001
                pass
            try:
                self._ser.close()
            except Exception:  # noqa: BLE001
                pass
        self._ser = None
        self._sampling = False
        self._rx_buf = bytearray()
        self._opened = False

    def read(self) -> Mapping[str, Any]:
        if not self._opened:
            raise RuntimeError(f"{self.id}: call open() before read()")
        self._tick += 1
        if self.ctx.dry_run:
            t = time.time()
            force = [
                0.1 * math.sin(t),
                0.05 * math.cos(t),
                -9.8 + 0.02 * math.sin(t * 0.5),
            ]
            torque = [
                0.01 * math.sin(t * 2),
                0.02 * math.cos(t),
                0.005 * math.sin(t * 3),
            ]
            wrench = force + torque
            return {
                "ts": t,
                "tick": self._tick,
                "dry_run": True,
                "port": self.port,
                "baudrate": self.baudrate,
                "force": force,
                "torque": torque,
                "wrench": wrench,
                "bias": list(self.bias),
            }

        force, torque = self._read_wrench()
        raw_wrench = force + torque
        wrench = [raw_wrench[i] - self.bias[i] for i in range(6)]
        force = wrench[:3]
        torque = wrench[3:]
        return {
            "ts": time.time(),
            "tick": self._tick,
            "port": self.port,
            "baudrate": self.baudrate,
            "force": force,
            "torque": torque,
            "wrench": wrench,
            "bias": list(self.bias),
            "timestamp_ms": int(time.time() * 1000),
        }

    def write(self, command: Mapping[str, Any]) -> Mapping[str, Any]:
        if not self._opened:
            raise RuntimeError(f"{self.id}: call open() before write()")
        if command.get("tare"):
            if self.ctx.dry_run:
                self.bias = [0.0] * 6
                return {"ok": True, "tared": True, "bias": list(self.bias)}
            force, torque = self._read_wrench()
            self.bias = force + torque
            return {"ok": True, "tared": True, "bias": list(self.bias)}
        if "bias" in command:
            self.bias = [float(x) for x in command["bias"]][:6]
            while len(self.bias) < 6:
                self.bias.append(0.0)
            return {"ok": True, "bias": list(self.bias)}
        return {"ok": False, "error": "unsupported command; use tare or bias"}

    def _write_cmd(self, payload: bytes, *, reset: bool = False) -> None:
        if self._ser is None or not self._ser.is_open:
            raise RuntimeError("serial port not open")
        if reset:
            self._ser.reset_input_buffer()
        self._ser.write(payload)
        self._ser.flush()

    def _drain_serial(self) -> None:
        if self._ser is None:
            return
        waiting = self._ser.in_waiting
        if waiting:
            self._rx_buf.extend(self._ser.read(min(waiting, 4096)))
            return
        chunk = self._ser.read(1)
        if chunk:
            self._rx_buf.extend(chunk)
            waiting = self._ser.in_waiting
            if waiting:
                self._rx_buf.extend(self._ser.read(min(waiting, 4096)))

    def _pop_frame_from_buffer(self, *, latest: bool = True) -> bytes | None:
        return pop_wrench_frame_from_buffer(self._rx_buf, latest=latest)

    def _recover_stream(self) -> None:
        """Re-send START once after a read timeout; do not clear RX mid-stream."""
        if not self._sampling or self._ser is None:
            return
        self._write_cmd(START_CMD, reset=False)
        time.sleep(0.05)

    def _read_wrench(self) -> tuple[list[float], list[float]]:
        if self._ser is None:
            raise RuntimeError("serial port not open")
        deadline = time.time() + max(0.3, self.timeout_ms / 1000.0)
        while time.time() < deadline:
            self._drain_serial()
            frame = self._pop_frame_from_buffer(latest=True)
            if frame is not None:
                return parse_wrench_frame(frame)
            time.sleep(0.002)
        if self._sampling:
            self._recover_stream()
            retry_deadline = time.time() + max(0.15, self.timeout_ms / 2000.0)
            while time.time() < retry_deadline:
                self._drain_serial()
                frame = self._pop_frame_from_buffer(latest=True)
                if frame is not None:
                    return parse_wrench_frame(frame)
                time.sleep(0.002)
        raise TimeoutError(f"no F/T frame from {self.port} (baud={self.baudrate}, timeout={self.timeout_ms}ms)")

    def probe_max_read_hz(self, duration_s: float = 5.0) -> dict[str, Any]:
        """Count wrench frames from the continuous serial stream (not paced UI ticks).

        Live: drain RX and pop every valid 28-byte frame for ``duration_s``.
        dry-run: synthetic tight loop so the probe still returns real timings.
        """
        duration_s = max(0.5, float(duration_s))
        if self.ctx.dry_run:
            times: list[float] = []
            t0 = time.perf_counter()
            deadline = t0 + duration_s
            while time.perf_counter() < deadline:
                self.read()
                times.append(time.perf_counter())
            span = times[-1] - times[0] if len(times) >= 2 else 0.0
            measured = ((len(times) - 1) / span) if span > 0 else None
            if measured is None:
                return {
                    "ok": False,
                    "error": "dry-run read probe produced too few samples",
                    "samples": len(times),
                    "errors": 0,
                    "duration_s": round(time.perf_counter() - t0, 3),
                    "method": "ft_dry_run_loop",
                    "dry_run": True,
                }
            cap = max(0.2, math.floor(measured * 0.95 * 10) / 10)
            return {
                "ok": True,
                "samples": len(times),
                "errors": 0,
                "duration_s": round(time.perf_counter() - t0, 3),
                "measured_hz": round(measured, 3),
                "read_cap_hz": cap,
                "method": "ft_dry_run_loop",
                "dry_run": True,
            }

        if self._ser is None:
            return {"ok": False, "error": "serial port not open", "method": "ft_stream_frames"}

        # Warm the stream briefly so START has produced frames before we count.
        warm_deadline = time.perf_counter() + 0.25
        while time.perf_counter() < warm_deadline:
            self._drain_serial()
            if self._pop_frame_from_buffer(latest=True) is not None:
                break
            time.sleep(0.002)

        times: list[float] = []
        errors = 0
        t0 = time.perf_counter()
        deadline = t0 + duration_s
        while time.perf_counter() < deadline:
            try:
                self._drain_serial()
                got = False
                while True:
                    frame = self._pop_frame_from_buffer(latest=False)
                    if frame is None:
                        break
                    parse_wrench_frame(frame)
                    times.append(time.perf_counter())
                    got = True
                if not got:
                    time.sleep(0.0005)
            except Exception:  # noqa: BLE001
                errors += 1
                if errors >= 8 and len(times) < 3:
                    break
        elapsed = max(1e-9, time.perf_counter() - t0)
        if len(times) < 3:
            return {
                "ok": False,
                "error": f"too few F/T frames ({len(times)}); check port/baud/START stream",
                "samples": len(times),
                "errors": errors,
                "duration_s": round(elapsed, 3),
                "method": "ft_stream_frames",
                "dry_run": False,
            }
        span = times[-1] - times[0]
        measured = (len(times) - 1) / span if span > 0 else len(times) / elapsed
        cap = max(0.2, math.floor(measured * 0.95 * 10) / 10)
        dts = sorted((times[i] - times[i - 1]) * 1000.0 for i in range(1, len(times)))
        mid = dts[len(dts) // 2]
        p95 = dts[min(len(dts) - 1, int(len(dts) * 0.95))]
        return {
            "ok": True,
            "samples": len(times),
            "errors": errors,
            "duration_s": round(elapsed, 3),
            "measured_hz": round(float(measured), 3),
            "read_cap_hz": cap,
            "method": "ft_stream_frames",
            "dry_run": False,
            "dt_ms_p50": round(mid, 3),
            "dt_ms_p95": round(p95, 3),
            "dt_ms_mean": round(sum(dts) / len(dts), 3),
        }
