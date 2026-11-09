"""PTS_CB01 serial protocol for PX-6AX GEN3 modules on one control box.

Frame: ``55 AA 7B 7B`` … LRC ``55 AA 7D 7D``. The baud setting is 460800 8N1;
on this USB CDC box the round trip is one USB frame, not the byte time.

Port masks ``0x01`` and ``0x04`` each latch one module into the application
registers. A combined mask is rejected (status ``0x11``). Address 1008 is the
resultant Fx/Fy/Fz. Address 1038 is taxel 0. Fx/Fy are signed int8, Fz is
unsigned. 1 LSB = 0.1 N.

The box does not report the module model. Addresses 1038..1475 are readable
and hold 146 complete taxels. No GEN3 model has 146 points; the 239-point pad
does not fit in that window. The host supplies the part code and this module
slices the window to that count.
"""

from __future__ import annotations

import time
from typing import Any

HEAD = bytes.fromhex("55 AA 7B 7B")
TAIL = bytes.fromhex("55 AA 7D 7D")
DEFAULT_BAUD = 460800
SENSORS = ((1, 0x01), (2, 0x04))
REST_ADDR = 1008
DIST_ADDR = 1038
WINDOW_END = 1475
READABLE_POINTS = (WINDOW_END - DIST_ADDR + 1) // 3  # 146
LSB_N = 0.1
_PREFIX = 6  # module status, 0x7B, addr LE, length LE

# Distinct point counts from the GEN3 manual. The part code is a host choice.
MODELS: dict[str, int] = {
    "PXSR-STDMC03A": 9,
    "PXSR-STDIP03B": 25,
    "PXSR-STDDP03F": 31,
    "PXSR-STDDP03D": 51,
    "PXSR-STDDP03G": 52,
    "PXSR-STDIP03A": 68,
    "PXSR-STDCP03B": 77,
    "PXSR-STDDP03E": 96,
    "PXSR-STDDP03C": 116,
    "PXSR-STDDP03B": 127,
    "PXSR-STDDP03A": 135,
    "PXSR-STDCP03A": 239,
}
DEFAULT_MODEL = "PXSR-STDDP03G"


def lrc(data: bytes) -> int:
    acc = 0
    for byte in data:
        acc = (acc + byte) & 0xFF
    return ((~acc) + 1) & 0xFF


def build(body: bytes) -> bytes:
    return HEAD + body + bytes([lrc(body)]) + TAIL


def reg_read_body(addr: int, length: int) -> bytes:
    param = bytes([0x7B]) + int(addr).to_bytes(2, "little") + int(length).to_bytes(2, "little")
    return bytes.fromhex("0E 00 70 C0 06") + len(param).to_bytes(2, "little") + param


def select_body(mask: int) -> bytes:
    return bytes.fromhex("0E 00 70 B1 0A 01 00") + bytes([mask & 0xFF])


def i8(value: int) -> int:
    return value - 256 if value >= 128 else value


def resolve_model(code: str | None) -> tuple[str, int]:
    """Return ``(part_code, n_points)``. Unknown codes raise ``KeyError``."""
    key = (code or DEFAULT_MODEL).strip().upper()
    if key not in MODELS:
        known = ", ".join(sorted(MODELS))
        raise KeyError(f"unknown Paxini part code {code!r}; known: {known}")
    return key, MODELS[key]


def full_read_length() -> int:
    """Register bytes from address 1008 through the readable taxel window."""
    return (DIST_ADDR - REST_ADDR) + READABLE_POINTS * 3


def payload_of(frame: bytes) -> bytes | None:
    if len(frame) < 16 or not frame.startswith(HEAD) or not frame.endswith(TAIL):
        return None
    if frame[9] != 0x00:
        return None
    length = int.from_bytes(frame[10:12], "little")
    data = frame[12 : 12 + length]
    return data if len(data) == length else None


def taxel_count(data: bytes) -> int:
    """Taxels at address 1038 that are force, not the unread-register fill.

    Past a module's own array the box sometimes returns ``n, n+1, n+2...``.
    Those bytes are not newtons. A zero tail does not reveal the boundary.
    """
    base = DIST_ADDR - REST_ADDR
    raw = data[base:]
    total = min(len(raw) // 3, READABLE_POINTS)
    usable = raw[: total * 3]
    if len(usable) < 24:
        return total
    end = len(usable) - 1
    start = end
    while start > 0 and ((usable[start - 1] + 1) & 0xFF) == usable[start]:
        start -= 1
    if end - start + 1 >= 24 and usable[start] != 0:
        return start // 3
    return total


def decode_registers(data: bytes, *, n_points: int | None = None) -> dict[str, Any]:
    """Decode register bytes that start at address 1008.

    ``n_points`` slices the distributed array. ``None`` keeps every force
    taxel, after dropping the unread-register counter fill.
    """
    fx = i8(data[0]) * LSB_N if len(data) > 0 else 0.0
    fy = i8(data[1]) * LSB_N if len(data) > 1 else 0.0
    fz = data[2] * LSB_N if len(data) > 2 else 0.0
    base = DIST_ADDR - REST_ADDR
    available = taxel_count(data)
    limit = available if n_points is None else max(0, min(available, int(n_points)))
    points: list[list[float]] = []
    for i in range(limit):
        off = base + i * 3
        if off + 2 >= len(data):
            break
        points.append(
            [
                round(i8(data[off]) * LSB_N, 3),
                round(i8(data[off + 1]) * LSB_N, 3),
                round(data[off + 2] * LSB_N, 3),
            ]
        )
    return {
        "fx": round(fx, 3),
        "fy": round(fy, 3),
        "fz": round(fz, 3),
        "points": points,
    }


def registers_from_payload(payload: bytes, *, n_points: int | None = None) -> dict[str, Any]:
    """Skip the 6-byte read prefix and decode the register bytes."""
    parsed = decode_registers(payload[_PREFIX:], n_points=n_points)
    parsed["module_status"] = payload[0] if payload else None
    return parsed


def read_frame(ser: Any, timeout_s: float) -> bytes:
    buf = bytearray()
    deadline = time.perf_counter() + timeout_s
    while time.perf_counter() < deadline:
        waiting = getattr(ser, "in_waiting", 0) or 0
        chunk = ser.read(waiting or 1)
        if not chunk:
            continue
        buf.extend(chunk)
        end = buf.rfind(TAIL)
        if end < 0:
            continue
        start = buf.rfind(HEAD, 0, end)
        if start >= 0:
            return bytes(buf[start : end + len(TAIL)])
    return bytes(buf)


class PtsCb01Box:
    """One open control box. Call from a single thread."""

    def __init__(self, port: str, baud: int = DEFAULT_BAUD) -> None:
        import serial

        self.port = port
        self.baud = int(baud)
        self.ser = serial.Serial(port, self.baud, timeout=0.05, write_timeout=0.2)
        time.sleep(0.1)
        self.ser.reset_input_buffer()
        version = payload_of(self._exchange(bytes.fromhex("0E 00 60 A0 01 00 00"), 1.2))
        self.version = (version or b"").decode("ascii", "replace").strip("\x00")
        mode = self._exchange(bytes.fromhex("0E 00 70 C0 0C 01 00 05"), 2.0)
        if len(mode) < 10 or mode[9] != 0x00:
            raise RuntimeError(f"set_mode failed: {mode.hex(' ')}")
        self.ser.timeout = 0
        length = full_read_length()
        self._full = build(reg_read_body(REST_ADDR, length))
        self._full_need = _PREFIX + length
        self._select = {mask: build(select_body(mask)) for _, mask in SENSORS}
        self._taxel_cap = {mask: READABLE_POINTS for _, mask in SENSORS}
        for _ in range(8):
            for index, mask in SENSORS:
                self.read_mask(index, mask, n_points=READABLE_POINTS)

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:  # noqa: BLE001
            pass

    def _exchange(self, body_or_frame: bytes, timeout_s: float, *, already_built: bool = False) -> bytes:
        frame = body_or_frame if already_built else build(body_or_frame)
        self.ser.write(frame)
        return read_frame(self.ser, timeout_s)

    def read_mask(self, index: int, mask: int, *, n_points: int) -> dict[str, Any]:
        host_ts = time.time()
        try:
            sel = self._exchange(self._select[mask], 0.05, already_built=True)
            if len(sel) < 10 or sel[9] != 0x00:
                raise RuntimeError(f"select 0x{mask:02X} status={sel[9] if len(sel) > 9 else -1}")
            frame = self._exchange(self._full, 0.05, already_built=True)
            data = payload_of(frame)
            if data is None or len(data) < self._full_need:
                status = frame[9] if len(frame) > 9 else -1
                raise RuntimeError(f"read status={status}")
            regs = data[_PREFIX:]
            found = taxel_count(regs)
            if found < self._taxel_cap[mask]:
                self._taxel_cap[mask] = found
            parsed = registers_from_payload(data, n_points=min(n_points, self._taxel_cap[mask]))
            return {
                "index": index,
                "mask": mask,
                "ok": True,
                "error": None,
                "host_ts": host_ts,
                **parsed,
            }
        except Exception as exc:  # noqa: BLE001
            try:
                self.ser.reset_input_buffer()
            except Exception:  # noqa: BLE001
                pass
            return {
                "index": index,
                "mask": mask,
                "ok": False,
                "error": str(exc),
                "host_ts": host_ts,
                "module_status": None,
                "fx": None,
                "fy": None,
                "fz": None,
                "points": [],
            }

    def read_both(self, *, n_points: int) -> list[dict[str, Any]]:
        return [self.read_mask(index, mask, n_points=n_points) for index, mask in SENSORS]

    def select_once(self, mask: int) -> bool:
        frame = self._exchange(self._select[mask], 0.05, already_built=True)
        return len(frame) >= 10 and frame[9] == 0x00

    def read_selected_once(self) -> bool:
        """One full register read. The port must already be selected."""
        frame = self._exchange(self._full, 0.03, already_built=True)
        data = payload_of(frame)
        return data is not None and len(data) >= self._full_need
