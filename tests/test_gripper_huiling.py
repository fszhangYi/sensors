"""Unit tests for 慧灵 Z-EFG-C65 driver (no hardware)."""

from __future__ import annotations

import struct

from sensors.core.base import SensorContext
from sensors.core.kinds import SensorKind
from sensors.drivers.gripper.huiling_zefg_c65 import (
    HuilingZefgC65Sensor,
    _crc16_modbus,
)


def test_kind_and_handbook_float_encoding() -> None:
    assert SensorKind.GRIPPER_HUILING.value == "gripper_huiling"
    assert struct.pack(">f", 50.0) == bytes.fromhex("42480000")
    assert struct.pack(">f", 0.3) == bytes.fromhex("3E99999A")
    # Handbook sample CRC (Reference.py selftest)
    body = bytes.fromhex("010600000001")
    assert struct.pack("<H", _crc16_modbus(body)) == bytes.fromhex("480A")


def test_dry_run_lifecycle() -> None:
    s = HuilingZefgC65Sensor(
        "gripper-huiling",
        {"endpoint": "COM3", "baudrate": 115200},
        ctx=SensorContext(dry_run=True),
    )
    assert s.kind == SensorKind.GRIPPER_HUILING
    rep = s.probe()
    assert rep.status.name in ("OK", "ok") or str(rep.status).endswith("OK")
    s.open()
    init = s.initialize()
    assert init.get("ok") is True
    sample = s.read()
    assert "position_mm" in sample
    wr = s.write({"position_mm": 20.0, "speed_mm_s": 40.0, "current_a": 0.3})
    assert wr.get("ok") is True
    sample2 = s.read()
    assert abs(float(sample2["position_mm"]) - 20.0) < 1e-6
    s.close()


def test_norm_roundtrip() -> None:
    s = HuilingZefgC65Sensor("g", {}, ctx=SensorContext(dry_run=True))
    assert abs(s._norm_to_mm(0.0) - 0.0) < 1e-9
    assert abs(s._norm_to_mm(1.0) - 65.0) < 1e-9
    assert abs(s._mm_to_norm(32.5) - 0.5) < 1e-9


def test_initialize_does_not_hold_io_lock_across_wait() -> None:
    """Regression: init used to hold _io_lock for the whole timeout and starve read."""
    import threading

    s = HuilingZefgC65Sensor(
        "gripper-huiling",
        {"endpoint": "COM3"},
        ctx=SensorContext(dry_run=True),
    )
    s.open()
    # Dry-run initialize returns immediately; assert lock is free afterward.
    assert s.initialize().get("ok") is True
    assert s._io_lock.acquire(blocking=False)
    s._io_lock.release()
    # Concurrent read while a dummy holder waits should still be possible via RLock owner.
    held = threading.Event()
    released = threading.Event()

    def holder() -> None:
        with s._io_lock:
            held.set()
            released.wait(timeout=1.0)

    t = threading.Thread(target=holder, daemon=True)
    t.start()
    assert held.wait(timeout=1.0)
    # Non-owner cannot take the lock non-blocking (same as gripper_read agent).
    assert s._io_lock.acquire(blocking=False) is False
    released.set()
    t.join(timeout=1.0)
    s.close()
