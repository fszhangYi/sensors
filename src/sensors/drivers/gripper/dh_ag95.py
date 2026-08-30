from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Mapping

from sensors.core.base import Sensor, SensorCapability, SensorContext
from sensors.core.health import CheckResult, HealthReport, HealthStatus
from sensors.core.kinds import SensorKind
from sensors.core.registry import register_sensor
from sensors.drivers.bus.serial_bus import list_serial_by_id, path_rw

# DH AG95 Modbus map (MegaCollect / dh_ag95)
REG_INIT = 0x0100
REG_FORCE = 0x0101
REG_POSITION = 0x0103
REG_SPEED = 0x0104
REG_INIT_STATE = 0x0200
REG_GRIP_STATE = 0x0201
REG_POSITION_FB = 0x0202
INIT_MAGIC = 0xA5
NORM_SCALE = 0.000637


def _crc16_modbus(data: bytes | list[int]) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte & 0xFF
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


@register_sensor(SensorKind.GRIPPER)
class DhAg95Sensor(Sensor):
    """DH AG95 Modbus RTU gripper (follower-side in MegaCollect)."""

    kind = SensorKind.GRIPPER
    capabilities = SensorCapability.PROBE | SensorCapability.SAMPLE | SensorCapability.CONTROL

    def __init__(self, sensor_id: str, config: Mapping[str, Any], ctx: SensorContext | None = None):
        super().__init__(sensor_id, config, ctx)
        self.port = str(
            config.get("port")
            or config.get("endpoint")
            or "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AB0MIFS5-if00-port0"
        )
        self.baudrate = int(config.get("baudrate", 115200))
        self.slave_id = int(config.get("slave_id", 0x01))
        self.port_substr = str(config.get("port_substr", "AB0MIFS5"))
        self.default_force = int(config.get("force", 100))
        self.default_speed = int(config.get("speed", 100))
        self.init_on_open = bool(config.get("init_on_open", False))
        self._ser = None

    def probe(self) -> HealthReport:
        checks: list[CheckResult] = []
        metrics: dict[str, Any] = {
            "port": self.port,
            "baudrate": self.baudrate,
            "slave_id": self.slave_id,
            "registers": {
                "init": hex(REG_INIT),
                "position_cmd": hex(REG_POSITION),
                "position_fb": hex(REG_POSITION_FB),
                "init_magic": hex(INIT_MAGIC),
            },
            "norm_scale": NORM_SCALE,
            "force_default": self.default_force,
            "speed_default": self.default_speed,
            "note": "target_force is command, NOT external F/T",
        }

        exists = Path(self.port).exists()
        checks.append(CheckResult("serial_path", exists, self.port, critical=True))

        by_id = list_serial_by_id()
        hit = any(self.port_substr in p for p in by_id) or (exists and self.port_substr in self.port)
        checks.append(CheckResult("ftdi_by_id", hit, f"substr={self.port_substr}"))
        if exists:
            checks.append(CheckResult("permissions", path_rw(self.port), "R/W check"))

        try:
            import serial  # noqa: F401

            metrics["pyserial"] = True
        except ImportError:
            metrics["pyserial"] = False
            checks.append(CheckResult("pyserial", False, "optional: pip install 'hik-sensors[serial]'"))

        status = HealthReport.aggregate_status(checks)
        if not exists:
            status = HealthStatus.OFFLINE

        return HealthReport(
            sensor_id=self.id,
            kind=self.kind.value,
            status=status,
            message="DH AG95 Modbus gripper",
            checks=checks,
            metrics=metrics,
            hints=[
                "Init: write 0x0100=0xA5, poll 0x0200 until 1",
                "Open≈0, Close≈0.637 as 7th joint dim",
            ],
        )

    def _write_register(self, index: int, value: int) -> bool:
        assert self._ser is not None
        buf = bytearray(8)
        buf[0] = self.slave_id & 0xFF
        buf[1] = 0x06
        buf[2] = (index >> 8) & 0xFF
        buf[3] = index & 0xFF
        buf[4] = (value >> 8) & 0xFF
        buf[5] = value & 0xFF
        crc = _crc16_modbus(buf[:6])
        buf[6] = crc & 0xFF
        buf[7] = (crc >> 8) & 0xFF
        for _ in range(3):
            self._ser.reset_input_buffer()
            written = self._ser.write(buf)
            if written != 8:
                continue
            resp = self._ser.read(8)
            if len(resp) == 8:
                return True
        return False

    def _read_register(self, index: int) -> int | None:
        assert self._ser is not None
        buf = bytearray(8)
        buf[0] = self.slave_id & 0xFF
        buf[1] = 0x03
        buf[2] = (index >> 8) & 0xFF
        buf[3] = index & 0xFF
        buf[4] = 0x00
        buf[5] = 0x01
        crc = _crc16_modbus(buf[:6])
        buf[6] = crc & 0xFF
        buf[7] = (crc >> 8) & 0xFF
        for _ in range(3):
            self._ser.reset_input_buffer()
            written = self._ser.write(buf)
            if written != 8:
                continue
            resp = self._ser.read(7)
            if len(resp) == 7:
                return ((resp[3] & 0xFF) << 8) | (resp[4] & 0xFF)
        return None

    def open(self) -> None:
        if self.ctx.dry_run:
            self._opened = True
            return
        try:
            import serial
        except ImportError as e:
            raise RuntimeError("pyserial required for gripper open()") from e
        self._ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=0.2,
        )
        if self.init_on_open:
            self._write_register(REG_INIT, INIT_MAGIC)
            deadline = time.time() + 10.0
            while time.time() < deadline:
                st = self._read_register(REG_INIT_STATE)
                if st == 1:
                    break
                time.sleep(0.2)
            self._write_register(REG_FORCE, self.default_force)
            self._write_register(REG_SPEED, self.default_speed)
        self._opened = True

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:  # noqa: BLE001
                pass
            self._ser = None
        self._opened = False

    def read(self) -> Mapping[str, Any]:
        super().read()
        ts = time.time()
        if self.ctx.dry_run or self._ser is None:
            return {
                "position_raw": None,
                "position_norm": None,
                "init_state": None,
                "grip_state": None,
                "dry_run": True,
                "ts": ts,
            }
        pos = self._read_register(REG_POSITION_FB)
        init_st = self._read_register(REG_INIT_STATE)
        grip_st = self._read_register(REG_GRIP_STATE)
        # MegaCollect: (1000 - g_state) * 0.000637  → open≈0, close≈0.637
        pos_norm = None if pos is None else (1000 - pos) * NORM_SCALE
        return {
            "position_raw": pos,
            "position_norm": pos_norm,
            "init_state": init_st,
            "grip_state": grip_st,
            "port": self.port,
            "ts": ts,
        }
