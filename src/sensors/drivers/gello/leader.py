from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

from sensors.core.base import Sensor, SensorCapability, SensorContext
from sensors.core.health import CheckResult, HealthReport, HealthStatus
from sensors.core.kinds import SensorKind
from sensors.core.registry import register_sensor
from sensors.drivers.bus.serial_bus import list_serial_by_id, path_rw

# Dynamixel Protocol 2.0 — Present Position (MegaCollect gello/dynamixel/driver.py)
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4


@register_sensor(SensorKind.GELLO)
class GelloLeaderSensor(Sensor):
    """Gello leader: Dynamixel over FTDI by-id (MegaCollect Client)."""

    kind = SensorKind.GELLO
    capabilities = SensorCapability.PROBE | SensorCapability.SAMPLE | SensorCapability.CONTROL

    def __init__(self, sensor_id: str, config: Mapping[str, Any], ctx: SensorContext | None = None):
        super().__init__(sensor_id, config, ctx)
        self.port = str(
            config.get("port")
            or config.get("endpoint")
            or "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAA088F-if00-port0"
        )
        self.baudrate = int(config.get("baudrate", 57600))
        self.joint_ids: Sequence[int] = list(config.get("joint_ids") or [1, 2, 3, 4, 5, 6, 7])
        self.max_delta_rad = float(config.get("max_delta_rad", 0.8))
        self.gripper_config = config.get("gripper_config")
        self.port_substr = str(config.get("port_substr", "FTAA088F"))
        self.joint_offsets = list(config.get("joint_offsets") or [0.0] * len(self.joint_ids))
        self.joint_signs = list(config.get("joint_signs") or [1] * len(self.joint_ids))
        self._port_handler = None
        self._packet_handler = None
        self._group_sync_read = None

    def probe(self) -> HealthReport:
        checks: list[CheckResult] = []
        metrics: dict[str, Any] = {
            "port": self.port,
            "baudrate": self.baudrate,
            "joint_ids": list(self.joint_ids),
            "max_delta_rad": self.max_delta_rad,
            "gripper_config": self.gripper_config,
        }

        exists = Path(self.port).exists()
        checks.append(CheckResult("serial_path", exists, self.port, critical=True))

        by_id = list_serial_by_id()
        hit = any(self.port_substr in p for p in by_id) or (exists and self.port_substr in self.port)
        checks.append(CheckResult("ftdi_by_id", hit, f"substr={self.port_substr}"))

        if exists:
            checks.append(CheckResult("permissions", path_rw(self.port), "R/W" if path_rw(self.port) else "denied"))

        try:
            import dynamixel_sdk  # noqa: F401

            metrics["dynamixel_sdk"] = True
        except ImportError:
            metrics["dynamixel_sdk"] = False
            checks.append(
                CheckResult("dynamixel_sdk", False, "optional: pip install 'hik-sensors[dynamixel]'")
            )

        status = HealthReport.aggregate_status(checks)
        if not exists:
            status = HealthStatus.OFFLINE

        return HealthReport(
            sensor_id=self.id,
            kind=self.kind.value,
            status=status,
            message="Gello Dynamixel leader",
            checks=checks,
            metrics=metrics,
            hints=[
                f"Refuse follow if max|leader−follower| > {self.max_delta_rad} rad",
                "Start Server before Client; align joints first",
            ],
        )

    def open(self) -> None:
        if self.ctx.dry_run:
            self._opened = True
            return
        try:
            from dynamixel_sdk import GroupSyncRead, PacketHandler, PortHandler
        except ImportError as e:
            raise RuntimeError("dynamixel_sdk required for gello open()") from e

        port_handler = PortHandler(self.port)
        if not port_handler.openPort():
            raise RuntimeError(f"Failed to open Dynamixel port {self.port}")
        if not port_handler.setBaudRate(self.baudrate):
            port_handler.closePort()
            raise RuntimeError(f"Failed to set baudrate {self.baudrate} on {self.port}")

        packet_handler = PacketHandler(2.0)
        group = GroupSyncRead(port_handler, packet_handler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        for dxl_id in self.joint_ids:
            if not group.addParam(dxl_id):
                port_handler.closePort()
                raise RuntimeError(f"Failed to add Dynamixel id={dxl_id} to GroupSyncRead")

        self._port_handler = port_handler
        self._packet_handler = packet_handler
        self._group_sync_read = group
        self._opened = True

    def close(self) -> None:
        if self._port_handler is not None:
            try:
                self._port_handler.closePort()
            except Exception:  # noqa: BLE001
                pass
        self._port_handler = None
        self._packet_handler = None
        self._group_sync_read = None
        self._opened = False

    def read(self) -> Mapping[str, Any]:
        super().read()
        ts = time.time()
        if self.ctx.dry_run or self._group_sync_read is None:
            return {
                "joints_raw_ticks": None,
                "joints_rad": None,
                "dry_run": True,
                "ts": ts,
            }

        from dynamixel_sdk.robotis_def import COMM_SUCCESS

        group = self._group_sync_read
        result = group.txRxPacket()
        if result != COMM_SUCCESS:
            raise RuntimeError(f"{self.id}: Dynamixel GroupSyncRead failed code={result}")

        ticks = []
        for dxl_id in self.joint_ids:
            if not group.isAvailable(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                raise RuntimeError(f"{self.id}: no Present Position for id={dxl_id}")
            raw = group.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            ticks.append(int(np.int32(np.uint32(raw))))

        rad = np.array(ticks, dtype=float) / 2048.0 * np.pi
        offsets = np.array(self.joint_offsets[: len(rad)], dtype=float)
        signs = np.array(self.joint_signs[: len(rad)], dtype=float)
        if len(offsets) < len(rad):
            offsets = np.pad(offsets, (0, len(rad) - len(offsets)))
        if len(signs) < len(rad):
            signs = np.pad(signs, (0, len(rad) - len(signs)), constant_values=1.0)
        calibrated = (rad - offsets) * signs

        return {
            "joints_raw_ticks": ticks,
            "joints_rad": calibrated.tolist(),
            "joints_rad_raw": rad.tolist(),
            "joint_ids": list(self.joint_ids),
            "port": self.port,
            "ts": ts,
        }

    def probe_max_read_hz(self, duration_s: float = 5.0) -> dict[str, Any]:
        """Tight Dynamixel GroupSyncRead loop — measures real bus round-trip rate."""
        import math

        duration_s = max(0.5, float(duration_s))
        times: list[float] = []
        errors = 0
        last_error: str | None = None
        t0 = time.perf_counter()
        deadline = t0 + duration_s
        while time.perf_counter() < deadline:
            try:
                self.read()
                times.append(time.perf_counter())
            except Exception as e:  # noqa: BLE001
                errors += 1
                last_error = str(e)
                if errors >= 8 and len(times) < 3:
                    break
        elapsed = max(1e-9, time.perf_counter() - t0)
        if len(times) < 3:
            return {
                "ok": False,
                "error": last_error or "gello read probe produced too few samples",
                "samples": len(times),
                "errors": errors,
                "duration_s": round(elapsed, 3),
                "method": "gello_sync_read_loop",
                "dry_run": bool(self.ctx.dry_run or self._group_sync_read is None),
            }
        span = times[-1] - times[0]
        measured = (len(times) - 1) / span if span > 0 else len(times) / elapsed
        cap = max(0.2, math.floor(measured * 0.95 * 10) / 10)
        dts = sorted((times[i] - times[i - 1]) * 1000.0 for i in range(1, len(times)))
        return {
            "ok": True,
            "samples": len(times),
            "errors": errors,
            "duration_s": round(elapsed, 3),
            "measured_hz": round(float(measured), 3),
            "read_cap_hz": cap,
            "method": "gello_sync_read_loop",
            "dry_run": bool(self.ctx.dry_run or self._group_sync_read is None),
            "dt_ms_p50": round(dts[len(dts) // 2], 3),
            "dt_ms_p95": round(dts[min(len(dts) - 1, int(len(dts) * 0.95))], 3),
            "dt_ms_mean": round(sum(dts) / len(dts), 3),
            "last_error": last_error,
        }
