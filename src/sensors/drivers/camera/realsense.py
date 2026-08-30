from __future__ import annotations

import time
from typing import Any, Mapping

import numpy as np

from sensors.core.base import Sensor, SensorCapability, SensorContext
from sensors.core.health import CheckResult, HealthReport, HealthStatus
from sensors.core.kinds import SensorKind
from sensors.core.registry import register_sensor

# MegaCollect save_data.py role tables (collect mosaic path)
DEFAULT_ROLE_SERIALS: dict[str, list[str]] = {
    "left": ["317222074437", "233622072962", "334622072861"],
    "right": ["317222073322", "233622076758", "337122074288"],
    "middle": [],
}


@register_sensor(SensorKind.REALSENSE)
class RealSenseSensor(Sensor):
    """Single RealSense node with a logical role (left|right|middle)."""

    kind = SensorKind.REALSENSE
    capabilities = SensorCapability.PROBE | SensorCapability.SAMPLE | SensorCapability.PREVIEW | SensorCapability.STREAM

    def __init__(self, sensor_id: str, config: Mapping[str, Any], ctx: SensorContext | None = None):
        super().__init__(sensor_id, config, ctx)
        self.role = str(config.get("role", "middle")).lower()
        self.serial = str(config.get("serial") or config.get("endpoint") or "")
        self.width = int(config.get("width", 1280))
        self.height = int(config.get("height", 720))
        self.fps = int(config.get("fps", 15))
        self.exposure = int(config.get("exposure", 200))
        self.gain = int(config.get("gain", 64))
        self.enable_depth = bool(config.get("enable_depth", True))
        self.align_to_color = bool(config.get("align_to_color", True))
        self.timeout_ms = int(config.get("timeout_ms", 5000))
        self.role_serials = dict(config.get("role_serials") or DEFAULT_ROLE_SERIALS)
        self._pipeline = None
        self._align = None
        self._active_serial: str | None = None

    def _match_role(self, serial: str) -> str:
        for role, serials in self.role_serials.items():
            if serial and serial in serials:
                return role
        return "middle"

    def _resolve_serial(self, devices: list[dict[str, str]]) -> str | None:
        if self.serial:
            return self.serial
        mapped = [d["serial"] for d in devices if d["inferred_role"] == self.role]
        if mapped:
            return mapped[0]
        if self.role == "middle" and devices:
            # Unmapped devices are treated as middle
            unmapped = [d["serial"] for d in devices if d["inferred_role"] == "middle"]
            return unmapped[0] if unmapped else None
        return None

    def probe(self) -> HealthReport:
        checks: list[CheckResult] = []
        metrics: dict[str, Any] = {
            "role": self.role,
            "serial_cfg": self.serial or None,
            "stream": f"{self.width}x{self.height}@{self.fps}",
            "exposure": self.exposure,
            "gain": self.gain,
            "role_serials": self.role_serials,
        }

        devices: list[dict[str, str]] = []
        rs_ok = False
        try:
            import pyrealsense2 as rs  # type: ignore

            rs_ok = True
            ctx = rs.context()
            for dev in ctx.query_devices():
                sn = dev.get_info(rs.camera_info.serial_number)
                name = dev.get_info(rs.camera_info.name)
                devices.append({"serial": sn, "name": name, "inferred_role": self._match_role(sn)})
        except ImportError:
            checks.append(CheckResult("pyrealsense2", False, "optional: pip install 'hik-sensors[realsense]'"))
        except Exception as e:  # noqa: BLE001
            checks.append(CheckResult("realsense_enum", False, str(e), critical=False))

        metrics["pyrealsense2"] = rs_ok
        metrics["devices"] = devices
        metrics["device_count"] = len(devices)
        metrics["resolved_serial"] = self._resolve_serial(devices)

        if self.serial:
            found = any(d["serial"] == self.serial for d in devices)
            checks.append(CheckResult("serial_present", found or not rs_ok, self.serial, critical=False))
            if found:
                inferred = self._match_role(self.serial)
                checks.append(
                    CheckResult("role_match", inferred == self.role, f"cfg={self.role} inferred={inferred}")
                )
        else:
            mapped = [d for d in devices if d["inferred_role"] == self.role]
            checks.append(
                CheckResult(
                    "role_device",
                    bool(mapped) or not rs_ok,
                    f"{len(mapped)} device(s) for role={self.role}",
                )
            )

        if rs_ok and not devices:
            status = HealthStatus.OFFLINE
        else:
            status = HealthReport.aggregate_status(checks) if checks else (
                HealthStatus.OK if devices else HealthStatus.UNKNOWN
            )

        return HealthReport(
            sensor_id=self.id,
            kind=self.kind.value,
            status=status,
            message=f"RealSense role={self.role}",
            checks=checks,
            metrics=metrics,
            hints=[
                "Collect mosaic: hconcat(L,R) over hconcat(M, black) → 1440×2560",
                "Exposure/gain defaults match MegaCollect Realsense(1280,720,15)",
            ],
        )

    def open(self) -> None:
        if self.ctx.dry_run:
            self._opened = True
            return
        try:
            import pyrealsense2 as rs  # type: ignore
        except ImportError as e:
            raise RuntimeError("pyrealsense2 required for realsense open()") from e

        ctx = rs.context()
        devices: list[dict[str, str]] = []
        for dev in ctx.query_devices():
            sn = dev.get_info(rs.camera_info.serial_number)
            name = dev.get_info(rs.camera_info.name)
            devices.append({"serial": sn, "name": name, "inferred_role": self._match_role(sn)})

        serial = self._resolve_serial(devices)
        if not serial:
            raise RuntimeError(f"{self.id}: no RealSense device for role={self.role} serial={self.serial!r}")

        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(serial)
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        if self.enable_depth:
            config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)

        profile = pipeline.start(config)
        # MegaCollect: disable auto-exposure, fixed exposure/gain
        try:
            device = profile.get_device()
            for sensor in device.query_sensors():
                if sensor.is_color_sensor():
                    if sensor.supports(rs.option.enable_auto_exposure):
                        sensor.set_option(rs.option.enable_auto_exposure, 0)
                    if sensor.supports(rs.option.exposure):
                        sensor.set_option(rs.option.exposure, float(self.exposure))
                    if sensor.supports(rs.option.gain):
                        sensor.set_option(rs.option.gain, float(self.gain))
        except Exception:  # noqa: BLE001
            pass

        self._pipeline = pipeline
        self._align = rs.align(rs.stream.color) if self.enable_depth and self.align_to_color else None
        self._active_serial = serial
        self._opened = True

    def close(self) -> None:
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:  # noqa: BLE001
                pass
            self._pipeline = None
        self._align = None
        self._active_serial = None
        self._opened = False

    def read(self) -> Mapping[str, Any]:
        super().read()
        ts = time.time()
        if self.ctx.dry_run or self._pipeline is None:
            return {
                "serial": self.serial or None,
                "role": self.role,
                "color": None,
                "depth": None,
                "dry_run": True,
                "ts": ts,
            }

        frames = self._pipeline.wait_for_frames(self.timeout_ms)
        if self._align is not None:
            frames = self._align.process(frames)

        color_frame = frames.get_color_frame()
        if not color_frame:
            raise RuntimeError(f"{self.id}: no color frame")
        color = np.asanyarray(color_frame.get_data())

        depth = None
        if self.enable_depth:
            depth_frame = frames.get_depth_frame()
            if depth_frame:
                depth = np.asanyarray(depth_frame.get_data())

        return {
            "serial": self._active_serial,
            "role": self.role,
            "color": color,
            "depth": depth,
            "color_shape": tuple(color.shape),
            "depth_shape": tuple(depth.shape) if depth is not None else None,
            "ts": ts,
        }
