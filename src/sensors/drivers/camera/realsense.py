from __future__ import annotations

import time
from typing import Any, Mapping

import numpy as np

from sensors.core.base import Sensor, SensorCapability, SensorContext
from sensors.core.config import coerce_bool
from sensors.core.health import CheckResult, HealthReport, HealthStatus
from sensors.core.kinds import SensorKind
from sensors.core.registry import register_sensor

# MegaCollect save_data.py role tables (collect mosaic path)
DEFAULT_ROLE_SERIALS: dict[str, list[str]] = {
    "left": ["317222074437", "233622072962", "334622072861"],
    "right": ["317222073322", "233622076758", "337122074288"],
    "middle": [],
}

# D400 color (BGR8) typically only accepts these discrete rates — not 5/10/20.
REALSENSE_COMMON_FPS = (6, 15, 30, 60)


def _fps_fallback_order(requested: int) -> list[int]:
    """Prefer the requested rate, then the nearest supported discrete rates (lower first on ties)."""
    req = max(1, int(requested))
    known = {req, *REALSENSE_COMMON_FPS}
    return sorted(known, key=lambda f: (abs(f - req), f))


@register_sensor(SensorKind.REALSENSE)
class RealSenseSensor(Sensor):
    """Single RealSense node with a logical role (left|right|middle)."""

    kind = SensorKind.REALSENSE
    capabilities = (
        SensorCapability.PROBE
        | SensorCapability.SAMPLE
        | SensorCapability.PREVIEW
        | SensorCapability.STREAM
        | SensorCapability.RATE_PROBE
    )

    def __init__(self, sensor_id: str, config: Mapping[str, Any], ctx: SensorContext | None = None):
        super().__init__(sensor_id, config, ctx)
        self.role = str(config.get("role", "middle")).lower()
        self.serial = str(config.get("serial") or config.get("endpoint") or "")
        self.width = int(config.get("width", 1280))
        self.height = int(config.get("height", 720))
        self.fps = int(config.get("fps", 15))
        self.exposure = int(config.get("exposure", 200))
        self.gain = int(config.get("gain", 64))
        self.enable_depth = coerce_bool(config.get("enable_depth"), True)
        self.align_to_color = coerce_bool(config.get("align_to_color"), True)
        self._stream_requested = (self.width, self.height, self.fps, self.enable_depth)
        self.timeout_ms = int(config.get("timeout_ms", 5000))
        self.role_serials = dict(config.get("role_serials") or DEFAULT_ROLE_SERIALS)
        self._pipeline = None
        self._align = None
        self._active_serial: str | None = None
        self._fps_requested = self.fps

    def _match_role(self, serial: str) -> str:
        """Optional role hint from MegaCollect tables (display only; never used to pick a device)."""
        for role, serials in self.role_serials.items():
            if serial and serial in serials:
                return role
        return "middle"

    def _resolve_serial(self, devices: list[dict[str, str]] | None = None) -> str | None:
        """Device selection is serial-only. Empty serial → no device (no role/USB fallback)."""
        _ = devices
        return self.serial or None

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

        if not self.serial:
            checks.append(
                CheckResult(
                    "serial_required",
                    False,
                    "set params.serial to the camera serial number (no role/USB auto-pick)",
                    critical=True,
                )
            )
        elif rs_ok:
            found = any(d["serial"] == self.serial for d in devices)
            checks.append(CheckResult("serial_present", found, self.serial, critical=True))
            if found:
                inferred = self._match_role(self.serial)
                known = any(self.serial in (serials or []) for serials in self.role_serials.values())
                if known:
                    checks.append(
                        CheckResult(
                            "role_match",
                            inferred == self.role,
                            f"cfg={self.role} inferred={inferred}",
                            critical=False,
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
            message=f"RealSense serial={self.serial or 'unset'} role={self.role}",
            checks=checks,
            metrics=metrics,
            hints=[
                "Match cameras by serial only — fill params.serial (or UI 相机序列号)",
                "Collect mosaic: hconcat(L,R) over hconcat(M, black) → 1440×2560",
                "Exposure/gain defaults match MegaCollect Realsense(1280,720,15)",
            ],
        )

    def _refresh_stream_config(self) -> None:
        self.width = int(self.config.get("width", self.width))
        self.height = int(self.config.get("height", self.height))
        self.fps = int(float(self.config.get("fps", self.fps)))
        self.exposure = int(float(self.config.get("exposure", self.exposure)))
        self.gain = int(float(self.config.get("gain", self.gain)))
        self.enable_depth = coerce_bool(self.config.get("enable_depth"), self.enable_depth)
        self.align_to_color = coerce_bool(self.config.get("align_to_color"), self.align_to_color)
        self._stream_requested = (self.width, self.height, self.fps, self.enable_depth)

    def _pipeline_candidates(self) -> list[tuple[int, int, int, bool]]:
        req_w, req_h, req_fps, req_depth = self._stream_requested
        seen: set[tuple[int, int, int, bool]] = set()
        out: list[tuple[int, int, int, bool]] = []

        def add(w: int, h: int, fps: int, depth: bool) -> None:
            key = (w, h, fps, depth)
            if key not in seen:
                seen.add(key)
                out.append(key)

        fps_order = _fps_fallback_order(req_fps)
        # 1) Exact request (and depth off) before changing resolution
        for depth in (req_depth, False):
            for fps in fps_order:
                add(req_w, req_h, fps, depth)
        # 2) Common resolutions with nearest fps
        for w, h in ((1280, 720), (640, 480)):
            for depth in (req_depth, False):
                for fps in fps_order:
                    add(w, h, fps, depth)
        return out

    def _actual_video_fps(self, rs: Any, profile: Any) -> int:
        """Read the fps librealsense actually negotiated for the color stream."""
        try:
            stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
            return int(stream.fps())
        except Exception:  # noqa: BLE001
            return int(self.fps)

    def open(self) -> None:
        if self.ctx.dry_run:
            self._refresh_stream_config()
            self._opened = True
            return
        try:
            import pyrealsense2 as rs  # type: ignore
        except ImportError as e:
            raise RuntimeError("pyrealsense2 required for realsense open()") from e

        serial = self._resolve_serial()
        if not serial:
            raise RuntimeError(
                f"{self.id}: RealSense serial is required "
                f"(set params.serial / 相机序列号; role={self.role!r} is not used to pick a device)"
            )

        ctx = rs.context()
        present = {
            dev.get_info(rs.camera_info.serial_number)
            for dev in ctx.query_devices()
        }
        if serial not in present:
            raise RuntimeError(
                f"{self.id}: RealSense serial={serial!r} not found "
                f"(connected={sorted(present) or 'none'})"
            )

        self._refresh_stream_config()
        pipeline, profile = self._start_pipeline(rs, serial)
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
        for _ in range(5):
            try:
                pipeline.wait_for_frames(300)
            except Exception:  # noqa: BLE001
                break

    def _start_pipeline(self, rs: Any, serial: str) -> tuple[Any, Any]:
        """Pick a supported profile (resolve first to avoid long failed starts)."""
        req_w, req_h, req_fps, req_depth = self._stream_requested
        self._fps_requested = req_fps
        candidates = self._pipeline_candidates()

        last_err: Exception | None = None
        for w, h, fps, depth in candidates:
            pipeline = rs.pipeline()
            cfg = rs.config()
            cfg.enable_device(serial)
            try:
                cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
                if depth:
                    cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
                cfg.resolve(pipeline)
                profile = pipeline.start(cfg)
                self.width = w
                self.height = h
                self.fps = self._actual_video_fps(rs, profile) or fps
                self.enable_depth = depth
                if not depth:
                    self.align_to_color = False
                return pipeline, profile
            except Exception as e:  # noqa: BLE001
                last_err = e
                try:
                    pipeline.stop()
                except Exception:  # noqa: BLE001
                    pass

        hint = f"requested {req_w}x{req_h}@{req_fps} depth={req_depth}"
        raise RuntimeError(
            f"{self.id}: couldn't resolve requests ({hint}): {last_err}"
        ) from last_err

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
                "width": self.width,
                "height": self.height,
                "fps": self.fps,
                "fps_requested": getattr(self, "_fps_requested", self.fps),
                "enable_depth": self.enable_depth,
                "align_to_color": self.align_to_color,
                "color": None,
                "depth": None,
                "dry_run": True,
                "ts": ts,
            }

        frames = None
        for _ in range(3):
            try:
                frames = self._pipeline.wait_for_frames(min(self.timeout_ms, 1500))
                break
            except Exception:  # noqa: BLE001
                time.sleep(0.05)
        if frames is None:
            raise RuntimeError(f"{self.id}: no frames within {min(self.timeout_ms, 1500)}ms")
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
            "width": self.width,
            "height": self.height,
            "fps": self.fps,
            "fps_requested": getattr(self, "_fps_requested", self.fps),
            "enable_depth": self.enable_depth,
            "align_to_color": self.align_to_color,
            "color": color,
            "depth": depth,
            "color_shape": tuple(color.shape),
            "depth_shape": tuple(depth.shape) if depth is not None else None,
            "ts": ts,
        }

    def _probe_grab_frame_once(self) -> None:
        """One pipeline wait + color frame (no align / numpy sample dict)."""
        if self.ctx.dry_run or self._pipeline is None:
            return
        frames = self._pipeline.wait_for_frames(min(self.timeout_ms, 1500))
        if not frames.get_color_frame():
            raise RuntimeError(f"{self.id}: no color frame")

    def probe_max_read_hz(self, duration_s: float = 3.0) -> dict[str, Any]:
        """Tight wait_for_frames loop — not sample read()."""
        from sensors.core.rate_probe import cap_from_measured, clamp_duration, rate_probe_fail, rate_probe_ok

        if not self._opened:
            raise RuntimeError(f"{self.id}: call open() before probe_max_read_hz()")
        duration_s = clamp_duration(duration_s)
        times: list[float] = []
        errors = 0
        last_error: str | None = None
        t0 = time.perf_counter()
        deadline = t0 + duration_s
        dry = bool(self.ctx.dry_run or self._pipeline is None)
        while time.perf_counter() < deadline:
            try:
                self._probe_grab_frame_once()
                times.append(time.perf_counter())
            except Exception as e:  # noqa: BLE001
                errors += 1
                last_error = str(e)
                if errors >= 8 and len(times) < 3:
                    break
        if len(times) < 3:
            return rate_probe_fail(
                error=last_error or "realsense read probe produced too few frames",
                method="realsense_wait_for_frames",
                samples=len(times),
                errors=errors,
                duration_s=time.perf_counter() - t0,
                dry_run=dry,
                last_error=last_error,
            )
        result = rate_probe_ok(
            times,
            method="realsense_wait_for_frames",
            errors=errors,
            t0=t0,
            dry_run=dry,
            last_error=last_error,
        )
        if result.get("ok") and self.fps > 0:
            hw_cap = float(self.fps)
            result["hardware_fps_cap"] = hw_cap
            cap = min(float(result["read_cap_hz"]), cap_from_measured(hw_cap, margin=1.0))
            result["read_cap_hz"] = round(cap, 3)
        return result
