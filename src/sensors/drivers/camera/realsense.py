from __future__ import annotations

import time
from typing import Any, Mapping

import numpy as np

from sensors.core.base import Sensor, SensorCapability, SensorContext
from sensors.core.config import coerce_bool
from sensors.core.health import CheckResult, HealthReport, HealthStatus
from sensors.core.kinds import SensorKind
from sensors.core.registry import register_sensor

# D400 color (BGR8) typically only accepts these discrete rates — not 5/10/20.
REALSENSE_COMMON_FPS = (6, 15, 30, 60)


def _normalize_role_serials(raw: Any) -> dict[str, list[str]]:
    """Optional role→serial hints only. Never used to pick or reject a device."""
    if not isinstance(raw, Mapping):
        return {}
    out: dict[str, list[str]] = {}
    for role, serials in raw.items():
        if serials is None:
            continue
        if isinstance(serials, (str, bytes)):
            items = [serials]
        elif isinstance(serials, (list, tuple, set)):
            items = list(serials)
        else:
            continue
        cleaned = [str(s).strip() for s in items if str(s).strip()]
        if cleaned:
            out[str(role).lower()] = cleaned
    return out


def _fps_fallback_order(requested: int) -> list[int]:
    """Prefer the requested rate, then the nearest supported discrete rates (lower first on ties)."""
    req = max(1, int(requested))
    known = {req, *REALSENSE_COMMON_FPS}
    return sorted(known, key=lambda f: (abs(f - req), f))


def _is_d405_name(name: str | None) -> bool:
    """True for Intel RealSense D405 (short-range wrist cams)."""
    n = (name or "").upper()
    return "D405" in n or "405" in n


def _colorize_depth_frame(colorizer: Any, depth_frame: Any) -> np.ndarray | None:
    """Pseudo-color depth via ``rs.colorizer`` (same path as show_d405.py)."""
    if colorizer is None or not depth_frame:
        return None
    try:
        colorized = colorizer.colorize(depth_frame)
        return np.asanyarray(colorized.get_data()).copy()
    except Exception:  # noqa: BLE001
        return None


def _intrinsics_list(intr: Any) -> list[float]:
    return [float(intr.fx), float(intr.fy), float(intr.ppx), float(intr.ppy)]


def _intrinsics_matrix(intr: Any) -> list[list[float]]:
    return [
        [float(intr.fx), 0.0, float(intr.ppx)],
        [0.0, float(intr.fy), float(intr.ppy)],
        [0.0, 0.0, 1.0],
    ]


def _extrinsics_matrix(extr: Any) -> list[list[float]]:
    """4x4 from RealSense extrinsics (rotation row-major 9 + translation 3)."""
    R = list(extr.rotation)
    t = list(extr.translation)
    return [
        [float(R[0]), float(R[1]), float(R[2]), float(t[0])],
        [float(R[3]), float(R[4]), float(R[5]), float(t[1])],
        [float(R[6]), float(R[7]), float(R[8]), float(t[2])],
        [0.0, 0.0, 0.0, 1.0],
    ]




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
        self.serial = ""
        self.width = int(config.get("width", 1280))
        self.height = int(config.get("height", 720))
        self.fps = int(config.get("fps", 15))
        self.exposure = int(config.get("exposure", 200))
        self.gain = int(config.get("gain", 64))
        self.enable_depth = coerce_bool(config.get("enable_depth"), True)
        self.align_to_color = coerce_bool(config.get("align_to_color"), True)
        self._stream_requested = (self.width, self.height, self.fps, self.enable_depth)
        self.timeout_ms = int(config.get("timeout_ms", 5000))
        # Optional display-only hints from YAML; empty by default (no lab whitelist).
        self.role_serials = _normalize_role_serials(config.get("role_serials"))
        self._pipeline = None
        self._align = None
        self._colorizer = None
        self._active_serial: str | None = None
        self._device_name: str | None = None
        self._fps_requested = self.fps
        self.depth_scale: float | None = None
        self.camera_infos: dict[str, Any] | None = None
        self._refresh_identity_config()

    def _match_role(self, serial: str) -> str | None:
        """Optional role hint from YAML ``role_serials`` (display only)."""
        for role, serials in self.role_serials.items():
            if serial and serial in serials:
                return role
        return None

    def _resolve_serial(self, devices: list[dict[str, str]] | None = None) -> str | None:
        """Device selection is serial-only from YAML/UI. No role/USB whitelist fallback."""
        _ = devices
        self._refresh_identity_config()
        return self.serial or None

    def probe(self) -> HealthReport:
        self._refresh_identity_config()
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
                inferred = self._match_role(sn)
                entry: dict[str, str] = {"serial": sn, "name": name}
                if inferred:
                    entry["inferred_role"] = inferred
                devices.append(entry)
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
                    "set params.serial to the camera serial number (any RealSense; no hardcoded whitelist)",
                    critical=True,
                )
            )
        elif rs_ok:
            found = any(d["serial"] == self.serial for d in devices)
            checks.append(CheckResult("serial_present", found, self.serial, critical=True))
            if found and self.role_serials:
                inferred = self._match_role(self.serial)
                if inferred is not None:
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
                "Cameras are matched by params.serial from YAML/UI only — edit serial freely per station",
                "Collect mosaic: hconcat(L,R) over hconcat(M, black) → 1440×2560",
                "Exposure/gain defaults match MegaCollect Realsense(1280,720,15)",
            ],
        )

    def _refresh_identity_config(self) -> None:
        """Re-read role/serial from ``self.config`` so YAML/UI edits always apply."""
        self.role = str(self.config.get("role", self.role or "middle")).lower()
        raw_serial = self.config.get("serial")
        if raw_serial is None or str(raw_serial).strip() == "":
            raw_serial = self.config.get("endpoint")
        self.serial = str(raw_serial).strip() if raw_serial is not None and str(raw_serial).strip() else ""
        if "role_serials" in self.config:
            self.role_serials = _normalize_role_serials(self.config.get("role_serials"))

    def _refresh_stream_config(self) -> None:
        self._refresh_identity_config()
        self.width = int(self.config.get("width", self.width))
        self.height = int(self.config.get("height", self.height))
        self.fps = int(float(self.config.get("fps", self.fps)))
        self.exposure = int(float(self.config.get("exposure", self.exposure)))
        self.gain = int(float(self.config.get("gain", self.gain)))
        self.enable_depth = coerce_bool(self.config.get("enable_depth"), self.enable_depth)
        self.align_to_color = coerce_bool(self.config.get("align_to_color"), self.align_to_color)
        self._stream_requested = (self.width, self.height, self.fps, self.enable_depth)

    def _pipeline_candidates(self) -> list[tuple[int, int, int, bool]]:
        """Ordered (w, h, fps, depth) tries.

        When depth is requested, exhaust depth-capable profiles (including
        640x480 like show_d405) **before** falling back to color-only. The old
        order tried ``depth=False`` at the requested resolution first, which
        silently disabled depth on D405 / picky USB setups.
        """
        req_w, req_h, req_fps, req_depth = self._stream_requested
        seen: set[tuple[int, int, int, bool]] = set()
        out: list[tuple[int, int, int, bool]] = []

        def add(w: int, h: int, fps: int, depth: bool) -> None:
            key = (w, h, fps, depth)
            if key not in seen:
                seen.add(key)
                out.append(key)

        fps_order = _fps_fallback_order(req_fps)
        res_order: list[tuple[int, int]] = []

        def add_res(w: int, h: int) -> None:
            key = (int(w), int(h))
            if key not in res_order:
                res_order.append(key)

        add_res(req_w, req_h)
        # show_d405 / D405: 640x480 color+depth is the reliable mode.
        if req_depth or _is_d405_name(self._device_name):
            add_res(640, 480)
        add_res(1280, 720)
        add_res(640, 480)

        # Pass 1: keep depth if requested.
        depth_flags = (True,) if req_depth else (False,)
        for w, h in res_order:
            for depth in depth_flags:
                for fps in fps_order:
                    add(w, h, fps, depth)
        # Pass 2: only then allow dropping depth.
        if req_depth:
            for w, h in res_order:
                for fps in fps_order:
                    add(w, h, fps, False)
        return out

    def _actual_video_fps(self, rs: Any, profile: Any) -> int:
        """Read the fps librealsense actually negotiated for the color stream."""
        try:
            stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
            return int(stream.fps())
        except Exception:  # noqa: BLE001
            return int(self.fps)


    def get_camera_infos(self) -> dict[str, Any] | None:
        """Intrinsics captured at pipeline start (None until open())."""
        return None if self.camera_infos is None else dict(self.camera_infos)

    def _capture_camera_infos(self, rs: Any, profile: Any, *, serial: str) -> dict[str, Any]:
        """Snapshot color/depth intrinsics (+ depth→color extrinsics) like hik_gello."""
        info: dict[str, Any] = {
            "serial": serial,
            "role": self.role,
            "width": int(self.width),
            "height": int(self.height),
            "fps": int(self.fps),
            "enable_depth": bool(self.enable_depth),
            "align_to_color": bool(self.align_to_color),
            "depth_scale": self.depth_scale,
        }
        try:
            device = profile.get_device()
            info["name"] = device.get_info(rs.camera_info.name)
            self._device_name = info["name"]
        except Exception:  # noqa: BLE001
            info["name"] = None

        try:
            color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
            color_intr = color_stream.get_intrinsics()
            info["color_intrinsics"] = _intrinsics_list(color_intr)
            info["intrinsic_matrix"] = _intrinsics_matrix(color_intr)
            info["color_distortion_coeffs"] = [float(c) for c in color_intr.coeffs]
            info["color_distortion_model"] = str(color_intr.model)
        except Exception as e:  # noqa: BLE001
            info["color_intrinsics_error"] = str(e)

        if self.enable_depth:
            try:
                depth_stream = profile.get_stream(rs.stream.depth).as_video_stream_profile()
                depth_intr = depth_stream.get_intrinsics()
                info["depth_intrinsics"] = _intrinsics_list(depth_intr)
                info["depth_intrinsic_matrix"] = _intrinsics_matrix(depth_intr)
                info["depth_distortion_coeffs"] = [float(c) for c in depth_intr.coeffs]
                info["depth_distortion_model"] = str(depth_intr.model)
                try:
                    color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
                    extr = depth_stream.get_extrinsics_to(color_stream)
                    info["depth_to_color"] = _extrinsics_matrix(extr)
                except Exception as e:  # noqa: BLE001
                    info["depth_to_color_error"] = str(e)
            except Exception as e:  # noqa: BLE001
                info["depth_intrinsics_error"] = str(e)
        return info

    def _dry_run_camera_infos(self) -> dict[str, Any]:
        """Placeholder K for dry-run (manifest still records stream geometry)."""
        w, h = float(self.width), float(self.height)
        fx = fy = 0.9 * max(w, h)
        cx, cy = w / 2.0, h / 2.0
        return {
            "serial": self.serial or None,
            "role": self.role,
            "width": int(self.width),
            "height": int(self.height),
            "fps": int(self.fps),
            "enable_depth": bool(self.enable_depth),
            "align_to_color": bool(self.align_to_color),
            "name": "dry-run",
            "dry_run": True,
            "color_intrinsics": [fx, fy, cx, cy],
            "intrinsic_matrix": [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
            "note": "synthetic intrinsics for dry_run; replace with real open() capture on hardware",
        }

    def open(self) -> None:
        if self.ctx.dry_run:
            self._refresh_stream_config()
            self.camera_infos = self._dry_run_camera_infos()
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
        present: dict[str, str] = {}
        for dev in ctx.query_devices():
            sn = dev.get_info(rs.camera_info.serial_number)
            present[sn] = dev.get_info(rs.camera_info.name)
        if serial not in present:
            raise RuntimeError(
                f"{self.id}: RealSense serial={serial!r} not found "
                f"(connected={sorted(present) or 'none'})"
            )
        self._device_name = present.get(serial)

        self._refresh_stream_config()
        # D405: show_d405 opens 640x480 color+depth; prefer that over failing 720p+depth.
        if self.enable_depth and _is_d405_name(self._device_name):
            if (self.width, self.height) == (1280, 720):
                self.width, self.height = 640, 480
                self._stream_requested = (self.width, self.height, self.fps, self.enable_depth)
        pipeline, profile = self._start_pipeline(rs, serial)
        # MegaCollect: disable auto-exposure, fixed exposure/gain
        try:
            device = profile.get_device()
            self._device_name = device.get_info(rs.camera_info.name)
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

        self.depth_scale = None
        if self.enable_depth:
            try:
                depth_sensor = profile.get_device().first_depth_sensor()
                self.depth_scale = float(depth_sensor.get_depth_scale())
            except Exception:  # noqa: BLE001
                self.depth_scale = None

        self._pipeline = pipeline
        self._align = rs.align(rs.stream.color) if self.enable_depth and self.align_to_color else None
        self._colorizer = rs.colorizer() if self.enable_depth else None
        self._active_serial = serial
        try:
            self.camera_infos = self._capture_camera_infos(rs, profile, serial=serial)
        except Exception as e:  # noqa: BLE001
            self.camera_infos = {
                "serial": serial,
                "role": self.role,
                "width": int(self.width),
                "height": int(self.height),
                "depth_scale": self.depth_scale,
                "error": str(e),
            }
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
        self._colorizer = None
        self._active_serial = None
        self.depth_scale = None
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
                "depth_scale": self.depth_scale,
                "color": None,
                "depth": None,
                "depth_colorized": None,
                "dry_run": True,
                "ts": ts,
                # HW timestamps absent in dry-run; kept for schema parity with live reads.
                "color_timestamp": None,
                "depth_timestamp": None,
                "color_timestamp_domain": None,
                "depth_timestamp_domain": None,
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
        # Hold frames while we extract color/depth (and colorize) — same as show_d405.
        try:
            frames.keep()
        except Exception:  # noqa: BLE001
            pass
        if self._align is not None:
            frames = self._align.process(frames)
            try:
                frames.keep()
            except Exception:  # noqa: BLE001
                pass

        color_frame = frames.get_color_frame()
        if not color_frame:
            raise RuntimeError(f"{self.id}: no color frame")
        color = np.asanyarray(color_frame.get_data()).copy()
        color_timestamp = float(color_frame.get_timestamp())
        color_timestamp_domain = str(color_frame.get_frame_timestamp_domain())

        depth = None
        depth_colorized = None
        depth_timestamp = None
        depth_timestamp_domain = None
        if self.enable_depth:
            depth_frame = frames.get_depth_frame()
            if depth_frame:
                depth = np.asanyarray(depth_frame.get_data()).copy()
                depth_timestamp = float(depth_frame.get_timestamp())
                depth_timestamp_domain = str(depth_frame.get_frame_timestamp_domain())
                depth_colorized = _colorize_depth_frame(self._colorizer, depth_frame)

        return {
            "serial": self._active_serial,
            "role": self.role,
            "width": self.width,
            "height": self.height,
            "fps": self.fps,
            "fps_requested": getattr(self, "_fps_requested", self.fps),
            "enable_depth": self.enable_depth,
            "align_to_color": self.align_to_color,
            "depth_scale": self.depth_scale,
            "color": color,
            "depth": depth,
            "depth_colorized": depth_colorized,
            "color_shape": tuple(color.shape),
            "depth_shape": tuple(depth.shape) if depth is not None else None,
            "ts": ts,
            # Librealsense device timestamps (usually ms). Alignment still uses wall ``ts``.
            "color_timestamp": color_timestamp,
            "depth_timestamp": depth_timestamp,
            "color_timestamp_domain": color_timestamp_domain,
            "depth_timestamp_domain": depth_timestamp_domain,
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
