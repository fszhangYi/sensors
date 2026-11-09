"""Paxini tactile.

Default transport is the PTS_CB01 control box (``paxini_pts``): two modules
on one serial port, resultant plus distributed taxels, 0.1 N per LSB.
``transport: dll`` keeps the older ReadPaxiniLib path (60-point
``PAXINI_OUTPUT``, often viewed as 6×10). That DLL is a different product
from the GEN3 UART box.

This is tactile, not wrist six-axis F/T (``HIK_LMM_FORCE_TORQUE``).
"""

from __future__ import annotations

import math
import os
import stat
import time
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from sensors.core.base import Sensor, SensorCapability, SensorContext
from sensors.core.health import CheckResult, HealthReport, HealthStatus
from sensors.core.kinds import SensorKind
from sensors.core.registry import register_sensor
from sensors.drivers.extensions.paxini_pts import (
    DEFAULT_BAUD,
    DEFAULT_MODEL,
    LSB_N,
    READABLE_POINTS,
    PtsCb01Box,
    resolve_model,
)

COMP_COUNT = 60
GRID_HW = (6, 10)  # reshape used in get_paxini_data.py comments


def resolve_transport(config: Mapping[str, Any]) -> str:
    """``pts_cb01`` unless the config asks for the ReadPaxiniLib DLL."""
    explicit = str(config.get("transport") or "").strip().lower()
    if explicit in ("dll", "paxini_dll", "readpaxinilib"):
        return "dll"
    if explicit in ("pts", "pts_cb01", "serial", "box"):
        return "pts_cb01"
    if str(config.get("dll_path") or "").strip():
        return "dll"
    return "pts_cb01"


def _port_available(port: str) -> bool:
    if not port:
        return False
    try:
        return all(
            [
                os.path.exists(port),
                os.access(port, os.R_OK),
                os.access(port, os.W_OK),
                stat.S_ISCHR(os.stat(port).st_mode),
            ]
        )
    except OSError:
        return False


@register_sensor(SensorKind.TACTILE)
class PaxiniTactileSensor(Sensor):
    """Paxini tactile. Default is the PTS_CB01 box; ``transport: dll`` is ReadPaxiniLib."""

    kind = SensorKind.TACTILE
    capabilities = (
        SensorCapability.PROBE
        | SensorCapability.SAMPLE
        | SensorCapability.PREVIEW
        | SensorCapability.RATE_PROBE
    )

    def __init__(self, sensor_id: str, config: Mapping[str, Any], ctx: SensorContext | None = None):
        super().__init__(sensor_id, config, ctx)
        self.dll_path = str(config.get("dll_path") or "")
        self.paxini_channel = int(config.get("paxini_channel", 5))
        self.port = str(config.get("port") or config.get("endpoint") or "")
        self.group_index = int(config.get("group_index", 0))
        self.transport = resolve_transport(config)
        self.baud = int(config.get("baud") or config.get("baudrate") or DEFAULT_BAUD)
        self.model = DEFAULT_MODEL
        self.n_points = 52
        if self.transport == "pts_cb01":
            self.model, self.n_points = resolve_model(str(config.get("model") or DEFAULT_MODEL))
        self._tick = 0
        self._lib = None
        self._handle = None
        self._buf = None
        self._box: PtsCb01Box | None = None

    def _synthetic_group(self, idx: int) -> dict[str, Any]:
        t = self._tick * 0.18 + idx * 0.7
        # Resultant contact-like force (N-scale), Z is primary like get_force()
        rest = [
            0.12 * math.sin(t * 0.9 + idx),
            0.10 * math.cos(t * 1.1),
            max(0.05, 0.8 + 0.6 * math.sin(t * 0.65)),
        ]
        # 6×10 contact patch on Z with small XY noise
        gy, gx = np.mgrid[0 : GRID_HW[0], 0 : GRID_HW[1]]
        cy, cx = 2.5 + 0.4 * math.sin(t), 4.5 + 0.6 * math.cos(t * 0.8)
        blob = np.exp(-((gy - cy) ** 2 + (gx - cx) ** 2) / 4.5) * rest[2]
        comp = np.zeros((COMP_COUNT, 3), dtype=np.float64)
        flat = blob.reshape(-1)
        for j in range(COMP_COUNT):
            comp[j, 0] = 0.02 * math.sin(t + j * 0.1)
            comp[j, 1] = 0.02 * math.cos(t + j * 0.13)
            comp[j, 2] = float(flat[j])
        ori = np.clip((comp / max(rest[2], 1e-3) * 80 + 40), 0, 255).astype(np.uint8)
        return {
            "rest_force": [float(x) for x in rest],
            "component_forces": comp.tolist(),
            "original_forces": ori.tolist(),
            "contact_force": float(rest[2]),
            "grid_shape": [GRID_HW[0], GRID_HW[1], 3],
        }

    def _try_bind_dll(self) -> tuple[bool, str]:
        if not self.dll_path:
            return False, "dll_path not set"
        path = Path(self.dll_path)
        if not path.is_file():
            return False, f"missing {self.dll_path}"
        try:
            import ctypes
            from ctypes import POINTER, Structure, c_float, c_int, c_void_p

            class FORCE_3D(Structure):
                _fields_ = [("x", c_float), ("y", c_float), ("z", c_float)]

            class FORCE_UINT8(Structure):
                _fields_ = [
                    ("x", ctypes.c_ubyte),
                    ("y", ctypes.c_ubyte),
                    ("z", ctypes.c_ubyte),
                ]

            class PAXINI_FORCE(Structure):
                _fields_ = [
                    ("ori_force", FORCE_UINT8 * COMP_COUNT),
                    ("comp_force", FORCE_3D * COMP_COUNT),
                    ("rest_force", FORCE_3D),
                ]

            class PAXINI_OUTPUT(Structure):
                _fields_ = [("force_group", PAXINI_FORCE * 10), ("num", c_int)]

            lib = ctypes.CDLL(str(path))
            lib.create_paxini_handle.argtypes = [POINTER(c_void_p), c_int]
            lib.create_paxini_handle.restype = c_int
            lib.destroy_paxini_handle.argtypes = [c_void_p]
            lib.destroy_paxini_handle.restype = c_int
            lib.get_paxini_data.argtypes = [c_void_p, POINTER(PAXINI_OUTPUT)]
            lib.get_paxini_data.restype = c_int

            handle = c_void_p()
            ret = lib.create_paxini_handle(ctypes.byref(handle), self.paxini_channel)
            if ret != 0:
                return False, f"create_paxini_handle={ret}"

            self._lib = lib
            self._handle = handle
            self._buf = PAXINI_OUTPUT()
            self._FORCE_3D = FORCE_3D  # keep for clarity
            return True, f"loaded {path.name} channel={self.paxini_channel}"
        except Exception as e:  # noqa: BLE001
            self._lib = None
            self._handle = None
            self._buf = None
            return False, str(e)

    def _read_live_groups(self) -> list[dict[str, Any]]:
        if self._lib is None or self._handle is None or self._buf is None:
            raise RuntimeError(f"{self.id}: Paxini not opened")
        import ctypes

        ret = self._lib.get_paxini_data(self._handle, ctypes.byref(self._buf))
        if ret != 0:
            raise RuntimeError(f"{self.id}: get_paxini_data={ret}")
        out = self._buf
        groups: list[dict[str, Any]] = []
        n = max(0, min(int(out.num), 10))
        for i in range(n):
            g = out.force_group[i]
            rest = [float(g.rest_force.x), float(g.rest_force.y), float(g.rest_force.z)]
            comp = []
            ori = []
            for j in range(COMP_COUNT):
                f = g.comp_force[j]
                comp.append([float(f.x), float(f.y), float(f.z)])
                o = g.ori_force[j]
                ori.append([int(o.x), int(o.y), int(o.z)])
            groups.append(
                {
                    "rest_force": rest,
                    "component_forces": comp,
                    "original_forces": ori,
                    "contact_force": float(rest[2]),
                    "grid_shape": [GRID_HW[0], GRID_HW[1], 3],
                }
            )
        return groups

    def _pts_module(self, index: int, mask: int, fx: float, fy: float, fz: float, points: list[list[float]]) -> dict[str, Any]:
        return {
            "index": index,
            "mask": mask,
            "ok": True,
            "error": None,
            "fx": round(fx, 3),
            "fy": round(fy, 3),
            "fz": round(fz, 3),
            "points": points,
            "rest_force": [round(fx, 3), round(fy, 3), round(fz, 3)],
            "contact_force": round(fz, 3),
        }

    def _synthetic_pts_modules(self) -> list[dict[str, Any]]:
        modules = []
        for slot, (index, mask) in enumerate(((1, 0x01), (2, 0x04))):
            t = self._tick * 0.18 + slot * 0.7
            fx = 0.1 * math.sin(t)
            fy = 0.1 * math.cos(t * 1.1)
            fz = max(0.0, 0.4 + 0.3 * math.sin(t * 0.65))
            points = []
            count = min(self.n_points, READABLE_POINTS)
            center = (count - 1) * (0.5 + 0.15 * math.sin(t))
            for i in range(count):
                blob = math.exp(-((i - center) ** 2) / 18.0) * fz
                points.append([round(0.02 * math.sin(t + i), 3), round(0.02 * math.cos(t + i), 3), round(blob, 3)])
            modules.append(self._pts_module(index, mask, fx, fy, fz, points))
        return modules

    def _sample_from_modules(self, modules: list[dict[str, Any]], *, version: str | None) -> dict[str, Any]:
        usable = [m for m in modules if m.get("points")]
        gi = 0
        if usable:
            gi = max(0, min(self.group_index, len(usable) - 1))
        primary = usable[gi] if usable else self._pts_module(1, 0x01, 0.0, 0.0, 0.0, [])
        points = primary.get("points") or []
        shown = len(points)
        return {
            "ts": time.time(),
            "dry_run": bool(self.ctx.dry_run),
            "transport": "pts_cb01",
            "version": version,
            "port": self.port or None,
            "baud": self.baud,
            "model": self.model,
            "n_points": self.n_points,
            "points_shown": shown,
            "points_truncated": self.n_points > READABLE_POINTS,
            "readable_points": READABLE_POINTS,
            "lsb_n": LSB_N,
            "num": len(modules),
            "group_index": gi,
            "modules": modules,
            "groups": modules,
            "rest_force": primary.get("rest_force") or [0.0, 0.0, 0.0],
            "contact_force": primary.get("contact_force", 0.0),
            "component_forces": points,
            "points": points,
            "grid_shape": None,
        }

    def _probe_pts(self) -> HealthReport:
        checks: list[CheckResult] = []
        metrics: dict[str, Any] = {
            "transport": "pts_cb01",
            "port": self.port or "—",
            "baud": self.baud,
            "model": self.model,
            "n_points": self.n_points,
            "readable_points": READABLE_POINTS,
            "lsb_n": LSB_N,
            "schema": "PTS_CB01 / 0x7B addr 1008+1038",
        }
        if self.ctx.dry_run:
            checks.append(CheckResult("dry_run", True, "synthetic PTS_CB01 modules"))
            return HealthReport(
                sensor_id=self.id,
                kind=self.kind.value,
                status=HealthStatus.OK,
                message="dry-run Paxini PTS_CB01 ready",
                checks=checks,
                metrics=metrics,
                hints=["Set port to the control box, for example COM19"],
            )
        if not self.port:
            checks.append(CheckResult("serial_port", False, "port not set", critical=True))
            return HealthReport(
                sensor_id=self.id,
                kind=self.kind.value,
                status=HealthStatus.OFFLINE,
                message="PTS_CB01 port not set",
                checks=checks,
                metrics=metrics,
                hints=["params.port: COM19", "params.model: PXSR-STDDP03G"],
            )
        try:
            import serial
        except ImportError:
            checks.append(CheckResult("pyserial", False, "pyserial not installed", critical=True))
            return HealthReport(
                sensor_id=self.id,
                kind=self.kind.value,
                status=HealthStatus.OFFLINE,
                message="pyserial missing",
                checks=checks,
                metrics=metrics,
            )
        listed = {p.device.upper() for p in serial.tools.list_ports.comports()}
        bare = self.port.upper().replace("\\\\.\\", "")
        present = bare in listed or self.port.upper() in listed
        checks.append(CheckResult("serial_port", present, self.port, critical=False))
        return HealthReport(
            sensor_id=self.id,
            kind=self.kind.value,
            status=HealthStatus.OK if present else HealthStatus.WARN,
            message="PTS_CB01 port listed" if present else "PTS_CB01 port not listed",
            checks=checks,
            metrics=metrics,
            hints=[
                "Masks 0x01 and 0x04. One response is one module.",
                "1 LSB = 0.1 N. The box does not report the model.",
            ],
        )

    def _open_pts(self) -> None:
        self._tick = 0
        if self.ctx.dry_run:
            super().open()
            return
        if not self.port:
            raise RuntimeError(f"{self.id}: PTS_CB01 port is not set")
        try:
            from sensors.drivers.extensions.force_torque import normalize_serial_port

            port = normalize_serial_port(self.port)
        except Exception:  # noqa: BLE001
            port = self.port
        self._box = PtsCb01Box(port, self.baud)
        super().open()

    def _close_pts(self) -> None:
        if self._box is not None:
            self._box.close()
        self._box = None
        super().close()

    def _read_pts(self) -> Mapping[str, Any]:
        if not self._opened:
            raise RuntimeError(f"{self.id}: call open() before read()")
        self._tick += 1
        if self.ctx.dry_run or self._box is None:
            modules = self._synthetic_pts_modules()
            version = None
        else:
            modules = self._box.read_both(n_points=min(self.n_points, READABLE_POINTS))
            for module in modules:
                pts = module.get("points") or []
                fx = float(module["fx"]) if module.get("fx") is not None else 0.0
                fy = float(module["fy"]) if module.get("fy") is not None else 0.0
                fz = float(module["fz"]) if module.get("fz") is not None else 0.0
                module["rest_force"] = [fx, fy, fz]
                module["contact_force"] = fz
                module["points"] = pts
            version = self._box.version
        return self._sample_from_modules(modules, version=version)

    def _probe_pts_hz(self, duration_s: float) -> dict[str, Any]:
        from sensors.core.rate_probe import clamp_duration, rate_probe_fail, rate_probe_ok

        if not self._opened:
            raise RuntimeError(f"{self.id}: call open() before probe_max_read_hz()")
        duration_s = clamp_duration(duration_s)
        dry = bool(self.ctx.dry_run or self._box is None)
        if not dry and self._box is not None and not self._box.select_once(0x01):
            return rate_probe_fail(
                error="select failed",
                method="pts_cb01_full_read",
                dry_run=False,
            )
        times: list[float] = []
        errors = 0
        last_error: str | None = None
        t0 = time.perf_counter()
        deadline = t0 + duration_s
        while time.perf_counter() < deadline:
            try:
                if dry:
                    self._tick += 1
                    self._synthetic_pts_modules()
                elif self._box is None or not self._box.read_selected_once():
                    raise RuntimeError("full read failed")
                times.append(time.perf_counter())
            except Exception as e:  # noqa: BLE001
                errors += 1
                last_error = str(e)
                if errors >= 8 and len(times) < 3:
                    break
        if len(times) < 3:
            return rate_probe_fail(
                error=last_error or "tactile read probe produced too few samples",
                method="pts_cb01_full_read",
                samples=len(times),
                errors=errors,
                duration_s=time.perf_counter() - t0,
                dry_run=dry,
                last_error=last_error,
            )
        return rate_probe_ok(
            times,
            method="pts_cb01_full_read",
            errors=errors,
            t0=t0,
            dry_run=dry,
            last_error=last_error,
            note="select once, then full reads of the selected module",
        )

    def probe(self) -> HealthReport:
        if self.transport == "pts_cb01":
            return self._probe_pts()
        checks: list[CheckResult] = []
        metrics: dict[str, Any] = {
            "dll_path": self.dll_path or "—",
            "paxini_channel": self.paxini_channel,
            "port": self.port or "—",
            "group_index": self.group_index,
            "comp_count": COMP_COUNT,
            "grid_shape": list(GRID_HW) + [3],
            "schema": "PaxiniHandler / PAXINI_OUTPUT",
            "note": "Finger tactile — not wrist F/T",
        }

        if self.port:
            ok = _port_available(self.port)
            checks.append(CheckResult("serial_port", ok, self.port, critical=False))

        if self.ctx.dry_run:
            checks.append(CheckResult("dry_run", True, "synthetic Paxini force groups"))
            return HealthReport(
                sensor_id=self.id,
                kind=self.kind.value,
                status=HealthStatus.OK,
                message="dry-run Paxini tactile ready (rest_force + 6×10 components)",
                checks=checks,
                metrics=metrics,
                hints=["Set dll_path to ReadPaxiniLib.dll for live reads"],
            )

        if self.dll_path:
            path = Path(self.dll_path)
            dll_ok = path.is_file()
            checks.append(CheckResult("dll", dll_ok, str(path) if dll_ok else f"missing {self.dll_path}", critical=True))
            status = HealthStatus.OK if dll_ok else HealthStatus.OFFLINE
            message = "Paxini DLL present" if dll_ok else "Paxini DLL missing"
        else:
            checks.append(CheckResult("dll", False, "dll_path not set", critical=False))
            status = HealthStatus.WARN
            message = "No dll_path — use dry_run or set ReadPaxiniLib.dll path"

        return HealthReport(
            sensor_id=self.id,
            kind=self.kind.value,
            status=status,
            message=message,
            checks=checks,
            metrics=metrics,
            hints=[
                "Aligned with get_paxini_data.PaxiniHandler",
                "get_force() ≡ rest_force.z of group 0",
            ],
        )

    def open(self) -> None:
        if self.transport == "pts_cb01":
            self._open_pts()
            return
        self._tick = 0
        if not self.ctx.dry_run:
            ok, detail = self._try_bind_dll()
            if not ok:
                raise RuntimeError(f"{self.id}: cannot open Paxini ({detail})")
        super().open()

    def close(self) -> None:
        if self.transport == "pts_cb01":
            self._close_pts()
            return
        if self._lib is not None and self._handle is not None:
            try:
                self._lib.destroy_paxini_handle(self._handle)
            except Exception:  # noqa: BLE001
                pass
        self._lib = None
        self._handle = None
        self._buf = None
        super().close()

    def read(self) -> Mapping[str, Any]:
        if self.transport == "pts_cb01":
            return self._read_pts()
        if not self._opened:
            raise RuntimeError(f"{self.id}: call open() before read()")
        self._tick += 1
        if self.ctx.dry_run or self._lib is None:
            # Match typical demo: at least one group (often 1–4 printed)
            n_groups = max(1, int(self.config.get("dry_run_groups", 2)))
            groups = [self._synthetic_group(i) for i in range(n_groups)]
        else:
            groups = self._read_live_groups()

        gi = max(0, min(self.group_index, len(groups) - 1)) if groups else 0
        primary = groups[gi] if groups else self._synthetic_group(0)
        return {
            "ts": time.time(),
            "dry_run": bool(self.ctx.dry_run),
            "num": len(groups),
            "group_index": gi,
            "groups": groups,
            "rest_force": primary["rest_force"],
            "contact_force": primary["contact_force"],
            "component_forces": primary["component_forces"],
            "original_forces": primary.get("original_forces"),
            "grid_shape": primary["grid_shape"],
            "dll_path": self.dll_path or None,
            "paxini_channel": self.paxini_channel,
            "port": self.port or None,
        }

    def _probe_read_once(self) -> None:
        """One Paxini DLL fetch or synthetic group (no sample dict)."""
        if self.ctx.dry_run or self._lib is None:
            self._tick += 1
            self._synthetic_group(0)
            return
        self._read_live_groups()

    def probe_max_read_hz(self, duration_s: float = 10.0) -> dict[str, Any]:
        """Tight native read loop — not sample read().

        PTS_CB01 selects the first module once, then times only the register
        read. The DLL path times ``get_paxini_data``.
        """
        if self.transport == "pts_cb01":
            return self._probe_pts_hz(duration_s)
        from sensors.core.rate_probe import clamp_duration, rate_probe_fail, rate_probe_ok

        if not self._opened:
            raise RuntimeError(f"{self.id}: call open() before probe_max_read_hz()")
        duration_s = clamp_duration(duration_s)
        times: list[float] = []
        errors = 0
        last_error: str | None = None
        t0 = time.perf_counter()
        deadline = t0 + duration_s
        dry = bool(self.ctx.dry_run or self._lib is None)
        while time.perf_counter() < deadline:
            try:
                self._probe_read_once()
                times.append(time.perf_counter())
            except Exception as e:  # noqa: BLE001
                errors += 1
                last_error = str(e)
                if errors >= 8 and len(times) < 3:
                    break
        if len(times) < 3:
            return rate_probe_fail(
                error=last_error or "tactile read probe produced too few samples",
                method="paxini_get_data_loop",
                samples=len(times),
                errors=errors,
                duration_s=time.perf_counter() - t0,
                dry_run=dry,
                last_error=last_error,
            )
        return rate_probe_ok(
            times,
            method="paxini_get_data_loop",
            errors=errors,
            t0=t0,
            dry_run=dry,
            last_error=last_error,
        )
