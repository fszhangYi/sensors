from __future__ import annotations

import socket
from multiprocessing import shared_memory
from pathlib import Path
from typing import Any, Mapping

from sensors.core.base import Sensor, SensorCapability, SensorContext
from sensors.core.health import CheckResult, HealthReport, HealthStatus
from sensors.core.kinds import SensorKind
from sensors.core.registry import register_sensor

# MegaCollect __main__ SHM layout
MOSAIC_SHAPE = (1440, 2560, 3)
JOINT_LEN = 12
FLAG_IDLE = 0
FLAG_RECORD = 1
FLAG_FAIL = 2


@register_sensor(SensorKind.PIPELINE)
class CollectPipelineSensor(Sensor):
    """Collect pipeline: collection_flag + mosaic/joint SHM + remote index sync."""

    kind = SensorKind.PIPELINE
    capabilities = SensorCapability.PROBE | SensorCapability.SAMPLE

    def __init__(self, sensor_id: str, config: Mapping[str, Any], ctx: SensorContext | None = None):
        super().__init__(sensor_id, config, ctx)
        self.flag_shm = str(config.get("flag_shm", "collection_flag"))
        self.mosaic_shm = str(config.get("mosaic_shm", "show_shared_array"))
        self.joint_shm = str(config.get("joint_shm", "joint_shared_array"))
        self.index_file = str(config.get("index_file", ""))
        self.remote_hosts = list(config.get("remote_hosts") or [])
        self.remote_port = int(config.get("remote_port", 12345))
        self.save_path = str(config.get("save_path", ""))
        self.mosaic_shape = tuple(config.get("mosaic_shape") or MOSAIC_SHAPE)
        self.joint_len = int(config.get("joint_len", JOINT_LEN))

    @staticmethod
    def _try_attach(name: str) -> tuple[bool, str]:
        try:
            shm = shared_memory.SharedMemory(name=name, create=False)
            size = shm.size
            shm.close()
            return True, f"size={size}"
        except FileNotFoundError:
            return False, "not found"
        except Exception as e:  # noqa: BLE001
            return False, str(e)

    def probe(self) -> HealthReport:
        checks: list[CheckResult] = []
        metrics: dict[str, Any] = {
            "flag_shm": self.flag_shm,
            "mosaic_shm": self.mosaic_shm,
            "joint_shm": self.joint_shm,
            "mosaic_shape": list(self.mosaic_shape),
            "joint_len": self.joint_len,
            "flag_semantics": {"0": "idle/stop", "1": "record", "2": "fail"},
            "remote_port": self.remote_port,
        }

        for label, name in (
            ("flag", self.flag_shm),
            ("mosaic", self.mosaic_shm),
            ("joint", self.joint_shm),
        ):
            ok, detail = self._try_attach(name)
            checks.append(CheckResult(f"shm:{label}", ok, f"{name} ({detail})"))

        if self.index_file:
            p = Path(self.index_file)
            checks.append(CheckResult("index_file", p.is_file(), self.index_file))
            if p.is_file():
                try:
                    metrics["data_index"] = int(p.read_text(encoding="utf-8").strip())
                except ValueError:
                    metrics["data_index"] = None

        if self.save_path:
            sp = Path(self.save_path)
            checks.append(CheckResult("save_path", sp.exists(), self.save_path))

        remote_ok = 0
        remote_detail = []
        for host in self.remote_hosts:
            try:
                with socket.create_connection((host, self.remote_port), timeout=0.4):
                    remote_ok += 1
                    remote_detail.append(f"{host}:ok")
            except OSError as e:
                remote_detail.append(f"{host}:{e.__class__.__name__}")
        if self.remote_hosts:
            checks.append(
                CheckResult(
                    "remote_sync",
                    remote_ok == len(self.remote_hosts),
                    f"{remote_ok}/{len(self.remote_hosts)} — " + ", ".join(remote_detail),
                )
            )
        metrics["remote"] = remote_detail

        # SHM often absent until MegaCollect runs — treat as offline not error
        shm_any = any(c.ok for c in checks if c.name.startswith("shm:"))
        status = HealthReport.aggregate_status(checks)
        if not shm_any:
            status = HealthStatus.OFFLINE

        return HealthReport(
            sensor_id=self.id,
            kind=self.kind.value,
            status=status,
            message="collect pipeline / SHM / remote sync",
            checks=checks,
            metrics=metrics,
            hints=[
                "SHM created by MegaCollect __main__; attach fails if app not running",
                f"Remote peers sync episode index on TCP :{self.remote_port}",
            ],
        )
