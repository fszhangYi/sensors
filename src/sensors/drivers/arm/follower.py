from __future__ import annotations

import pickle
import socket
import time
from typing import Any, Mapping

import numpy as np

from sensors.core.base import Sensor, SensorCapability, SensorContext
from sensors.core.health import CheckResult, HealthReport, HealthStatus
from sensors.core.kinds import SensorKind
from sensors.core.registry import register_sensor


@register_sensor(SensorKind.ARM)
class FollowerArmSensor(Sensor):
    """Follower arm: MegaCollect Server, ZMQ REP default :6001 (UR | Elite)."""

    kind = SensorKind.ARM
    capabilities = SensorCapability.PROBE | SensorCapability.SAMPLE | SensorCapability.CONTROL

    def __init__(self, sensor_id: str, config: Mapping[str, Any], ctx: SensorContext | None = None):
        super().__init__(sensor_id, config, ctx)
        self.backend = str(config.get("backend", "elite"))
        self.robot_ip = str(config.get("robot_ip", "10.111.34.200"))
        self.zmq_host = str(config.get("zmq_host", "127.0.0.1"))
        self.zmq_port = int(config.get("zmq_port", 6001))
        self.io_collect_do = str(config.get("io_collect_do", "Y005"))
        self.io_collect_di = str(config.get("io_collect_di", "X009"))
        self.robot_tcp_port = int(config.get("robot_tcp_port", 54321))
        self.rcv_timeout_ms = int(config.get("rcv_timeout_ms", 1000))
        self.snd_timeout_ms = int(config.get("snd_timeout_ms", 1000))
        self._socket = None

    @property
    def endpoint(self) -> str:
        return f"tcp://{self.zmq_host}:{self.zmq_port}"

    @staticmethod
    def _tcp_open(host: str, port: int, timeout: float = 0.4) -> bool:
        try:
            with socket.create_connection((host, port), timeout=timeout):
                return True
        except OSError:
            return False

    def probe(self) -> HealthReport:
        checks: list[CheckResult] = []
        metrics: dict[str, Any] = {
            "backend": self.backend,
            "robot_ip": self.robot_ip,
            "zmq": self.endpoint,
            "io_do": self.io_collect_do,
            "io_di": self.io_collect_di,
        }

        zmq_ok = self._tcp_open(self.zmq_host, self.zmq_port)
        checks.append(CheckResult("zmq_port", zmq_ok, f"{self.endpoint} {'open' if zmq_ok else 'closed'}"))

        robot_ok = self._tcp_open(self.robot_ip, self.robot_tcp_port, timeout=0.3)
        checks.append(CheckResult("robot_tcp", robot_ok, f"{self.robot_ip}:{self.robot_tcp_port}"))

        try:
            import zmq  # noqa: F401

            metrics["pyzmq"] = True
        except ImportError:
            metrics["pyzmq"] = False
            checks.append(CheckResult("pyzmq", False, "optional: pip install 'hik-sensors[zmq]'"))

        status = HealthStatus.OK if zmq_ok else HealthStatus.OFFLINE
        if zmq_ok and not robot_ok:
            status = HealthStatus.WARN

        return HealthReport(
            sensor_id=self.id,
            kind=self.kind.value,
            status=status,
            message=f"follower {self.backend} via {self.endpoint}",
            checks=checks,
            metrics=metrics,
            hints=[
                "MegaCollect: start Server after port mapping",
                f"{self.io_collect_do}=collect DO, {self.io_collect_di}=collect DI (Elite)",
            ],
        )

    def open(self) -> None:
        if self.ctx.dry_run:
            self._opened = True
            return
        try:
            import zmq
        except ImportError as e:
            raise RuntimeError("pyzmq required for arm open()") from e
        ctx = zmq.Context.instance()
        sock = ctx.socket(zmq.REQ)
        sock.setsockopt(zmq.RCVTIMEO, self.rcv_timeout_ms)
        sock.setsockopt(zmq.SNDTIMEO, self.snd_timeout_ms)
        sock.setsockopt(zmq.LINGER, 0)
        sock.connect(self.endpoint)
        self._socket = sock
        self._opened = True

    def close(self) -> None:
        if self._socket is not None:
            try:
                self._socket.close(0)
            except Exception:  # noqa: BLE001
                pass
            self._socket = None
        self._opened = False

    def _rpc(self, method: str, args: Mapping[str, Any] | None = None) -> Any:
        if self._socket is None:
            raise RuntimeError(f"{self.id}: socket not open")
        import zmq

        request = {"method": method, "args": dict(args or {})}
        try:
            self._socket.send(pickle.dumps(request))
            raw = self._socket.recv()
        except zmq.Again as e:
            raise TimeoutError(f"ZMQ timeout calling {method} on {self.endpoint}") from e
        return pickle.loads(raw)

    def read(self) -> Mapping[str, Any]:
        super().read()
        ts = time.time()
        if self.ctx.dry_run or self._socket is None:
            return {
                "joints": None,
                "endpoint": self.endpoint,
                "backend": self.backend,
                "dry_run": True,
                "ts": ts,
            }
        joints = self._rpc("get_joint_state")
        if isinstance(joints, np.ndarray):
            joints_out: Any = joints.tolist()
        elif hasattr(joints, "tolist"):
            joints_out = joints.tolist()
        else:
            joints_out = joints
        return {
            "joints": joints_out,
            "endpoint": self.endpoint,
            "backend": self.backend,
            "ts": ts,
        }
