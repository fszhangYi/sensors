from __future__ import annotations

import os
import shutil
import subprocess
import time
from pathlib import Path
from typing import Any, Mapping

from sensors.core.base import Sensor, SensorCapability, SensorContext
from sensors.core.health import CheckResult, HealthReport, HealthStatus
from sensors.core.kinds import SensorKind
from sensors.core.registry import register_sensor


def list_serial_by_id() -> list[str]:
    root = Path("/dev/serial/by-id")
    if not root.is_dir():
        return []
    return sorted(str(p) for p in root.iterdir() if p.is_symlink() or p.exists())


def path_rw(path: str) -> bool:
    try:
        return os.access(path, os.R_OK | os.W_OK)
    except OSError:
        return False


@register_sensor(SensorKind.BUS)
class SerialBusSensor(Sensor):
    """Port-mapping hub: socat PTY + USB by-id (MegaCollect 「端口映射」)."""

    kind = SensorKind.BUS
    capabilities = SensorCapability.PROBE | SensorCapability.SAMPLE | SensorCapability.CONTROL

    def __init__(self, sensor_id: str, config: Mapping[str, Any], ctx: SensorContext | None = None):
        super().__init__(sensor_id, config, ctx)
        self.robot_ip = str(config.get("robot_ip", "10.111.34.200"))
        self.robot_tcp_port = int(config.get("robot_tcp_port", 54321))
        self.local_pty = str(config.get("local_pty", "/tmp/ttyUR"))
        self.socat_script = str(config.get("socat_script", ""))
        self.expect_by_id = list(config.get("expect_by_id") or [])
        self.kill_existing = bool(config.get("kill_existing", True))
        self._proc: subprocess.Popen[str] | None = None

    def probe(self) -> HealthReport:
        checks: list[CheckResult] = []
        metrics: dict[str, Any] = {
            "robot_ip": self.robot_ip,
            "robot_tcp_port": self.robot_tcp_port,
            "local_pty": self.local_pty,
        }

        pty_ok = Path(self.local_pty).exists()
        checks.append(CheckResult("local_pty", pty_ok, self.local_pty))

        socat_bin = shutil.which("socat")
        checks.append(CheckResult("socat_binary", socat_bin is not None, socat_bin or "not on PATH"))

        socat_running = False
        try:
            r = subprocess.run(["pgrep", "-a", "socat"], capture_output=True, text=True, timeout=2)
            socat_running = r.returncode == 0 and bool(r.stdout.strip())
            metrics["socat_procs"] = r.stdout.strip().splitlines()[:5]
        except (OSError, subprocess.SubprocessError) as e:
            metrics["socat_procs_error"] = str(e)
        checks.append(CheckResult("socat_process", socat_running, "running" if socat_running else "not running"))

        by_id = list_serial_by_id()
        metrics["serial_by_id"] = by_id
        for needle in self.expect_by_id:
            hit = any(needle in p for p in by_id)
            checks.append(CheckResult(f"by_id:{needle}", hit, "found" if hit else "missing"))

        if self.socat_script:
            checks.append(CheckResult("socat_script", Path(self.socat_script).is_file(), self.socat_script))

        status = HealthReport.aggregate_status(checks)
        return HealthReport(
            sensor_id=self.id,
            kind=self.kind.value,
            status=status,
            message="serial / port-mapping hub",
            checks=checks,
            metrics=metrics,
            hints=[
                f"socat pty,link={self.local_pty} tcp:{self.robot_ip}:{self.robot_tcp_port}",
                "Start port mapping before Elite tools that need /tmp/ttyUR",
            ],
        )

    def _socat_cmd(self) -> list[str]:
        if self.socat_script:
            return ["bash", self.socat_script]
        return [
            "socat",
            f"pty,link={self.local_pty},raw,ignoreeof,waitslave",
            f"tcp:{self.robot_ip}:{self.robot_tcp_port}",
        ]

    def open(self) -> None:
        if self.ctx.dry_run:
            self._opened = True
            return
        if shutil.which("socat") is None and not self.socat_script:
            raise RuntimeError("socat not on PATH; install socat or set socat_script")

        if self.kill_existing:
            try:
                subprocess.run(["killall", "-9", "socat"], capture_output=True, timeout=2)
            except (OSError, subprocess.SubprocessError):
                pass

        cmd = self._socat_cmd()
        self._proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
        )
        # waitslave may block until peer connects; give PTY a moment to appear
        deadline = time.time() + 1.5
        while time.time() < deadline:
            if self._proc.poll() is not None:
                err = ""
                try:
                    err = (self._proc.stderr.read() or "")[:200] if self._proc.stderr else ""
                except Exception:  # noqa: BLE001
                    pass
                raise RuntimeError(f"socat exited early code={self._proc.returncode}: {err}")
            if Path(self.local_pty).exists() or self.socat_script:
                break
            time.sleep(0.05)
        self._opened = True

    def close(self) -> None:
        if self._proc is not None:
            try:
                if self._proc.poll() is None:
                    self._proc.terminate()
                    try:
                        self._proc.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        self._proc.kill()
            except Exception:  # noqa: BLE001
                pass
            self._proc = None
        self._opened = False

    def read(self) -> Mapping[str, Any]:
        super().read()
        by_id = list_serial_by_id()
        proc_alive = self._proc is not None and self._proc.poll() is None
        socat_running = False
        try:
            r = subprocess.run(["pgrep", "-a", "socat"], capture_output=True, text=True, timeout=2)
            socat_running = r.returncode == 0 and bool(r.stdout.strip())
        except (OSError, subprocess.SubprocessError):
            pass
        return {
            "local_pty": self.local_pty,
            "pty_exists": Path(self.local_pty).exists(),
            "proc_alive": proc_alive,
            "socat_running": socat_running,
            "robot": f"{self.robot_ip}:{self.robot_tcp_port}",
            "serial_by_id": by_id,
            "expect_hits": {n: any(n in p for p in by_id) for n in self.expect_by_id},
            "dry_run": self.ctx.dry_run,
            "ts": time.time(),
        }
