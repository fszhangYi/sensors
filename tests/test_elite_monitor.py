"""Elite 8056 monitor reconnect helper (no real robot)."""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest


def test_machine_pos_ready() -> None:
    from sensors.drivers.arm._elite_monitor import machine_pos_ready

    assert not machine_pos_ready(SimpleNamespace(monitor_info=None), num_joints=6)
    info = SimpleNamespace(machinePos=[None] * 6)
    assert not machine_pos_ready(SimpleNamespace(monitor_info=info), num_joints=6)
    info.machinePos = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0, 0.0]
    assert machine_pos_ready(SimpleNamespace(monitor_info=info), num_joints=6)


def test_open_ec_retries_when_monitor_thread_dies(monkeypatch) -> None:
    from sensors.drivers.arm import _elite_monitor as mod

    calls = {"n": 0}

    class FakeThread:
        def __init__(self, alive: bool):
            self._alive = alive

        def is_alive(self) -> bool:
            return self._alive

        def join(self, timeout=None) -> None:  # noqa: ANN001
            return None

    class FakeEC:
        def __init__(self, ip: str, auto_connect: bool = False):
            del ip, auto_connect
            calls["n"] += 1
            self.monitor_info = SimpleNamespace(machinePos=[None] * 8)
            self.monitor_thread = FakeThread(alive=False)
            self.monitor_run_state = True

        def monitor_thread_run(self) -> None:
            # Simulate SDK thread dying in __first_connect (struct.error).
            self.monitor_thread = FakeThread(alive=False)

        def monitor_thread_stop(self) -> None:
            self.monitor_run_state = False

    fake_elite = MagicMock()
    fake_elite.EC = FakeEC
    monkeypatch.setitem(__import__("sys").modules, "elite", fake_elite)

    logs: list[str] = []
    with pytest.raises(TimeoutError, match="8056 monitor handshake failed"):
        mod.open_ec_with_monitor(
            robot_ip="10.0.0.1",
            num_joints=6,
            monitor_wait_s=0.2,
            monitor_retries=3,
            monitor_retry_backoff_s=0.01,
            post_close_cooldown_s=0.0,
            sensor_id="arm-test",
            log=logs.append,
        )
    assert calls["n"] == 3
    assert any("attempt 1/3" in m for m in logs)


def test_open_ec_succeeds_when_machine_pos_appears(monkeypatch) -> None:
    from sensors.drivers.arm import _elite_monitor as mod

    class FakeThread:
        def is_alive(self) -> bool:
            return True

        def join(self, timeout=None) -> None:  # noqa: ANN001
            return None

    class FakeEC:
        def __init__(self, ip: str, auto_connect: bool = False):
            del ip, auto_connect
            self.monitor_info = SimpleNamespace(machinePos=[None] * 8)
            self.monitor_thread = FakeThread()
            self._ticks = 0

        def monitor_thread_run(self) -> None:
            return None

        def monitor_thread_stop(self) -> None:
            return None

    # Patch machine_pos_ready path by filling pos after a few polls via property
    real_ec = FakeEC("x")

    def factory(ip: str, auto_connect: bool = False):
        del ip, auto_connect
        ec = FakeEC("x")

        def run() -> None:
            ec.monitor_info.machinePos = [10.0] * 8

        ec.monitor_thread_run = run  # type: ignore[method-assign]
        return ec

    fake_elite = MagicMock()
    fake_elite.EC = factory
    monkeypatch.setitem(__import__("sys").modules, "elite", fake_elite)

    robot = mod.open_ec_with_monitor(
        robot_ip="10.0.0.1",
        num_joints=6,
        monitor_wait_s=2.0,
        monitor_retries=2,
        monitor_retry_backoff_s=0.01,
        post_close_cooldown_s=0.0,
        sensor_id="arm-ok",
        log=lambda _m: None,
    )
    assert robot.monitor_info.machinePos[0] == 10.0
    del real_ec
