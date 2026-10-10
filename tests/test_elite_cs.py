"""Elite CS drivers (arm_cs_*) — dry_run only; no elite_cs_sdk required."""

from __future__ import annotations

import pytest

from sensors.core.base import SensorContext
from sensors.core.registry import list_registered_kinds
from sensors.drivers import load_all_drivers
from sensors.drivers.arm.cs_read import EliteCsArmReadSensor
from sensors.drivers.arm.cs_write import EliteCsArmWriteSensor


def test_cs_kinds_registered():
    load_all_drivers()
    kinds = set(list_registered_kinds())
    assert "arm_cs_read" in kinds
    assert "arm_cs_write" in kinds
    assert "arm_read" in kinds
    assert "arm_write" in kinds


def test_cs_write_dry_run_arm_servoj():
    s = EliteCsArmWriteSensor(
        "cs-w",
        {"robot_ip": "10.111.34.201", "num_joints": 6, "max_delta_deg": 15.0},
        SensorContext(dry_run=True),
    )
    s.open()
    assert s.probe().status.value == "ok"
    assert s.read()["backend"] == "elite_cs"
    assert s.write({"joints_rad": [0.1] * 6})["ok"] is False  # not armed
    assert s.write({"arm": True})["ok"] is True
    r = s.write(
        {
            "joints_rad": [0.1, -0.1, 0.05, 0.0, 0.05, 0.0],
            "reference_joints_rad": [0.0] * 6,
        }
    )
    assert r["ok"] is True, r
    assert r["backend"] == "elite_cs"
    assert s.read()["joints_rad"][0] == pytest.approx(0.1)
    assert s.write({"disarm": True})["ok"] is True
    s.close()


def test_cs_read_dry_run_rejects_write():
    s = EliteCsArmReadSensor(
        "cs-r",
        {"robot_ip": "10.111.34.201"},
        SensorContext(dry_run=True),
    )
    s.open()
    sample = s.read()
    assert sample["dry_run"] is True
    assert sample["backend"] == "elite_cs"
    with pytest.raises(RuntimeError, match="read-only"):
        s.write({"joints_rad": [0.0] * 6})
    s.close()


def test_cs_write_endpoint_uses_primary_30001():
    s = EliteCsArmWriteSensor("cs", {"robot_ip": "192.168.1.10"}, SensorContext(dry_run=True))
    assert s.robot_tcp_port == 30001
    assert s.endpoint == "192.168.1.10:30001"
