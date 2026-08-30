from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import pytest

from sensors.core.base import SensorContext
from sensors.core.config import BundleConfig, load_bundle_config
from sensors.core.registry import list_registered_kinds
from sensors.drivers import load_all_drivers
from sensors.drivers.arm.follower import FollowerArmSensor
from sensors.drivers.bus.serial_bus import SerialBusSensor
from sensors.drivers.camera.realsense import RealSenseSensor
from sensors.drivers.gello.leader import GelloLeaderSensor
from sensors.drivers.gripper.dh_ag95 import DhAg95Sensor, _crc16_modbus
from sensors.runtime.manager import SensorManager

ROOT = Path(__file__).resolve().parents[1]
CFG = ROOT / "configs" / "default.yaml"


def test_register_all_kinds():
    load_all_drivers()
    kinds = set(list_registered_kinds())
    for k in ("bus", "arm", "gello", "gripper", "realsense", "pipeline", "ft", "tactile"):
        assert k in kinds


def test_load_default_config():
    bundle = load_bundle_config(CFG)
    assert isinstance(bundle, BundleConfig)
    assert bundle.site == "lab-default"
    assert any(d.kind == "arm" for d in bundle.devices)


def test_probe_all_smoke():
    mgr = SensorManager.from_yaml(CFG, dry_run=True)
    reports = mgr.probe_all()
    assert len(reports) >= 8
    by_id = {r.sensor_id: r for r in reports}
    assert by_id["ft-wrist"].status.value == "unsupported"
    assert "arm-follower" in by_id
    assert "bus-main" in by_id


def test_dry_run_open_read_serial_zmq_realsense():
    mgr = SensorManager.from_yaml(CFG, dry_run=True)
    for sid in ("bus-main", "arm-follower", "gripper-dh", "gello-leader", "rs-left"):
        sample = mgr.read(sid)
        assert sample.get("dry_run") is True
        assert "ts" in sample


def test_modbus_crc_stable():
    frame = [0x01, 0x06, 0x01, 0x00, 0x00, 0xA5]
    assert _crc16_modbus(frame) == _crc16_modbus(bytes(frame))
    assert 0 <= _crc16_modbus(frame) <= 0xFFFF


def test_arm_zmq_rpc_read():
    sensor = FollowerArmSensor("arm", {"zmq_host": "127.0.0.1", "zmq_port": 6001})
    joints = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.0])
    sock = MagicMock()
    sock.recv.return_value = __import__("pickle").dumps(joints)
    sensor._socket = sock
    sensor._opened = True
    out = sensor.read()
    assert out["joints"] == pytest.approx(joints.tolist())
    sock.send.assert_called_once()
    req = __import__("pickle").loads(sock.send.call_args[0][0])
    assert req["method"] == "get_joint_state"


def test_bus_read_without_hardware():
    sensor = SerialBusSensor("bus", {"local_pty": "/tmp/ttyUR_nonexistent_hik"})
    sensor._opened = True
    sample = sensor.read()
    assert sample["pty_exists"] is False
    assert "serial_by_id" in sample


def test_realsense_dry_read_fields():
    sensor = RealSenseSensor("rs", {"role": "left"}, SensorContext(dry_run=True))
    sensor.open()
    sample = sensor.read()
    assert sample["color"] is None
    assert sample["role"] == "left"
    sensor.close()


def test_gripper_dry_read():
    sensor = DhAg95Sensor("g", {}, SensorContext(dry_run=True))
    sensor.open()
    sample = sensor.read()
    assert sample["position_raw"] is None
    sensor.close()


def test_gello_dry_read():
    sensor = GelloLeaderSensor("gello", {}, SensorContext(dry_run=True))
    sensor.open()
    sample = sensor.read()
    assert sample["joints_rad"] is None
    sensor.close()
