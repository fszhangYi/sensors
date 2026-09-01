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
    assert by_id["ft-wrist"].status.value == "ok"
    assert by_id["tactile-paxini"].status.value == "ok"
    assert "arm-follower" in by_id
    assert "bus-main" in by_id


def test_dry_run_open_read_serial_zmq_realsense():
    mgr = SensorManager.from_yaml(CFG, dry_run=True)
    for sid in ("bus-main", "arm-follower", "gripper-dh", "gello-leader", "rs-left", "ft-wrist", "tactile-paxini"):
        sample = mgr.read(sid)
        assert sample.get("dry_run") is True
        assert "ts" in sample
    ft = mgr.read("ft-wrist")
    assert len(ft["force"]) == 3 and len(ft["torque"]) == 3
    assert len(ft["wrench"]) == 6
    assert "force_smooth" not in ft
    tac = mgr.read("tactile-paxini")
    assert len(tac["rest_force"]) == 3
    assert "contact_force" in tac
    assert len(tac["component_forces"]) == 60
    assert tac["num"] >= 1


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
    assert sample["position_raw"] is not None
    sensor.close()


def test_gripper_calibrate_dry_run():
    sensor = DhAg95Sensor("g", {}, SensorContext(dry_run=True))
    sensor.open()
    result = sensor.write({"calibrate": True})
    assert result["ok"] is True
    assert result["position_raw_min"] is not None
    assert result["position_raw_max"] > result["position_raw_min"]
    sensor.close()


def test_gripper_initialize_dry_run():
    sensor = DhAg95Sensor("g", {}, SensorContext(dry_run=True))
    sensor.open()
    assert sensor.initialized is False
    result = sensor.initialize()
    assert result["ok"] is True
    assert result["initialized"] is True
    assert sensor.initialized is True
    legacy = sensor.write({"initialize": True})
    assert legacy["ok"] is True
    assert legacy["initialized"] is True
    sensor.close()
    assert sensor.initialized is False


def test_base_initialize_default():
    sensor = SerialBusSensor("bus", {}, SensorContext(dry_run=True))
    sensor.open()
    result = sensor.initialize()
    assert result["ok"] is True
    assert result["skipped"] is True
    assert sensor.initialized is True
    sensor.close()


def test_ft_serial_frame_parse():
    import struct
    import sys

    from sensors.drivers.extensions.force_torque import (
        FRAME_SIZE,
        normalize_serial_port,
        parse_wrench_frame,
        pop_wrench_frame_from_buffer,
    )

    if sys.platform == "win32":
        assert normalize_serial_port("COM5") == "\\\\.\\COM5"
    else:
        assert normalize_serial_port("COM5") == "COM5"
    payload = struct.pack("<6f", 1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
    frame = bytes([0x49, 0xAA]) + payload + bytes([0x0D, 0x0A])
    assert len(frame) == FRAME_SIZE
    force, torque = parse_wrench_frame(frame)
    assert force[0] == pytest.approx(1.0)
    assert torque[2] == pytest.approx(0.3)


def _ft_test_frame(seq: float) -> bytes:
    import struct

    from sensors.drivers.extensions.force_torque import FRAME_SIZE

    payload = struct.pack("<6f", seq, 0.0, 0.0, 0.0, 0.0, 0.0)
    frame = bytes([0x49, 0xAA]) + payload + bytes([0x0D, 0x0A])
    assert len(frame) == FRAME_SIZE
    return frame


def test_ft_buffer_pop_latest_skips_stale():
    from sensors.drivers.extensions.force_torque import pop_wrench_frame_from_buffer

    buf = bytearray(_ft_test_frame(1.0) + _ft_test_frame(2.0) + _ft_test_frame(3.0))
    frame = pop_wrench_frame_from_buffer(buf, latest=True)
    assert frame is not None
    import struct

    fx = struct.unpack("<6f", frame[2:26])[0]
    assert fx == pytest.approx(3.0)
    assert len(buf) == 0


def test_ft_continuous_read_does_not_retrigger_start():
    from sensors.drivers.extensions.force_torque import ForceTorqueSensor

    sensor = ForceTorqueSensor("ft", {"port": "COM5", "timeout_ms": 200}, SensorContext(dry_run=False))
    mock_ser = MagicMock()
    mock_ser.is_open = True
    frames = [_ft_test_frame(1.0), _ft_test_frame(2.0)]

    def read_side_effect(size=-1):
        if mock_ser.in_waiting:
            data = frames.pop(0) if frames else b""
            mock_ser.in_waiting = 0
            return data
        return b""

    def in_waiting_side_effect():
        return len(frames) * 28 if frames else 0

    mock_ser.in_waiting = 28
    mock_ser.read.side_effect = read_side_effect
    sensor._ser = mock_ser
    sensor._opened = True
    sensor._sampling = True
    sensor._rx_buf = bytearray()

    fx1, _ = sensor._read_wrench()
    assert fx1[0] == pytest.approx(1.0)
    assert mock_ser.write.call_count == 0

    mock_ser.in_waiting = 28
    fx2, _ = sensor._read_wrench()
    assert fx2[0] == pytest.approx(2.0)
    for call in mock_ser.write.call_args_list:
        assert call.args[0] != bytes([0x49, 0xAA, 0x0D, 0x0A])


def test_ft_dry_run_tare():
    from sensors.drivers.extensions.force_torque import ForceTorqueSensor

    sensor = ForceTorqueSensor("ft", {}, SensorContext(dry_run=True))
    sensor.open()
    sample = sensor.read()
    assert len(sample["wrench"]) == 6
    result = sensor.write({"tare": True})
    assert result["ok"] is True
    sensor.close()


def test_realsense_fps_fallback_prefers_nearest():
    from sensors.drivers.camera.realsense import _fps_fallback_order

    assert _fps_fallback_order(5)[0] == 5
    # nearest discrete after exact miss should be 6, not 30
    assert _fps_fallback_order(5)[1] == 6
    assert _fps_fallback_order(5)[2] == 15
    assert 30 in _fps_fallback_order(5)
    assert _fps_fallback_order(15)[0] == 15
    assert _fps_fallback_order(12)[0] == 12
    assert _fps_fallback_order(12)[1] == 15


def test_realsense_dry_read_stream_params():
    sensor = RealSenseSensor("rs", {"role": "left", "enable_depth": False}, SensorContext(dry_run=True))
    sensor.open()
    sample = sensor.read()
    assert sample["width"] == 1280
    assert sample["enable_depth"] is False
    sensor.close()


def test_gello_dry_read():
    sensor = GelloLeaderSensor("gello", {}, SensorContext(dry_run=True))
    sensor.open()
    sample = sensor.read()
    assert sample["joints_rad"] is None
    sensor.close()


def test_rate_probe_dry_run_all_kinds():
    load_all_drivers()
    cases = [
        (GelloLeaderSensor, "gello", {}),
        (DhAg95Sensor, "gripper", {}),
        (RealSenseSensor, "realsense", {"role": "left"}),
        (FollowerArmSensor, "arm", {}),
        (SerialBusSensor, "bus", {}),
    ]
    from sensors.drivers.extensions.force_torque import ForceTorqueSensor
    from sensors.drivers.extensions.paxini import PaxiniTactileSensor
    from sensors.drivers.pipeline.collect import CollectPipelineSensor

    cases.extend(
        [
            (ForceTorqueSensor, "ft", {}),
            (PaxiniTactileSensor, "tactile", {}),
            (CollectPipelineSensor, "pipeline", {}),
        ]
    )
    for cls, kind, cfg in cases:
        sensor = cls(f"probe-{kind}", cfg, SensorContext(dry_run=True))
        sensor.open()
        probe = sensor.probe_max_read_hz(duration_s=0.6)
        if kind in ("bus", "pipeline"):
            assert probe.get("unsupported") is True
            assert probe.get("ok") is False
        else:
            assert probe.get("ok") is True, (kind, probe)
            assert probe.get("measured_hz", 0) > 0
            assert probe.get("read_cap_hz", 0) > 0
            assert probe.get("method")
        sensor.close()


def test_gripper_sync_write_probe_dry_run():
    sensor = DhAg95Sensor("g", {}, SensorContext(dry_run=True))
    sensor.open()
    probe = sensor.probe_max_sync_write_hz(duration_s=0.6, baseline_s=0.5)
    assert probe.get("ok") is True
    assert probe.get("max_sync_write_hz", 0) > 0
    assert probe.get("method") == "gripper_modbus_sync_write_read"
    sensor.close()
