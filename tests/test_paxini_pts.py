"""PTS_CB01 codec and the tactile driver's two transports."""

from sensors.core.base import SensorContext
from sensors.drivers.extensions.paxini import PaxiniTactileSensor, resolve_transport
from sensors.drivers.extensions.paxini_pts import (
    READABLE_POINTS,
    build,
    decode_registers,
    lrc,
    payload_of,
    reg_read_body,
    resolve_model,
    select_body,
)


def test_lrc_matches_live_frames():
    read_body = reg_read_body(1008, 156)
    assert lrc(read_body) == 0xAD
    assert build(read_body)[-5] == 0xAD
    # Response to a successful select is status 0, length 0, LRC 0xC7.
    assert lrc(bytes.fromhex("0E 00 70 B1 0A 00 00 00")) == 0xC7
    assert lrc(select_body(0x01)) == 0xC5
    assert lrc(select_body(0x04)) != lrc(select_body(0x01))


def test_decode_resultant_and_first_taxel():
    data = bytearray(30 + 3)
    data[0] = 10  # Fx 1.0 N
    data[1] = 0xFE  # Fy -0.2 N
    data[2] = 10  # Fz 1.0 N
    data[30] = 0xFF  # taxel Fx -0.1 N
    data[31] = 0
    data[32] = 25  # taxel Fz 2.5 N
    parsed = decode_registers(bytes(data), n_points=1)
    assert parsed["fx"] == 1.0
    assert parsed["fy"] == -0.2
    assert parsed["fz"] == 1.0
    assert parsed["points"] == [[-0.1, 0.0, 2.5]]


def test_counter_fill_after_the_array_is_not_force():
    data = bytearray(30 + 146 * 3)
    data[30] = 10
    data[32] = 10
    start = 30 + 60 * 3
    for i in range(146 * 3 - 60 * 3):
        data[start + i] = (0x12 + i) & 0xFF
    parsed = decode_registers(bytes(data))
    assert len(parsed["points"]) == 60
    assert parsed["points"][0] == [1.0, 0.0, 1.0]
    assert parsed["points"][1:] == [[0.0, 0.0, 0.0]] * 59


def test_payload_status_and_prefix():
    body = bytes.fromhex("0E 00 70 C0 06 00 00 00")
    frame = build(body[:6] + b"\x00" + (0).to_bytes(2, "little"))
    # status is not zero on this hand-built body; payload_of must reject it
    bad = bytearray(build(bytes.fromhex("0E 00 70 C0 06 11 00 00")))
    assert payload_of(bytes(bad)) is None
    assert frame.startswith(b"\x55\xaa\x7b\x7b")


def test_model_table_and_window():
    code, n = resolve_model("pxsr-stddp03g")
    assert code == "PXSR-STDDP03G"
    assert n == 52
    assert resolve_model(None)[1] == 52
    assert READABLE_POINTS == 146
    try:
        resolve_model("not-a-sensor")
    except KeyError:
        pass
    else:
        raise AssertionError("unknown model should fail")


def test_dry_run_pts_sample():
    sensor = PaxiniTactileSensor("tac", {"port": "COM19", "model": "PXSR-STDDP03F"}, SensorContext(dry_run=True))
    assert sensor.transport == "pts_cb01"
    assert sensor.n_points == 31
    sensor.open()
    sample = sensor.read()
    sensor.close()
    assert sample["transport"] == "pts_cb01"
    assert sample["num"] == 2
    assert [m["mask"] for m in sample["modules"]] == [0x01, 0x04]
    assert len(sample["component_forces"]) == 31
    assert len(sample["rest_force"]) == 3
    assert sample["points_truncated"] is False


def test_omega_pad_is_marked_truncated():
    sensor = PaxiniTactileSensor("tac", {"model": "PXSR-STDCP03A"}, SensorContext(dry_run=True))
    sensor.open()
    sample = sensor.read()
    sensor.close()
    assert sample["n_points"] == 239
    assert sample["points_shown"] == 146
    assert sample["points_truncated"] is True


def test_dll_transport_still_sixty_points():
    assert resolve_transport({"dll_path": "ReadPaxiniLib.dll"}) == "dll"
    assert resolve_transport({}) == "pts_cb01"
    sensor = PaxiniTactileSensor("tac", {"transport": "dll"}, SensorContext(dry_run=True))
    sensor.open()
    sample = sensor.read()
    sensor.close()
    assert len(sample["component_forces"]) == 60
    assert sample["grid_shape"] == [6, 10, 3]
