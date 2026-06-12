import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "firmware",
                                "shared"))

import protocol


def test_telemetry_roundtrip():
    frame = protocol.encode_telemetry(
        seq=42, phase=protocol.PHASE_CLIMB, force=-123456, angle_deg=37.3,
        pressure_hpa=979.7, batt_mv=4970,
        flags=protocol.FLAG_GPS_FIX | protocol.FLAG_FORCE_UNCALIBRATED)
    msg = protocol.decode(frame)
    assert msg["type"] == protocol.TELEMETRY
    assert msg["seq"] == 42
    assert msg["phase"] == protocol.PHASE_CLIMB
    assert msg["force"] == -123456
    assert abs(msg["angle_deg"] - 37.5) < 0.26  # half-degree resolution
    assert abs(msg["pressure_hpa"] - 979.7) < 0.05
    assert abs(msg["batt_v"] - 4.9) < 0.11
    assert msg["flags"] & protocol.FLAG_GPS_FIX
    assert msg["flags"] & protocol.FLAG_FORCE_UNCALIBRATED


def test_telemetry_clamping():
    frame = protocol.encode_telemetry(seq=0, phase=0, force=0,
                                      angle_deg=200.0, pressure_hpa=-5,
                                      batt_mv=99999, flags=0)
    msg = protocol.decode(frame)
    assert msg["angle_deg"] == 127.5
    assert msg["pressure_hpa"] == 0
    assert msg["batt_v"] == 25.5


def test_seq_wraps():
    frame = protocol.encode_telemetry(seq=70000, phase=0, force=0,
                                      angle_deg=0, pressure_hpa=0,
                                      batt_mv=0, flags=0)
    assert protocol.decode(frame)["seq"] == 70000 & 0xFFFF


def test_time_sync_roundtrip():
    frame = protocol.encode_time_sync(seq=7, epoch_s=1781300467)
    msg = protocol.decode(frame)
    assert msg["type"] == protocol.TIME_SYNC
    assert msg["seq"] == 7
    assert msg["epoch_s"] == 1781300467


def test_mass_roundtrip():
    frame = protocol.encode_mass(seq=8, mass_kg=540.3, confidence_pct=85)
    msg = protocol.decode(frame)
    assert msg["type"] == protocol.MASS
    assert abs(msg["mass_kg"] - 540.3) < 0.05
    assert msg["confidence_pct"] == 85


def test_summary_roundtrip():
    frame = protocol.encode_summary(seq=9, duration_s=38.4, max_force=812345,
                                    release_alt_m=412, mass_kg=540.3)
    msg = protocol.decode(frame)
    assert msg["type"] == protocol.SUMMARY
    assert abs(msg["duration_s"] - 38.4) < 0.05
    assert msg["max_force"] == 812345
    assert msg["release_alt_m"] == 412
    assert abs(msg["mass_kg"] - 540.3) < 0.05


def test_rejects_unknown_version():
    frame = bytearray(protocol.encode_time_sync(0, 1))
    frame[0] = 99
    assert protocol.decode(bytes(frame)) is None


def test_rejects_unknown_type():
    frame = bytearray(protocol.encode_time_sync(0, 1))
    frame[1] = 99
    assert protocol.decode(bytes(frame)) is None


def test_rejects_truncated_and_garbage():
    frame = protocol.encode_telemetry(0, 0, 0, 0, 0, 0, 0)
    assert protocol.decode(frame[:-1]) is None
    assert protocol.decode(b"") is None
    assert protocol.decode(None) is None
    assert protocol.decode(b"\xff\xfe\x85\x97&G\t") is None  # old 7-byte format
