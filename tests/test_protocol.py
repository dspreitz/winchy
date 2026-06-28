# Winchy - glider winch rope force & advice system
# Copyright (C) 2026 Dominic Spreitz
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version. Distributed WITHOUT ANY WARRANTY.
# See the GNU General Public License for more details, and the LICENSE
# file or <https://www.gnu.org/licenses/> for the full text.
#
# SPDX-License-Identifier: GPL-3.0-or-later

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "firmware",
                                "shared"))

import protocol


def test_telemetry_roundtrip():
    frame = protocol.encode_telemetry(
        seq=42, phase=protocol.PHASE_CLIMB, force=-123456, angle_deg=37.3,
        altitude_m=412, batt_mv=4970,
        flags=protocol.FLAG_GPS_FIX | protocol.FLAG_FORCE_UNCALIBRATED,
        batt_pct=87, glider_speed_ms=28.4)
    msg = protocol.decode(frame)
    assert msg["type"] == protocol.TELEMETRY
    assert msg["seq"] == 42
    assert msg["phase"] == protocol.PHASE_CLIMB
    assert msg["force"] == -123456
    assert abs(msg["angle_deg"] - 37.5) < 0.26  # half-degree resolution
    assert msg["altitude_m"] == 412
    assert abs(msg["batt_v"] - 4.9) < 0.11
    assert msg["batt_pct"] == 87
    assert abs(msg["glider_speed_ms"] - 28.4) < 0.06  # 0.1 m/s resolution
    assert msg["flags"] & protocol.FLAG_GPS_FIX
    assert msg["flags"] & protocol.FLAG_FORCE_UNCALIBRATED


def test_telemetry_clamping():
    frame = protocol.encode_telemetry(seq=0, phase=0, force=0,
                                      angle_deg=200.0, altitude_m=-5,
                                      batt_mv=99999, flags=0)
    msg = protocol.decode(frame)
    assert msg["angle_deg"] == 127.5
    assert msg["altitude_m"] == 0
    assert msg["batt_v"] == 25.5


def test_seq_wraps():
    frame = protocol.encode_telemetry(seq=70000, phase=0, force=0,
                                      angle_deg=0, altitude_m=0,
                                      batt_mv=0, flags=0)
    assert protocol.decode(frame)["seq"] == 70000 & 0xFFFF


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


def test_winch_pos_roundtrip():
    frame = protocol.encode_winch_pos(
        seq=11, lat_deg=48.1234567, lon_deg=11.7654321, altitude_m=520,
        hacc_m=1.4, status=protocol.WINCH_FIX | protocol.WINCH_SURVEY_DONE)
    msg = protocol.decode(frame)
    assert msg["type"] == protocol.WINCH_POS
    assert msg["seq"] == 11
    assert abs(msg["lat"] - 48.1234567) < 1e-6   # 1e-7 deg ~ 1.1 cm
    assert abs(msg["lon"] - 11.7654321) < 1e-6
    assert msg["altitude_m"] == 520
    assert abs(msg["hacc_m"] - 1.4) < 0.06
    assert msg["status"] & protocol.WINCH_FIX
    assert msg["status"] & protocol.WINCH_SURVEY_DONE


def test_winch_pos_negative_and_clamp():
    frame = protocol.encode_winch_pos(seq=0, lat_deg=-33.8688, lon_deg=151.2093,
                                      altitude_m=-10, hacc_m=99.0)
    msg = protocol.decode(frame)
    assert abs(msg["lat"] + 33.8688) < 1e-6   # southern hemisphere
    assert abs(msg["lon"] - 151.2093) < 1e-6
    assert msg["altitude_m"] == 0             # clamped at 0
    assert msg["hacc_m"] == 25.5              # saturates (byte, 0.1 m)
    assert msg["status"] == 0


def test_rejects_unknown_version():
    frame = bytearray(protocol.encode_mass(0, 100.0, 50))
    frame[0] = 99
    assert protocol.decode(bytes(frame)) is None


def test_rejects_unknown_type():
    frame = bytearray(protocol.encode_mass(0, 100.0, 50))
    frame[1] = 99
    assert protocol.decode(bytes(frame)) is None


def test_rejects_truncated_and_garbage():
    frame = protocol.encode_telemetry(0, 0, 0, 0, 0, 0, 0)
    assert protocol.decode(frame[:-1]) is None
    assert protocol.decode(b"") is None
    assert protocol.decode(None) is None
    assert protocol.decode(b"\xff\xfe\x85\x97&G\t") is None  # old 7-byte format


def test_upload_cmd_roundtrip():
    msg = protocol.decode(protocol.encode_upload_cmd(seq=12, nonce=200))
    assert msg["type"] == protocol.UPLOAD_CMD
    assert msg["seq"] == 12
    assert msg["nonce"] == 200


def test_upload_ack_roundtrip():
    msg = protocol.decode(protocol.encode_upload_ack(seq=13, nonce=200))
    assert msg["type"] == protocol.UPLOAD_ACK
    assert msg["nonce"] == 200


def test_upload_nonce_wraps():
    assert protocol.decode(
        protocol.encode_upload_cmd(0, 300))["nonce"] == 300 & 0xFF


def test_upload_cmd_and_ack_distinct_5_byte_frames():
    cmd = protocol.encode_upload_cmd(1, 1)
    ack = protocol.encode_upload_ack(1, 1)
    assert len(cmd) == 5 and len(ack) == 5      # tiny, negligible duty cycle
    assert cmd != ack                           # different type byte
    assert protocol.decode(cmd)["type"] == protocol.UPLOAD_CMD
    assert protocol.decode(ack)["type"] == protocol.UPLOAD_ACK
