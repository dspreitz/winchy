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

# Host tests for the u-blox M10 driver (winchy/sensors/gps.py): both the
# CONFIGURATION it sends (UBX-CFG-VALSET keys/values, UBX framing + checksum)
# and the FUNCTIONALITY it parses (UBX-NAV-PVT, the UBX byte-stream framer, and
# the legacy NMEA fallback). The driver is pure (struct + time only), so it runs
# unchanged on CPython once the MicroPython time shims below are in place.

import asyncio
import os
import struct
import sys
import time

if not hasattr(time, "sleep_ms"):                 # MicroPython time shims
    time.sleep_ms = lambda ms: None
    time.ticks_ms = lambda: int(time.monotonic() * 1000)
    time.ticks_add = lambda t, d: t + d
    time.ticks_diff = lambda a, b: a - b

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "firmware",
                                "rope"))

from winchy.sensors import gps


# --- helpers ---------------------------------------------------------------

def make_navpvt(fix_type=3, gnss_ok=True, num_sv=12, lon_deg=11.5, lat_deg=48.5,
                hmsl_mm=512345, hacc_mm=1500, vacc_mm=2500, veld_mm=-300,
                gspeed_mm=28400, sacc_mm=500, pdop=123,
                dt=(2026, 6, 27, 16, 45, 10), resolved=True):
    """Build a 92-byte UBX-NAV-PVT payload with known fields (see the u-blox M10
    interface description for the offsets)."""
    pl = bytearray(92)
    struct.pack_into("<I", pl, 0, 0)                       # iTOW
    y, mo, d, h, mi, s = dt
    struct.pack_into("<HBBBBB", pl, 4, y, mo, d, h, mi, s)
    pl[11] = 0x01 | 0x02 | (0x04 if resolved else 0x00)   # validDate|Time|Resolved
    pl[20] = fix_type
    pl[21] = 0x01 if gnss_ok else 0x00                     # flags: gnssFixOK
    pl[23] = num_sv
    struct.pack_into("<i", pl, 24, round(lon_deg * 1e7))
    struct.pack_into("<i", pl, 28, round(lat_deg * 1e7))
    struct.pack_into("<i", pl, 36, hmsl_mm)               # hMSL
    struct.pack_into("<I", pl, 40, hacc_mm)
    struct.pack_into("<I", pl, 44, vacc_mm)
    struct.pack_into("<i", pl, 56, veld_mm)               # velD (+down)
    struct.pack_into("<i", pl, 60, gspeed_mm)             # gSpeed (2-D)
    struct.pack_into("<I", pl, 68, sacc_mm)
    struct.pack_into("<H", pl, 76, pdop)
    return bytes(pl)


def fletcher(body):
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return a, b


def decode_valset(msg):
    """Return (layers, {key: value_bytes}) from a UBX-CFG-VALSET frame, sizing
    each value from the key's type nibble (0x1/0x2 -> 1 B, 0x3 -> 2 B, 0x4 -> 4 B)."""
    assert msg[0:2] == b"\xb5\x62"
    assert msg[2] == 0x06 and msg[3] == 0x8A          # CFG-VALSET
    ln = msg[4] | (msg[5] << 8)
    p = msg[6:6 + ln]
    layers = p[1]
    items, i = {}, 4
    while i < len(p):
        key = int.from_bytes(p[i:i + 4], "little")
        i += 4
        size = {1: 1, 2: 1, 3: 2, 4: 4}[(key >> 28) & 0x7]
        items[key] = p[i:i + size]
        i += size
    return layers, items


class RecUart:
    def __init__(self):
        self.writes = []

    def write(self, b):
        self.writes.append(bytes(b))


class FakeReader:
    def __init__(self, *chunks):
        self.chunks = list(chunks)

    async def read(self, n):
        return self.chunks.pop(0) if self.chunks else b""


def read_one_ubx(*chunks):
    gps._ubx_buf = b""                                # reset the module framer
    return asyncio.run(asyncio.wait_for(gps.read_ubx(FakeReader(*chunks)), 2))


# --- NAV-PVT parsing (functionality) ---------------------------------------

def test_navpvt_3d_fix_fields():
    d = gps.parse_nav_pvt(make_navpvt())
    assert d["fix"] == 3
    assert d["sats"] == 12
    assert abs(d["lon"] - 11.5) < 1e-6
    assert abs(d["lat"] - 48.5) < 1e-6
    assert abs(d["alt_m"] - 512.345) < 1e-3
    assert abs(d["hacc_m"] - 1.5) < 1e-6
    assert abs(d["vacc_m"] - 2.5) < 1e-6
    assert abs(d["gspeed_ms"] - 28.4) < 1e-6
    assert abs(d["climb_ms"] - 0.3) < 1e-6          # velD -300 mm/s -> +0.3 up
    assert abs(d["pdop"] - 1.23) < 1e-6
    assert d["datetime"] == (2026, 6, 27, 16, 45, 10)


def test_navpvt_no_fix_when_gnssfixok_clear():
    # fixType says 3D but the gnssFixOK flag is clear -> report no fix, so a
    # half-acquired solution is never trusted for position/QNH.
    d = gps.parse_nav_pvt(make_navpvt(fix_type=3, gnss_ok=False))
    assert d["fix"] == 0
    assert d["sats"] == 12                           # sats still reported


def test_navpvt_datetime_gated_on_fullyresolved():
    assert gps.parse_nav_pvt(make_navpvt(resolved=False))["datetime"] is None
    assert gps.parse_nav_pvt(make_navpvt(resolved=True))["datetime"] is not None


def test_navpvt_short_payload_rejected():
    assert gps.parse_nav_pvt(b"\x00" * 91) is None


def test_navpvt_real_capture_parses():
    # A real frame captured from the bench M10 while re-acquiring (fix 0, sats 0).
    # Its date/time fields are populated from backup RAM, but the fullyResolved
    # bit in `valid` (0xf3) is CLEAR, so parse_nav_pvt must gate datetime to None
    # - precisely the protection we want: never sync the RTC off an unresolved
    # GPS time. Guards the real-world byte layout against the struct offsets.
    pl = bytes.fromhex(
        "40b07e22ea07061b102d0af3ffffffff642501000000240073e171e1"
        "8649852114f1e0006595e000ffffffff007884df00000000000000000000"
        "0000000000000000000000cbbd000080a812010f2700005c40562f00000000"
        "00000000")
    d = gps.parse_nav_pvt(pl)
    assert d is not None
    assert d["fix"] == 0
    assert d["sats"] == 0
    assert d["datetime"] is None          # valid byte 0xf3 -> fullyResolved clear


# --- CFG-VALSET configuration (the config the driver sends) -----------------

def test_configure_enables_navpvt_and_disables_all_nmea():
    u = RecUart()
    gps.configure(u, rate_hz=5)
    layers, items = decode_valset(u.writes[0])
    assert layers == 0x03                              # RAM | BBR
    assert items[gps._CFG_MSGOUT_NAV_PVT] == b"\x01"   # binary NAV-PVT ON
    # Every NMEA sentence must be OFF - the app reads UBX only; a stray NMEA
    # 'on' is exactly what made the old high-baud check (which looked for '$G')
    # misfire.
    for key, _ in gps._CFG_MSGOUT_UART1:
        assert items[key] == b"\x00"


def test_configure_sets_airborne_model_and_signals():
    u = RecUart()
    gps.configure(u, rate_hz=5)
    _, items = decode_valset(u.writes[0])
    assert items[gps._CFG_DYNMODEL] == bytes((gps._DYN_AIRBORNE_2G,))
    for sig in (gps._CFG_SIG_GPS, gps._CFG_SIG_SBAS,
                gps._CFG_SIG_GAL, gps._CFG_SIG_BDS):
        assert items[sig] == b"\x01"


def test_configure_rate_encoding():
    u = RecUart()
    gps.configure(u, rate_hz=5)                        # 1000/5 = 200 ms
    _, items = decode_valset(u.writes[0])
    assert items[gps._CFG_RATE_MEAS] == (200).to_bytes(2, "little")
    assert items[gps._CFG_RATE_NAV] == (1).to_bytes(2, "little")


def test_configure_clamps_fast_rate():
    u = RecUart()
    gps.configure(u, rate_hz=50)                       # 1000/50 = 20 -> clamp 50
    _, items = decode_valset(u.writes[0])
    assert items[gps._CFG_RATE_MEAS] == (50).to_bytes(2, "little")


def test_set_baud_message():
    u = RecUart()
    gps.set_baud(u, 115200)
    layers, items = decode_valset(u.writes[0])
    assert layers == 0x01                              # RAM only
    assert items[0x40520001] == (115200).to_bytes(4, "little")


def test_mga_ini_time_message_well_formed():
    u = RecUart()
    gps.mga_ini_time_utc(u, (2026, 6, 27, 16, 45, 10), acc_s=4)
    msg = u.writes[0]
    assert msg[0:2] == b"\xb5\x62"
    assert msg[2] == 0x13 and msg[3] == 0x40           # MGA-INI-TIME_UTC
    ln = msg[4] | (msg[5] << 8)
    a, b = fletcher(msg[2:6 + ln])
    assert (a, b) == (msg[-2], msg[-1])                # valid checksum
    assert struct.unpack_from("<H", msg, 6 + 4)[0] == 2026   # year field


# --- UBX framing (the _ubx builder + read_ubx stream framer) ---------------

def test_ubx_builder_checksum_and_sync():
    msg = gps._ubx(0x06, 0x8A, b"\x01\x02\x03")
    assert msg[0:2] == b"\xb5\x62"
    assert msg[2:4] == b"\x06\x8a"
    assert msg[4:6] == (3).to_bytes(2, "little")
    a, b = fletcher(msg[2:-2])
    assert (a, b) == (msg[-2], msg[-1])


def test_read_ubx_extracts_navpvt():
    frame = gps._ubx(0x01, 0x07, make_navpvt(num_sv=9))
    cls, mid, payload = read_one_ubx(frame)
    assert (cls, mid) == (0x01, 0x07)
    assert gps.parse_nav_pvt(payload)["sats"] == 9


def test_read_ubx_resyncs_past_garbage_and_split_reads():
    frame = gps._ubx(0x01, 0x07, make_navpvt())
    # leading noise (no UBX sync) then the frame split across two reads
    cls, mid, payload = read_one_ubx(b"\x00\x11\x22noise", frame[:20], frame[20:])
    assert (cls, mid) == (0x01, 0x07)
    assert len(payload) == 92


def test_read_ubx_drops_bad_checksum_then_returns_good():
    good = gps._ubx(0x01, 0x07, make_navpvt(num_sv=7))
    bad = bytearray(gps._ubx(0x01, 0x07, make_navpvt(num_sv=3)))
    bad[-1] ^= 0xFF                                    # corrupt the checksum
    cls, mid, payload = read_one_ubx(bytes(bad), good)
    assert (cls, mid) == (0x01, 0x07)
    assert gps.parse_nav_pvt(payload)["sats"] == 7     # the good frame won


def test_read_ubx_skips_bogus_length():
    # A sync followed by an absurd length must not be trusted as a frame.
    frame = gps._ubx(0x01, 0x07, make_navpvt())
    bogus = b"\xb5\x62\x01\x07\x00\xff"                # length 0xff00 = 65280
    cls, mid, payload = read_one_ubx(bogus + frame)
    assert (cls, mid) == (0x01, 0x07)
    assert len(payload) == 92


# --- legacy NMEA fallback (still used as a liveness path) -------------------

def test_parse_nmea_gga():
    d = gps.parse_nmea(
        "$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47")
    assert d["type"] == "GGA"
    assert d["fix"] == 1 and d["sats"] == 8
    assert abs(d["lat"] - (48 + 7.038 / 60)) < 1e-6
    assert abs(d["lon"] - (11 + 31.0 / 60)) < 1e-6
    assert abs(d["alt_m"] - 545.4) < 1e-6


def test_parse_nmea_rmc_datetime_and_speed():
    d = gps.parse_nmea(
        "$GNRMC,081836,A,4807.038,N,01131.000,E,10.0,084.4,260606,,,A*43")
    assert d["type"] == "RMC" and d["valid"] is True
    assert d["datetime"] == (2006, 6, 26, 8, 18, 36)
    assert abs(d["speed_ms"] - 10.0 * 0.514444) < 1e-6


def test_parse_nmea_ignores_other_sentences():
    assert gps.parse_nmea("$GNVTG,,T,,M,0.0,N,0.0,K,A*23") is None
    assert gps.parse_nmea("not a sentence") is None
