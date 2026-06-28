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

# u-blox M10S GPS: UART wrapper plus minimal NMEA parsing (GGA for fix and
# position, RMC for UTC date/time used to sync the RTC at startup).

import struct
import time


class GPS:
    def __init__(self, uart):
        self._uart = uart

    def any(self):
        return self._uart.any()

    def readline(self):
        return self._uart.readline()

    def dump(self, lines=10, timeout_ms=5000):
        """Print NMEA traffic as a boot-time liveness check.

        Unlike the old monolith this cannot hang forever if the GPS is
        silent - it gives up after timeout_ms.
        """
        deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
        n = 0
        while n < lines:
            if self._uart.any():
                print(self._uart.readline())
                n += 1
            elif time.ticks_diff(deadline, time.ticks_ms()) < 0:
                print("GPS: no NMEA traffic within %d ms" % timeout_ms)
                return


def _ubx(cls, msg_id, payload=b""):
    """Frame a UBX message (sync chars + class/id/len + Fletcher checksum)."""
    body = bytes([cls, msg_id]) + len(payload).to_bytes(2, "little") + payload
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return b"\xb5\x62" + body + bytes([a, b])


def set_baud(uart, baud):
    """Set the module's UART1 baud (UBX-CFG-VALSET, RAM + BBR layers). Sent at the
    port's current baud; the caller must then reopen the host UART at `baud`.

    Written to BBR as well as RAM (layers 0x03) so the baud PERSISTS across power
    cycles - retained by the AXP2101 backup-domain charge (board.py), the same
    rail that keeps the ephemeris/almanac alive. The MAX-M10S has NO config flash
    (a Flash-layer write is NAKed - verified on-device), so BBR is the only
    non-volatile option. gps_task probes the actual baud at boot, so a lost BBR
    (full de-power) just falls back to 9600 and re-raises + re-persists here.
    """
    key = 0x40520001  # CFG-UART1-BAUDRATE (U4)
    payload = (b"\x00\x03\x00\x00"            # version 0, layers=RAM|BBR, reserved
               + key.to_bytes(4, "little") + baud.to_bytes(4, "little"))
    uart.write(_ubx(0x06, 0x8A, payload))
    time.sleep_ms(150)


# M10 config-database keys (UBX-CFG-VALSET). Sizes are encoded in the key:
# 0x3.. = U2, 0x2.. = U1/E1. NMEA message output is per (id, port).
_CFG_RATE_MEAS = 0x30210001          # U2, ms between measurements
_CFG_RATE_NAV = 0x30210002           # U2, measurements per nav solution
_CFG_DYNMODEL = 0x20110021           # E1, navigation dynamic model
_CFG_SIG_GPS = 0x1031001F            # L, GPS enable
_CFG_SIG_SBAS = 0x10310020           # L, SBAS (EGNOS) enable
_CFG_SIG_GAL = 0x10310021            # L, Galileo enable
_CFG_SIG_BDS = 0x10310022            # L, BeiDou enable
_DYN_AIRBORNE_2G = 7                 # allow launch dynamics (climb + accel)
_CFG_MSGOUT_NAV_PVT = 0x20910007     # U1, UBX-NAV-PVT output on UART1
_CFG_MSGOUT_UART1 = (                # NMEA off: we read binary UBX-NAV-PVT now
    (0x209100BB, False),             # GGA
    (0x209100AC, False),             # RMC
    (0x209100CA, False),             # GLL
    (0x209100C0, False),             # GSA
    (0x209100C5, False),             # GSV
    (0x209100B1, False),             # VTG
)


def _valset(layers, items):
    """Frame a UBX-CFG-VALSET. layers: bit0 RAM, bit1 BBR, bit2 Flash.
    items: iterable of (key_u32, value_bytes)."""
    p = bytes((0x00, layers, 0x00, 0x00))
    for key, val in items:
        p += key.to_bytes(4, "little") + val
    return _ubx(0x06, 0x8A, p)


def configure(uart, rate_hz=5):
    """Configure the u-blox M10 via the config database (UBX-CFG-VALSET): emit
    only the sentences we parse (GGA + RMC) and set the nav rate. M10 dropped
    the legacy UBX-CFG-MSG/CFG-RATE, so the keyed interface is the supported one
    (its acceptance is ACK-verified on this module).

    Written to RAM + BBR (layers 0x03): RAM applies it now; BBR persists it
    across power cycles WITHOUT flash wear, retained by the AXP2101 backup-domain
    charge (board.py) - the same rail that keeps the receiver's ephemeris/almanac
    alive for a warm start. The baud is persisted to BBR too now (set_baud), and
    gps_task probes the live baud at boot. Volume is ~150 B/epoch, so the
    baud must support rate_hz * 150 B/s (10 Hz needs the raised baud, not 9600).
    """
    meas = max(50, min(1000, 1000 // rate_hz))
    items = [(_CFG_RATE_MEAS, meas.to_bytes(2, "little")),
             (_CFG_RATE_NAV, (1).to_bytes(2, "little"))]
    for key, keep in _CFG_MSGOUT_UART1:
        items.append((key, b"\x01" if keep else b"\x00"))
    # Airborne dynamic model: the rope rides a winch launch (hard acceleration,
    # fast climb through rotation), so the default 'portable' model would smooth
    # away exactly the dynamics we want. Pin the multi-GNSS + SBAS set too
    # (already the M10 default here: GPS + Galileo + BeiDou + EGNOS) so it is
    # guaranteed and documented; GLONASS is left off - the M10 concurrent-GNSS
    # budget is better spent on Galileo+BeiDou for Europe.
    items.append((_CFG_DYNMODEL, bytes((_DYN_AIRBORNE_2G,))))
    items.append((_CFG_SIG_GPS, b"\x01"))
    items.append((_CFG_SIG_SBAS, b"\x01"))
    items.append((_CFG_SIG_GAL, b"\x01"))
    items.append((_CFG_SIG_BDS, b"\x01"))
    items.append((_CFG_MSGOUT_NAV_PVT, b"\x01"))   # one binary msg, all we need
    uart.write(_valset(0x03, items))     # layers: RAM | BBR (no flash wear)
    time.sleep_ms(50)


def poll_ubx(uart, cls, mid, timeout_ms=1500):
    """Send a UBX poll (zero-length) and return the full matching UBX frame
    (sync..checksum) or None. Used to read UBX-SEC-UNIQID / UBX-MON-VER for the
    AssistNow ZTP device registration. Call before the NMEA read loop starts so
    the binary reply isn't consumed by the line reader."""
    while uart.any():                    # flush stale NMEA
        uart.read(uart.any())
    uart.write(_ubx(cls, mid))
    deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
    buf = b""
    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        if uart.any():
            buf += uart.read(uart.any())
        time.sleep_ms(20)
    i = buf.find(bytes((0xB5, 0x62, cls, mid)))
    if i < 0 or i + 6 > len(buf):
        return None
    end = i + 6 + (buf[i + 4] | (buf[i + 5] << 8)) + 2
    return buf[i:end] if end <= len(buf) else None


def feed_mga(uart, blob, chunk=128, gap_ms=15):
    """Stream a UBX MGA blob (AssistNow Predictive Orbits = concatenated
    UBX-MGA-ANO predicted-orbit messages) to the receiver. Already framed;
    the receiver stores the ones matching the current date and uses them at the
    next acquisition instead of downloading ephemeris over the air. Chunked with
    small gaps so the module's input buffer keeps up at the working baud."""
    for i in range(0, len(blob), chunk):
        uart.write(blob[i:i + chunk])
        time.sleep_ms(gap_ms)


def mga_ini_time_utc(uart, t, acc_s=2):
    """UBX-MGA-INI-TIME_UTC: coarse UTC time aiding so the predicted orbits
    can be used immediately (the right day is picked) instead of waiting for the
    satellite time-of-week. t = (year, month, day, hour, minute, second)."""
    y, mo, d, h, mi, s = t
    p = bytes((0x10, 0x00, 0x00, 0x80))             # type=UTC, ver=0, ref=0, leapSec=unknown
    p += y.to_bytes(2, "little") + bytes((mo, d, h, mi, s, 0x00))
    p += (0).to_bytes(4, "little")                  # ns
    p += acc_s.to_bytes(2, "little")                # tAccS (seconds)
    p += (0).to_bytes(2, "little")                  # reserved
    p += (0).to_bytes(4, "little")                  # tAccNs
    uart.write(_ubx(0x13, 0x40, p))
    time.sleep_ms(20)


def _coord(value, hemisphere, degree_digits):
    """ddmm.mmmm / dddmm.mmmm -> signed decimal degrees."""
    if not value:
        return None
    degrees = int(value[:degree_digits]) + float(value[degree_digits:]) / 60
    if hemisphere in ("S", "W"):
        degrees = -degrees
    return degrees


def parse_nmea(line):
    """Parse one NMEA sentence (bytes or str).

    Returns a dict for the sentences we use, else None:
      {'type': 'GGA', 'fix': int, 'sats': int, 'lat': f|None,
       'lon': f|None, 'alt_m': f|None, 'hdop': f|None}
      {'type': 'RMC', 'valid': bool,
       'datetime': (y, mo, d, h, mi, s) | None,    # UTC
       'speed_ms': f | None}                       # ground speed
    """
    try:
        if isinstance(line, bytes):
            line = line.decode("ascii")
        line = line.strip()
        if not line.startswith("$"):
            return None
        fields = line.split("*")[0].split(",")
        sentence = fields[0][-3:]

        if sentence == "GGA":
            return {
                "type": "GGA",
                "fix": int(fields[6] or 0),
                "sats": int(fields[7] or 0),
                "lat": _coord(fields[2], fields[3], 2),
                "lon": _coord(fields[4], fields[5], 3),
                "alt_m": float(fields[9]) if fields[9] else None,
                "hdop": float(fields[8]) if fields[8] else None,
            }

        if sentence == "RMC":
            valid = fields[2] == "A"
            dt = None
            if valid and fields[1] and fields[9]:
                t, d = fields[1], fields[9]
                dt = (2000 + int(d[4:6]), int(d[2:4]), int(d[0:2]),
                      int(t[0:2]), int(t[2:4]), int(float(t[4:])))
            # field 7 = speed over ground in knots -> m/s (1 kn = 0.514444 m/s)
            speed_ms = (float(fields[7]) * 0.514444
                        if len(fields) > 7 and fields[7] else None)
            return {"type": "RMC", "valid": valid, "datetime": dt,
                    "speed_ms": speed_ms}
    except (ValueError, IndexError, UnicodeError):
        pass
    return None


def parse_nav_pvt(pl):
    """Parse a UBX-NAV-PVT payload (>=92 B) into a dict, or None.

    One binary message carries everything the old GGA+RMC pair did, plus a real
    3-D velocity, direct accuracy estimates and pDOP:
      fix (0=none / 2=2D / 3=3D, only if the gnssFixOK flag is set), sats,
      lat, lon (deg), alt_m (MSL), pdop, hacc_m / vacc_m (position accuracy),
      gspeed_ms (2-D ground speed), climb_ms (GPS vertical rate), sacc_ms
      (speed accuracy), datetime (UTC, only once time is fully resolved).
    """
    if len(pl) < 92:
        return None
    year, month, day, hour, mn, sec, valid = struct.unpack_from("<HBBBBBB", pl, 4)
    t_acc = struct.unpack_from("<I", pl, 12)[0]   # time accuracy estimate, ns
    fix_type, flags, _f2, num_sv = struct.unpack_from("<BBBB", pl, 20)
    lon, lat, _h, hmsl = struct.unpack_from("<iiii", pl, 24)
    hacc, vacc = struct.unpack_from("<II", pl, 40)
    _vn, _ve, veld, gspeed = struct.unpack_from("<iiii", pl, 48)
    sacc = struct.unpack_from("<I", pl, 68)[0]
    pdop = struct.unpack_from("<H", pl, 76)[0]
    gnss_ok = bool(flags & 0x01)
    return {
        "fix": fix_type if gnss_ok else 0,
        "sats": num_sv,
        "lat": lat * 1e-7, "lon": lon * 1e-7,
        "alt_m": hmsl / 1000.0,
        "pdop": pdop * 0.01,
        "hacc_m": hacc / 1000.0, "vacc_m": vacc / 1000.0,
        "gspeed_ms": gspeed / 1000.0,
        "climb_ms": -veld / 1000.0,            # velD is +down; climb is +up
        "sacc_ms": sacc / 1000.0,
        "datetime": ((year, month, day, hour, mn, sec)
                     if (valid & 0x04) else None),    # bit2 = fullyResolved
        "t_acc_ns": t_acc,                            # time accuracy estimate, ns
    }


_ubx_buf = b""


async def read_ubx(reader):
    """Await one valid UBX frame on `reader` (asyncio StreamReader) and return
    (cls, id, payload). Accumulates with read() and frames in a persistent
    buffer - readexactly() does not drain this UART stream, while read() (what
    readline used) does. Resyncs past noise and drops bad-checksum frames."""
    global _ubx_buf
    while True:
        b = _ubx_buf
        i = b.find(b"\xb5\x62")
        if i < 0:
            _ubx_buf = b[-1:] if b else b          # keep a possible lone sync
        elif len(b) - i < 6:
            _ubx_buf = b[i:]                        # need the header
        else:
            ln = b[i + 4] | (b[i + 5] << 8)
            if ln > 512:
                _ubx_buf = b[i + 2:]               # bogus length -> skip sync
                continue
            end = i + 6 + ln + 2
            if len(b) < end:
                _ubx_buf = b[i:]                   # need the rest of the frame
            else:
                frame = b[i:end]
                _ubx_buf = b[end:]
                ca = cb = 0
                for x in frame[2:6 + ln]:
                    ca = (ca + x) & 0xFF
                    cb = (cb + ca) & 0xFF
                if ca == frame[-2] and cb == frame[-1]:
                    return frame[2], frame[3], frame[6:6 + ln]
                continue                           # bad checksum -> rescan
        data = await reader.read(128)
        if data:
            _ubx_buf += data


def has_gps_frame(buf):
    """True if `buf` holds at least one CHECKSUM-VALID UBX frame OR NMEA
    sentence - i.e. real GPS framing, not a chance 2-byte sync match in line
    noise. A bare "b5 62" / "$G" substring turns up ~10% of the time in the
    garbage you read at the WRONG baud, which made the old liveness check
    false-positive and leave the app/GPS baud mismatched (no sats). Requiring a
    valid checksum drops the false-positive rate to ~1/65536 (UBX) / ~1/256
    (NMEA). Used by the baud detector (_gps_alive)."""
    return _buf_has_ubx(buf) or _buf_has_nmea(buf)


def _buf_has_ubx(buf):
    n = len(buf)
    i = 0
    while True:
        j = buf.find(b"\xb5\x62", i)
        if j < 0:
            return False
        i = j + 2
        if j + 8 > n:                       # sync+class+id+len(2)+cksum(2) min
            continue
        ln = buf[j + 4] | (buf[j + 5] << 8)
        if ln > 512 or j + 6 + ln + 2 > n:  # implausible or incomplete -> skip
            continue
        ca = cb = 0
        for x in buf[j + 2:j + 6 + ln]:     # Fletcher over class..payload
            ca = (ca + x) & 0xFF
            cb = (cb + ca) & 0xFF
        if ca == buf[j + 6 + ln] and cb == buf[j + 7 + ln]:
            return True


def _buf_has_nmea(buf):
    for seg in buf.replace(b"\r", b"\n").split(b"\n"):
        d = seg.find(b"$")
        if d < 0:
            continue
        star = seg.find(b"*", d)
        if star < 0 or star + 3 > len(seg) or star == d + 1:
            continue
        try:
            given = int(seg[star + 1:star + 3], 16)
        except ValueError:
            continue
        cs = 0
        for c in seg[d + 1:star]:           # XOR of chars between $ and *
            cs ^= c
        if cs == given:
            return True
    return False
