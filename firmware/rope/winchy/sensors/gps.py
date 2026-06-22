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
    """Set the module's UART1 baud (UBX-CFG-VALSET, RAM layer). Sent at the
    port's current baud; the caller must then reopen the host UART at `baud`.
    A power cycle resets the module to 9600, so this is part of the boot
    config and is re-sent every boot.
    """
    key = 0x40520001  # CFG-UART1-BAUDRATE (U4)
    payload = (b"\x00\x01\x00\x00"            # version 0, layers=RAM, reserved
               + key.to_bytes(4, "little") + baud.to_bytes(4, "little"))
    uart.write(_ubx(0x06, 0x8A, payload))
    time.sleep_ms(150)


def configure(uart, rate_hz=5):
    """Configure a u-blox M10 over UART: emit only the sentences we parse
    (GGA + RMC) and raise the nav rate. RAM-only (resets on power cycle), so
    call this at every boot. Volume is ~150 B per epoch, so the baud must
    support rate_hz * 150 B/s (10 Hz needs the raised baud, not 9600).
    """
    # NMEA message rates (class 0xF0): drop GLL/GSA/GSV/VTG, keep GGA/RMC.
    for msg_class, msg_id, on in ((0xF0, 0x01, 0), (0xF0, 0x02, 0),
                                  (0xF0, 0x03, 0), (0xF0, 0x05, 0),
                                  (0xF0, 0x00, 1), (0xF0, 0x04, 1)):
        uart.write(_ubx(0x06, 0x01, bytes([msg_class, msg_id, on])))
        time.sleep_ms(20)
    # CFG-RATE: measRate ms, navRate 1 cycle, timeRef 1 (GPS).
    meas = max(50, min(1000, 1000 // rate_hz))
    uart.write(_ubx(0x06, 0x08, meas.to_bytes(2, "little") + b"\x01\x00\x01\x00"))
    time.sleep_ms(20)


def save_config(uart):
    """Persist the current navigation/message config to non-volatile storage
    (UBX-CFG-CFG save) so a power cycle keeps the nav rate and GGA/RMC message
    set instead of re-running the cold config dance. Pairs with the AXP2101
    backup-domain charge (board.py) that keeps the receiver's battery-backed
    RAM - and thus its ephemeris/almanac - alive between runs for a warm start.

    saveMask 0xFFFE = all config sections except ioPort: the UART baud stays
    volatile (resets to 9600) on purpose, so the tested 9600 -> high-baud boot
    bring-up in gps_task is unchanged. deviceMask 0x17 = BBR | Flash | EEPROM |
    SPI flash (bits for absent devices are ignored).
    """
    payload = (b"\x00\x00\x00\x00"      # clearMask: clear nothing
               + b"\xfe\xff\x00\x00"     # saveMask: all sections except ioPort
               + b"\x00\x00\x00\x00"     # loadMask: load nothing
               + b"\x17")                # deviceMask: BBR | Flash | EEPROM | SPI
    uart.write(_ubx(0x06, 0x09, payload))
    time.sleep_ms(100)


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
       'lon': f|None, 'alt_m': f|None}
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
