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

# Minimal NMEA parsing shared by both segments. Pure (no hardware), so it runs
# and tests on desktop CPython and on either board. Handles the only sentences
# Winchy uses: GGA (fix/sats/position) and RMC (UTC + ground speed). The talker
# prefix is ignored, so $GPGGA and $GNGGA both parse.
#
# NOTE: the rope still carries its own copy in winchy/sensors/gps.py; that
# should migrate to this module when the two firmware trees are unified onto
# the identical T-Beam Supreme hardware.


def _coord(value, hemisphere, degree_digits):
    """ddmm.mmmm / dddmm.mmmm -> signed decimal degrees, or None if blank."""
    if not value:
        return None
    degrees = int(value[:degree_digits]) + float(value[degree_digits:]) / 60
    if hemisphere in ("S", "W"):
        degrees = -degrees
    return degrees


def parse_nmea(line):
    """Parse one NMEA sentence (bytes or str). Returns a dict for GGA/RMC,
    else None:
      {'type': 'GGA', 'fix': int, 'sats': int,
       'lat': f|None, 'lon': f|None, 'alt_m': f|None}
      {'type': 'RMC', 'valid': bool,
       'datetime': (y, mo, d, h, mi, s)|None, 'speed_ms': f|None}
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
            speed_ms = (float(fields[7]) * 0.514444
                        if len(fields) > 7 and fields[7] else None)
            return {"type": "RMC", "valid": valid, "datetime": dt,
                    "speed_ms": speed_ms}
    except (ValueError, IndexError, UnicodeError):
        pass
    return None
