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


def configure(uart, rate_hz=5):
    """Configure a u-blox M10 over UART: emit only the sentences we parse
    (GGA + RMC) and raise the nav rate. RAM-only (resets on power cycle), so
    call this at every boot.

    At 9600 baud only GGA+RMC fit at 5 Hz (~770 B/s of the ~960 B/s budget) -
    do NOT re-enable the other sentences here without a faster baud.
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
