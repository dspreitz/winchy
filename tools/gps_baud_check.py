import machine, time, struct


def check(baud):
    u = machine.UART(1, baudrate=baud, rx=machine.Pin(9), tx=machine.Pin(8), timeout=200)
    time.sleep_ms(200)
    while u.any():
        u.read(u.any())
    t = time.ticks_ms(); buf = b""
    while time.ticks_diff(time.ticks_ms(), t) < 1500:
        if u.any():
            buf += u.read(u.any())
        time.sleep_ms(20)
    sync = buf.count(b"\xb5\x62")
    pvt = buf.count(b"\xb5\x62\x01\x07")
    nmea = buf.count(b"$G")
    print("baud %6d: %d bytes, UBX-sync=%d NAV-PVT=%d NMEA=%d" % (baud, len(buf), sync, pvt, nmea))
    i = buf.find(b"\xb5\x62\x01\x07")
    if i >= 0 and len(buf) >= i + 6:
        ln = buf[i + 4] | (buf[i + 5] << 8); end = i + 6 + ln + 2
        if len(buf) >= end:
            pl = buf[i + 6:i + 6 + ln]
            if len(pl) >= 92:
                ft, fl, _, nsv = struct.unpack_from("<BBBB", pl, 20)
                lon, lat, _h, hmsl = struct.unpack_from("<iiii", pl, 24)
                hacc = struct.unpack_from("<I", pl, 40)[0]
                print("   NAV-PVT: fix=%d ok=%d sats=%d lat=%.6f lon=%.6f alt=%.1f hacc=%.1f" % (
                    ft, fl & 1, nsv, lat * 1e-7, lon * 1e-7, hmsl / 1000, hacc / 1000))
    return sync


for b in (115200, 9600):
    if check(b) > 0:
        break
