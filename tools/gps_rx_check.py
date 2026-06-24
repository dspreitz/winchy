# All-binary GPS receive check: parse a streamed NAV-PVT (fix + position) and
# poll NAV-SAT (satellites tracked/used + SNR). No NMEA/ASCII decode.
import machine, time, struct


def _ubx(cls, mid, p=b""):
    body = bytes([cls, mid, len(p) & 0xFF, (len(p) >> 8) & 0xFF]) + p
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return bytes([0xB5, 0x62]) + body + bytes([a, b])


u = machine.UART(1, baudrate=115200, rx=machine.Pin(9), tx=machine.Pin(8), timeout=200)
time.sleep_ms(150)


def readbuf(ms):
    while u.any():
        u.read(u.any())
    t = time.ticks_ms(); buf = b""
    while time.ticks_diff(time.ticks_ms(), t) < ms:
        if u.any():
            buf += u.read(u.any())
        time.sleep_ms(20)
    return buf


buf = readbuf(1500)
i = buf.find(b"\xb5\x62\x01\x07")
if i >= 0 and len(buf) >= i + 6:
    ln = buf[i + 4] | (buf[i + 5] << 8); pl = buf[i + 6:i + 6 + ln]
    if len(pl) >= 92:
        yr, mo, d, h, mi, s, valid = struct.unpack_from("<HBBBBBB", pl, 4)
        ft, fl, _, nsv = struct.unpack_from("<BBBB", pl, 20)
        lon, lat, _e, hmsl = struct.unpack_from("<iiii", pl, 24)
        hacc = struct.unpack_from("<I", pl, 40)[0]
        vd, gs = struct.unpack_from("<ii", pl, 56)
        print("NAV-PVT: fixType=%d gnssOK=%d numSV=%d lat=%.6f lon=%.6f alt=%.1f "
              "hacc=%.1fm gspd=%.2f t=%04d-%02d-%02d %02d:%02d:%02d valid=0x%02x" % (
              ft, fl & 1, nsv, lat * 1e-7, lon * 1e-7, hmsl / 1000, hacc / 1000,
              gs / 1000, yr, mo, d, h, mi, s, valid))
else:
    print("no NAV-PVT frame; UBX-sync in 1.5s:", buf.count(b"\xb5\x62"))

u.write(_ubx(0x01, 0x35))
buf = readbuf(1500)
i = buf.find(b"\xb5\x62\x01\x35")
if i >= 0 and len(buf) >= i + 8:
    ln = buf[i + 4] | (buf[i + 5] << 8); pl = buf[i + 6:i + 6 + ln]
    if len(pl) >= 8:
        n = pl[5]; cnos = []; used = 0
        for k in range(8, min(len(pl), 8 + n * 12), 12):
            cnos.append(pl[k + 2])
            f = pl[k + 8] | (pl[k + 9] << 8) | (pl[k + 10] << 16) | (pl[k + 11] << 24)
            if f & 0x08:
                used += 1
        cnos.sort(reverse=True)
        print("NAV-SAT: tracked=%d used_in_fix=%d topSNR=%s" % (n, used, cnos[:8]))
else:
    print("no NAV-SAT reply")
