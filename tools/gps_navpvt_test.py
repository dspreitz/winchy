import machine, time, struct

RX, TX, BAUD = 9, 8, 115200


def _ubx(cls, mid, p=b""):
    body = bytes([cls, mid, len(p) & 0xFF, (len(p) >> 8) & 0xFF]) + p
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return bytes([0xB5, 0x62]) + body + bytes([a, b])


u = machine.UART(1, baudrate=BAUD, rx=machine.Pin(RX), tx=machine.Pin(TX), timeout=200)
time.sleep_ms(150)

# VALSET (RAM): NAV-PVT on, GGA/RMC off
p = bytes((0, 1, 0, 0))
for k, v in [(0x20910007, b"\x01"), (0x209100BB, b"\x00"), (0x209100AC, b"\x00")]:
    p += k.to_bytes(4, "little") + v
u.write(_ubx(0x06, 0x8A, p))
time.sleep_ms(250)

buf = b""; t = time.ticks_ms(); found = 0
while time.ticks_diff(time.ticks_ms(), t) < 6000 and found < 2:
    if u.any():
        buf += u.read(u.any())
    i = buf.find(b"\xb5\x62\x01\x07")
    if i >= 0 and len(buf) >= i + 6:
        ln = buf[i + 4] | (buf[i + 5] << 8); end = i + 6 + ln + 2
        if len(buf) >= end:
            pl = buf[i + 6:i + 6 + ln]; buf = buf[end:]
            if len(pl) >= 92:
                yr, mo, d, h, mi, s, valid = struct.unpack_from("<HBBBBBB", pl, 4)
                ft, fl, _, nsv = struct.unpack_from("<BBBB", pl, 20)
                lon, lat, _, hmsl = struct.unpack_from("<iiii", pl, 24)
                hacc = struct.unpack_from("<I", pl, 40)[0]
                vd, gs = struct.unpack_from("<ii", pl, 56)
                sacc = struct.unpack_from("<I", pl, 68)[0]
                pdop = struct.unpack_from("<H", pl, 76)[0]
                print("fix=%d ok=%d sats=%d lat=%.7f lon=%.7f alt=%.1f pdop=%.2f "
                      "hacc=%.1fm gspd=%.2f climb=%.2f sacc=%.2f t=%04d-%02d-%02d "
                      "%02d:%02d:%02d valid=0x%02x" % (ft, fl & 1, nsv, lat * 1e-7,
                      lon * 1e-7, hmsl / 1000, pdop * 0.01, hacc / 1000, gs / 1000,
                      -vd / 1000, sacc / 1000, yr, mo, d, h, mi, s, valid))
                found += 1
    time.sleep_ms(50)
if not found:
    print("no NAV-PVT seen; sample:", buf[:60])
