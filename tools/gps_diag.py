# Deep GPS health probe: baud/liveness, GGA fix, UBX-NAV-SAT (satellites
# tracked + SNR = sky view, independent of a fix), and the configured dynModel.
import machine, time


def _ubx(cls, mid, p=b""):
    body = bytes([cls, mid, len(p) & 0xFF, (len(p) >> 8) & 0xFF]) + p
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return bytes([0xB5, 0x62]) + body + bytes([a, b])


def uart(b):
    return machine.UART(1, baudrate=b, rx=machine.Pin(9), tx=machine.Pin(8), timeout=200)


def read(u, ms):
    while u.any():
        u.read(u.any())
    t = time.ticks_ms(); buf = b""
    while time.ticks_diff(time.ticks_ms(), t) < ms:
        if u.any():
            buf += u.read(u.any())
        time.sleep_ms(20)
    return buf


baud = None
for b in (115200, 9600):
    u = uart(b); time.sleep_ms(200)
    buf = read(u, 1500)
    nmea = buf.count(b"$G"); ubx = buf.count(b"\xb5\x62")
    print("baud %6d: %d bytes, NMEA=%d UBX=%d" % (b, len(buf), nmea, ubx))
    if nmea or ubx:
        baud = b; break

if baud is None:
    print("=> GPS SILENT at both bauds (no UART data at all)")
else:
    u = uart(baud); time.sleep_ms(150)
    txt = read(u, 3000).decode("ascii", "replace")
    gga = [l for l in txt.split("\n") if "GGA" in l]
    if gga:
        f = gga[-1].split(",")
        print("GGA: fixQ=%s sats_used=%s hdop=%s alt=%s" % (
            f[6] if len(f) > 6 else "?", f[7] if len(f) > 7 else "?",
            f[8] if len(f) > 8 else "?", f[9] if len(f) > 9 else "?"))
    else:
        print("no GGA (is GGA output enabled? still UBX?)")

    pl = None
    buf = read_resp = None
    u.write(_ubx(0x01, 0x35))                 # UBX-NAV-SAT
    buf = read(u, 2000)
    i = buf.find(b"\xb5\x62\x01\x35")
    if i >= 0 and len(buf) >= i + 8:
        ln = buf[i + 4] | (buf[i + 5] << 8); pl = buf[i + 6:i + 6 + ln]
    if pl and len(pl) >= 8:
        n = pl[5]; cnos = []; used = 0
        for k in range(8, min(len(pl), 8 + n * 12), 12):
            cnos.append(pl[k + 2])
            fl = pl[k + 8] | (pl[k + 9] << 8) | (pl[k + 10] << 16) | (pl[k + 11] << 24)
            if fl & 0x08:
                used += 1
        cnos.sort(reverse=True)
        print("NAV-SAT: tracked=%d used_in_fix=%d  top SNR(dBHz)=%s" % (n, used, cnos[:10]))
    else:
        print("no NAV-SAT reply")

    p = bytes((0, 0, 0, 0)) + (0x20110021).to_bytes(4, "little")
    u.write(_ubx(0x06, 0x8B, p))              # VALGET dynModel
    buf = read(u, 1500)
    i = buf.find(b"\xb5\x62\x06\x8b")
    if i >= 0:
        ln = buf[i + 4] | (buf[i + 5] << 8); pl = buf[i + 6:i + 6 + ln]
        if len(pl) >= 9:
            m = {0: "portable", 2: "stationary", 7: "air<2g", 8: "air<4g"}
            print("dynModel=%d (%s)" % (pl[8], m.get(pl[8], "?")))
    else:
        print("no dynModel reply")
