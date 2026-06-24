# Live GPS signal watcher: poll UBX-NAV-SAT every ~3 s and print tracked sats,
# sats used in fix, and the top carrier-to-noise values. Watch the SNR jump
# when the antenna seats. Runs ~135 s.
import machine, time


def _ubx(cls, mid, p=b""):
    body = bytes([cls, mid, len(p) & 0xFF, (len(p) >> 8) & 0xFF]) + p
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return bytes([0xB5, 0x62]) + body + bytes([a, b])


u = machine.UART(1, baudrate=115200, rx=machine.Pin(9), tx=machine.Pin(8), timeout=200)
time.sleep_ms(150)


def navsat():
    while u.any():
        u.read(u.any())
    u.write(_ubx(0x01, 0x35))
    t = time.ticks_ms(); buf = b""
    while time.ticks_diff(time.ticks_ms(), t) < 1200:
        if u.any():
            buf += u.read(u.any())
        time.sleep_ms(20)
    i = buf.find(b"\xb5\x62\x01\x35")
    if i < 0 or len(buf) < i + 8:
        return None
    ln = buf[i + 4] | (buf[i + 5] << 8); pl = buf[i + 6:i + 6 + ln]
    if len(pl) < 8:
        return None
    n = pl[5]; cnos = []; used = 0
    for k in range(8, min(len(pl), 8 + n * 12), 12):
        cnos.append(pl[k + 2])
        fl = pl[k + 8] | (pl[k + 9] << 8) | (pl[k + 10] << 16) | (pl[k + 11] << 24)
        if fl & 0x08:
            used += 1
    cnos.sort(reverse=True)
    return n, used, cnos


print("watching NAV-SAT - reseat the antenna now (fix needs ~4 sats >=35 dBHz)")
for i in range(45):
    r = navsat()
    if r:
        n, used, cnos = r
        strong = len([c for c in cnos if c >= 35])
        flag = "  <== FIX-READY" if used >= 4 or strong >= 4 else ""
        print("t=%3ds  tracked=%2d  used=%d  strong(>=35)=%d  topSNR=%s%s" % (
            i * 3, n, used, strong, cnos[:6], flag))
    else:
        print("t=%3ds  (no NAV-SAT reply)" % (i * 3))
    time.sleep(3)
print("done")
