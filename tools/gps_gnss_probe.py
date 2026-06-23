import machine, time

RX, TX, BAUD = 42, 41, 9600
NAMES = {0: "GPS", 1: "SBAS", 2: "Galileo", 3: "BeiDou", 4: "IMES", 5: "QZSS", 6: "GLONASS"}


def _msg(cls, mid, payload=b""):
    body = bytes([cls, mid, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF]) + payload
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return bytes([0xB5, 0x62]) + body + bytes([a, b])


def poll(u, cls, mid, wait_ms=1500):
    while u.any():
        u.read(u.any())
    u.write(_msg(cls, mid))
    t = time.ticks_ms(); buf = b""
    while time.ticks_diff(time.ticks_ms(), t) < wait_ms:
        if u.any():
            buf += u.read(u.any())
        time.sleep_ms(20)
    i = buf.find(bytes([0xB5, 0x62, cls, mid]))
    if i < 0:
        return None
    ln = buf[i + 4] | (buf[i + 5] << 8)
    return buf[i + 6:i + 6 + ln]


u = machine.UART(1, baudrate=BAUD, rx=machine.Pin(RX), tx=machine.Pin(TX), timeout=200)
time.sleep_ms(150)

g = poll(u, 0x06, 0x3E)
if g:
    print("CFG-GNSS: ver=%d numTrkChHw=%d numTrkChUse=%d blocks=%d" % (g[0], g[1], g[2], g[3]))
    for k in range(4, len(g), 8):
        b = g[k:k + 8]
        if len(b) < 8:
            break
        flags = b[4] | (b[5] << 8) | (b[6] << 16) | (b[7] << 24)
        print("  %-8s res=%d max=%d enable=%d sigCfg=0x%02X" % (
            NAMES.get(b[0], str(b[0])), b[1], b[2], flags & 1, (flags >> 16) & 0xFF))
else:
    print("no CFG-GNSS reply")

s = poll(u, 0x06, 0x16)
if s:
    print("CFG-SBAS: mode=0x%02X usage=0x%02X maxSBAS=%d" % (s[0], s[1], s[2]))
else:
    print("no CFG-SBAS reply")
