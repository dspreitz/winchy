import machine, time

RX, TX, BAUD = 9, 8, 115200   # rope M10: ESP32 rx=GPIO9, tx=GPIO8, raised baud

KEYS = [("DYNMODEL", 0x20110021),
        ("GPS",  0x1031001F), ("SBAS", 0x10310020), ("GAL", 0x10310021),
        ("BDS",  0x10310022), ("QZSS", 0x10310024), ("GLO", 0x10310025)]
MODELS = {0: "portable", 2: "stationary", 3: "pedestrian", 4: "automotive",
          5: "sea", 6: "air<1g", 7: "air<2g", 8: "air<4g"}


def _ubx(cls, mid, payload=b""):
    body = bytes([cls, mid, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF]) + payload
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return bytes([0xB5, 0x62]) + body + bytes([a, b])


u = machine.UART(1, baudrate=BAUD, rx=machine.Pin(RX), tx=machine.Pin(TX), timeout=200)
time.sleep_ms(150)
while u.any():
    u.read(u.any())

payload = bytes([0x00, 0x00, 0x00, 0x00])     # version=0 (request), layer=0 (RAM)
for _, k in KEYS:
    payload += k.to_bytes(4, "little")
u.write(_ubx(0x06, 0x8B, payload))

t = time.ticks_ms(); buf = b""
while time.ticks_diff(time.ticks_ms(), t) < 1500:
    if u.any():
        buf += u.read(u.any())
    time.sleep_ms(20)

i = buf.find(bytes([0xB5, 0x62, 0x06, 0x8B]))
if i < 0:
    print("no VALGET reply; sample:", buf[:80])
else:
    ln = buf[i + 4] | (buf[i + 5] << 8)
    pl = buf[i + 6:i + 6 + ln]
    got = {}
    o = 4                                      # skip version/layer/position
    while o + 5 <= len(pl):
        k = pl[o] | (pl[o + 1] << 8) | (pl[o + 2] << 16) | (pl[o + 3] << 24)
        got[k] = pl[o + 4]
        o += 5
    for name, k in KEYS:
        v = got.get(k)
        if name == "DYNMODEL":
            print("DYNMODEL =", v, "(%s)" % MODELS.get(v, "?"))
        else:
            print("%-5s ENA = %s" % (name, v))
