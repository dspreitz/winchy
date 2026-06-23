import machine, time

RX, TX, BAUD = 9, 8, 115200

ITEMS = [(0x20110021, b"\x07"),   # CFG-NAVSPG-DYNMODEL = airborne <2g
         (0x1031001F, b"\x01"),   # GPS
         (0x10310020, b"\x01"),   # SBAS
         (0x10310021, b"\x01"),   # Galileo
         (0x10310022, b"\x01")]   # BeiDou
KEYS = [("DYNMODEL", 0x20110021), ("GPS", 0x1031001F), ("SBAS", 0x10310020),
        ("GAL", 0x10310021), ("BDS", 0x10310022), ("GLO", 0x10310025)]
MODELS = {0: "portable", 7: "air<2g", 8: "air<4g"}


def _ubx(cls, mid, payload=b""):
    body = bytes([cls, mid, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF]) + payload
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return bytes([0xB5, 0x62]) + body + bytes([a, b])


u = machine.UART(1, baudrate=BAUD, rx=machine.Pin(RX), tx=machine.Pin(TX), timeout=200)
time.sleep_ms(150)

# VALSET to RAM | BBR (layers 0x03)
p = bytes((0x00, 0x03, 0x00, 0x00))
for k, v in ITEMS:
    p += k.to_bytes(4, "little") + v
while u.any():
    u.read(u.any())
u.write(_ubx(0x06, 0x8A, p))
time.sleep_ms(120)
ack = b""
t = time.ticks_ms()
while time.ticks_diff(time.ticks_ms(), t) < 800:
    if u.any():
        ack += u.read(u.any())
    time.sleep_ms(20)
print("VALSET ACK:", bytes([0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x8A]) in ack,
      "NAK:", bytes([0xB5, 0x62, 0x05, 0x00, 0x02, 0x00, 0x06, 0x8A]) in ack)

# VALGET readback
p = bytes((0x00, 0x00, 0x00, 0x00))
for _, k in KEYS:
    p += k.to_bytes(4, "little")
while u.any():
    u.read(u.any())
u.write(_ubx(0x06, 0x8B, p))
t = time.ticks_ms(); buf = b""
while time.ticks_diff(time.ticks_ms(), t) < 1500:
    if u.any():
        buf += u.read(u.any())
    time.sleep_ms(20)
i = buf.find(bytes([0xB5, 0x62, 0x06, 0x8B]))
if i < 0:
    print("no VALGET reply")
else:
    ln = buf[i + 4] | (buf[i + 5] << 8)
    pl = buf[i + 6:i + 6 + ln]
    got = {}
    o = 4
    while o + 5 <= len(pl):
        k = pl[o] | (pl[o + 1] << 8) | (pl[o + 2] << 16) | (pl[o + 3] << 24)
        got[k] = pl[o + 4]; o += 5
    for name, k in KEYS:
        v = got.get(k)
        print("%-9s = %s%s" % (name, v, "  (" + MODELS.get(v, "?") + ")" if name == "DYNMODEL" else ""))
