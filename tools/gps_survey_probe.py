import machine, time

RX, TX, BAUD = 42, 41, 9600


def _ck(body):
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return bytes([0xB5, 0x62]) + body + bytes([a, b])


def poll(u, cls, mid, payload=b"", wait_ms=1500):
    while u.any():
        u.read(u.any())
    body = bytes([cls, mid, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF]) + payload
    u.write(_ck(body))
    t = time.ticks_ms()
    buf = b""
    while time.ticks_diff(time.ticks_ms(), t) < wait_ms:
        if u.any():
            buf += u.read(u.any())
        time.sleep_ms(20)
    return buf


def find(buf, cls, mid):
    i = buf.find(bytes([0xB5, 0x62, cls, mid]))
    if i < 0:
        return None
    ln = buf[i + 4] | (buf[i + 5] << 8)
    return buf[i + 6:i + 6 + ln]


u = machine.UART(1, baudrate=BAUD, rx=machine.Pin(RX), tx=machine.Pin(TX), timeout=200)
time.sleep_ms(150)

# --- MON-VER: identify the product ---
b = poll(u, 0x0A, 0x04)
pl = find(b, 0x0A, 0x04)
if pl:
    print("SW:", bytes(pl[0:30]).split(b"\x00")[0])
    print("HW:", bytes(pl[30:40]).split(b"\x00")[0])
    for k in range(40, len(pl), 30):
        s = bytes(pl[k:k + 30]).split(b"\x00")[0]
        if s:
            print("EXT:", s)
else:
    print("no MON-VER reply")

# --- CFG-TMODE2 (0x06 0x3D): hardware Survey-In / Time Mode ---
b = poll(u, 0x06, 0x3D)
tm = find(b, 0x06, 0x3D)
ack = b.find(bytes([0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x3D])) >= 0
nak = b.find(bytes([0xB5, 0x62, 0x05, 0x00, 0x02, 0x00, 0x06, 0x3D])) >= 0
if tm is not None:
    print("CFG-TMODE2: SUPPORTED (Survey-In available), mode byte=%d len=%d" % (tm[0], len(tm)))
elif nak:
    print("CFG-TMODE2: NAK -> NOT supported (no hardware Survey-In)")
elif ack:
    print("CFG-TMODE2: ACK but no payload")
else:
    print("CFG-TMODE2: no reply")

# --- CFG-NAV5 (0x06 0x24): current dynamic model ---
b = poll(u, 0x06, 0x24)
n5 = find(b, 0x06, 0x24)
if n5 and len(n5) >= 3:
    models = {0: "portable", 2: "stationary", 3: "pedestrian", 4: "automotive",
              5: "sea", 6: "airborne<1g", 7: "airborne<2g", 8: "airborne<4g"}
    print("CFG-NAV5: dynModel=%d (%s)  fixMode=%d" % (
        n5[2], models.get(n5[2], "?"), n5[3] if len(n5) > 3 else -1))
else:
    print("CFG-NAV5: no reply")
