import machine, time

RX, TX, BAUD = 42, 41, 9600


def poll():
    u = machine.UART(1, baudrate=BAUD, rx=machine.Pin(RX), tx=machine.Pin(TX),
                     timeout=200)
    time.sleep_ms(150)
    while u.any():
        u.read(u.any())
    body = bytes([0x0A, 0x04, 0x00, 0x00])   # UBX-MON-VER poll
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    u.write(bytes([0xB5, 0x62]) + body + bytes([a, b]))
    t = time.ticks_ms()
    buf = b""
    while time.ticks_diff(time.ticks_ms(), t) < 2000:
        if u.any():
            buf += u.read(u.any())
        time.sleep_ms(20)
    return buf


buf = poll()
i = buf.find(b"\xb5\x62\x0a\x04")
print("rx", len(buf), "bytes, MON-VER at", i)
if i >= 0:
    ln = buf[i + 4] | (buf[i + 5] << 8)
    pl = buf[i + 6:i + 6 + ln]
    print("SW:", bytes(pl[0:30]).split(b"\x00")[0])
    print("HW:", bytes(pl[30:40]).split(b"\x00")[0])
    ext = pl[40:]
    for k in range(0, len(ext), 30):
        s = bytes(ext[k:k + 30]).split(b"\x00")[0]
        if s:
            print("EXT:", s)
else:
    # no UBX reply - show any NMEA talker IDs we did see
    print("no UBX; sample:", buf[:120])
