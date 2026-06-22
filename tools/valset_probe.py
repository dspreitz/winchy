import machine, time

TX, RX = 8, 9


def _ck(body):
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return bytes([0xB5, 0x62]) + body + bytes([a, b])


def _valset(layers, items):
    # items: list of (key_u32, value_bytes)
    p = bytes([0x00, layers, 0x00, 0x00])
    for key, val in items:
        p += key.to_bytes(4, "little") + val
    return _ck(bytes([0x06, 0x8A]) + len(p).to_bytes(2, "little") + p)


# rate + NMEA message-output keys we want to persist
ITEMS = [
    (0x30210001, (100).to_bytes(2, "little")),   # CFG-RATE-MEAS = 100 ms
    (0x30210002, (1).to_bytes(2, "little")),     # CFG-RATE-NAV = 1
    (0x209100BB, b"\x01"),                        # NMEA GGA UART1 on
    (0x209100AC, b"\x01"),                        # NMEA RMC UART1 on
    (0x209100CA, b"\x00"),                        # NMEA GLL UART1 off
    (0x209100C0, b"\x00"),                        # NMEA GSA UART1 off
    (0x209100C5, b"\x00"),                        # NMEA GSV UART1 off
    (0x209100B1, b"\x00"),                        # NMEA VTG UART1 off
]

u = machine.UART(1, baudrate=115200, tx=machine.Pin(TX), rx=machine.Pin(RX),
                 timeout=200)
time.sleep_ms(120)
while u.any():
    u.read(u.any())
u.write(_valset(0x01, ITEMS))           # RAM layer only (probe, no flash write)
t = time.ticks_ms()
buf = b""
while time.ticks_diff(time.ticks_ms(), t) < 1500:
    if u.any():
        buf += u.read(u.any())
    time.sleep_ms(20)
ack = buf.find(b"\xb5\x62\x05\x01\x02\x00\x06\x8a")   # ACK-ACK for CFG (06 8A)
nak = buf.find(b"\xb5\x62\x05\x00\x02\x00\x06\x8a")   # ACK-NAK
print("rx", len(buf), "bytes  ACK@", ack, " NAK@", nak)
print("RESULT:", "ACK-ACK (keys valid)" if ack >= 0 else
      ("ACK-NAK (a key is wrong)" if nak >= 0 else "no ack seen"))
