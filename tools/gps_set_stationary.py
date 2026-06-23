import machine, time, struct

RX, TX, BAUD = 42, 41, 9600


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


# CFG-NAV5: Stationary dynamic model (2) + static hold (freeze when ~still).
nav5 = struct.pack("<HBBiIbBHHHHBBBBHHB5s",
                   0x0041,   # mask: dynModel (0x01) + staticHold (0x40)
                   2,        # dynModel = stationary
                   0,        # fixMode (ignored)
                   0, 0,     # fixedAlt, fixedAltVar
                   0, 0,     # minElev, drLimit
                   0, 0, 0, 0,  # pDop, tDop, pAcc, tAcc
                   50,       # staticHoldThresh = 50 cm/s (0.5 m/s)
                   0, 0, 0,  # dgpsTimeOut, cnoThreshNumSVs, cnoThresh
                   0,        # reserved1
                   20,       # staticHoldMaxDist = 20 m
                   0, b"\x00" * 5)
# CFG-CFG: save to BBR + Flash + EEPROM + SPI flash.
cfg_save = struct.pack("<IIIB", 0x00000000, 0x0000FFFF, 0x00000000, 0x17)

u = machine.UART(1, baudrate=BAUD, rx=machine.Pin(RX), tx=machine.Pin(TX), timeout=200)
time.sleep_ms(150)

before = poll(u, 0x06, 0x24)
print("dynModel before:", before[2] if before else "?")

u.write(_msg(0x06, 0x24, nav5))
time.sleep_ms(80)
u.write(_msg(0x06, 0x09, cfg_save))
time.sleep_ms(120)

after = poll(u, 0x06, 0x24)
if after:
    print("dynModel after :", after[2], "(2=stationary)  staticHoldThresh=%d cm/s" % after[20])
    print("OK" if after[2] == 2 else "FAILED to set stationary")
else:
    print("no NAV5 readback")
