import time

import serial

LOG = r"C:\Users\dom14\vs_code\Winchy\tools\motion_test.log"

s = serial.Serial("COM6", 115200, timeout=1)
s.write(b"\x04")  # soft reset, fresh boot
t0 = time.time()
with open(LOG, "wb") as f:
    f.write(b"# capture start (t=0 at soft reset)\n")
    while time.time() - t0 < 290:
        data = s.read(s.in_waiting or 1)
        if data:
            stamp = ("[%6.1f] " % (time.time() - t0)).encode()
            f.write(data.replace(b"\n", b"\n" + stamp))
s.close()
print("capture done ->", LOG)
