# Winchy - glider winch rope force & advice system
# Copyright (C) 2026 Dominic Spreitz
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version. Distributed WITHOUT ANY WARRANTY.
# See the GNU General Public License for more details, and the LICENSE
# file or <https://www.gnu.org/licenses/> for the full text.
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""Hard-reset the winch and confirm main.py auto-runs and starts flash logging.

Triggers machine.reset() (a clean power-on-equivalent boot, unlike a Ctrl-C
soft reset which can interrupt the auto-run), waits out the USB CDC
re-enumeration, then reads the boot output read-only.
"""
import time

import serial
from serial.tools import list_ports


def winch_port():
    for x in list_ports.comports():
        if x.vid == 0x303A and x.serial_number != "123456":  # 123456 = rope
            return x.device
    return None


p = winch_port()
s = serial.Serial(p, 115200, timeout=0.3)
for _ in range(3):
    s.write(b"\x03")
    time.sleep(0.2)
s.write(b"import machine; machine.reset()\r\n")
time.sleep(0.5)
s.close()
print("reset sent to", p)

time.sleep(4)  # USB re-enumeration
p = None
for _ in range(20):
    p = winch_port()
    if p:
        break
    time.sleep(0.5)
print("re-enumerated as", p)

s = serial.Serial(p, 115200, timeout=0.3)
end = time.time() + 12
out = b""
while time.time() < end:
    out += s.read(512)
s.close()
text = out.decode("utf-8", "replace")
print("--- boot tail ---")
print("\n".join(text.splitlines()[-12:]))
print("--- ready:", "Winch receiver ready" in text,
      "| RX frames:", text.count("[RX]"))
