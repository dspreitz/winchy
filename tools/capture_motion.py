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
