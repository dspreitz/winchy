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

# Bench probe: does the T3S3 winch read NMEA from the GPS on UART1 (rx=42)?
# Run with:  mpremote connect COM6 run tools/gps_uart_probe.py
# Prints the byte count and the first NMEA seen in a 4 s window.

from machine import UART
import time

RX, TX, BAUD = 42, 41, 9600

u = UART(1, baudrate=BAUD, rx=RX, tx=TX, timeout=200)
time.sleep_ms(200)

buf = b""
t = time.ticks_ms()
while time.ticks_diff(time.ticks_ms(), t) < 4000:
    if u.any():
        buf += u.read()
    time.sleep_ms(20)

print("GPS UART rx=%d @ %d -> %d bytes" % (RX, BAUD, len(buf)))
if buf:
    text = buf.decode("ascii", "replace")
    for line in text.splitlines():
        if line.startswith("$"):
            print(line)
