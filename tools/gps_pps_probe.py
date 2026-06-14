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

# Bench probe: is the GPS 1PPS reaching the T3S3 winch on GPIO 40?
# Run with:  mpremote connect COM6 run tools/gps_pps_probe.py
# Counts rising edges over 4 s (expect ~4 if a 1 Hz PPS is present and the GPS
# has a fix - most u-blox modules gate PPS on lock). Then reset the winch to
# resume normal operation.

from machine import Pin
import time

PPS = 40
count = 0


def _cb(p):
    global count
    count += 1


pin = Pin(PPS, Pin.IN)
pin.irq(trigger=Pin.IRQ_RISING, handler=_cb)
time.sleep(4)
pin.irq(handler=None)

print("PPS GPIO %d: %d rising edges in 4 s (expect ~4 at 1 Hz)" % (PPS, count))
print("PPS level now: %d" % pin.value())
