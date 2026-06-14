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

"""Soft-reset the winch unit and capture boot + first RX frames.

Sends Ctrl-C (interrupt running app) then Ctrl-D (soft reset) over the
native USB CDC, then prints everything the board emits for a fixed window.
Leaves the app running on exit. COM port passed as argv[1].
"""
import sys
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else "COM8"
window_s = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0

ser = serial.Serial(port, 115200, timeout=0.2)
time.sleep(0.3)
ser.write(b"\x03")          # Ctrl-C: drop running app to REPL
time.sleep(0.4)
ser.reset_input_buffer()
ser.write(b"\x04")          # Ctrl-D: soft reset -> re-run main.py
print("=== soft reset sent on %s, capturing %.0fs ===" % (port, window_s))

end = time.time() + window_s
while time.time() < end:
    data = ser.read(4096)
    if data:
        sys.stdout.write(data.decode("utf-8", "replace"))
        sys.stdout.flush()
ser.close()
print("\n=== capture end ===")
