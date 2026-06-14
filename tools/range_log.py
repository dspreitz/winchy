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

"""Read-only serial logger for the rope unit during a range test.

Attaches to the rope's USB CDC WITHOUT resetting it (no Ctrl-C/Ctrl-D, no
DTR toggle that matters on native USB), timestamps every line to a log file,
and echoes the interesting lines (link reports, ADR power changes, TX/RX
trouble) to the console so you can watch progress while walking.

Usage:  python tools/range_log.py [COM6] [logfile]
Stop with Ctrl-C; the full timestamped log is kept in the file.
"""
import sys
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else "COM6"
logfile = (sys.argv[2] if len(sys.argv) > 2
           else "range_log_%s.txt" % time.strftime("%Y%m%d_%H%M%S"))

# Lines worth surfacing live (everything still goes to the file).
ECHO = ("Link report:", "ADR:", "TX deferred", "radio cb error",
        "QNH", "Baro alt")

ser = serial.Serial(port, 115200, timeout=0.5)
print("Logging %s -> %s   (Ctrl-C to stop)" % (port, logfile))
buf = b""
with open(logfile, "a") as f:
    f.write("\n=== range log start %s on %s ===\n"
            % (time.strftime("%Y-%m-%d %H:%M:%S"), port))
    f.flush()
    try:
        while True:
            data = ser.read(512)
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("utf-8", "replace").rstrip("\r")
                stamp = time.strftime("%H:%M:%S")
                f.write("%s  %s\n" % (stamp, text))
                f.flush()
                if any(k in text for k in ECHO):
                    print("%s  %s" % (stamp, text))
    except KeyboardInterrupt:
        print("\nstopped; log saved to", logfile)
    finally:
        ser.close()
