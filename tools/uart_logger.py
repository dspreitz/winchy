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

"""Continuous UART0 console/panic logger for the rope (host side).

The rope's Guru-Meditation/panic output goes to UART0 (GPIO43), captured via
a 3.3V USB-UART adapter - NOT to USB-CDC (the TinyUSB console cannot emit
panic dumps). Run this logger in the background and the next C-level crash
is on disk with a timestamp instead of lost.

Usage: python tools/uart_logger.py [PORT] [LOGFILE]
Defaults: COM16, _devlogs/uart0_rope.log

- Timestamps every line; flushes promptly (a panic must hit the disk even if
  the host dies right after).
- Survives adapter replug / port vanishing (retries every 5 s, logs markers).
- Rotates at 20 MB to <name>.old (the running app's garbled console mirror
  produces ~160 B/s of noise; panic/boot output arrives clean).
"""
import os
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM16"
LOGFILE = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
    os.path.dirname(__file__), "..", "_devlogs", "uart0_rope.log")
BAUD = 115200
ROTATE_BYTES = 20 * 1024 * 1024


def stamp():
    return time.strftime("%Y-%m-%d %H:%M:%S")


def log_line(f, text):
    f.write("%s | %s\n" % (stamp(), text))
    f.flush()


def rotate_if_needed(path):
    try:
        if os.path.getsize(path) > ROTATE_BYTES:
            old = path + ".old"
            if os.path.exists(old):
                os.remove(old)
            os.replace(path, old)
    except OSError:
        pass


def main():
    buf = b""
    while True:
        rotate_if_needed(LOGFILE)
        f = open(LOGFILE, "a", encoding="utf-8", errors="replace")
        try:
            s = serial.Serial(PORT, BAUD, timeout=1)
        except Exception as e:
            log_line(f, "## logger: %s not available (%s), retry in 5 s"
                     % (PORT, type(e).__name__))
            f.close()
            time.sleep(5)
            continue
        log_line(f, "## logger: connected to %s @ %d" % (PORT, BAUD))
        try:
            last_rotate = time.time()
            while True:
                chunk = s.read(4096)
                if chunk:
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        log_line(f, line.rstrip(b"\r").decode(
                            "utf-8", "replace"))
                if len(buf) > 8192:          # binary flood without newlines
                    log_line(f, "<%d B no-newline data>" % len(buf))
                    buf = b""
                if time.time() - last_rotate > 300:
                    f.close()
                    rotate_if_needed(LOGFILE)
                    f = open(LOGFILE, "a", encoding="utf-8",
                             errors="replace")
                    last_rotate = time.time()
        except Exception as e:
            log_line(f, "## logger: port lost (%s), reconnecting"
                     % type(e).__name__)
            try:
                s.close()
            except Exception:
                pass
            f.close()
            time.sleep(5)


if __name__ == "__main__":
    main()
