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

# Tiny persistent event log (WiFi joins/drops/roams, upload results, boot
# reasons). Field test #2 (2026-07-07) was diagnosed nearly blind: every
# WiFi/upload event existed only on the console, which nobody captures in
# the field. Events are RARE (a handful per session), so each line is
# flushed immediately - a line must survive a power loss seconds later.
# The file is bounded: past max_bytes the newest keep_bytes are rewritten
# (line-aligned), so it never eats the flash and never rotates a whole
# session away like the 4 MB raw log can.

import time


def _default_stamp():
    t = time.localtime()
    if t[0] >= 2024:                      # RTC synced (GPS/NTP)
        return ("%04d-%02d-%02dT%02d:%02d:%02dZ" % t[:6])
    return "t+%d" % int(time.time())      # pre-sync: relative seconds


class EventLog:
    def __init__(self, path="events.log", max_bytes=16384, keep_bytes=8192,
                 stamp=None):
        self.path = path
        self.max_bytes = max_bytes
        self.keep_bytes = keep_bytes
        self._stamp = stamp or _default_stamp

    def log(self, msg):
        """Append one event; never raises (a logging failure must not take
        an app task with it)."""
        try:
            line = "%s | %s\n" % (self._stamp(), msg)
            with open(self.path, "a") as f:
                f.write(line)
                sz = f.tell()
            if sz > self.max_bytes:
                self._rotate()
        except Exception:
            pass

    def _rotate(self):
        """Keep the newest keep_bytes, line-aligned, dropping the rest."""
        with open(self.path, "rb") as f:
            f.seek(0, 2)
            size = f.tell()
            f.seek(size - self.keep_bytes if size > self.keep_bytes else 0)
            tail = f.read()
        i = tail.find(b"\n")               # drop the torn first line
        tail = tail[i + 1:] if i >= 0 else tail
        with open(self.path, "wb") as f:
            f.write(b"# rotated\n")
            f.write(tail)
