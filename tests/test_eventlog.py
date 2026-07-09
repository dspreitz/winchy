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

# Host tests for the persistent event log (firmware/shared/eventlog.py).
# The contract that matters in the field: events survive (flushed per line),
# the file stays BOUNDED (it must never eat the flash), rotation keeps the
# NEWEST events, and a logging failure never propagates into an app task.

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "firmware",
                                "shared"))

from eventlog import EventLog


def test_lines_are_stamped_and_appended(tmp_path):
    p = str(tmp_path / "events.log")
    ev = EventLog(p, stamp=lambda: "S1")
    ev.log("wifi joined X")
    ev.log("upload ok")
    lines = open(p).read().splitlines()
    assert lines == ["S1 | wifi joined X", "S1 | upload ok"]


def test_size_stays_bounded_and_newest_survive(tmp_path):
    p = str(tmp_path / "events.log")
    ev = EventLog(p, max_bytes=2000, keep_bytes=800, stamp=lambda: "T")
    for i in range(200):
        ev.log("event number %04d" % i)
    size = os.path.getsize(p)
    assert size <= 2000 + 100              # bounded (one line of slack)
    txt = open(p).read()
    assert "event number 0199" in txt      # newest kept
    assert "event number 0000" not in txt  # oldest dropped
    assert txt.startswith("# rotated\n")


def test_rotation_is_line_aligned(tmp_path):
    p = str(tmp_path / "events.log")
    ev = EventLog(p, max_bytes=500, keep_bytes=200, stamp=lambda: "T")
    for i in range(50):
        ev.log("evt %04d" % i)
    lines = open(p).read().splitlines()
    # every kept line after the marker is complete: "T | evt NNNN"
    for ln in lines[1:]:
        assert ln.startswith("T | evt "), ln


def test_log_failure_never_raises(tmp_path):
    # Path is a DIRECTORY -> open() fails; log() must swallow it.
    ev = EventLog(str(tmp_path), stamp=lambda: "T")
    ev.log("does not explode")


def test_default_stamp_smoke(tmp_path):
    # Host clock is synced (year >= 2024) -> ISO stamp with the separator.
    p = str(tmp_path / "events.log")
    EventLog(p).log("x")
    line = open(p).read()
    assert " | x" in line and line[:2] == "20"
