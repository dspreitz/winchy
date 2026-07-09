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

# Host tests for the last-episode locator (firmware/shared/logtail.py). This
# decides WHAT a manual field upload sends - a wrong offset silently uploads
# a torn or wrong slice of the ride the operator wants to inspect.

import io
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "firmware",
                                "shared"))

from logtail import align_to_line, last_marker_offset

M = b"# motion-start"


def buf(*parts):
    return io.BytesIO(b"".join(parts))


def test_single_marker_found():
    f = buf(b"# boot\nrow1\n", M + b" t=1\n", b"row2\nrow3\n")
    assert last_marker_offset(f, M) == len(b"# boot\nrow1\n")


def test_last_of_many_markers_wins():
    a = b"# boot\n" + M + b" t=1\n" + b"r\n" * 100
    b = M + b" t=2\n" + b"r\n" * 5
    f = buf(a, b)
    assert last_marker_offset(f, M) == len(a)


def test_marker_at_file_start():
    f = buf(M + b" t=0\nrow\n")
    assert last_marker_offset(f, M) == 0


def test_no_marker_returns_none():
    f = buf(b"# boot\n" + b"row\n" * 1000)
    assert last_marker_offset(f, M) is None


def test_empty_file_returns_none():
    assert last_marker_offset(buf(), M) is None


def test_marker_straddling_chunk_boundary():
    # Place the marker so it crosses a 4096-byte chunk edge: pad so the
    # marker starts a few bytes before a multiple of 4096.
    pad = b"x" * 20 + b"\n"
    rows = pad * 300                       # > 4096 bytes
    lead = rows[:4096 - 7]                 # marker starts 7 B before the edge
    lead = lead[:lead.rfind(b"\n") + 1]    # keep it line-aligned
    f = buf(lead, M + b" t=9\n", b"tail\n" * 50)
    assert last_marker_offset(f, M, chunk=4096) == len(lead)


def test_mid_line_occurrence_is_not_a_marker():
    # The marker text inside a data row (not at line start) must not match.
    f = buf(b"# boot\n", b"data,data,", M, b",data\n", b"row\n")
    assert last_marker_offset(f, M) is None


def test_cap_excludes_old_markers():
    early = M + b" t=old\n"
    filler = b"r" * 300 + b"\n"
    f = buf(early, filler * 50)            # marker only in the early part
    total = len(early) + 50 * len(filler)
    assert last_marker_offset(f, M, cap_bytes=total - len(early) - 10) is None


def test_align_to_line_zero_stays_zero():
    assert align_to_line(buf(b"abc\ndef\n"), 0) == 0


def test_align_to_line_snaps_forward_to_next_row():
    data = b"row-one\nrow-two\nrow-three\n"
    f = buf(data)
    # offset lands mid "row-two" -> aligned to start of "row-three"
    mid = data.find(b"row-two") + 3
    assert align_to_line(f, mid) == data.find(b"row-three")


def test_align_to_line_already_aligned():
    data = b"aaa\nbbb\n"
    f = buf(data)
    assert align_to_line(f, 4) == 4        # 'bbb' starts at 4
