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

# Find the start of the LAST recorded episode in the raw log, so a manual
# field upload can send just the ride that was flown instead of the whole
# (up to 4 MB) file - field test #2, 2026-07-07: gzip+POST of ~4 MB over a
# phone hotspot blocked the app for minutes and then failed. Pure and
# host-testable: works on any binary file object (seek/read/tell).


def last_marker_offset(f, marker=b"# motion-start", cap_bytes=2 * 1024 * 1024,
                       chunk=4096):
    """Byte offset of the last line STARTING WITH marker, or None.

    Scans backwards in chunks with an overlap so a marker straddling a
    chunk boundary is still found; only the final cap_bytes of the file are
    searched (bounds the flash reads on a capped 4 MB log). The marker must
    sit at a line start (preceded by a newline, or at offset 0)."""
    f.seek(0, 2)
    size = f.tell()
    if size == 0:
        return None
    lo = size - cap_bytes if size > cap_bytes else 0
    # extend each chunk by the marker length so a straddling hit is seen
    overlap = len(marker) + 1              # +1 for the preceding newline
    pos = size
    while pos > lo:
        start = pos - chunk
        if start < lo:
            start = lo
        f.seek(start)
        data = f.read(min(pos - start + overlap, size - start))
        # newline-anchored search; a line-start marker at file offset 0 has
        # no preceding newline, so handle that case explicitly
        i = data.rfind(b"\n" + marker)
        if i >= 0:
            return start + i + 1
        if start == 0 and data.startswith(marker):
            return 0
        pos = start
    return None


def align_to_line(f, offset, max_scan=65536):
    """Smallest offset >= `offset` that begins a line (0 stays 0). Used when
    falling back to a plain byte-window tail: the first row must not be a
    torn half-line."""
    if offset <= 0:
        return 0
    f.seek(offset - 1)
    scanned = 0
    while scanned < max_scan:
        b = f.read(1024)
        if not b:
            return offset + scanned        # EOF: nothing after -> harmless
        i = b.find(b"\n")
        if i >= 0:
            return offset - 1 + scanned + i + 1
        scanned += len(b)
    return offset
