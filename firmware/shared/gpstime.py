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

# Defensive GPS-time acceptance, shared by the rope and the winch.
#
# A u-blox module can briefly report a "fully resolved" UTC time that is minutes
# wrong - seen on the rope: a NAV-PVT time ~37 min behind real UTC during the
# aided / partial-lock phase, which was then latched for the whole session and
# overrode the (correct) NTP fallback. This decides whether to (re)set the RTC
# from a GPS time sample so that:
#   1. a GPS time that disagrees with an already-set NTP clock by minutes is
#      rejected (NTP cross-check),
#   2. a low-confidence (large tAcc) or single-outlier time is not trusted until
#      a second, consistent frame confirms it,
#   3. a bad value that was nonetheless adopted self-heals: once the module
#      settles, a consistent new time re-syncs the RTC instead of being stuck.
#
# Pure function (no time/RTC access) so it is unit-testable on the host; the
# caller converts the GPS time to epoch seconds and applies the action.


_HTTP_MONTHS = ("Jan", "Feb", "Mar", "Apr", "May", "Jun",
                "Jul", "Aug", "Sep", "Oct", "Nov", "Dec")


def parse_http_date(s):
    """RFC 7231 date ("Mon, 22 Jun 2026 20:43:21 GMT") -> (y, mo, d, h, mi, s)
    UTC, or None. Shared by both units (was duplicated in each app). The year
    is RANGE-CHECKED (2024..2050): downstream this feeds time.mktime, which on
    MicroPython overflows a 32-bit machine word past ~2068 - the same glitched-
    date crash class already guarded in _pps_arm_next."""
    try:
        p = s.split()
        d, y = int(p[1]), int(p[3])
        mo = _HTTP_MONTHS.index(p[2]) + 1
        hh, mm, ss = (int(x) for x in p[4].split(":"))
        if not (2024 <= y <= 2050 and 1 <= mo <= 12 and 1 <= d <= 31):
            return None
        return (y, mo, d, hh, mm, ss)
    except (ValueError, IndexError, AttributeError):
        return None


def time_fix_decision(source, gps_epoch, rtc_epoch, tacc_ns, cand_epoch,
                      max_tacc_ns=1000000000, ntp_skew_s=5,
                      drift_s=3, consist_s=2):
    """Decide what to do with one GPS UTC time sample.

    source     : current RTC time source - "gps" | "ntp" | None
    gps_epoch  : the GPS UTC time as epoch seconds (int)
    rtc_epoch  : the current RTC as epoch seconds (only meaningful if source set)
    tacc_ns    : NAV-PVT time-accuracy estimate, ns (0 = unknown, e.g. NMEA)
    cand_epoch : pending consistency-candidate epoch, or None

    Returns (action, new_cand):
      action  : "set"    - adopt this GPS time as the RTC
                "arm"    - already on GPS and it agrees; keep PPS disciplined only
                "reject" - distrust this sample; keep the current clock
                "wait"   - hold; need one more consistent frame to trust GPS
      new_cand: candidate epoch to remember for the next call (None to clear)
    """
    if tacc_ns and tacc_ns > max_tacc_ns:           # low-confidence time -> ignore
        return ("reject", cand_epoch)
    if source == "gps":
        if abs(gps_epoch - rtc_epoch) <= drift_s:
            return ("arm", None)                    # agrees -> just discipline PPS
        if cand_epoch is not None and abs(gps_epoch - cand_epoch) <= consist_s:
            return ("set", None)                    # drift confirmed -> self-heal
        return ("wait", gps_epoch)                  # one outlier; wait for a 2nd
    if source == "ntp":
        if abs(gps_epoch - rtc_epoch) <= ntp_skew_s:
            return ("set", None)                    # agrees with NTP -> adopt GPS
        return ("reject", cand_epoch)               # minutes off NTP -> bogus
    # No clock yet: require two consecutive consistent frames before trusting GPS.
    if cand_epoch is not None and abs(gps_epoch - cand_epoch) <= consist_s:
        return ("set", None)
    return ("wait", gps_epoch)
