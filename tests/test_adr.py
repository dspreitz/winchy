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

# Host tests for the ADR (TX-power control) decision (firmware/shared/adr.py).
# This drives the radio link's transmit power in flight, so a regression means
# a dead link at range. Encodes the hardening the comments describe: fail loud
# (jump to max) on stale/lossy feedback, raise fast, trim slow, never walk the
# link down to the sensitivity floor.

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "firmware",
                                "shared"))

from adr import fspl_db, next_tx_power, power_cap_dbm


def test_stale_report_jumps_to_max():
    # No fresh feedback -> the survivorship-biased loop must fail LOUD: full
    # power at once, not a gentle step (the link may already be gone).
    assert next_tx_power(0, False, 0, -75) == 22


def test_high_loss_jumps_to_max():
    assert next_tx_power(0, True, 20, -75) == 22     # at the threshold
    assert next_tx_power(-9, True, 80, -60) == 22    # way over, from the floor


def test_low_rssi_raises_fast_by_step_up():
    assert next_tx_power(10, True, 0, -95) == 14     # +4 dB per cycle


def test_raise_clamps_at_max():
    assert next_tx_power(21, True, 0, -95) == 22     # never past the PA max


def test_high_rssi_trims_slowly_by_one_db():
    assert next_tx_power(10, True, 0, -60) == 9      # -1 dB per cycle


def test_trim_clamps_at_min():
    assert next_tx_power(-9, True, 0, -40) == -9     # never below the floor


def test_rssi_in_window_holds_power():
    assert next_tx_power(10, True, 0, -80) == 10     # comfortable -> hold
    assert next_tx_power(10, True, 0, -90) == 10     # exactly at the low edge
    assert next_tx_power(10, True, 0, -70) == 10     # exactly at the high edge


def test_loss_beats_good_rssi():
    # Good RSSI but heavy loss: the frames that DID get through report "fine" -
    # that is exactly the survivorship bias, so loss must win and jump to max.
    assert next_tx_power(5, True, 50, -60) == 22


# --- distance-aware hardening (bench saturation latch-up, 2026-07-06) ---

def test_fspl_sane():
    # ~31.5 dB at 1 m, +20 dB per decade @ 869.525 MHz
    assert 31 < fspl_db(1) < 32
    assert 51 < fspl_db(10) < 52
    assert 91 < fspl_db(1000) < 92


def test_power_cap_curve():
    assert power_cap_dbm(1) == -9         # bench: clamped to the floor
    assert power_cap_dbm(50) in (0, 1)    # staging area: single digits
    assert 15 <= power_cap_dbm(300) <= 17
    assert power_cap_dbm(600) == 22       # field: cap == max, no change
    assert power_cap_dbm(2000) == 22


def test_near_stale_fails_quiet_to_cap():
    # THE latch-up fix: lossy/stale at bench distance means RX saturation,
    # so go to the distance cap (the floor), NOT to max.
    assert next_tx_power(22, False, 0, 0, distance_m=2) == -9
    assert next_tx_power(22, True, 80, 0, distance_m=2) == -9


def test_far_stale_still_fails_loud():
    # At range the old behaviour is sacred: stale -> max, immediately.
    assert next_tx_power(5, False, 0, -80, distance_m=800) == 22


def test_unknown_distance_is_legacy():
    assert next_tx_power(5, False, 0, -80) == 22
    assert next_tx_power(10, True, 0, -60) == 9


def test_cap_bounds_good_link_too():
    # Even with feedback, never exceed what the distance justifies.
    assert next_tx_power(10, True, 0, -95, distance_m=2) == -9   # raise capped
    assert next_tx_power(10, True, 0, -80, distance_m=2) == -9   # hold capped


def test_latch_probe_ladder_when_stale_without_distance():
    # Stale forever at unknown distance: after probe_start cycles the power
    # walks max -> mid -> min (cycling) instead of latching at max.
    assert next_tx_power(22, False, 0, 0, stale_count=0) == 22
    assert next_tx_power(22, False, 0, 0, stale_count=23) == 22
    assert next_tx_power(22, False, 0, 0, stale_count=24) == 22   # step 0: max
    assert next_tx_power(22, False, 0, 0, stale_count=40) == 6    # step 1: mid
    assert next_tx_power(22, False, 0, 0, stale_count=56) == -9   # step 2: min
    assert next_tx_power(22, False, 0, 0, stale_count=72) == 22   # cycles
