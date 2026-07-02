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

from adr import next_tx_power


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
