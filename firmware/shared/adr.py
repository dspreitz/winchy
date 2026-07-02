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

# Closed-loop TX-power control (ADR) decision, extracted from the rope app so
# it is pure and host-testable: this drives the radio link's transmit power in
# flight, so a regression here means a dead link at range. Hardened after an
# earlier version starved the link:
#   * control on RSSI (linear with power), NOT SNR - SNR saturates (~12 dB) and
#     let it keep cutting power until RSSI was at the noise floor;
#   * recover FAST: jump straight to max on a stale/lossy link, so it out-runs
#     survivorship-biased feedback (only frames that got through report "good");
#   * optimise SLOW: trim 1 dB at a time, and only when RSSI is well clear of
#     the floor, so it never walks the link down to the edge.
# The caller owns the timing (report freshness) and the constants.


def next_tx_power(power_dbm, report_fresh, loss_pct, rssi_dbm,
                  power_min=-9, power_max=22,
                  rssi_low=-90, rssi_high=-70,
                  step_up=4, step_down=1, loss_high_pct=20):
    """Next TX power (dBm), clamped. Fail loud (jump to max) on stale/lossy
    feedback so the link recovers at once; otherwise keep the receiver-reported
    RSSI in a comfortable window - raise fast, trim slow."""
    if (not report_fresh) or loss_pct >= loss_high_pct:
        return power_max
    if rssi_dbm < rssi_low:
        return min(power_max, power_dbm + step_up)
    if rssi_dbm > rssi_high:
        return max(power_min, power_dbm - step_down)
    return power_dbm  # within the window: hold
