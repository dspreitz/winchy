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
# Bench night 2026-07-06 exposed the failure mode of blind fail-loud: at short
# range LOSS can mean "too LOUD" (RX front-end saturation), and jumping to max
# then kills the link permanently - no reports ever come back, so the stale
# branch latches max power forever (mutual saturation death spiral, RSSI ~0,
# 1-3 frames decoding per reset). Hardening, in priority order:
#   * DISTANCE PRIOR (winch_dist from GPS + last-known winch position - the
#     winch is parked, so the value stays valid with the radio dead): a
#     free-space-path-loss derived CAP on every output, generous +margin so
#     the field (>= ~600 m -> cap == max) is untouched; and when the link is
#     stale/lossy at NEAR distance, go to the cap instead of max (saturation
#     is more plausible than range there).
#   * LATCH PROBE (no distance available): when stale at max for a while,
#     step the power down a probe ladder (max -> mid -> min, cycling) until
#     feedback returns, instead of holding max forever.
# The caller owns the timing (report freshness, stale-cycle count) and the
# constants.

import math


def fspl_db(distance_m, freq_mhz=869.525):
    """Free-space path loss in dB (floor the distance at 1 m)."""
    if distance_m < 1.0:
        distance_m = 1.0
    return 32.45 + 20 * math.log10(freq_mhz) + 20 * math.log10(
        distance_m / 1000.0)


def power_cap_dbm(distance_m, power_min=-9, power_max=22,
                  target_rssi=-80, margin_db=15, freq_mhz=869.525):
    """Distance-derived TX power cap: enough power to hit target_rssi at
    distance_m plus a generous fading/obstruction margin. Reaches power_max
    at roughly 600 m, so field behaviour is unchanged; only the near range
    (bench, staging area) gets tamed below saturation."""
    need = target_rssi + fspl_db(distance_m, freq_mhz) + margin_db
    return max(power_min, min(power_max, int(round(need))))


def next_tx_power(power_dbm, report_fresh, loss_pct, rssi_dbm,
                  power_min=-9, power_max=22,
                  rssi_low=-90, rssi_high=-70,
                  step_up=4, step_down=1, loss_high_pct=20,
                  distance_m=None, near_m=50,
                  target_rssi=-80, margin_db=15,
                  stale_count=0, probe_start=24, probe_hold=16):
    """Next TX power (dBm), clamped.

    distance_m: winch<->rope distance (None = unknown -> legacy behaviour
    plus the latch probe). stale_count: consecutive decision cycles without
    fresh feedback (caller-maintained, reset on fresh)."""
    cap = power_max
    if distance_m is not None:
        cap = power_cap_dbm(distance_m, power_min, power_max,
                            target_rssi, margin_db)
    if (not report_fresh) or loss_pct >= loss_high_pct:
        if distance_m is not None and distance_m < near_m:
            # Near range: loss most likely means RX saturation, not range -
            # fail QUIET to the distance cap (bench: the minimum).
            return cap
        # Far/unknown: fail loud as before, but do not latch - after
        # probe_start stale cycles walk a probe ladder (max -> mid -> min,
        # cycling, probe_hold cycles per step) until feedback returns.
        if stale_count >= probe_start:
            mid = (power_max + power_min) // 2
            idx = ((stale_count - probe_start) // probe_hold) % 3
            return min((power_max, mid, power_min)[idx], cap)
        return min(power_max, cap)
    if rssi_dbm < rssi_low:
        return min(min(power_max, power_dbm + step_up), cap)
    if rssi_dbm > rssi_high:
        return max(power_min, min(power_dbm - step_down, cap))
    return min(power_dbm, cap)  # within the window: hold (capped)
