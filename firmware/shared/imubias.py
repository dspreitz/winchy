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

# Gyro-bias estimation, extracted pure and host-testable. This feeds the
# motion gate that decides when the rope records/announces/uploads, so a
# regression here silently loses launch data (field test #2, 2026-07-07:
# the boot estimate averaged 250 ms of HANDLING motion into the bias, the
# corrected gyro then sat above the motion threshold forever -> the raw log
# rotated the ride away and announce/roam/upload never ran).
#
# Two pieces:
#   * window_bias(): judge one boot-time sample window - is it STILL enough
#     to trust as a bias estimate? Uses per-axis spread (motion shows up as
#     spread regardless of the unknown true bias) plus a magnitude sanity
#     bound (a steady rotation has low spread but a large mean - reject).
#   * BiasTracker: live self-healing learner. Tracks EMA mean/variance of
#     the RAW gyro - variance is bias-independent, so it recognises rest
#     even when the current bias is poisoned - and pulls the bias toward
#     the raw mean while at rest. Replaces the old accel-norm-gated learner
#     (the per-axis accel scale error kept |a|-1 above its 0.05 window in
#     some orientations, so it never learned at all).


def window_bias(samples, max_spread_dps=4.0, max_mean_dps=10.0):
    """Judge one boot-time gyro window. samples: iterable of (gx, gy, gz)
    in dps. Returns (mean3, still): mean3 is the per-axis mean (the bias
    candidate), still is True when the window is trustworthy - low per-axis
    spread (max-min) AND plausible magnitude (a real QMI8658 bias is a few
    dps; more means the unit is turning)."""
    n = 0
    mins = [1e9, 1e9, 1e9]
    maxs = [-1e9, -1e9, -1e9]
    sums = [0.0, 0.0, 0.0]
    for s in samples:
        for i in range(3):
            v = s[i]
            sums[i] += v
            if v < mins[i]:
                mins[i] = v
            if v > maxs[i]:
                maxs[i] = v
        n += 1
    if n == 0:
        return (0.0, 0.0, 0.0), False
    mean = tuple(v / n for v in sums)
    still = True
    for i in range(3):
        if maxs[i] - mins[i] > max_spread_dps or abs(mean[i]) > max_mean_dps:
            still = False
            break
    return mean, still


class BiasTracker:
    """Self-healing live gyro-bias learner (call update() per IMU sample).

    Keeps an EMA mean + variance of the RAW gyro. When every axis' variance
    is below still_var_dps2, the unit is at rest - a judgement that does NOT
    depend on the current bias, so a poisoned boot bias heals within seconds
    of real rest. While at rest the bias eases toward the raw mean (and is
    magnitude-capped: a slow steady rotation must not be learned away).

    .bias is a live list - hand it to the consumer once; updates mutate it
    in place."""

    def __init__(self, bias=(0.0, 0.0, 0.0), alpha_stat=0.02,
                 alpha_bias=0.01, still_var_dps2=4.0, max_bias_dps=10.0):
        self.bias = list(bias)
        self._mean = list(bias)
        self._var = [still_var_dps2] * 3   # start "not proven still"
        self._a = alpha_stat
        self._ab = alpha_bias
        self._still_var = still_var_dps2
        self._max = max_bias_dps

    @property
    def still(self):
        v = self._var
        s = self._still_var
        return v[0] < s and v[1] < s and v[2] < s

    def update(self, gyro):
        a = self._a
        m = self._mean
        v = self._var
        for i in range(3):
            d = gyro[i] - m[i]
            m[i] += a * d
            v[i] += a * (d * d - v[i])
        if self.still:
            ab = self._ab
            b = self.bias
            mx = self._max
            for i in range(3):
                t = m[i]
                if t > mx:
                    t = mx
                elif t < -mx:
                    t = -mx
                b[i] += ab * (t - b[i])
