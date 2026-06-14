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

# Survey-in: average a stationary receiver's fixes over time to beat down the
# jitter on a fixed point - exactly what an RTK base does, here for the parked
# winch. Pure: no hardware and no clock; the caller feeds accepted fixes and
# reads the running estimate. Deliberately receiver-agnostic so the same logic
# works for the interim u-blox 7 now and the Supreme's M10 later - the swap is
# glue only.
#
# Averaging removes the random part of the error (~1/sqrt(n)); a correlated
# bias (multipath, slow ionosphere) remains, so accuracy_m is floored rather
# than allowed to shrink to zero. The reposition guard catches the winch being
# towed between sessions: a fix that sits far from the current mean for several
# consecutive samples re-seeds the average at the new spot.

import math

_EARTH_R = 6371000.0  # mean Earth radius, m


def _ground_dist_m(lat1, lon1, lat2, lon2):
    mlat = math.radians((lat1 + lat2) / 2)
    east = math.radians(lon2 - lon1) * math.cos(mlat) * _EARTH_R
    north = math.radians(lat2 - lat1) * _EARTH_R
    return math.sqrt(east * east + north * north)


class SurveyIn:
    """Running-mean position of a stationary receiver, with a reposition guard.

    Feed accepted fixes with add(); read .lat / .lon / .alt / .accuracy_m and
    .converged. accuracy_m is an indicator (standard error of the mean, floored
    at ACCURACY_FLOOR_M), not a rigorous CEP.
    """

    ACCURACY_FLOOR_M = 0.5

    def __init__(self, reset_dist_m=15.0, reset_hits=5, min_samples=30,
                 target_accuracy_m=2.0):
        self.reset_dist_m = reset_dist_m      # a fix beyond this may be a move
        self.reset_hits = reset_hits          # consecutive far fixes -> re-seed
        self.min_samples = min_samples        # before convergence can be claimed
        self.target_accuracy_m = target_accuracy_m
        self.reset()

    def reset(self):
        self.n = 0
        self.lat = None
        self.lon = None
        self.alt = None
        self._far = 0
        self._var_m2 = 0.0   # running estimate of single-fix spread (m^2)

    def _seed(self, lat, lon, alt):
        self.lat, self.lon, self.alt = lat, lon, alt
        self.n = 1
        self._var_m2 = 0.0
        self._far = 0

    @property
    def accuracy_m(self):
        if self.n < 2:
            return float("inf")
        sem = math.sqrt(self._var_m2 / self.n)   # std error of the mean
        return max(sem, self.ACCURACY_FLOOR_M)

    @property
    def converged(self):
        return (self.n >= self.min_samples
                and self.accuracy_m <= self.target_accuracy_m)

    def add(self, lat, lon, alt=0.0):
        """Feed one fix. Returns True if it was averaged in, False if it was
        rejected as an outlier or triggered a reposition re-seed."""
        if self.lat is None:
            self._seed(lat, lon, alt)
            return True

        d = _ground_dist_m(self.lat, self.lon, lat, lon)
        if d > self.reset_dist_m:
            self._far += 1
            if self._far >= self.reset_hits:
                self._seed(lat, lon, alt)   # sustained move -> re-survey here
            return False                    # never average a far fix in
        self._far = 0

        # incremental mean of lat/lon/alt and EMA of the squared offset, both
        # weighted 1/n so early samples count and the spread tracks the scatter.
        self.n += 1
        k = 1.0 / self.n
        self.lat += (lat - self.lat) * k
        self.lon += (lon - self.lon) * k
        self.alt += (alt - self.alt) * k
        self._var_m2 += (d * d - self._var_m2) * k
        return True
