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

import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "firmware",
                                "shared"))

from survey import SurveyIn, _ground_dist_m

M_PER_DEG_LAT = 6371000.0 * math.pi / 180


def _jitter(lat, lon, sigma_m, rng):
    """A noisy fix ~sigma_m metres off (lat, lon)."""
    dn = rng.gauss(0, sigma_m)
    de = rng.gauss(0, sigma_m)
    return (lat + dn / M_PER_DEG_LAT,
            lon + de / (M_PER_DEG_LAT * math.cos(math.radians(lat))))


def test_converges_near_truth():
    rng = random.Random(1)
    s = SurveyIn(min_samples=30, target_accuracy_m=2.0)
    lat0, lon0 = 48.5, 11.3
    for _ in range(200):
        s.add(*_jitter(lat0, lon0, 2.5, rng), 500.0)
    assert _ground_dist_m(lat0, lon0, s.lat, s.lon) < 1.0
    assert s.converged
    assert s.accuracy_m <= 2.0
    assert abs(s.alt - 500.0) < 5.0


def test_accuracy_shrinks_with_samples():
    rng = random.Random(2)
    s = SurveyIn()
    for _ in range(5):
        s.add(*_jitter(48.0, 11.0, 3.0, rng), 0.0)
    early = s.accuracy_m
    for _ in range(300):
        s.add(*_jitter(48.0, 11.0, 3.0, rng), 0.0)
    assert s.accuracy_m < early


def test_single_outlier_ignored():
    s = SurveyIn(reset_dist_m=15.0, reset_hits=5)
    for _ in range(40):
        s.add(48.0, 11.0, 0.0)
    lat_before, n_before = s.lat, s.n
    accepted = s.add(48.01, 11.0, 0.0)   # ~1.1 km away
    assert accepted is False
    assert s.lat == lat_before
    assert s.n == n_before


def test_reposition_reseeds_after_sustained_move():
    s = SurveyIn(reset_dist_m=15.0, reset_hits=5)
    for _ in range(40):
        s.add(48.0, 11.0, 0.0)
    assert s.converged
    new_lat, new_lon = 48.01, 11.0      # winch towed ~1.1 km
    for _ in range(5):
        s.add(new_lat, new_lon, 0.0)
    assert s.n == 1                     # average discarded, re-seeded here
    assert s.lat == new_lat
    assert not s.converged


def test_empty_is_not_converged():
    s = SurveyIn()
    assert s.lat is None
    assert s.accuracy_m == float("inf")
    assert not s.converged
