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

# Host tests for the gyro-bias estimation (firmware/shared/imubias.py).
# Regression anchor: field test #2 (2026-07-07) - the boot bias was averaged
# from 250 ms of HANDLING motion ("the unit is at rest at power-on"), the
# corrected gyro then exceeded the 10 dps motion threshold permanently, the
# raw log recorded continuously (rotating ride 1 away at the 4 MB cap) and
# still=False disabled announce/roam/upload for the whole field session.
# Contract under test:
#   * window_bias() must REJECT a window containing handling motion, and a
#     steady rotation (low spread, large mean).
#   * BiasTracker must heal a poisoned bias once the unit truly rests - its
#     stillness judgement must not depend on the (possibly wrong) bias.

import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "firmware",
                                "shared"))

from imubias import BiasTracker, window_bias

# Bench-measured rest characteristics (2026-06-13 motion test): true bias a
# few dps, sample-to-sample wiggle well under 1 dps.
TRUE_BIAS = (2.3, 1.8, 0.1)


def rest_samples(n, rng, noise=0.3):
    return [tuple(TRUE_BIAS[i] + rng.uniform(-noise, noise) for i in range(3))
            for _ in range(n)]


def handling_samples(n, rng, amp=40.0):
    # Handling/mounting motion: tens of dps, sign-changing.
    return [tuple(rng.uniform(-amp, amp) for _ in range(3)) for _ in range(n)]


# --- window_bias: the boot-time window judgement ------------------------------

def test_still_window_accepted_with_true_bias():
    rng = random.Random(1)
    mean, still = window_bias(rest_samples(50, rng))
    assert still
    for i in range(3):
        assert abs(mean[i] - TRUE_BIAS[i]) < 0.2


def test_handling_window_rejected():
    # THE field-test-#2 poison scenario: power-on while mounting. The mean of
    # this window is a garbage bias - it must never be accepted.
    rng = random.Random(2)
    _, still = window_bias(handling_samples(50, rng))
    assert not still


def test_partial_motion_window_rejected():
    # A single jolt inside an otherwise calm window still spoils the mean.
    rng = random.Random(3)
    win = rest_samples(40, rng) + [(25.0, -18.0, 30.0)] + rest_samples(9, rng)
    _, still = window_bias(win)
    assert not still


def test_steady_rotation_rejected_despite_low_spread():
    # Constant turn (e.g. on a slowly rotating drum): spread ~0 but the mean
    # is a rate, not a bias.
    rng = random.Random(4)
    win = [tuple(20.0 + rng.uniform(-0.2, 0.2) for _ in range(3))
           for _ in range(50)]
    _, still = window_bias(win)
    assert not still


def test_empty_window_is_not_still():
    mean, still = window_bias([])
    assert not still and mean == (0.0, 0.0, 0.0)


# --- BiasTracker: the live self-healing learner --------------------------------

def test_tracker_holds_bias_during_motion():
    rng = random.Random(5)
    tr = BiasTracker(TRUE_BIAS)
    before = list(tr.bias)
    for g in handling_samples(500, rng):
        tr.update(g)
    # High variance -> never "still" -> bias untouched by motion data.
    assert tr.bias == before


def test_tracker_heals_poisoned_bias_at_rest():
    # THE self-heal requirement: boot bias poisoned to ~40 dps (the old code
    # could never recover - its accel-norm gate stayed shut). Feed real rest:
    # the variance judgement sees stillness regardless of the wrong bias and
    # pulls it to the true value within ~1 min of 50 Hz samples.
    rng = random.Random(6)
    tr = BiasTracker((41.0, -38.0, 25.0))
    for g in rest_samples(3000, rng):    # 60 s at 50 Hz
        tr.update(g)
    for i in range(3):
        assert abs(tr.bias[i] - TRUE_BIAS[i]) < 1.0


def test_tracker_converges_from_zero_fallback():
    # The moving-boot fallback hands the task zeros; at rest the tracker must
    # find the real bias.
    rng = random.Random(7)
    tr = BiasTracker((0.0, 0.0, 0.0))
    for g in rest_samples(3000, rng):
        tr.update(g)
    for i in range(3):
        assert abs(tr.bias[i] - TRUE_BIAS[i]) < 1.0


def test_tracker_never_learns_a_rate_as_bias():
    # A perfectly steady rotation is indistinguishable from rest by variance
    # alone - the magnitude cap must keep the learned bias plausible (<= the
    # 10 dps motion threshold) so a real turn can never gate itself off.
    rng = random.Random(8)
    tr = BiasTracker(TRUE_BIAS)
    steady = [tuple(35.0 + rng.uniform(-0.1, 0.1) for _ in range(3))
              for _ in range(3000)]
    for g in steady:
        tr.update(g)
    for i in range(3):
        assert abs(tr.bias[i]) <= 10.0


def test_tracker_recovers_stillness_after_motion():
    # motion -> rest -> the variance decays and learning resumes.
    rng = random.Random(9)
    tr = BiasTracker((10.0, 10.0, 10.0))
    for g in handling_samples(500, rng):
        tr.update(g)
    for g in rest_samples(4000, rng):
        tr.update(g)
    assert tr.still
    for i in range(3):
        assert abs(tr.bias[i] - TRUE_BIAS[i]) < 1.0
