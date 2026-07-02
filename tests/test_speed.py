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

# Host tests for the glider (CG-hook) speed estimate
# (firmware/rope/winchy/fusion/speed.py) - the rigid-link relation
# v_H = v_P + L*(thetadot x r). Was the only fusion module without tests.

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "firmware",
                                "rope"))

from winchy.fusion.speed import glider_speed


def test_no_rotation_reduces_to_3d_speed():
    # thetadot = 0: the hook moves with the segment -> plain sqrt(gs^2+climb^2).
    assert abs(glider_speed(3.0, 4.0, 30.0, 0.0) - 5.0) < 1e-9


def test_zero_everything_is_zero():
    assert glider_speed(0.0, 0.0, 0.0, 0.0) == 0.0


def test_rotation_at_horizontal_rope_adds_vertical_term():
    # theta=0 (rope horizontal): the correction is all vertical,
    # v_Hz = climb + L*wr*cos(0) = L*wr with gs=climb=0.
    wr_dps = 30.0
    L = 5.0
    expect = L * math.radians(wr_dps)
    assert abs(glider_speed(0.0, 0.0, 0.0, wr_dps, L) - expect) < 1e-9


def test_rotation_at_vertical_rope_subtracts_horizontal_term():
    # theta=90: v_Hx = gs - L*wr*sin(90) = gs - L*wr; vz = climb.
    wr_dps = 20.0
    L = 5.0
    gs = 10.0
    expect = abs(gs - L * math.radians(wr_dps))
    assert abs(glider_speed(gs, 0.0, 90.0, wr_dps, L) - expect) < 1e-9


def test_result_is_never_negative():
    # Magnitude of a vector: a fast back-rotation can't yield negative speed.
    assert glider_speed(1.0, 0.0, 90.0, 100.0, 5.0) >= 0.0
