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
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "firmware",
                                "rope"))

from winchy.fusion import geometry as g

LAT, LON = 48.0, 11.0
M_PER_DEG_LAT = 6371000.0 * math.pi / 180  # ~111195 m


def test_north_offset_distance_and_bearing():
    dlat = 1000.0 / M_PER_DEG_LAT
    assert abs(g.horizontal_distance_m(LAT, LON, LAT + dlat, LON) - 1000.0) < 1.0
    b = g.bearing_deg(LAT, LON, LAT + dlat, LON)
    assert b < 0.5 or b > 359.5


def test_east_offset_distance_and_bearing():
    dlon = 1000.0 / (M_PER_DEG_LAT * math.cos(math.radians(LAT)))
    assert abs(g.horizontal_distance_m(LAT, LON, LAT, LON + dlon) - 1000.0) < 1.0
    assert abs(g.bearing_deg(LAT, LON, LAT, LON + dlon) - 90.0) < 0.5


def test_cardinal_bearings_south_and_west():
    dlat = 1000.0 / M_PER_DEG_LAT
    dlon = 1000.0 / (M_PER_DEG_LAT * math.cos(math.radians(LAT)))
    assert abs(g.bearing_deg(LAT, LON, LAT - dlat, LON) - 180.0) < 0.5
    assert abs(g.bearing_deg(LAT, LON, LAT, LON - dlon) - 270.0) < 0.5


def test_elevation_and_slant():
    assert abs(g.elevation_angle_deg(100.0, 100.0) - 45.0) < 1e-9
    assert abs(g.elevation_angle_deg(100.0, 0.0)) < 1e-9
    assert abs(g.slant_distance_m(3.0, 4.0) - 5.0) < 1e-9


def test_winch_relative_at_force_onset():
    # both ends on the ground, rope 1000 m north -> initial cable length
    dlat = 1000.0 / M_PER_DEG_LAT
    r = g.winch_relative(LAT, LON, 500.0, LAT + dlat, LON, 500.0,
                         hook_distance_m=5.0)
    assert abs(r["horizontal_m"] - 1000.0) < 1.0
    assert abs(r["dh_m"]) < 1e-6
    assert abs(r["slant_m"] - 1000.0) < 1.0           # dh~0 -> slant ~ horizontal
    assert abs(r["elevation_deg"]) < 1e-3
    assert r["bearing_deg"] < 0.5 or r["bearing_deg"] > 359.5
    assert abs(r["cable_length_m"] - 1005.0) < 1.0    # + 5 m hook


def test_winch_relative_in_climb():
    # rope 600 m downrange and 400 m above the winch
    dlat = 600.0 / M_PER_DEG_LAT
    r = g.winch_relative(LAT, LON, 500.0, LAT + dlat, LON, 900.0)
    assert abs(r["horizontal_m"] - 600.0) < 1.0
    assert abs(r["dh_m"] - 400.0) < 1e-6
    assert abs(r["slant_m"] - math.hypot(600.0, 400.0)) < 1.0
    assert abs(r["elevation_deg"] - math.degrees(math.atan2(400, 600))) < 0.1
