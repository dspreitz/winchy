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

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "firmware",
                                "rope"))

from winchy.fusion.altitude import (pressure_to_altitude_m,
                                    sea_level_pressure_hpa)


def test_zero_altitude_at_reference():
    assert abs(pressure_to_altitude_m(1013.25, 1013.25)) < 0.001


def test_isa_1000m():
    # ISA: 898.75 hPa at 1000 m with standard sea level 1013.25 hPa.
    alt = pressure_to_altitude_m(898.75, 1013.25)
    assert abs(alt - 1000) < 5


def test_calibration_roundtrip():
    # Values from the bench device: 979.7 hPa at GPS altitude 373.2 m.
    ref = sea_level_pressure_hpa(979.7, 373.2)
    assert abs(pressure_to_altitude_m(979.7, ref) - 373.2) < 0.01
    # Climbing 100 m from there drops pressure by roughly 11.5 hPa.
    alt_higher = pressure_to_altitude_m(979.7 - 11.5, ref)
    assert 90 < alt_higher - 373.2 < 110


def test_lower_pressure_means_higher_altitude():
    ref = 1020.0
    assert (pressure_to_altitude_m(900.0, ref)
            > pressure_to_altitude_m(950.0, ref)
            > pressure_to_altitude_m(1000.0, ref))


# --- totality: these must NEVER return complex/inf (2026-07-09 crash loop:
# a poisoned last_fix.json with alt=58316 m made (1 - alt/44330) negative,
# the fractional power went COMPLEX and took down both Kalman filters and
# the telemetry encoder on every boot).

def test_regression_58km_altitude_stays_real():
    qnh = sea_level_pressure_hpa(974.2, 58316.3)   # the exact field values
    assert isinstance(qnh, float) and qnh > 0
    alt = pressure_to_altitude_m(974.2, qnh)
    assert isinstance(alt, float)


def test_negative_and_zero_qnh_stay_real():
    for ref in (0.0, -50.0, 1e-9):
        alt = pressure_to_altitude_m(974.2, ref)
        assert isinstance(alt, float)


def test_garbage_pressure_stays_real():
    # BMP280 occasionally returns 0 hPa (known glitch).
    alt = pressure_to_altitude_m(0.0, 1013.25)
    assert isinstance(alt, float)
    qnh = sea_level_pressure_hpa(0.0, 370.0)
    assert isinstance(qnh, float) and qnh > 0


def test_clamps_do_not_disturb_sane_values():
    ref = sea_level_pressure_hpa(979.7, 373.2)     # bench round-trip intact
    assert abs(pressure_to_altitude_m(979.7, ref) - 373.2) < 0.01
