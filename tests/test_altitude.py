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
