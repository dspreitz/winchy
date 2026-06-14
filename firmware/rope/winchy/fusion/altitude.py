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

# Barometric altitude. International Standard Atmosphere model, valid in
# the troposphere. Pure functions, no hardware imports - runs and tests on
# desktop CPython as well as on the device.

_ISA_SCALE_M = 44330.0
_ISA_EXPONENT = 5.255


def pressure_to_altitude_m(pressure_hpa, ref_sea_level_hpa):
    """Altitude in m for a pressure, given the sea-level reference (QNH)."""
    return _ISA_SCALE_M * (1.0 -
                           (pressure_hpa / ref_sea_level_hpa)
                           ** (1.0 / _ISA_EXPONENT))


def sea_level_pressure_hpa(pressure_hpa, altitude_m):
    """Sea-level reference (QNH) such that pressure_to_altitude_m() returns
    altitude_m for this pressure. Used to calibrate against GPS altitude."""
    return pressure_hpa / (1.0 - altitude_m / _ISA_SCALE_M) ** _ISA_EXPONENT
