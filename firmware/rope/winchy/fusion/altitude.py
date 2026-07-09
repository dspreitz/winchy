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

# Domain clamps - these functions must be TOTAL. Found 2026-07-09: a
# poisoned last_fix.json (alt=58316 m, a garbage GPS fix that got persisted)
# made (1 - alt/44330) NEGATIVE, and a negative base to a fractional power
# is COMPLEX in Python - the complex QNH then propagated through both
# Kalman filters into telemetry encoding and crash-looped the app on every
# boot. The ISA troposphere model is only valid to ~11 km anyway, so inputs
# are clamped to the physically sane envelope instead of ever going complex.
_ALT_MIN_M = -1000.0     # below the Dead Sea, with margin
_ALT_MAX_M = 11000.0     # troposphere ceiling
_P_MIN_HPA = 100.0       # ~16 km; anything below is a sensor glitch
_P_MAX_HPA = 1100.0


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def pressure_to_altitude_m(pressure_hpa, ref_sea_level_hpa):
    """Altitude in m for a pressure, given the sea-level reference (QNH)."""
    p = _clamp(pressure_hpa, _P_MIN_HPA, _P_MAX_HPA)
    ref = _clamp(ref_sea_level_hpa, _P_MIN_HPA, _P_MAX_HPA)
    return _ISA_SCALE_M * (1.0 - (p / ref) ** (1.0 / _ISA_EXPONENT))


def sea_level_pressure_hpa(pressure_hpa, altitude_m):
    """Sea-level reference (QNH) such that pressure_to_altitude_m() returns
    altitude_m for this pressure. Used to calibrate against GPS altitude."""
    p = _clamp(pressure_hpa, _P_MIN_HPA, _P_MAX_HPA)
    alt = _clamp(altitude_m, _ALT_MIN_M, _ALT_MAX_M)
    return p / (1.0 - alt / _ISA_SCALE_M) ** _ISA_EXPONENT
