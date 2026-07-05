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

# Attitude math. Pure functions, no hardware imports - runs and tests on
# desktop CPython as well as on the device.

import math

# @micropython.native: called at the 50 Hz IMU rate; no-op shim on CPython.
# @micropython.native DISABLED (soak bisect 2026-07-05): with the
# decorators active the rope hit rst=WDT/PANIC every ~30-40 min in
# serial-free idle soaks, with the C sampler ON (soak C) and OFF
# (soak D) alike - the native emitter is the common suspect. The
# decorator is kept as a no-op so re-enabling is a one-line change
# once verified against a newer MicroPython.
def _native(f):
    return f


@_native
def rope_inclination(ax, ay, az):
    """Angle between the rope (device Y axis) and gravity, in degrees.

    Immune to the rope spinning around its own axis. Input is specific
    force in g. 0 = rope vertical, 90 = rope horizontal.

    Only valid while kinematic acceleration is small compared to gravity;
    during ground roll / rotation the Kalman filter must take over (see
    docs/winch_launch_physics.md).
    """
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    angle_rad = math.acos(abs(ay / norm))
    return math.degrees(angle_rad)


@_native
def rope_angle_above_ground(ax, ay, az):
    """Rope elevation angle relative to the ground ("Seilwinkel")."""
    return 90 - rope_inclination(ax, ay, az)
