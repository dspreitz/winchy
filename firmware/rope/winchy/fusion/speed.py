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

# Glider-speed estimate. Pure functions, no hardware imports - runs and tests
# on desktop CPython as well as on the device.

import math


def glider_speed(ground_speed_ms, climb_rate_ms, angle_deg, theta_rate_dps,
                 hook_distance_m=5.0):
    """Speed of the ring in the glider's CG hook (~ glider speed).

    Wind is ignored and the launch is assumed planar (one vertical plane).
    The rope segment moves at v_P = (ground_speed, climb_rate) in that plane
    (horizontal along-track, vertical). The CG hook sits hook_distance_m up
    the taut rope, so it is a rigid offset whose orientation is the rope
    angle theta; differentiating that offset gives the rigid-link relation

        v_H = v_P + L * (thetadot x r)

    i.e. a term perpendicular to the rope, proportional to how fast the rope
    is rotating:

        v_Hx = ground_speed - L*wr*sin(theta)
        v_Hz = climb_rate   + L*wr*cos(theta)

    with theta the rope angle above horizontal and wr = thetadot in rad/s.
    The correction is ~L*thetadot (a metre or so per second), significant only
    while the rope is rotating fast (the ground-roll/rotation phase); in steady
    climb (thetadot ~ 0) it reduces to the segment's own 3-D speed.
    """
    th = math.radians(angle_deg)
    wr = math.radians(theta_rate_dps)
    vx = ground_speed_ms - hook_distance_m * wr * math.sin(th)
    vz = climb_rate_ms + hook_distance_m * wr * math.cos(th)
    return math.sqrt(vx * vx + vz * vz)
