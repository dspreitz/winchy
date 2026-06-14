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
                                "rope"))

from winchy.fusion.kalman import GravityKalman, VerticalKalman
from winchy.fusion.attitude import rope_angle_above_ground, rope_inclination

DT = 0.02  # 50 Hz, the device IMU rate


def angle_of(filt):
    return rope_inclination(*filt.gravity)


def test_static_convergence():
    random.seed(1)
    filt = GravityKalman()
    for _ in range(200):
        accel = (random.gauss(0, 0.02), 1.0 + random.gauss(0, 0.02),
                 random.gauss(0, 0.02))
        filt.predict((0.0, 0.0, 0.0), DT)
        filt.update(accel)
    # Residual is folded measurement noise; sub-degree is well within the
    # protocol's half-degree resolution.
    assert abs(angle_of(filt)) < 0.8  # rope vertical


def test_tracks_clean_rotation():
    # Inclination ramps 0 -> 45 deg over 3 s, rotation about body X.
    # g_body = (0, cos(theta), -sin(theta)), gyro_x = d(theta)/dt.
    random.seed(2)
    filt = GravityKalman()
    rate_dps = 15.0
    theta = 0.0
    for _ in range(int(3.0 / DT)):
        theta += rate_dps * DT
        t = math.radians(theta)
        accel = (0.0, math.cos(t), -math.sin(t))
        filt.predict((rate_dps, 0.0, 0.0), DT)
        filt.update(accel)
    assert abs(angle_of(filt) - 45.0) < 1.5


def test_gyro_carries_through_launch_acceleration():
    # Ground roll/rotation: inclination ramps 0 -> 40 deg over 4 s while a
    # 0.8 g kinematic acceleration along the rope (body Y) corrupts the
    # accelerometer between t=1 s and t=3 s. The accel-only angle is wrong
    # by >10 deg there; the filter must stay within a few degrees.
    random.seed(3)
    filt = GravityKalman()
    # settle the filter at theta=0 first
    for _ in range(100):
        filt.predict((0.0, 0.0, 0.0), DT)
        filt.update((0.0, 1.0, 0.0))

    rate_dps = 10.0
    theta = 0.0
    max_filter_err = 0.0
    max_accel_only_err = 0.0
    for i in range(int(4.0 / DT)):
        t_now = i * DT
        theta += rate_dps * DT
        t = math.radians(theta)
        gravity = (0.0, math.cos(t), -math.sin(t))
        kin = 0.8 if 1.0 < t_now < 3.0 else 0.0  # pull along the rope axis
        accel = (gravity[0], gravity[1] + kin, gravity[2])
        filt.predict((rate_dps, 0.0, 0.0), DT)
        filt.update(accel)
        if 1.0 < t_now < 3.0:
            max_filter_err = max(max_filter_err,
                                 abs(angle_of(filt) - theta))
            max_accel_only_err = max(max_accel_only_err,
                                     abs(rope_inclination(*accel) - theta))
    assert max_accel_only_err > 10.0   # the problem is real
    assert max_filter_err < 3.0        # and the filter solves it


def test_spin_immunity():
    # Rope spinning around its own axis at 360 dps, constant 20 deg
    # inclination: g_body = (sin(t)*sin(phi), cos(t), -sin(t)*cos(phi)),
    # gyro = (0, spin, 0). The inclination estimate must not move.
    random.seed(4)
    filt = GravityKalman()
    t = math.radians(20.0)
    spin_dps = 360.0
    phi = 0.0
    for _ in range(100):  # settle at phi=0
        filt.predict((0.0, 0.0, 0.0), DT)
        filt.update((0.0, math.cos(t), -math.sin(t)))
    for _ in range(int(5.0 / DT)):
        phi += math.radians(spin_dps) * DT
        accel = (math.sin(t) * math.sin(phi) + random.gauss(0, 0.02),
                 math.cos(t) + random.gauss(0, 0.02),
                 -math.sin(t) * math.cos(phi) + random.gauss(0, 0.02))
        filt.predict((0.0, spin_dps, 0.0), DT)
        filt.update(accel)
        assert abs(angle_of(filt) - 20.0) < 2.0


def test_vertical_kalman_estimates_climb_rate():
    # Climb at 4 m/s, baro altitude at 1 Hz with 0.7 m noise.
    random.seed(5)
    vk = VerticalKalman()
    alt = 100.0
    for i in range(30):
        alt += 4.0
        vk.predict(1.0)
        vk.update(alt + random.gauss(0, 0.7))
    assert abs(vk.vrate - 4.0) < 0.6
    assert abs(vk.alt - alt) < 1.5


def test_vertical_kalman_stationary():
    random.seed(6)
    vk = VerticalKalman()
    for _ in range(30):
        vk.predict(1.0)
        vk.update(360.0 + random.gauss(0, 0.7))
    assert abs(vk.vrate) < 0.4
    assert abs(vk.alt - 360.0) < 1.0
