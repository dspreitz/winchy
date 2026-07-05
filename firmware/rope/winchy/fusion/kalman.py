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

# Sensor fusion filters. Pure Python, no hardware imports - runs and tests
# on desktop CPython as well as on the device.

import math

# @micropython.native compiles the hot methods (called at the 50 Hz IMU rate)
# to machine code (~2x). On CPython (host tests) the decorator is a no-op, so
# behaviour and tests are identical on both.
# @micropython.native DISABLED (soak bisect 2026-07-05): with the
# decorators active the rope hit rst=WDT/PANIC every ~30-40 min in
# serial-free idle soaks, with the C sampler ON (soak C) and OFF
# (soak D) alike - the native emitter is the common suspect. The
# decorator is kept as a no-op so re-enabling is a one-line change
# once verified against a newer MicroPython.
def _native(f):
    return f

_DEG2RAD = math.pi / 180.0


class GravityKalman:
    """Tracks the gravity direction as a unit vector in the body frame.

    Prediction rotates the estimate with the gyro (dg = -w x g * dt);
    the measurement is the normalized accelerometer vector. With isotropic
    process and measurement noise, and rotation as the only dynamics, the
    covariance remains a scalar multiple of the identity - so the scalar
    form used here is exact, not an approximation.

    The measurement variance grows with | |a| - 1g |: under launch
    acceleration the accelerometer no longer points along gravity, the
    update is de-weighted and the gyro carries the estimate through
    (docs/winch_launch_physics.md, "consequences" item 2).
    """

    def __init__(self, q=0.01, r_static=0.05, r_dynamic=50.0):
        self.g = None        # gravity unit vector estimate, body frame
        self.p = 1.0         # scalar covariance
        self._q = q          # process noise per second
        self._r_static = r_static
        self._r_dynamic = r_dynamic

    @_native
    def predict(self, gyro_dps, dt):
        if self.g is None:
            return
        wx = gyro_dps[0] * _DEG2RAD
        wy = gyro_dps[1] * _DEG2RAD
        wz = gyro_dps[2] * _DEG2RAD
        rate = math.sqrt(wx * wx + wy * wy + wz * wz)
        angle = rate * dt
        if angle > 1e-9:
            # Exact Rodrigues rotation of g by -angle about the gyro axis
            # (the body rotates by +angle, so gravity rotates by -angle in
            # the body frame). A first-order Euler step underrotates
            # noticeably at rope-spin rates (several deg per step).
            kx, ky, kz = wx / rate, wy / rate, wz / rate
            gx, gy, gz = self.g
            c = math.cos(angle)
            s = math.sin(angle)
            dot = (kx * gx + ky * gy + kz * gz) * (1.0 - c)
            cx = ky * gz - kz * gy
            cy = kz * gx - kx * gz
            cz = kx * gy - ky * gx
            gx2 = gx * c - cx * s + kx * dot
            gy2 = gy * c - cy * s + ky * dot
            gz2 = gz * c - cz * s + kz * dot
            norm = math.sqrt(gx2 * gx2 + gy2 * gy2 + gz2 * gz2)
            if norm > 1e-9:
                self.g = (gx2 / norm, gy2 / norm, gz2 / norm)
        self.p += self._q * dt

    @_native
    def update(self, accel_g):
        ax, ay, az = accel_g
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm < 1e-6:
            return
        zx, zy, zz = ax / norm, ay / norm, az / norm
        if self.g is None:
            self.g = (zx, zy, zz)
            return
        # Adaptive measurement variance: distrust the accelerometer in
        # proportion to how far its magnitude is from 1 g.
        dyn = norm - 1.0
        r = self._r_static + self._r_dynamic * dyn * dyn
        k = self.p / (self.p + r)
        gx, gy, gz = self.g
        gx += k * (zx - gx)
        gy += k * (zy - gy)
        gz += k * (zz - gz)
        n = math.sqrt(gx * gx + gy * gy + gz * gz)
        if n > 1e-9:
            self.g = (gx / n, gy / n, gz / n)
        self.p *= (1.0 - k)

    @property
    def gravity(self):
        """Current estimate (unit vector), or straight down if unset."""
        return self.g if self.g is not None else (0.0, 1.0, 0.0)


class VerticalKalman:
    """Altitude and climb rate from a noisy altitude source.

    Constant-velocity model driven by white acceleration noise; standard
    2-state Kalman filter with explicit 2x2 algebra.
    """

    def __init__(self, accel_var=0.4, meas_var=1.0):
        self.alt = None
        self.vrate = 0.0
        self._q = accel_var   # (m/s^2)^2 white acceleration density
        self._r = meas_var    # m^2 altitude measurement variance
        self._p00 = 100.0
        self._p01 = 0.0
        self._p11 = 10.0

    @_native
    def predict(self, dt):
        if self.alt is None:
            return
        self.alt += self.vrate * dt
        # P = F P F' + Q, F = [[1, dt], [0, 1]]
        p00 = self._p00 + dt * (self._p01 + self._p01 + dt * self._p11)
        p01 = self._p01 + dt * self._p11
        p11 = self._p11
        # Q for white acceleration noise
        q = self._q
        self._p00 = p00 + q * dt ** 4 / 4
        self._p01 = p01 + q * dt ** 3 / 2
        self._p11 = p11 + q * dt ** 2

    @_native
    def update(self, alt_m):
        if self.alt is None:
            self.alt = alt_m
            return
        s = self._p00 + self._r
        k0 = self._p00 / s
        k1 = self._p01 / s
        innovation = alt_m - self.alt
        self.alt += k0 * innovation
        self.vrate += k1 * innovation
        p00, p01, p11 = self._p00, self._p01, self._p11
        self._p00 = (1 - k0) * p00
        self._p01 = (1 - k0) * p01
        self._p11 = p11 - k1 * p01
