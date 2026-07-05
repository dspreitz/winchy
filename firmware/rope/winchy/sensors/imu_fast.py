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

# Adapter for the winchy_fast C sampler (firmware/cmodules/winchy_fast):
# the QMI8658 is sampled by an esp_timer task at a hardware-exact rate into a
# ring buffer; Python only drains finished samples. Importing this module
# raises ImportError on builds without the C module - app.py then falls back
# to the legacy Python driver (winchy/sensors/qmi8658.py).

import time

import winchy_fast

_ACCEL_LSB_PER_G = 16384   # CTRL2: +/-2 g  (same scaling as qmi8658.py)
_GYRO_LSB_PER_DPS = 128    # CTRL3: +/-256 dps


class FastIMU:
    """QMI8658 via the winchy_fast C sampler.

    next_sample() -> (t_ms, (ax, ay, az) g, (gx, gy, gz) dps) or None. t_ms
    is ticks_ms-compatible (masked to the 2^30 ticks period in C), and the
    sample SPACING is exact regardless of the asyncio loop's cadence."""

    DRAIN = 6   # imu_task drains up to this many queued samples per wake

    def __init__(self, sck, mosi, miso, cs, hz=50):
        who = winchy_fast.imu_init(sck, mosi, miso, cs, hz)
        print("IMU: winchy_fast C sampler (WHO_AM_I=0x%02x, %d Hz)" % (who, hz))

    def next_sample(self):
        s = winchy_fast.imu_next()
        if s is None:
            return None
        return (s[0],
                (s[1] / _ACCEL_LSB_PER_G, s[2] / _ACCEL_LSB_PER_G,
                 s[3] / _ACCEL_LSB_PER_G),
                (s[4] / _GYRO_LSB_PER_DPS, s[5] / _GYRO_LSB_PER_DPS,
                 s[6] / _GYRO_LSB_PER_DPS))

    def pending(self):
        return winchy_fast.imu_count()

    # --- boot-time helpers (accel print + gyro-bias estimate in app.run) ----

    def _wait_sample(self):
        while True:
            s = self.next_sample()
            if s is not None:
                return s
            time.sleep_ms(5)

    def read_accel_avg(self, samples=10, delay_ms=10):
        sx = sy = sz = 0.0
        for _ in range(samples):
            _, a, _g = self._wait_sample()
            sx += a[0]
            sy += a[1]
            sz += a[2]
        return (sx / samples, sy / samples, sz / samples)

    def read_gyro(self):
        return self._wait_sample()[2]
