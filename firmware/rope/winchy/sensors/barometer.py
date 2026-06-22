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

# BMP280 barometric pressure sensor, forced-measurement mode.

import asyncio
import time

from bmp280 import (BMP280, BMP280_CASE_WEATHER,
                    BMP280_TEMP_OS_8, BMP280_PRES_OS_16, BMP280_STANDBY_250,
                    BMP280_IIR_FILTER_2, BMP280_POWER_FORCED)


class Barometer:
    def __init__(self, i2c):
        bmp = BMP280(i2c)
        bmp.use_case(BMP280_CASE_WEATHER)
        # x16 pressure oversampling: averages 16 samples *within* one forced
        # conversion (~44 ms), cutting per-reading noise with no temporal lag -
        # unlike a heavier IIR filter, which would lag the climb in a launch.
        # The explicit setters below fully define the config (the lib's
        # oversample() helper writes the wrong register bits, so don't use it).
        bmp.temp_os = BMP280_TEMP_OS_8
        bmp.press_os = BMP280_PRES_OS_16
        bmp.standby = BMP280_STANDBY_250
        bmp.iir = BMP280_IIR_FILTER_2
        bmp.power_mode = BMP280_POWER_FORCED
        self._bmp = bmp
        self._last_good = 1013.25   # standard pressure until the first read

    def _validate(self):
        """Reject a 0/implausible reading (the BMP280 occasionally returns 0,
        which with QNH set computes a ~44 km altitude spike); hold last good."""
        p = self._bmp.pressure / 100
        if 300.0 < p < 1100.0:      # BMP280 operating range
            self._last_good = p
        return self._last_good

    def pressure_hpa(self):
        """Blocking forced measurement (~100-200 ms). Returns hPa."""
        try:
            self._bmp.force_measure()
        except Exception:
            print("BMP Force measure not working.")
        while self._bmp.is_measuring:
            time.sleep(0.1)
        while self._bmp.is_updating:
            time.sleep(0.1)
        self._bmp.sleep()
        return self._validate()

    async def apressure_hpa(self):
        """Forced measurement that yields to other tasks while waiting."""
        try:
            self._bmp.force_measure()
        except Exception:
            print("BMP Force measure not working.")
        while self._bmp.is_measuring:
            await asyncio.sleep_ms(20)
        while self._bmp.is_updating:
            await asyncio.sleep_ms(20)
        self._bmp.sleep()
        return self._validate()
