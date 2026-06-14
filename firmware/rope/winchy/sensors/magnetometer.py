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

# QMC6310 magnetometer via the vendored PiicoDev driver.
#
# The PiicoDev driver insists on constructing its own machine.I2C instance,
# so it re-initialises I2C bus 0. We pass the same pins and frequency as
# board.i2c0, which makes the re-init a no-op for the other devices on the
# bus (the old monolith switched the shared bus to 1 MHz here). A native
# driver on board.i2c0 is planned together with the Kalman filter work.

from machine import Pin

import config
from PiicoDev_QMC6310 import PiicoDev_QMC6310


class Magnetometer:
    def __init__(self, range_ut=3000):
        # Loads offsets from calibration.cal (PiicoDev default) if present.
        self._mag = PiicoDev_QMC6310(bus=0, freq=config.I2C0_FREQ,
                                     scl=Pin(config.I2C0_SCL),
                                     sda=Pin(config.I2C0_SDA),
                                     range=range_ut)

    def read(self):
        """Field vector in microtesla: dict with keys x, y, z."""
        return self._mag.read()

    def magnitude(self):
        return self._mag.readMagnitude()

    def heading(self):
        return self._mag.readHeading()

    def calibrate(self):
        """Interactive figure-eight calibration; writes calibration.cal."""
        self._mag.calibrate()
