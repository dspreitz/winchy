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

# QMI8658 IMU on SPI. Accelerometer in use; gyro is enabled with the same
# settings as the original code but not yet read (needed for the Kalman
# filter later).

import time

_REG_WHO_AM_I = 0x00
_REG_CTRL1 = 0x02  # serial interface / sensor enable
_REG_CTRL2 = 0x03  # accelerometer ODR/range
_REG_CTRL3 = 0x04  # gyroscope ODR/range
_REG_CTRL5 = 0x06  # data processing (low-pass filters)
_REG_CTRL7 = 0x08  # sensor enable / data reads
_REG_TEMP_L = 0x33
_REG_AX_L = 0x35
_REG_AY_L = 0x37
_REG_AZ_L = 0x39
_REG_GX_L = 0x3B
_REG_GY_L = 0x3D
_REG_GZ_L = 0x3F

_WHO_AM_I_VALUE = 0x05
_ACCEL_LSB_PER_G = 16384   # CTRL2 range bits: +/-2 g
_GYRO_LSB_PER_DPS = 128    # CTRL3 range bits: +/-256 dps


class QMI8658:
    def __init__(self, spi, cs):
        self._spi = spi
        self._cs = cs
        # The IMU can read back 0x00 for the first few ms after power-up (SPI
        # not ready yet), which used to crash the boot. Poll WHO_AM_I a few
        # times with a short settle before giving up.
        who = 0
        for _ in range(20):
            who = self._read(_REG_WHO_AM_I)[0]
            if who == _WHO_AM_I_VALUE:
                break
            time.sleep_ms(10)
        if who != _WHO_AM_I_VALUE:
            raise RuntimeError("QMI8658 not found (WHO_AM_I=0x%02x)" % who)
        # Same register configuration as the original monolith.
        self._write(_REG_CTRL1, 0b00100000)
        self._write(_REG_CTRL2, 0b10001000)
        self._write(_REG_CTRL3, 0b11001000)
        self._write(_REG_CTRL7, 0b10000011)
        self._write(_REG_CTRL5, 0b00011001)

    def _read(self, reg, length=1):
        self._cs.value(0)
        self._spi.write(bytearray([reg | 0x80]))
        data = self._spi.read(length)
        self._cs.value(1)
        return data

    def _write(self, reg, value):
        self._cs.value(0)
        self._spi.write(bytes([reg & 0x7F]))
        self._spi.write(bytes([value]))
        self._cs.value(1)

    def _read_int16(self, reg):
        value = (self._read(reg + 1)[0] << 8) | self._read(reg)[0]
        if value & 0x8000:
            value -= 65536
        return value

    def read_accel(self):
        """(x, y, z) specific force in g."""
        return (self._read_int16(_REG_AX_L) / _ACCEL_LSB_PER_G,
                self._read_int16(_REG_AY_L) / _ACCEL_LSB_PER_G,
                self._read_int16(_REG_AZ_L) / _ACCEL_LSB_PER_G)

    def read_gyro(self):
        """(x, y, z) angular rate in degrees per second."""
        return (self._read_int16(_REG_GX_L) / _GYRO_LSB_PER_DPS,
                self._read_int16(_REG_GY_L) / _GYRO_LSB_PER_DPS,
                self._read_int16(_REG_GZ_L) / _GYRO_LSB_PER_DPS)

    def read_accel_avg(self, samples=10, delay_ms=10):
        sum_x = sum_y = sum_z = 0.0
        for _ in range(samples):
            x, y, z = self.read_accel()
            sum_x += x
            sum_y += y
            sum_z += z
            time.sleep_ms(delay_ms)
        return (sum_x / samples, sum_y / samples, sum_z / samples)

    def read_temperature(self):
        return self._read_int16(_REG_TEMP_L) / 256.0
