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
        # CTRL2/3 set ODR 250 Hz (was 31.25) and clear the stray self-test bits;
        # CTRL5 widens the low-pass to ~9 Hz (mode 01 = 3.59% of ODR, was 0.82 Hz
        # = 2.62% of 31.25 Hz) so rotation-rate / oscillation dynamics aren't
        # smoothed away. ~9 Hz stays under Nyquist at the current ~21 Hz read
        # loop; raising the read rate (then a wider LPF) is a future action.
        # CTRL1 bit6 = ADDR_AI (register address auto-increment): enables the
        # 12-byte BURST read in read_accel_gyro() - one SPI transaction per
        # sample instead of 12, which was the read-loop bottleneck (~21 Hz).
        # Bit5 kept as before (chip byte-order setting; the byte assembly in
        # the reads matches it unchanged).
        self._write(_REG_CTRL1, 0b01100000)
        self._write(_REG_CTRL2, 0x05)   # accel: +/-2g, 250 Hz ODR, self-test off
        self._write(_REG_CTRL3, 0x45)   # gyro:  +/-256 dps, 250 Hz ODR, self-test off
        self._write(_REG_CTRL7, 0b10000011)  # accel + gyro enabled
        self._write(_REG_CTRL5, 0x33)   # LPF on, mode 01 = 3.59% of ODR (~9 Hz)
        # Preallocated burst-read buffers (no per-sample allocation at 50 Hz).
        self._burst = bytearray(12)              # AX_L..GZ_H (0x35..0x40)
        self._cmd_ax = bytes([_REG_AX_L | 0x80])  # read command, start at AX_L

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

    def read_accel_gyro(self):
        """Accel (g) + gyro (dps) in ONE 12-byte burst read (auto-increment
        from AX_L): ((ax, ay, az), (gx, gy, gz)).

        The old per-register path costs 12 SPI transactions each with ~0.5-1 ms
        of Python overhead - that capped the 50 Hz IMU loop at ~21 Hz. One
        transaction + local byte assembly removes the bottleneck. Byte order
        matches _read_int16 exactly (value = byte[addr+1] << 8 | byte[addr]),
        so the readings are bit-identical to the old path."""
        b = self._burst
        self._cs.value(0)
        self._spi.write(self._cmd_ax)
        self._spi.readinto(b)
        self._cs.value(1)
        ax = (b[1] << 8) | b[0]
        ay = (b[3] << 8) | b[2]
        az = (b[5] << 8) | b[4]
        gx = (b[7] << 8) | b[6]
        gy = (b[9] << 8) | b[8]
        gz = (b[11] << 8) | b[10]
        if ax & 0x8000:
            ax -= 65536
        if ay & 0x8000:
            ay -= 65536
        if az & 0x8000:
            az -= 65536
        if gx & 0x8000:
            gx -= 65536
        if gy & 0x8000:
            gy -= 65536
        if gz & 0x8000:
            gz -= 65536
        return ((ax / _ACCEL_LSB_PER_G, ay / _ACCEL_LSB_PER_G,
                 az / _ACCEL_LSB_PER_G),
                (gx / _GYRO_LSB_PER_DPS, gy / _GYRO_LSB_PER_DPS,
                 gz / _GYRO_LSB_PER_DPS))

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
