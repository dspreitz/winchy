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

# ADS1232 24-bit bridge ADC (rope force), bit-banged serial interface.

import asyncio
import time

import machine
from machine import Pin


class ADS1232:
    """Force ADC.

    Data-ready is signalled by DOUT falling; a pin interrupt latches that into a
    flag so waiting for a sample sleeps instead of busy-spinning.

    The serial readout runs with interrupts masked: a stretched SCLK high phase
    would otherwise risk an unintended standby (the ADS1232 enters standby if
    SCLK is held high past t10 ~ one conversion, i.e. ~12.5 ms at 80 SPS;
    datasheet 7.4.2), and masking also keeps the bit-banged pulses clean.
    """

    _GAINS = {1: (0, 0), 2: (1, 0), 64: (0, 1), 128: (1, 1)}

    def __init__(self, pdwn, sclk, dout, gain0, gain1, gain=128, speed=None):
        self._pdwn = Pin(pdwn, Pin.OUT)
        self._sclk = Pin(sclk, Pin.OUT)
        self._dout = Pin(dout, Pin.IN)
        self._gain0 = Pin(gain0, Pin.OUT)
        self._gain1 = Pin(gain1, Pin.OUT)
        # SPEED pin: None = hardwired on the daughterboard (DGND -> 10 SPS).
        # Wire it to a GPIO to select the data rate via set_speed().
        self._speed = Pin(speed, Pin.OUT) if speed is not None else None
        self.offset = 0
        self._ready = False
        self._flag = asyncio.ThreadSafeFlag()
        self._pdwn.value(1)  # wake from power-down
        self._sclk.value(0)
        self.set_gain(gain)
        self._dout.irq(trigger=Pin.IRQ_FALLING, handler=self._on_drdy)

    def _on_drdy(self, _pin):
        self._ready = True
        self._flag.set()

    def set_gain(self, gain):
        if gain not in self._GAINS:
            raise ValueError("gain must be 1, 2, 64 or 128")
        g0, g1 = self._GAINS[gain]
        self._gain0.value(g0)
        self._gain1.value(g1)
        print("Gain set to %dx" % gain)

    def set_speed(self, sps):
        """Set the data rate via the SPEED pin: 10 or 80 SPS. Returns False if
        SPEED is hardwired (speed=None at construction). After a change the
        digital filter needs ~4 conversions to resettle (discard them)."""
        if self._speed is None:
            return False
        if sps not in (10, 80):
            raise ValueError("sps must be 10 or 80")
        self._speed.value(1 if sps == 80 else 0)
        return True

    def _clock_out(self):
        """Shift out the 24-bit conversion (DRDY/DOUT must be low). Runs with
        interrupts masked so a pulse can't be stretched into a standby event;
        a 25th pulse forces DOUT high to end the frame."""
        irq = machine.disable_irq()
        result = 0
        for _ in range(24):
            self._sclk.value(1)
            result = (result << 1) | self._dout.value()
            self._sclk.value(0)
        self._sclk.value(1)   # 25th pulse: force DOUT high
        self._sclk.value(0)
        machine.enable_irq(irq)
        if result & 0x800000:  # sign-extend
            result |= ~0xFFFFFF
        self._ready = False
        return result

    def _wait_drdy(self, timeout_ms, what="read"):
        deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
        while not self._ready and self._dout.value() == 1:
            if time.ticks_diff(deadline, time.ticks_ms()) < 0:
                raise OSError("ADS1232 DRDY timeout (%s)" % what)
            time.sleep_ms(1)

    def read_raw(self, timeout_ms=250):
        """One signed 24-bit conversion, no offset applied. Blocking."""
        self._wait_drdy(timeout_ms)
        return self._clock_out()

    async def aread_raw(self, timeout_ms=250):
        """One signed 24-bit conversion, yielding to other tasks while the
        conversion completes. Raises asyncio.TimeoutError if DRDY stays away."""
        # A stale flag (set by a previous shift's DOUT edges) returns
        # immediately while DOUT is still high, so wait again on the cleared
        # flag for the genuine DRDY edge.
        while self._dout.value() == 1:
            await asyncio.wait_for_ms(self._flag.wait(), timeout_ms)
        return self._clock_out()

    def read(self, timeout_ms=250):
        """Offset-corrected (tared) reading."""
        return self.read_raw(timeout_ms) - self.offset

    def calibrate_offset(self, timeout_ms=500):
        """Run the ADS1232 on-chip offset calibration (datasheet 7.4.1): after
        the 24 data bits a 25th SCLK forces DOUT high and the falling edge of a
        26th SCLK starts calibration. The analog inputs are disconnected
        internally, so this is load-independent; DRDY/DOUT returns low when done
        and the first conversion after is fully settled. Re-zeros the ADC's
        internal offset and its thermal drift.

        Run once at boot BEFORE tare(); the tare baseline then stays valid across
        later recalibrations (each returns the ADC offset to ~0, so no re-tare is
        needed). Blocks ~1-2 conversion periods. Raises OSError on timeout."""
        self._wait_drdy(timeout_ms, "calibrate")
        irq = machine.disable_irq()
        for _ in range(26):   # 24 data + 25th (DOUT high) + 26th (start cal)
            self._sclk.value(1)
            self._sclk.value(0)
        machine.enable_irq(irq)
        self._ready = False
        self._wait_drdy(timeout_ms, "calibrate done")  # DRDY low = cal complete
        self._ready = False

    def tare(self, samples=10):
        # The ADC needs a moment to start converting after wake/gain-set, so
        # the first conversions can time out at boot. Average `samples` good
        # reads, tolerating timeouts (extra time on the first), instead of
        # letting one DRDY timeout abort startup. Raises only if it gets none.
        total = 0
        got = 0
        attempts = 0
        while got < samples and attempts < samples * 3 + 5:
            attempts += 1
            try:
                total += self.read_raw(500 if got == 0 else 250)
                got += 1
            except OSError:
                time.sleep_ms(20)
        if got == 0:
            raise OSError("ADS1232 DRDY timeout (tare)")
        self.offset = total // got
        return self.offset
