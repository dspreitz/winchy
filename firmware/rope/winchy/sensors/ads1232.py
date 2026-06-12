# ADS1232 24-bit bridge ADC (rope force), bit-banged serial interface.

import time

from machine import Pin


class ADS1232:
    """Force ADC.

    Data-ready is signalled by DOUT falling; a pin interrupt latches that
    into a flag so waiting for a sample sleeps instead of busy-spinning
    (the old monolith's tight DOUT poll blocked Ctrl-C and starved
    everything else). Ready for an asyncio ThreadSafeFlag in step 4.
    """

    _GAINS = {1: (0, 0), 2: (1, 0), 64: (0, 1), 128: (1, 1)}

    def __init__(self, pdwn, sclk, dout, gain0, gain1, gain=128):
        self._pdwn = Pin(pdwn, Pin.OUT)
        self._sclk = Pin(sclk, Pin.OUT)
        self._dout = Pin(dout, Pin.IN)
        self._gain0 = Pin(gain0, Pin.OUT)
        self._gain1 = Pin(gain1, Pin.OUT)
        self.offset = 0
        self._ready = False
        self._pdwn.value(1)  # wake from power-down
        self._sclk.value(0)
        self.set_gain(gain)
        self._dout.irq(trigger=Pin.IRQ_FALLING, handler=self._on_drdy)

    def _on_drdy(self, _pin):
        self._ready = True

    def set_gain(self, gain):
        if gain not in self._GAINS:
            raise ValueError("gain must be 1, 2, 64 or 128")
        g0, g1 = self._GAINS[gain]
        self._gain0.value(g0)
        self._gain1.value(g1)
        print("Gain set to %dx" % gain)

    def read_raw(self, timeout_ms=250):
        """One signed 24-bit conversion, no offset applied."""
        deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
        while not self._ready and self._dout.value() == 1:
            if time.ticks_diff(deadline, time.ticks_ms()) < 0:
                raise OSError("ADS1232 DRDY timeout")
            time.sleep_ms(1)

        result = 0
        for _ in range(24):
            self._sclk.value(1)
            result = (result << 1) | self._dout.value()
            self._sclk.value(0)
        if result & 0x800000:  # sign-extend
            result |= ~0xFFFFFF
        # One extra clock pulse completes the cycle and forces DOUT high.
        self._sclk.value(1)
        self._sclk.value(0)
        # Clear only after clocking: the data shift itself produces falling
        # edges on DOUT that re-trigger the IRQ.
        self._ready = False
        return result

    def read(self, timeout_ms=250):
        """Offset-corrected (tared) reading."""
        return self.read_raw(timeout_ms) - self.offset

    def tare(self, samples=10):
        self.offset = sum(self.read_raw() for _ in range(samples)) // samples
        return self.offset
