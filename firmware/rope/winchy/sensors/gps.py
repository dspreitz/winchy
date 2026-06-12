# u-blox M10S GPS, NMEA over UART.
#
# NMEA parsing and GPS->RTC time sync arrive with the asyncio runtime;
# for now this wraps the UART and the boot-time liveness dump.

import time


class GPS:
    def __init__(self, uart):
        self._uart = uart

    def any(self):
        return self._uart.any()

    def readline(self):
        return self._uart.readline()

    def dump(self, lines=10, timeout_ms=5000):
        """Print NMEA traffic as a boot-time liveness check.

        Unlike the old monolith this cannot hang forever if the GPS is
        silent - it gives up after timeout_ms.
        """
        deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
        n = 0
        while n < lines:
            if self._uart.any():
                print(self._uart.readline())
                n += 1
            elif time.ticks_diff(deadline, time.ticks_ms()) < 0:
                print("GPS: no NMEA traffic within %d ms" % timeout_ms)
                return
