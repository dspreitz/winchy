# BMP280 barometric pressure sensor, forced-measurement mode.

import time

from bmp280 import (BMP280, BMP280_CASE_WEATHER, BMP280_OS_HIGH,
                    BMP280_TEMP_OS_8, BMP280_PRES_OS_4, BMP280_STANDBY_250,
                    BMP280_IIR_FILTER_2, BMP280_POWER_FORCED)


class Barometer:
    def __init__(self, i2c):
        bmp = BMP280(i2c)
        bmp.use_case(BMP280_CASE_WEATHER)
        bmp.oversample(BMP280_OS_HIGH)
        bmp.temp_os = BMP280_TEMP_OS_8
        bmp.press_os = BMP280_PRES_OS_4
        bmp.standby = BMP280_STANDBY_250
        bmp.iir = BMP280_IIR_FILTER_2
        bmp.power_mode = BMP280_POWER_FORCED
        self._bmp = bmp

    def pressure_hpa(self):
        """Blocking forced measurement (~100-200 ms). Returns hPa.

        Becomes a kick/collect pair in the asyncio runtime (step 4) so the
        wait does not stall the force sampling.
        """
        try:
            self._bmp.force_measure()
        except Exception:
            print("BMP Force measure not working.")
        while self._bmp.is_measuring:
            time.sleep(0.1)
        while self._bmp.is_updating:
            time.sleep(0.1)
        self._bmp.sleep()
        return self._bmp.pressure / 100
