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

# T-Beam S3 Supreme board definition: power rails and bus singletons.
#
# Only this module and config.py know how the board is wired. Drivers and
# the application get their buses from here instead of constructing their
# own (the old monolith created three different setups of the same I2C bus).

from machine import I2C, Pin, SPI, SoftI2C, UART

import config
from AXP2101 import AXP2101, AXP2101_SLAVE_ADDRESS, XPOWERS_AXP2101_CHIP_ID

# Bus singletons. Constructing these does not touch the attached chips, so
# it is safe before init_power() has enabled the sensor rails.
i2c0 = I2C(0, scl=Pin(config.I2C0_SCL), sda=Pin(config.I2C0_SDA),
           freq=config.I2C0_FREQ)
gps_uart = UART(1, baudrate=config.GPS_BAUD, bits=8, parity=None, stop=1,
                tx=Pin(config.GPS_TX), rx=Pin(config.GPS_RX), timeout=300)


def gps_reopen(baud):
    """Reconfigure the GPS UART to a new baud (after the module switched)."""
    gps_uart.init(baudrate=baud, bits=8, parity=None, stop=1,
                  tx=Pin(config.GPS_TX), rx=Pin(config.GPS_RX), timeout=300)
# IMU SPI: when config.IMU_FAST is on, the winchy_fast C module owns the bus
# exclusively (IDF spi_master on SPI3_HOST, the same controller machine.SPI(2)
# maps to) - creating machine.SPI(2) alongside it would double-drive the
# controller. The flag (not an import probe) decides, so the C module can stay
# in the image while disabled (stability bisect, see config.IMU_FAST).
if config.IMU_FAST:
    qmi_spi = None
    qmi_cs = None
else:
    qmi_spi = SPI(2, baudrate=1000000, polarity=1, phase=1,
                  sck=Pin(config.QMI_SCK), mosi=Pin(config.QMI_MOSI),
                  miso=Pin(config.QMI_MISO))
    qmi_cs = Pin(config.QMI_CS, Pin.OUT, value=1)

# AXP2101 rail plan, FACTORY-PARITY since 2026-07-21: (rail, mV, state) -
# state True = enable, False = explicitly DISABLE (overrides whatever the
# PMU held), None = set voltage only (DC1 feeds the ESP32-S3 - never toggle).
#
# The old plan enabled EVERY rail ("replicate the monolith") including four
# that LilyGo's factory firmware pointedly keeps OFF on the Supreme (DC2,
# CPUSLDO, DLDO1, DLDO2 - all unloaded on this board; DLDO2/CPUSLDO hang
# off DC4 which we also ran at 1.0 V instead of the factory 1.84 V).
# Board #2 powered itself off ~55 s into that config on every boot while
# running LilyGo's factory firmware indefinitely - prime suspect: a marginal
# defect on one of those never-factory-enabled rails tripping the PMU's
# protection. Factory parity is also simply correct: nothing of ours is on
# them. Loads: ALDO1/2 sensors, ALDO3 radio, ALDO4 GPS, BLDO1 SD slot,
# BLDO2 pin header (ADS1232 breakout), DC3/DC4/DC5 M.2 socket (empty; kept
# at factory settings).
_RAILS = (
    ("DC1", 3300, None),
    ("DC2", 1000, False),
    ("DC3", 3300, True),
    ("DC4", 1840, True),
    ("DC5", 3300, True),
    ("ALDO1", 3300, True),
    ("ALDO2", 3300, True),
    ("ALDO3", 3300, True),
    ("ALDO4", 3300, True),
    ("BLDO1", 3300, True),
    ("BLDO2", 3300, True),
    ("CPUSLDO", 1000, False),
    ("DLDO1", 3300, False),
    ("DLDO2", 3300, False),
)


def init_power():
    """Configure the AXP2101 PMU and enable all peripheral rails.

    Returns the PMU object. Settings replicate the original monolith:
    same rail voltages, charger profile, IRQ mask and watchdog state.
    """
    pmu_i2c = SoftI2C(scl=Pin(config.PMU_SCL), sda=Pin(config.PMU_SDA))
    pmu = AXP2101(pmu_i2c, addr=AXP2101_SLAVE_ADDRESS)
    if pmu.getChipID() != XPOWERS_AXP2101_CHIP_ID:
        raise RuntimeError("AXP2101 PMU not found")

    pmu.setVbusVoltageLimit(pmu.XPOWERS_AXP2101_VBUS_VOL_LIM_4V36)
    pmu.setVbusCurrentLimit(pmu.XPOWERS_AXP2101_VBUS_CUR_LIM_1500MA)
    pmu.setSysPowerDownVoltage(2600)

    for rail, millivolts, state in _RAILS:
        getattr(pmu, "set" + rail + "Voltage")(millivolts)
        if state is True:
            getattr(pmu, "enable" + rail)()
        elif state is False:
            getattr(pmu, "disable" + rail)()

    pmu.setPowerKeyPressOffTime(pmu.XPOWERS_POWEROFF_6S)
    pmu.setPowerKeyPressOnTime(pmu.XPOWERS_POWERON_2S)
    # Power-off is HARDWARE again (field test 2026-07-03): the software path
    # (button_task polling PKEY IRQs, then shutdown() on release) proved
    # unreliable on battery-only power, leaving no way to switch the unit off.
    # The AXP2101 long-press auto-off is authoritative and works even if the
    # app is crashed/hung: hold the key 6 s -> PMIC powers off (LED goes dark).
    # Known quirk: the PMIC powers off WHILE the key is held, so keep holding
    # past ~2 s after the LED goes dark and the press-to-power-on (2 s) can
    # restart it - RELEASE the key promptly once the LED turns off.
    pmu.setLongPressPowerOFF()          # long press = power off (not restart)
    pmu.enableLongPressShutdown()

    # No battery temperature sensor fitted; leaving TS measurement on
    # disturbs charging.
    pmu.disableTSPinMeasure()
    pmu.enableBattDetection()
    pmu.enableVbusVoltageMeasure()
    pmu.enableBattVoltageMeasure()
    pmu.enableSystemVoltageMeasure()
    pmu.setChargingLedMode(pmu.XPOWERS_CHG_LED_ON)  # steady on = rope powered on

    # Charge the GPS backup-domain cell/supercap on the AXP2101 VRTC rail so the
    # u-blox keeps its ephemeris/almanac/time/last-fix across power cycles ->
    # warm/hot GPS starts instead of a full cold start every boot (~30 s TTFF).
    # Retention needs an energy source between runs (18650 fitted, or a charged
    # backup cap), so this only helps when the unit isn't fully de-powered.
    pmu.setButtonBatteryChargeVoltage(3300)
    pmu.enableButtonBatteryCharge()

    pmu.disableIRQ(pmu.XPOWERS_AXP2101_ALL_IRQ)
    pmu.clearIrqStatus()
    pmu.enableIRQ(
        pmu.XPOWERS_AXP2101_BAT_INSERT_IRQ | pmu.XPOWERS_AXP2101_BAT_REMOVE_IRQ |
        pmu.XPOWERS_AXP2101_VBUS_INSERT_IRQ | pmu.XPOWERS_AXP2101_VBUS_REMOVE_IRQ |
        pmu.XPOWERS_AXP2101_PKEY_SHORT_IRQ | pmu.XPOWERS_AXP2101_PKEY_LONG_IRQ |
        pmu.XPOWERS_AXP2101_BAT_CHG_DONE_IRQ | pmu.XPOWERS_AXP2101_BAT_CHG_START_IRQ
    )   # PKEY IRQs are informational now - power-off is the PMIC's own long-press

    pmu.setPrechargeCurr(pmu.XPOWERS_AXP2101_PRECHARGE_50MA)
    # 1 A ~= 0.29C for the Samsung INR18650-35E (3500 mAh); needs a >=1.3 A
    # USB source to actually reach it (a weak laptop port will throttle).
    pmu.setChargerConstantCurr(pmu.XPOWERS_AXP2101_CHG_CUR_1000MA)
    pmu.setChargerTerminationCurr(pmu.XPOWERS_AXP2101_CHG_ITERM_25MA)
    pmu.setChargeTargetVoltage(pmu.XPOWERS_AXP2101_CHG_VOL_4V1)

    pmu.setWatchdogConfig(pmu.XPOWERS_AXP2101_WDT_IRQ_TO_PIN)
    pmu.setWatchdogTimeout(pmu.XPOWERS_AXP2101_WDT_TIMEOUT_4S)
    pmu.disableWatchdog()
    pmu.clearIrqStatus()

    print("PMU: system %dmV, battery %dmV (%d%%)" % (
        pmu.getSystemVoltage(), pmu.getBattVoltage(), pmu.getBatteryPercent()))
    return pmu
