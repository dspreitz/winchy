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

# Read live AXP2101 battery/charger status on the rope unit. Run with:
#   mpremote connect COMx resume run tools/check_pmu.py
# Reads status registers only (does not re-run init_power), so it is safe to
# call while the app is interrupted at the REPL.
from machine import Pin, SoftI2C
import config
from AXP2101 import AXP2101, AXP2101_SLAVE_ADDRESS

pmu = AXP2101(SoftI2C(scl=Pin(config.PMU_SCL), sda=Pin(config.PMU_SDA)),
              addr=AXP2101_SLAVE_ADDRESS)
print("battery_connected:", pmu.isBatteryConnect())
print("vbus_in:", pmu.isVbusIn())
print("charging:", pmu.isCharging())
print("batt_mv:", pmu.getBattVoltage())
print("batt_pct:", pmu.getBatteryPercent())
print("system_mv:", pmu.getSystemVoltage())
