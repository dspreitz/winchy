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

# Rope segment configuration: pin map (LilyGo T-Beam S3 Supreme) and
# tunables. Schematic:
# https://github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series/blob/master/schematic/LilyGo_T-BeamS3Supreme.pdf

# I2C bus 0: BMP280 (0x77), QMC6310 magnetometer (0x1c), SH1106 OLED (0x3c)
I2C0_SCL = 18
I2C0_SDA = 17
I2C0_FREQ = 400000

# AXP2101 PMU on its own SoftI2C bus
PMU_SCL = 41
PMU_SDA = 42
PMU_IRQ = 40

# u-blox M10S GPS
GPS_TX = 8
GPS_RX = 9
GPS_BAUD = 9600        # u-blox cold-boot default; the UART opens here, then
                       # gps.set_baud() raises the module to GPS_BAUD_HIGH
GPS_BAUD_HIGH = 115200 # ~11.5 kB/s budget; 10 Hz GGA+RMC is only ~1.5 kB/s
GPS_PPS = 6   # u-blox 1PPS (TIMEPULSE) -> ESP32 GPIO6, disciplines the RTC
# Nav/output rate. gps.configure() trims NMEA to GGA+RMC and applies this.
# 10 Hz needs the raised baud (at 9600 only ~5 Hz fits).
GPS_NAV_RATE_HZ = 10

# IMU sampling path. True = winchy_fast C sampler (esp_timer task, hardware-
# exact 50 Hz; the C module stays in the image either way). False = legacy
# Python driver via machine.SPI(2).
# v1 (esp_timer service task + unbounded polling SPI) froze the VM under
# WiFi-rejoin + motion + streaming (hotspot-dance repro, bisected 2026-07-05)
# and caused rst=WDT/PANIC in the field - fixed by v2 (dedicated priority-10
# task, bounded interrupt SPI, imu_stats), which passed two hotspot dances at
# exact 50 Hz. The remaining "post-v2 panics" turned out to be a BENCH
# ARTIFACT, not firmware: a wedged USB-CDC connection with an attached-but-
# stalled host reader makes every print() block, crawling the whole VM into
# watchdog territory (proven live: closing the host's COM handle restored
# 0.1 Hz -> 2 Hz telemetry instantly). Lesson: never leave long-held serial
# readers attached to the rope; use WiFi/log-based observation instead.
# Serial-free soak bisect 2026-07-05: soak C (C sampler ON) crashed at
# ~32 min, soak D (C sampler OFF) crashed at ~36 min - both with the
# @micropython.native decorators active -> the C MODULE IS EXONERATED and the
# native emitter is the prime suspect (disabled in fusion/ for soak E, which
# runs with the C sampler back ON).
IMU_FAST = True

# QMI8658 IMU on SPI2
QMI_SCK = 36
QMI_MOSI = 35
QMI_MISO = 37
QMI_CS = 34
QMI_INT = 33

# SX1262 LoRa radio
LORA_SPI_BUS = 1
LORA_CLK = 12
LORA_MOSI = 11
LORA_MISO = 13
LORA_CS = 10
LORA_IRQ = 1
LORA_RST = 5
LORA_BUSY = 4  # 'gpio' parameter of the sx1262 driver

# LoRa link parameters - SF/BW/CR/sync/freq must match the winch segment.
# Band g3 (869.4-869.65 MHz: 500 mW ERP, 10% duty cycle), chosen over g1
# (868.0-868.6) for: 10x the duty-cycle headroom, power/range headroom, and to
# clear the FLARM collision-avoidance band (868.2-868.4 MHz, on every glider).
# g3 is only 250 kHz wide, so BW must be <=250: 869.525 +/- 125 kHz fits it
# exactly. SF7 keeps airtime low; BW250 is ~2x airtime vs BW500 but +3 dB more
# sensitive - fine for the ~17-byte telemetry frames.
LORA_FREQ_MHZ = 869.525
LORA_BW_KHZ = 250.0
LORA_SF = 7
LORA_CR = 8
LORA_SYNC_WORD = 0x12
LORA_TX_POWER_DBM = 14  # initial power; ADR ramps up to ADR_TX_POWER_MAX_DBM
                        # (now +22 dBm in g3) when the link needs it, trims down
                        # when it doesn't. +22 dBm needs OCP >= 140 mA (begin()).

# ADS1232 force ADC
ADS_PDWN = 39
ADS_SCLK = 45
ADS_DOUT = 46
ADS_GAIN0 = 38
ADS_GAIN1 = 2
ADS_GAIN = 128
# SPEED pin is hardwired to DGND on the daughterboard -> 10 SPS. To enable
# 80 SPS (tension-oscillation capture), rework the board (cut SPEED from DGND,
# run it to a free GPIO), then set ADS_SPEED to that GPIO. None = leave as-is.
ADS_SPEED = None
ADS_SPEED_HZ = 80      # data rate driven when ADS_SPEED is wired (10 or 80)

# SH1106 OLED. Disabled: the panel is physically blocked by the ADS1232
# breakout PCB, so it only wastes power. Set True to re-enable the status
# page and the display task.
DISPLAY_ENABLED = False
OLED_RST = 16
OLED_ADDR = 0x3C
OLED_WIDTH = 128
OLED_HEIGHT = 64

# Rope WiFi status dashboard (winchy/dashboard.py). With the OLED disabled this
# small web page is the only live status view for walk/ground tests. True keeps
# WiFi continuously on and serves the page (accepts the battery cost); False
# falls back to the duty-cycled WiFi behaviour (power saving, upload-only).
# Set False for flight. See app.py dashboard_task / wifi_task.
ROPE_DASHBOARD = True

# Radio master switch. False = radio never initialized, no telemetry/radio
# task (kept as a diagnostic lever; it was how the 2026-07-06 panic hunt
# isolated the radio path). The hunt's conclusion: the old micropySX126X
# driver's IRQ architecture caused stochastic WDT panics; fixed by migrating
# to the official micropython-lib lora driver (single-context, + explicit
# calibrate() after TCXO enable). Verified 2026-07-07: 8+ h soak, 0 panics.
RADIO_ENABLED = True
