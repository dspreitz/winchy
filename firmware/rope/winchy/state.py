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

# Shared application state: the latest value of each measured quantity
# plus its timestamp (time.ticks_ms). Each task writes only its own fields
# and reads any others; under cooperative (asyncio) scheduling no locking
# is needed.


class State:
    def __init__(self):
        # Force (ADS1232)
        self.force_raw = 0       # last raw conversion (untared)
        self.force_offset = 0    # tare offset
        self.force_ts = 0
        self.force_errors = 0    # DRDY timeouts
        # IMU / attitude
        self.accel = (0.0, 0.0, 0.0)     # instantaneous specific force, g
        self.gyro_dps = (0.0, 0.0, 0.0)  # bias-corrected angular rate
        self.mag = (0.0, 0.0, 0.0)       # magnetometer field, uT (x, y, z)
        self.angle_deg = 0.0             # rope angle above ground (Kalman)
        self.angle_rate_dps = 0.0        # rope-angle rate (EMA), for glider speed
        self.glider_speed_ms = 0.0       # CG-hook speed (rope speed + 5 m lever)
        self.rope_speed_ms = 0.0         # rope-segment 3-D speed (GPS+climb)
        self.accel_ts = 0
        # Tow phase (protocol.PHASE_*; the state machine will drive this)
        self.phase = 0  # PHASE_IDLE
        # Barometer
        self.pressure_hpa = 0.0
        self.baro_ts = 0
        self.qnh_hpa = 0.0      # sea-level reference; 0 = none (then seed/default)
        self.qnh_gps_cal = False  # True once a real GPS fix has set qnh_hpa
        self.baro_alt_m = 0.0   # altitude from pressure + qnh_hpa
        self.climb_rate_ms = 0.0  # vertical Kalman estimate
        # GPS
        self.gps_fix = 0         # GGA fix quality (0 = none)
        self.gps_sats = 0
        self.gps_hdop = 99.0     # GGA HDOP; high = poor geometry (default bad)
        self.lat = 0.0
        self.lon = 0.0
        self.alt_m = 0.0
        self.ground_speed_ms = 0.0  # GPS speed over ground (RMC)
        self.gps_ts = 0
        self.time_synced = False        # rope RTC set from its own GPS+PPS
        self.log_start = None    # "yyyymmdd-hhmm" session start (first GPS time)
        self.raw_uploaded_bytes = 0  # >0 = raw.csv offloaded; writer resets it
        self.raw_q = []          # pending raw-log lines; drained by raw_writer_task
        self.raw_recording = False   # imu_task is mid-episode (gates the reset)
        # Power
        self.system_mv = 0
        self.batt_mv = 0
        self.batt_pct = 0        # AXP2101 fuel gauge %, -1 if no cell
        self.batt_low = False    # rope battery low (checked while IDLE)
        self.charging = False    # AXP2101 reports the cell is charging
        # Radio link
        self.tx_count = 0
        # Downlink quality as reported by the winch (LINK_REPORT); for ADR.
        self.link_rssi_dbm = 0
        self.link_snr_db = 0
        self.link_loss_pct = 0
        self.link_report_ts = 0  # time.ticks_ms of last report; 0 = never
        self.tx_power_dbm = 0    # current radio TX power, driven by ADR
        # Winch position (from WINCH_POS over the radio) + derived geometry.
        self.winch_lat = 0.0
        self.winch_lon = 0.0
        self.winch_alt_m = 0.0
        self.winch_acc_m = 0.0   # surveyed horizontal accuracy, m
        self.winch_status = 0    # protocol.WINCH_* bits (fix / survey-done)
        self.winch_pos_ts = 0    # time.ticks_ms of last WINCH_POS; 0 = never
        # Winch-relative geometry (computed in gps_task once both fixes exist).
        self.cable_length_m = 0.0  # winch -> glider hook (slant + hook)
        self.winch_dist_m = 0.0    # straight-line winch -> rope segment
        self.elevation_deg = 0.0   # elevation of the rope seen from the winch
