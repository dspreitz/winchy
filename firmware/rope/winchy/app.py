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

# Rope segment application: asyncio runtime (migration step 4).
#
# Each measurement source runs as its own task at its own rate and
# publishes into the shared State; telemetry and display sample the latest
# values. The force ADC is never stalled by the slow barometer read or the
# radio anymore. See docs/rope_segment_architecture.md.

import asyncio
import gc
import math
import micropython
import os
import time

from machine import Pin, RTC
import sh1106
from sx1262 import SX1262

import config
import protocol
import wifi
from winchy import board
from winchy.fusion import altitude
from winchy.fusion.attitude import rope_angle_above_ground
from winchy.fusion.geometry import winch_relative
from winchy.fusion.kalman import GravityKalman, VerticalKalman
from winchy.fusion.speed import glider_speed
from winchy.sensors.ads1232 import ADS1232
from winchy.sensors.barometer import Barometer
from winchy.sensors.gps import (GPS, parse_nmea, configure as gps_configure,
                                 set_baud as gps_set_baud)
from winchy.sensors.magnetometer import Magnetometer
from winchy.sensors.qmi8658 import QMI8658
from winchy.state import State

FORCE_TIMEOUT_MS = 1000
IMU_PERIOD_MS = 20          # 50 Hz sampling
MAG_PERIOD_MS = 50          # 20 Hz magnetometer (slow I2C; kept off the IMU loop)
IMU_WINDOW = 20             # samples for the boot-time accel print
BARO_PERIOD_MS = 1000

# Gyro bias: learned slowly while IDLE and still, frozen during the tow so
# it cannot chase real rotation.
GYRO_BIAS_ALPHA = 0.01
GYRO_STILL_TOLERANCE_G = 0.05
# Glider (CG-hook) speed = the rope-segment's own 3-D speed (sqrt(ground^2 +
# climb^2)), which is the glider speed to within a small correction. The 5 m
# rigid-link rotation term (L*thetadot) is DISABLED by default: it amplifies
# ANY rotation of the segment - hand-spinning it on the bench, or the segment
# swinging/spinning on the cable in flight - into large fake airspeed, which
# is not the glider's motion. Re-enable only after validating against a real
# launch (see [[winchy-next-features]]).
GLIDER_SPEED_5M_CORRECTION = False
GLIDER_HOOK_DIST_M = 5.0    # rope segment -> CG-hook ring, along the rope
ANGLE_RATE_ALPHA = 0.3      # EMA on the rope-angle rate
ANGLE_RATE_MAX_DPS = 200.0  # clamp to reject finite-difference spikes
TELEMETRY_PERIOD_MS = 500   # 2 Hz. SF7/BW250 airtime ~30 ms -> ~6% duty, well
                            # under g3's 10% limit; phase-driven rate switching
                            # (roadmap) would burst higher during a launch.
REPORT_REQUEST_EVERY = 2    # ask the winch for a LINK_REPORT every Nth frame
                            # (~1 Hz at 2 Hz telemetry); 0 disables feedback
REPORT_WINDOW_MS = 150      # extra RX dwell after a request so the winch's
                            # half-duplex reply lands before the next TX; must
                            # exceed reply airtime + winch processing (~60 ms
                            # at SF7, incl. the winch OLED update)
DISPLAY_PERIOD_MS = 500
SUPERVISOR_PERIOD_MS = 5000

# Low-battery warning (rope cell terminal voltage from the AXP2101, mV).
# Checked only while IDLE; hysteresis avoids flicker near the threshold.
BATT_PRESENT_MV = 2500      # at/below this, treat as no cell fitted (USB only)
BATT_LOW_MV = 3500          # warn below this (~20% on a Li-ion)
BATT_LOW_CLEAR_MV = 3650    # clear above this

# Range-test GPS logging: record this unit's position per transmitted frame,
# keyed by the telemetry seq so it joins against the winch's per-frame RSSI
# log (rope walks with GPS; winch stays tethered and logs the downlink).
# Clear rope_gpslog.csv before a run; disable for routine use (flash wear).
GPS_LOG = False             # set True for a range test (logs position per frame)
GPS_LOG_PATH = "rope_gpslog.csv"
GPS_LOG_FLUSH_EVERY = 4     # flush after this many records (~2 s at 2 Hz)

# Raw sensor log (rope-side, for offline Kalman validation): every filter input
# (raw accel/gyro/mag, pressure, force, GPS alt) plus the on-device outputs
# (baro alt/climb, angle) at the IMU cadence, to flash. Offloaded later (USB
# now; WiFi->GitHub planned). ~16 min at 50 Hz fills the ~5.8 MB FS, so logging
# stops at RAW_LOG_MAX_BYTES to protect it - clear/offload raw.csv per session.
RAW_LOG = True
RAW_LOG_PATH = "raw.csv"
RAW_LOG_FLUSH_EVERY = 100   # rows between flash flushes (flush itself is ~1 ms)
RAW_LOG_MAX_BYTES = 4000000
# Each f.write() to littlefs costs ~3 ms, so writing is done in raw_writer_task,
# not the 50 Hz imu_task: imu_task only appends formatted lines to state.raw_q,
# and the writer drains them a few at a time, yielding between chunks so a burst
# (e.g. the pre-roll dump) never stalls sampling. (Single-core cooperative, so
# each small chunk still blocks briefly - just ~10 ms, not one ~330 ms batch.)
RAW_WRITE_CHUNK = 8         # rows written per writer pass before it yields
RAW_WRITE_IDLE_MS = 50      # writer poll interval while idle / deferring
RAW_GC_EVERY = 20           # gc.collect() every N flushes in the writer
RAW_Q_MAX = 4000            # safety cap on the pending-write queue (~80 s @50 Hz)
# Written at file open and after an offload-reset; a file of exactly this size
# holds no episodes, so the uploader skips it.
# t_ms is the monotonic boot clock (for filter dt); utc_ms is unix-epoch
# milliseconds (UTC), valid once the GPS has synced the RTC, so rope and winch
# logs can be aligned in absolute time.
RAW_LOG_HEADER = ("# boot\n# t_ms,utc_ms,ax,ay,az,gx,gy,gz,mx,my,mz,force,"
                  "pressure_hpa,baro_alt_m,climb_ms,gps_alt_m,gps_lat,"
                  "gps_lon,gps_fix,gps_sats,gps_speed_ms,angle_deg,"
                  "glider_speed_ms,cable_len_m,winch_dist_m,elev_deg\n")
# Motion gating: a winch sits idle most of the time, so logging only around
# movement keeps the log (and the planned WiFi->GitHub upload) minimal. A
# time-based ring buffer holds the last RAW_LOG_PREROLL_S so the START of a
# launch is captured; once the unit has been at rest RAW_LOG_REST_HOLD_S, the
# segment is closed. Set RAW_LOG_MOTION_GATED False to log everything (e.g. a
# dedicated full-rate Kalman-validation capture).
RAW_LOG_MOTION_GATED = True
RAW_LOG_PREROLL_S = 5       # rolling pre-roll captured before motion starts
RAW_LOG_REST_HOLD_S = 5     # continuous rest this long closes a segment
MOTION_ACCEL_DEV_G = 0.10   # |norm-1g| over this = moving (rest <= 0.04)
MOTION_GYRO_DPS = 10.0      # bias-corrected gyro magnitude over this = moving

# Offload the raw log over WiFi when in range. The rope is battery-powered on
# the cable, so WiFi is duty-cycled: only while IDLE (never during a launch),
# only when the cell can spare it, and powered down between attempts. When it
# joins a known network it pushes raw.csv to the same winchy-logs release as
# the winch (separate asset). secrets.py (gitignored, on the device only):
# WIFI_NETWORKS = [(ssid, password), ...] and GITHUB_TOKEN. No secrets -> off.
# It connects to the strongest in-range network from the list (see wifi.py).
WIFI_ENABLED = True
WIFI_JOIN_TIMEOUT_S = 15    # wait this long per network for join + DHCP
WIFI_PERIOD_S = 600         # try to offload this often while idle (10 min)
GITHUB_REPO = "dspreitz/winchy-logs"
GITHUB_RELEASE_TAG = "logs"
GITHUB_ASSET = "rope_rawlog.csv"
try:
    from secrets import GITHUB_TOKEN
except ImportError:
    GITHUB_TOKEN = None
try:
    from secrets import WIFI_NETWORKS               # [(ssid, password), ...]
except ImportError:
    try:                                            # back-compat: single AP
        from secrets import WIFI_SSID, WIFI_PASSWORD
        WIFI_NETWORKS = [(WIFI_SSID, WIFI_PASSWORD)]
    except ImportError:
        WIFI_NETWORKS = []

# Closed-loop TX-power control (ADR). Drives the rope's output power from the
# winch's reported downlink quality. Hardened after an earlier version starved
# the link:
#   * control on RSSI (linear with power), NOT SNR - SNR saturates (~12 dB) and
#     let it keep cutting power until RSSI was at the noise floor;
#   * recover FAST: jump straight to max on a stale/lossy link, so it out-runs
#     survivorship-biased feedback (only frames that got through report "good");
#   * optimise SLOW: trim 1 dB at a time, and only when RSSI is well clear of
#     the floor, so it never walks the link down to the edge.
# Only power is adapted; SF/BW stay fixed (would need both ends retuned).
ADR_ENABLED = True
ADR_TX_POWER_MIN_DBM = -9     # SX1262 floor (driver clamps below this)
ADR_TX_POWER_MAX_DBM = 22     # SX1262 max (+22 dBm / 158 mW), within g3's 500 mW
                              # ERP cap. Needs OCP >= 140 mA in sx.begin() or the
                              # PA can't deliver it. Battery cost on the rope is
                              # ~7 mA average (118 mA x ~6% TX duty).
ADR_RSSI_LOW_DBM = -90        # winch RSSI below this -> raise (keeps ~20 dB above
                              # the ~-110 dBm SF7/BW500 sensitivity floor)
ADR_RSSI_HIGH_DBM = -70       # winch RSSI above this -> trim (margin to spare)
ADR_STEP_UP_DB = 4            # raise quickly to restore margin
ADR_STEP_DOWN_DB = 1          # trim gently
ADR_LOSS_HIGH_PCT = 20        # loss above this -> jump to max
ADR_REPORT_TIMEOUT_MS = 4000  # no fresh report this long -> jump to max


def _adr_next_power(state):
    """Next TX power (dBm), clamped. Fail loud (jump to max) on stale/lossy
    feedback so the link recovers at once; otherwise keep the winch-reported
    RSSI in a comfortable window - raise fast, trim slow."""
    power = state.tx_power_dbm
    fresh = (state.link_report_ts != 0 and time.ticks_diff(
        time.ticks_ms(), state.link_report_ts) <= ADR_REPORT_TIMEOUT_MS)
    if (not fresh) or state.link_loss_pct >= ADR_LOSS_HIGH_PCT:
        return ADR_TX_POWER_MAX_DBM
    rssi = state.link_rssi_dbm
    if rssi < ADR_RSSI_LOW_DBM:
        return min(ADR_TX_POWER_MAX_DBM, power + ADR_STEP_UP_DB)
    if rssi > ADR_RSSI_HIGH_DBM:
        return max(ADR_TX_POWER_MIN_DBM, power - ADR_STEP_DOWN_DB)
    return power  # within the window: hold

# Barometric altitude calibration against GPS, only while IDLE (during a
# launch the unit climbs and GPS/baro lag differently, so the reference is
# frozen). EMA per 1 Hz barometer sample: ~30 s to settle.
BARO_CAL_ALPHA = 0.1
BARO_CAL_GPS_MAX_AGE_MS = 5000


async def force_task(adc, state):
    # Free-running at the ADC conversion rate (10 SPS at gain 128).
    while True:
        try:
            state.force_raw = await adc.aread_raw(FORCE_TIMEOUT_MS)
            state.force_ts = time.ticks_ms()
        except asyncio.TimeoutError:
            state.force_errors += 1


async def mag_task(mag, state):
    # Magnetometer read is a slow I2C transaction (~10 ms); run it in its own
    # task at 20 Hz so it never drags down the 50 Hz IMU/Kalman loop. The IMU
    # loop logs the held state.mag value into each raw row.
    while True:
        try:
            m = mag.read()
            state.mag = (m["x"], m["y"], m["z"])
        except Exception:
            pass
        await asyncio.sleep_ms(MAG_PERIOD_MS)


async def imu_task(imu, state, filt, gyro_bias):
    bias = list(gyro_bias)
    last = time.ticks_ms()
    ring = []              # rolling pre-roll: (t_ms, rowstr), idle only
    recording = False
    last_motion_ts = 0
    prev_angle = None      # for the rope-angle rate (glider speed)
    while True:
        accel = imu.read_accel()
        gyro = imu.read_gyro()
        now = time.ticks_ms()
        dt = time.ticks_diff(now, last) / 1000.0
        last = now

        norm = math.sqrt(accel[0] ** 2 + accel[1] ** 2 + accel[2] ** 2)
        if (state.phase == protocol.PHASE_IDLE
                and abs(norm - 1.0) < GYRO_STILL_TOLERANCE_G):
            for i in range(3):
                bias[i] += GYRO_BIAS_ALPHA * (gyro[i] - bias[i])
        corrected = (gyro[0] - bias[0], gyro[1] - bias[1],
                     gyro[2] - bias[2])

        filt.predict(corrected, dt)
        filt.update(accel)
        state.accel = accel
        state.gyro_dps = corrected
        state.angle_deg = rope_angle_above_ground(*filt.gravity)
        state.accel_ts = now

        # Rope-angle rate (EMA + clamp) and glider (CG-hook) speed estimate.
        if prev_angle is not None and dt > 0:
            rate = (state.angle_deg - prev_angle) / dt
            if rate > ANGLE_RATE_MAX_DPS:
                rate = ANGLE_RATE_MAX_DPS
            elif rate < -ANGLE_RATE_MAX_DPS:
                rate = -ANGLE_RATE_MAX_DPS
            state.angle_rate_dps += ANGLE_RATE_ALPHA * (rate - state.angle_rate_dps)
        prev_angle = state.angle_deg
        vh = state.ground_speed_ms
        vv = state.climb_rate_ms
        state.rope_speed_ms = math.sqrt(vh * vh + vv * vv)
        if GLIDER_SPEED_5M_CORRECTION:
            state.glider_speed_ms = glider_speed(
                vh, vv, state.angle_deg, state.angle_rate_dps,
                GLIDER_HOOK_DIST_M)
        else:                              # segment 3-D speed; rotation-immune
            state.glider_speed_ms = state.rope_speed_ms

        # Raw log: filter inputs (raw accel/gyro/mag, pressure, force, GPS alt)
        # + on-device outputs (baro alt/climb, angle) for offline replay. gyro
        # is logged RAW (un-bias-corrected) so the bias estimation replays too.
        # No file I/O here - lines are queued for raw_writer_task so the slow
        # flash writes never stall this 50 Hz loop.
        if RAW_LOG and len(state.raw_q) < RAW_Q_MAX:
            mx, my, mz = state.mag   # held; mag_task refreshes it at ~20 Hz
            utc_ms = time.time_ns() // 1000000 + 946684800000  # unix epoch ms
            row = ("%d,%d,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%d,"
                   "%.2f,%.1f,%.2f,%.1f,%.7f,%.7f,%d,%d,%.2f,%.1f,%.2f,"
                   "%.1f,%.1f,%.2f\n" % (
                       now, utc_ms, accel[0], accel[1], accel[2], gyro[0], gyro[1],
                       gyro[2], mx, my, mz, state.force_raw, state.pressure_hpa,
                       state.baro_alt_m, state.climb_rate_ms, state.alt_m,
                       state.lat, state.lon, state.gps_fix, state.gps_sats,
                       state.ground_speed_ms, state.angle_deg,
                       state.glider_speed_ms, state.cable_length_m,
                       state.winch_dist_m, state.elevation_deg))

            # Motion gate. Rest baselines (motion-test): accel_dev <= 0.04 g,
            # gyro_mag < 1 dps. A flat spin keeps |a| ~ 1 g, so the gyro term is
            # what catches it - both terms are needed.
            moving = (not RAW_LOG_MOTION_GATED) or (
                abs(norm - 1.0) > MOTION_ACCEL_DEV_G
                or max(abs(corrected[0]), abs(corrected[1]),
                       abs(corrected[2])) > MOTION_GYRO_DPS)

            if not recording:
                ring.append((now, row))
                # trim the rolling pre-roll to the last RAW_LOG_PREROLL_S
                while time.ticks_diff(now, ring[0][0]) > RAW_LOG_PREROLL_S * 1000:
                    ring.pop(0)
                if moving:
                    state.raw_q.append("# motion-start t=%d\n" % now)
                    for _, r in ring:
                        state.raw_q.append(r)
                    ring = []
                    recording = True
                    state.raw_recording = True
                    last_motion_ts = now
            else:
                state.raw_q.append(row)
                if moving:
                    last_motion_ts = now
                if time.ticks_diff(now, last_motion_ts) >= RAW_LOG_REST_HOLD_S * 1000:
                    state.raw_q.append("# rest t=%d\n" % now)
                    recording = False
                    state.raw_recording = False
        await asyncio.sleep_ms(IMU_PERIOD_MS)


async def raw_writer_task(state):
    # Owns raw.csv. Drains state.raw_q a few lines at a time, yielding between
    # chunks so the slow per-line flash writes never block the 50 Hz imu_task.
    # Also performs the offload-reset (it is now the file's only writer).
    if not RAW_LOG:
        return
    rawf = open(RAW_LOG_PATH, "a")
    try:
        raw_bytes = os.stat(RAW_LOG_PATH)[6]   # cap across reboots (append)
    except OSError:
        raw_bytes = 0
    rawf.write(RAW_LOG_HEADER)
    rawf.flush()
    unflushed = 0
    flushes = 0
    while True:
        # Offload-reset: wifi_task sets raw_uploaded_bytes after a good upload.
        # Reclaim the flash, but only between episodes, with the queue drained,
        # and only if nothing new is on disk since the upload.
        if (state.raw_uploaded_bytes and not state.raw_recording
                and not state.raw_q):
            try:
                cur = os.stat(RAW_LOG_PATH)[6]
            except OSError:
                cur = 0
            if cur <= state.raw_uploaded_bytes:
                rawf.close()
                try:
                    os.remove(RAW_LOG_PATH)
                except OSError:
                    pass
                rawf = open(RAW_LOG_PATH, "a")
                rawf.write(RAW_LOG_HEADER)
                rawf.flush()
                raw_bytes = 0
                print("raw.csv offloaded; reset")
            state.raw_uploaded_bytes = 0

        # In motion-gated mode, defer all flash writes while an episode is
        # recording so the launch is sampled at full rate with no write stalls.
        # The buffered lines (RAM is plentiful - PSRAM) drain below once idle.
        # In continuous mode there is no idle, so drain as we go instead.
        if RAW_LOG_MOTION_GATED and state.raw_recording:
            await asyncio.sleep_ms(RAW_WRITE_IDLE_MS)
            continue

        q = state.raw_q
        if not q:
            if unflushed:
                rawf.flush()
                unflushed = 0
            await asyncio.sleep_ms(RAW_WRITE_IDLE_MS)
            continue
        chunk = q[:RAW_WRITE_CHUNK]    # atomic slice+del (no await between)
        del q[:RAW_WRITE_CHUNK]
        for r in chunk:
            if raw_bytes < RAW_LOG_MAX_BYTES:
                rawf.write(r)
                raw_bytes += len(r)
                unflushed += 1
        if unflushed >= RAW_LOG_FLUSH_EVERY:
            rawf.flush()
            unflushed = 0
            flushes += 1
            if flushes % RAW_GC_EVERY == 0:
                gc.collect()
        await asyncio.sleep_ms(0)      # yield so imu_task samples between chunks


async def baro_task(baro, state, vertical):
    last = time.ticks_ms()
    while True:
        pressure = await baro.apressure_hpa()
        now = time.ticks_ms()
        state.pressure_hpa = pressure
        state.baro_ts = now

        gps_fresh = (state.gps_fix and time.ticks_diff(
            time.ticks_ms(), state.gps_ts) < BARO_CAL_GPS_MAX_AGE_MS)
        if state.phase == protocol.PHASE_IDLE and gps_fresh:
            ref = altitude.sea_level_pressure_hpa(pressure, state.alt_m)
            if state.qnh_hpa == 0:
                state.qnh_hpa = ref
                print("Baro reference initialised: QNH %.1f hPa "
                      "(GPS alt %.1f m)" % (ref, state.alt_m))
            else:
                state.qnh_hpa += BARO_CAL_ALPHA * (ref - state.qnh_hpa)
        if state.qnh_hpa:
            state.baro_alt_m = altitude.pressure_to_altitude_m(
                pressure, state.qnh_hpa)
            vertical.predict(time.ticks_diff(now, last) / 1000.0)
            vertical.update(state.baro_alt_m)
            state.climb_rate_ms = vertical.vrate
        last = now
        await asyncio.sleep_ms(BARO_PERIOD_MS)


# GPS 1PPS discipline: the rising edge on config.GPS_PPS is the exact UTC
# second boundary, so we set the whole-second RTC value on the edge instead of
# when the (latency-jittered) NMEA sentence arrives. The IRQ only schedules the
# apply; the pre-built tuple is written in a safe context.
_pps_rtc = None         # RTC() handle
_pps_armed = None       # datetime tuple to write on the next edge, or None
_pps_count = 0          # rising edges seen (liveness / debug)


def _pps_apply(_):
    global _pps_armed
    a = _pps_armed
    if a is not None and _pps_rtc is not None:
        _pps_rtc.datetime(a)
        _pps_armed = None


def _on_pps(pin):
    global _pps_count
    _pps_count += 1
    micropython.schedule(_pps_apply, 0)


def _pps_arm_next(y, mo, d, h, mi, s):
    # Arm the RTC for the NEXT whole second (the upcoming PPS edge), UTC.
    global _pps_armed
    nxt = time.gmtime(time.mktime((y, mo, d, h, mi, s, 0, 0)) + 1)
    _pps_armed = (nxt[0], nxt[1], nxt[2], nxt[6], nxt[3], nxt[4], nxt[5], 0)


def _gps_alive(uart, timeout_ms=1500):
    """True if NMEA is arriving - used to confirm the high-baud switch took."""
    t0 = time.ticks_ms()
    buf = b""
    while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
        n = uart.any()
        if n:
            buf += uart.read(n)
            if b"$G" in buf:
                return True
        time.sleep_ms(10)
    return False


async def gps_task(state):
    global _pps_rtc
    # Raise the module baud (sent at the boot baud; ignored if already raised),
    # reopen the host UART to match, then trim to GGA+RMC and set the nav rate.
    gps_set_baud(board.gps_uart, config.GPS_BAUD_HIGH)
    board.gps_reopen(config.GPS_BAUD_HIGH)
    gps_configure(board.gps_uart, config.GPS_NAV_RATE_HZ)
    if not _gps_alive(board.gps_uart):       # baud switch didn't take - revert
        board.gps_reopen(config.GPS_BAUD)
        gps_configure(board.gps_uart, 5)
        print("GPS: high baud failed, fell back to %d/5 Hz" % config.GPS_BAUD)
    reader = asyncio.StreamReader(board.gps_uart)
    rtc = RTC()
    _pps_rtc = rtc
    Pin(config.GPS_PPS, Pin.IN).irq(trigger=Pin.IRQ_RISING, handler=_on_pps)
    while True:
        line = await reader.readline()
        update = parse_nmea(line)
        if not update:
            continue
        if update["type"] == "GGA":
            state.gps_fix = update["fix"]
            state.gps_sats = update["sats"]
            if update["lat"] is not None:
                state.lat = update["lat"]
                state.lon = update["lon"]
            if update["alt_m"] is not None:
                state.alt_m = update["alt_m"]
            state.gps_ts = time.ticks_ms()
            # Winch-relative geometry, once the winch has sent its position and
            # we have our own fix. Updates at the GPS rate (cheap trig); the
            # 50 Hz imu_task just logs the latest values. Reel-in rate is the
            # offline derivative of winch_dist_m, so it needs nothing here.
            if state.winch_pos_ts and state.gps_fix:
                rel = winch_relative(
                    state.winch_lat, state.winch_lon, state.winch_alt_m,
                    state.lat, state.lon, state.alt_m, GLIDER_HOOK_DIST_M)
                state.cable_length_m = rel["cable_length_m"]
                state.winch_dist_m = rel["slant_m"]
                state.elevation_deg = rel["elevation_deg"]
        elif update["type"] == "RMC":
            if update["speed_ms"] is not None:
                state.ground_speed_ms = update["speed_ms"]
            if update["datetime"]:
                y, mo, d, h, mi, s = update["datetime"]
                if not state.time_synced:
                    # Rough initial set (NMEA-latency) so we have time at once;
                    # PPS then refines the second boundary on each edge.
                    rtc.datetime((y, mo, d, 0, h, mi, s, 0))
                    state.time_synced = True
                    print("RTC synced from GPS: %04d-%02d-%02d %02d:%02d:%02dZ"
                          % (y, mo, d, h, mi, s))
                _pps_arm_next(y, mo, d, h, mi, s)


async def telemetry_task(sx, state):
    seq = 0
    frame_count = 0
    gpslog = open(GPS_LOG_PATH, "a") if GPS_LOG else None
    if gpslog:
        gpslog.write("# boot ticks_ms=%d\n"
                     "seq,ticks_ms,lat,lon,alt_m,sats,fix,tx_dbm\n"
                     % time.ticks_ms())
        gpslog.flush()
    gps_buf = []
    while True:
        force = state.force_raw - state.force_offset  # tared counts
        flags = protocol.FLAG_FORCE_UNCALIBRATED  # until calibration lands
        if state.gps_fix:
            flags |= protocol.FLAG_GPS_FIX
        if state.time_synced:
            flags |= protocol.FLAG_TIME_SYNCED
        if state.batt_low:
            flags |= protocol.FLAG_BATTERY_LOW
        if state.charging:
            flags |= protocol.FLAG_CHARGING
        frame_count += 1
        report_requested = bool(
            REPORT_REQUEST_EVERY and frame_count % REPORT_REQUEST_EVERY == 0)
        if report_requested:
            flags |= protocol.FLAG_REQUEST_REPORT
        # Tow phase detection arrives with the state machine (step 6).
        frame = protocol.encode_telemetry(
            seq, protocol.PHASE_IDLE, force, state.angle_deg,
            state.baro_alt_m, state.batt_mv, flags,  # real cell, not sys rail
            state.batt_pct, state.glider_speed_ms)
        print("ADC value:", force)
        print("Seilwinkel:", state.angle_deg)
        ax, ay, az = state.accel
        gx, gy, gz = state.gyro_dps
        print("Motion: |a|=%.3f g a=(%.2f,%.2f,%.2f) gyro=(%.1f,%.1f,%.1f) dps"
              % (math.sqrt(ax * ax + ay * ay + az * az), ax, ay, az,
                 gx, gy, gz))
        if state.qnh_hpa:
            print("Baro alt: %.1f m (GPS %.1f m, %+.1f m/s)"
                  % (state.baro_alt_m, state.alt_m, state.climb_rate_ms))
        if ADR_ENABLED:
            new_power = _adr_next_power(state)
            if new_power != state.tx_power_dbm:
                # setOutputPower must run from standby (it errors in RX). The
                # SPI sequence can be corrupted if the RX IRQ (on_radio) fires
                # mid-command, so guard it: on a collision, leave power as-is
                # and retry next cycle. send() below recovers from any state.
                try:
                    sx.standby()
                    sx.setOutputPower(new_power)
                    print("ADR: tx power %d -> %d dBm (snr=%d loss=%d%%)"
                          % (state.tx_power_dbm, new_power, state.link_snr_db,
                             state.link_loss_pct))
                    state.tx_power_dbm = new_power
                except Exception as e:
                    print("ADR power change deferred:", e)
        # Driver auto-returns to RX after this TX (non-blocking mode), so the
        # winch's reply is caught by on_radio with no explicit listen window;
        # we just dwell longer on report cycles so it isn't clobbered.
        try:
            sx.send(frame)  # non-blocking; TX_DONE arrives via radio callback
        except Exception as e:
            # SPI collided with the RX IRQ; drop this frame, recover to RX.
            print("TX deferred:", e)
            try:
                sx.startReceive()
            except Exception:
                pass
        if gpslog:  # this frame's position, keyed by seq for the post-walk join
            gps_buf.append("%d,%d,%.6f,%.6f,%.1f,%d,%d,%d\n" % (
                seq, time.ticks_ms(), state.lat, state.lon, state.alt_m,
                state.gps_sats, state.gps_fix, state.tx_power_dbm))
            if len(gps_buf) >= GPS_LOG_FLUSH_EVERY:
                for r in gps_buf:
                    gpslog.write(r)
                gpslog.flush()
                gps_buf = []
        seq = (seq + 1) & 0xFFFF
        state.tx_count += 1
        print("Sent packet (binary):", frame)
        await asyncio.sleep_ms(TELEMETRY_PERIOD_MS
                               + (REPORT_WINDOW_MS if report_requested else 0))


async def display_task(display, state):
    while True:
        display.fill(0)
        display.text("F: {}".format(state.force_raw - state.force_offset), 0, 0)
        display.text("Seilwinkel:", 0, 16)
        display.text("{:.1f} deg".format(state.angle_deg), 0, 26)
        display.text("{:.0f}hPa {:.0f}m".format(state.pressure_hpa,
                                                state.baro_alt_m), 0, 42)
        display.text("Sat:{} {}mV".format(state.gps_sats, state.system_mv),
                     0, 54)
        display.show()
        await asyncio.sleep_ms(DISPLAY_PERIOD_MS)


async def supervisor_task(pmu, state):
    while True:
        state.system_mv = pmu.getSystemVoltage()
        state.batt_mv = pmu.getBattVoltage()
        state.batt_pct = pmu.getBatteryPercent()
        state.charging = pmu.isCharging()
        # Recurring low-battery check - only meaningful while IDLE, and only
        # when a cell is actually fitted (0 mV = USB-only, not "empty").
        if state.phase == protocol.PHASE_IDLE and state.batt_mv > BATT_PRESENT_MV:
            if state.batt_mv < BATT_LOW_MV:
                state.batt_low = True
            elif state.batt_mv > BATT_LOW_CLEAR_MV:
                state.batt_low = False
        else:
            state.batt_low = False
        # Latch the session start (first valid wall clock) for the log filename.
        if state.time_synced and state.log_start is None:
            state.log_start = _log_stamp()
        await asyncio.sleep_ms(SUPERVISOR_PERIOD_MS)


def _log_stamp():
    # "yyyymmdd-hhmm" from the UTC RTC, or None if the clock isn't set yet.
    t = time.localtime()
    if t[0] < 2024:
        return None
    return "%04d%02d%02d-%02d%02d" % (t[0], t[1], t[2], t[3], t[4])


def _github_upload_raw(asset):
    # Replace the rolling rope-log asset on the 'logs' release: look up the
    # release, delete any existing asset of the same name, then upload raw.csv.
    # Blocks the asyncio loop for the round-trips (~secs) - only called while
    # IDLE, so the launch path is never affected.
    import urequests
    hdr = {"Authorization": "Bearer " + GITHUB_TOKEN, "User-Agent": "winchy",
           "Accept": "application/vnd.github+json"}
    ok = False
    try:
        r = urequests.get("https://api.github.com/repos/%s/releases/tags/%s"
                          % (GITHUB_REPO, GITHUB_RELEASE_TAG), headers=hdr)
        rel = r.json()
        r.close()
        rid = rel["id"]
        for a in rel.get("assets", ()):
            if a.get("name") == asset:
                urequests.delete(
                    "https://api.github.com/repos/%s/releases/assets/%d"
                    % (GITHUB_REPO, a["id"]), headers=hdr).close()
        gc.collect()
        body = open(RAW_LOG_PATH, "rb").read()
        h2 = dict(hdr)
        h2["Content-Type"] = "text/csv"
        u = urequests.post(
            "https://uploads.github.com/repos/%s/releases/%d/assets?name=%s"
            % (GITHUB_REPO, rid, asset), data=body, headers=h2)
        ok = 200 <= u.status_code < 300
        print("GitHub upload %s: HTTP %d" % (asset, u.status_code))
        u.close()
    except Exception as e:
        print("GitHub upload failed:", e)
    gc.collect()
    return ok


async def wifi_task(state):
    # Duty-cycled offload: while idle and the battery can spare it, bring WiFi
    # up, try to join the known network, push raw.csv if it has new data, then
    # power WiFi back down. WiFi is separate from the SX1262 LoRa link.
    import network
    wlan = network.WLAN(network.STA_IF)
    while True:
        await asyncio.sleep_ms(WIFI_PERIOD_S * 1000)
        # Never during a launch; skip if the cell is low and not charging.
        if state.phase != protocol.PHASE_IDLE:
            continue
        if state.batt_low and not state.charging:
            continue
        try:
            size = os.stat(RAW_LOG_PATH)[6]
        except OSError:
            continue
        if size <= len(RAW_LOG_HEADER):  # header only, no episodes to offload
            continue
        try:
            wlan.active(True)
            joined = await wifi.connect_any(wlan, WIFI_NETWORKS,
                                            WIFI_JOIN_TIMEOUT_S)
            if joined:
                # Prepend the session start (first GPS time) to the asset so
                # each session is a distinct, dated file on the release.
                stamp = state.log_start or _log_stamp()
                asset = (stamp + "_" if stamp else "") + GITHUB_ASSET
                print("WiFi '%s' joined (%s); uploading %s"
                      % (joined, wlan.ifconfig()[0], asset))
                if _github_upload_raw(asset):
                    # Tell imu_task (the file owner) it may reclaim the flash.
                    state.raw_uploaded_bytes = size
            else:
                print("WiFi: no known network in range")
        except Exception as e:
            print("WiFi/upload error:", e)
        finally:
            wlan.active(False)           # power WiFi down to save the battery
            gc.collect()


async def _main(pmu, adc, imu, baro, sx, display, state, gyro_bias, mag):
    # The app is up: re-enable Ctrl-C (main.py disabled it for the startup
    # window so a host attaching/probing during boot can't abort it). Now a
    # deliberate Ctrl-C interrupts the running app as usual (for deploys).
    micropython.kbd_intr(3)
    gravity_filter = GravityKalman()
    vertical_filter = VerticalKalman()
    tasks = [
        force_task(adc, state),
        imu_task(imu, state, gravity_filter, gyro_bias),
        raw_writer_task(state),
        mag_task(mag, state),
        baro_task(baro, state, vertical_filter),
        gps_task(state),
        telemetry_task(sx, state),
        supervisor_task(pmu, state),
    ]
    if display:
        tasks.append(display_task(display, state))
    if WIFI_ENABLED and WIFI_NETWORKS and GITHUB_TOKEN:
        tasks.append(wifi_task(state))
    await asyncio.gather(*tasks)


def run():
    pmu = board.init_power()
    state = State()

    # --- Sensors (with boot-time liveness output)
    baro = Barometer(board.i2c0)
    print(baro.pressure_hpa(), "hPa")

    gps = GPS(board.gps_uart)
    gps.dump(10)

    imu = QMI8658(board.qmi_spi, board.qmi_cs)
    print("Accelerations:", imu.read_accel_avg(IMU_WINDOW))
    # Initial gyro bias from 50 samples; the unit is at rest at power-on.
    sums = [0.0, 0.0, 0.0]
    for _ in range(50):
        g = imu.read_gyro()
        for i in range(3):
            sums[i] += g[i]
        time.sleep_ms(5)
    gyro_bias = tuple(v / 50 for v in sums)
    print("Gyro bias (dps): (%.2f, %.2f, %.2f)" % gyro_bias)

    mag = Magnetometer()  # loads calibration.cal; input for the Kalman work

    adc = ADS1232(pdwn=config.ADS_PDWN, sclk=config.ADS_SCLK,
                  dout=config.ADS_DOUT, gain0=config.ADS_GAIN0,
                  gain1=config.ADS_GAIN1, gain=config.ADS_GAIN)
    state.force_offset = adc.tare()
    print("Force ADC tared, offset", state.force_offset)

    # --- Display
    display = sh1106.SH1106_I2C(config.OLED_WIDTH, config.OLED_HEIGHT,
                                board.i2c0, Pin(config.OLED_RST),
                                config.OLED_ADDR)
    if config.DISPLAY_ENABLED:
        display.sleep(False)
        display.fill(0)
        display.text("Winchy rope unit", 0, 0)
        display.show()
    else:
        display.sleep(True)  # panel off
        display = None
        print("Display disabled (config.DISPLAY_ENABLED)")

    # --- LoRa
    def on_radio(events):
        # Runs from the DIO1 IRQ and can land mid-SPI relative to a task's
        # send(); a corrupted command raises ERR_CHIP_NOT_FOUND. Catch it and
        # re-arm RX so a collision drops one frame instead of crashing.
        try:
            if events & SX1262.RX_DONE:
                frame, err = sx.recv()
                msg = protocol.decode(frame)
                if msg and msg["type"] == protocol.LINK_REPORT:
                    state.link_rssi_dbm = msg["rssi_dbm"]
                    state.link_snr_db = msg["snr_db"]
                    state.link_loss_pct = msg["loss_pct"]
                    state.link_report_ts = time.ticks_ms()
                    print("Link report: rssi={} dBm snr={} dB loss={}%".format(
                        msg["rssi_dbm"], msg["snr_db"], msg["loss_pct"]))
                elif msg and msg["type"] == protocol.WINCH_POS:
                    state.winch_lat = msg["lat"]
                    state.winch_lon = msg["lon"]
                    state.winch_alt_m = msg["altitude_m"]
                    state.winch_acc_m = msg["hacc_m"]
                    state.winch_status = msg["status"]
                    state.winch_pos_ts = time.ticks_ms()
                    print("Winch pos: %.6f %.6f acc=%.1f m status=%d" % (
                        msg["lat"], msg["lon"], msg["hacc_m"], msg["status"]))
                else:
                    print("Receive: {}, {}".format(frame, SX1262.STATUS[err]))
            elif events & SX1262.TX_DONE:
                print("TX done.")
        except Exception as e:
            print("radio cb error:", e)
            try:
                sx.startReceive()
            except Exception:
                pass

    sx = SX1262(spi_bus=config.LORA_SPI_BUS, clk=config.LORA_CLK,
                mosi=config.LORA_MOSI, miso=config.LORA_MISO,
                cs=config.LORA_CS, irq=config.LORA_IRQ,
                rst=config.LORA_RST, gpio=config.LORA_BUSY)
    sx.begin(freq=config.LORA_FREQ_MHZ, bw=config.LORA_BW_KHZ,
             sf=config.LORA_SF, cr=config.LORA_CR,
             syncWord=config.LORA_SYNC_WORD,
             power=config.LORA_TX_POWER_DBM, currentLimit=140.0,
             preambleLength=8, implicit=False, implicitLen=0xFF,
             crcOn=True, txIq=False, rxIq=False,
             tcxoVoltage=1.7, useRegulatorLDO=False, blocking=True)
    sx.setBlockingCallback(False, on_radio)
    state.tx_power_dbm = config.LORA_TX_POWER_DBM  # ADR adjusts from here

    print("Starting asyncio runtime")
    asyncio.run(_main(pmu, adc, imu, baro, sx, display, state, gyro_bias, mag))
