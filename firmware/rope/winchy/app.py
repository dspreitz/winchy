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
from winchy import board, dashboard
from winchy.fusion import altitude
from winchy.fusion.attitude import rope_angle_above_ground
from winchy.fusion.geometry import winch_relative
from winchy.fusion.kalman import GravityKalman, VerticalKalman
from winchy.fusion.speed import glider_speed
from winchy.sensors.ads1232 import ADS1232
from winchy.sensors.barometer import Barometer
from winchy.sensors.gps import (GPS, parse_nav_pvt, read_ubx,
                                 configure as gps_configure,
                                 set_baud as gps_set_baud,
                                 feed_mga as gps_feed_mga,
                                 mga_ini_time_utc as gps_ini_time,
                                 poll_ubx as gps_poll_ubx)
from winchy.sensors.magnetometer import Magnetometer
from winchy.sensors.qmi8658 import QMI8658
from winchy.state import State

FORCE_TIMEOUT_MS = 1000
# Periodic ADS1232 on-chip offset recalibration (sheds thermal offset drift).
# Load-independent and returns the offset to the boot baseline, so no re-tare;
# gated to IDLE + near-zero tared load so a launch is never interrupted.
ADC_RECAL_PERIOD_S = 600       # recalibrate at most this often
ADC_RECAL_QUIET_COUNTS = 500000  # only when |tared force| is below this (~6% FS)
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
BUTTON_POLL_MS = 300        # power-key long-press poll (off via long press)

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
# now; WiFi->GitHub). At RAW_LOG_MAX_BYTES the log ROTATES (resets + keeps
# logging) rather than halting, so the most recent ~RAW_LOG_MAX_BYTES is always
# captured - a stuck/failed upload can never silently stop recording a test.
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
                  "glider_speed_ms,cable_len_m,winch_dist_m,elev_deg,"
                  "gps_hacc_m,gps_climb_ms\n")
# Motion gating: a winch sits idle most of the time, so logging only around
# movement keeps the log (and the planned WiFi->GitHub upload) minimal. A
# time-based ring buffer holds the last RAW_LOG_PREROLL_S so the START of a
# launch is captured; once the unit has been at rest RAW_LOG_REST_HOLD_S, the
# segment is closed. Set RAW_LOG_MOTION_GATED False to log everything (e.g. a
# dedicated full-rate Kalman-validation capture).
RAW_LOG_MOTION_GATED = True   # only log around movement; False = log everything
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

# AssistNow Predictive Orbits (u-blox MGA-ANO). Opportunistic: when the rope has
# internet (the dashboard keeps WiFi up), fetch predicted orbit data and feed it
# to the GPS - so a later cold start uses predicted ephemeris instead of waiting
# ~30 s to download it over the air. The blob is cached on flash and re-fed at
# every boot, so it helps even with no internet then.
#
# The legacy Online/Offline token flow was retired (EoS 2026-05-31); the new
# service uses Zero-Touch Provisioning: the device registers once with its
# UBX-SEC-UNIQID + UBX-MON-VER and a Device-Profile token to get a permanent
# chipcode, then fetches data with the chipcode. secrets.py (device only):
# UBLOX_ZTP_TOKEN (from the Thingstream Device Profile). No token -> the
# download is skipped (any cached blob is still fed at boot).
ASSISTNOW_ZTP_URL = "https://api.thingstream.io/ztp/assistnow/credentials"
ASSISTNOW_GNSS = "gps"             # add ',gal,glo,bds' for more (bigger blob)
ASSISTNOW_DATA = "uporb_7,ualm"    # 7-day predicted orbits + almanac (per plan's allowedData)
ASSISTNOW_MAX_AGE_S = 3 * 86400    # re-download when the cache is older than this
ASSISTNOW_PATH = "mga_offline.ubx"      # cached MGA blob
ASSISTNOW_TS_PATH = "mga_offline.ts"    # blob download time (s, 2000-epoch)
ASSISTNOW_CHIP_PATH = "mga_chip.json"   # cached ZTP chipcode + data serviceUrl
_assistnow_done = False            # one download attempt per boot
_gps_uniqid_hex = None             # UBX-SEC-UNIQID frame (hex), for ZTP register
_gps_monver_hex = None             # UBX-MON-VER frame (hex), for ZTP register
try:
    from secrets import UBLOX_ZTP_TOKEN
except ImportError:
    UBLOX_ZTP_TOKEN = None
try:
    from secrets import UBLOX_ZTP_URL   # optional override of the endpoint
    ASSISTNOW_ZTP_URL = UBLOX_ZTP_URL
except ImportError:
    pass

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
# Fallback reference if QNH can't be seeded (first boot ever, no stored fix):
# standard sea-level pressure. Absolute altitude is then off by the local QNH
# offset, but climb/relative are still correct, so baro/climb work before a fix.
BARO_QNH_DEFAULT_HPA = 1013.25
# Last good GPS fix, persisted to flash, so QNH can be seeded from the known
# field elevation at the next boot (before any fix). Saved while IDLE, throttled.
LAST_FIX_PATH = "last_fix.json"
LAST_FIX_SAVE_S = 120         # min seconds between saves (flash wear)
LAST_FIX_MIN_SATS = 5         # only persist a confident 3D fix
# QNH GPS-calibration gating: the receiver's altitude can be hundreds of metres
# off for a few seconds right at first lock, so don't calibrate QNH off a weak
# or outlier fix (it would yank baro_alt and slowly fade back).
QNH_CAL_MIN_SATS = 5          # need a confident fix to (re)calibrate QNH
QNH_CAL_MAX_HACC_M = 5.0      # ... and a tight fix (NAV-PVT horizontal accuracy,
                              # metres - a direct quality measure, unlike DOP)
QNH_CAL_MAX_JUMP_M = 100      # ignore a GPS alt this far off the baro alt ...
QNH_CAL_FAR_HITS = 10         # ... unless it persists (the unit really moved)


async def force_task(adc, state):
    # Free-running at the ADC conversion rate (10 SPS at gain 128).
    last_recal = time.ticks_ms()
    while True:
        try:
            state.force_raw = await adc.aread_raw(FORCE_TIMEOUT_MS)
            state.force_ts = time.ticks_ms()
        except asyncio.TimeoutError:
            state.force_errors += 1
        # Periodically re-run the ADC's on-chip offset calibration to shed
        # thermal offset drift. Gated to IDLE + near-zero tared load so it never
        # interrupts a launch; load-independent + no re-tare needed (see driver).
        if (state.phase == protocol.PHASE_IDLE
                and time.ticks_diff(time.ticks_ms(), last_recal)
                    > ADC_RECAL_PERIOD_S * 1000
                and abs(state.force_raw - state.force_offset)
                    < ADC_RECAL_QUIET_COUNTS):
            try:
                adc.calibrate_offset()
            except OSError:
                state.force_errors += 1
            last_recal = time.ticks_ms()


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
                   "%.1f,%.1f,%.1f,%.1f,%.2f\n" % (
                       now, utc_ms, accel[0], accel[1], accel[2], gyro[0], gyro[1],
                       gyro[2], mx, my, mz, state.force_raw, state.pressure_hpa,
                       state.baro_alt_m, state.climb_rate_ms, state.alt_m,
                       state.lat, state.lon, state.gps_fix, state.gps_sats,
                       state.ground_speed_ms, state.angle_deg,
                       state.glider_speed_ms, state.cable_length_m,
                       state.winch_dist_m, state.elevation_deg,
                       state.gps_hacc_m, state.gps_climb_ms))

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


def _fw_line(role, app_path):
    # One-time firmware fingerprint written to the log at boot, so a later
    # debugger can tell which build produced a log: MicroPython version + build
    # date, whether this build has deflate compression (only the custom Winchy
    # builds do), and whether the app is frozen into the image (app source is
    # absent from the filesystem). Written only at boot, not on the cap/offload
    # rotation, so the "header-only = no episodes" checks stay valid.
    import sys
    try:
        import deflate
        import io
        deflate.DeflateIO(io.BytesIO(), deflate.GZIP).write
        comp = "y"
    except (ImportError, AttributeError):
        comp = "n"
    try:
        os.stat(app_path)
        frozen = "n"
    except OSError:
        frozen = "y"
    return "# fw: %s | %s | deflate=%s frozen=%s\n" % (
        role, sys.version, comp, frozen)


def _reset_raw_file(rawf):
    """Close, delete, and reopen raw.csv with a fresh header; return the new
    handle. Shared by the offload-reset and the cap rotation."""
    try:
        rawf.close()
    except Exception:
        pass
    try:
        os.remove(RAW_LOG_PATH)
    except OSError:
        pass
    rawf = open(RAW_LOG_PATH, "a")
    rawf.write(RAW_LOG_HEADER)
    rawf.flush()
    return rawf


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
    rawf.write(_fw_line("rope", "winchy/app.py"))   # one-time fw fingerprint
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
                rawf = _reset_raw_file(rawf)
                raw_bytes = len(RAW_LOG_HEADER)
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
            if raw_bytes >= RAW_LOG_MAX_BYTES:
                # Rolling cap: rotate (drop the oldest) instead of halting, so a
                # stuck/failed upload can never silently stop recording. Keeps
                # the most recent ~RAW_LOG_MAX_BYTES.
                rawf = _reset_raw_file(rawf)
                raw_bytes = len(RAW_LOG_HEADER)
                unflushed = 0
                print("raw.csv hit cap; rotated (keeping recent data)")
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


def _save_last_fix(lat, lon, alt):
    try:
        with open(LAST_FIX_PATH, "w") as f:
            f.write('{"lat":%.6f,"lon":%.6f,"alt":%.1f}' % (lat, lon, alt))
    except OSError:
        pass


def _load_last_fix_alt():
    # Return the last persisted GPS MSL altitude (m), or None.
    try:
        import json
        return float(json.loads(open(LAST_FIX_PATH).read())["alt"])
    except (OSError, ValueError, KeyError):
        return None


async def baro_task(baro, state, vertical):
    last = time.ticks_ms()
    seeded = False
    qnh_far = 0          # consecutive GPS fixes whose alt is far from baro alt
    while True:
        pressure = await baro.apressure_hpa()
        now = time.ticks_ms()
        state.pressure_hpa = pressure
        state.baro_ts = now

        # One-time boot seed: with no GPS fix yet, derive QNH from the last known
        # field elevation (persisted to flash) + today's pressure, so the
        # absolute altitude is right immediately and weather-aware (no sea-level
        # assumption). Only falls back to standard pressure if there is no stored
        # fix (first boot ever). A real GPS fix below replaces it.
        if state.qnh_hpa == 0 and not seeded:
            seed_alt = _load_last_fix_alt()
            if seed_alt is not None:
                state.qnh_hpa = altitude.sea_level_pressure_hpa(pressure, seed_alt)
                print("Baro QNH seeded from last fix %.1f m -> %.1f hPa"
                      % (seed_alt, state.qnh_hpa))
            seeded = True

        gps_fresh = (state.gps_fix and time.ticks_diff(
            time.ticks_ms(), state.gps_ts) < BARO_CAL_GPS_MAX_AGE_MS)
        if (state.phase == protocol.PHASE_IDLE and gps_fresh
                and state.gps_sats >= QNH_CAL_MIN_SATS
                and state.gps_hacc_m <= QNH_CAL_MAX_HACC_M):
            ref = altitude.sea_level_pressure_hpa(pressure, state.alt_m)
            # Reject a transient bad fix: a GPS altitude far off the current
            # (seeded/calibrated) baro altitude is the large vertical error seen
            # right at first lock. Accept anyway only if it persists, which means
            # the unit really moved to a new field.
            if (abs(state.alt_m - state.baro_alt_m) > QNH_CAL_MAX_JUMP_M
                    and qnh_far < QNH_CAL_FAR_HITS):
                qnh_far += 1
            else:
                qnh_far = 0
                if not state.qnh_gps_cal:
                    # First good fix: snap QNH to truth (replaces seed/default)
                    # and re-seed the filter so the jump isn't read as a climb.
                    state.qnh_hpa = ref
                    state.qnh_gps_cal = True
                    vertical.alt = None
                    vertical.vrate = 0.0
                    print("Baro QNH from GPS: %.1f hPa (alt %.1f m, %d sat)"
                          % (ref, state.alt_m, state.gps_sats))
                else:
                    state.qnh_hpa += BARO_CAL_ALPHA * (ref - state.qnh_hpa)
        # Until a fix calibrates QNH, use the seed; absent any stored fix, fall
        # back to standard pressure so baro/climb still work.
        qnh = state.qnh_hpa or BARO_QNH_DEFAULT_HPA
        raw_alt = altitude.pressure_to_altitude_m(pressure, qnh)
        vertical.predict(time.ticks_diff(now, last) / 1000.0)
        vertical.update(raw_alt)
        # Publish the Kalman-filtered altitude, not the noisy per-sample ISA
        # value: removes sensor jitter while still tracking a real climb.
        # raw.csv still logs raw pressure_hpa for offline replay.
        state.baro_alt_m = vertical.alt
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
    # Guard a glitched GPS date: time.mktime() overflows a 32-bit machine word
    # for years beyond ~2068, which used to crash gps_task (crash-guard reset
    # loop). Reject implausible dates and never let a bad fix take the unit down.
    global _pps_armed
    if not (2024 <= y <= 2050 and 1 <= mo <= 12 and 1 <= d <= 31):
        return
    try:
        nxt = time.gmtime(time.mktime((y, mo, d, h, mi, s, 0, 0)) + 1)
    except (OverflowError, ValueError):
        return
    _pps_armed = (nxt[0], nxt[1], nxt[2], nxt[6], nxt[3], nxt[4], nxt[5], 0)


def _gps_alive(uart, timeout_ms=1500):
    """True if recognisable GPS framing is arriving at the current baud - used to
    confirm the high-baud switch took. Checked BEFORE (re)configuring, when the
    module is streaming its existing config, so accept EITHER UBX (b5 62, the
    NAV-PVT we set, retained in BBR) OR NMEA ($G, the factory default) - whichever
    the module currently emits. (Checking only NMEA broke once NMEA was turned
    off; checking only UBX broke right after a CFG change briefly stalled the UBX
    stream - hence the check now runs before configure, see gps_task.)"""
    t0 = time.ticks_ms()
    buf = b""
    while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
        n = uart.any()
        if n:
            buf += uart.read(n)
            if b"\xb5\x62" in buf or b"$G" in buf:
                return True
        time.sleep_ms(10)
    return False


def _gps_detect_baud():
    """Probe the module's actual baud and bring it to high baud; return the nav
    rate to use. The M10 baud is RAM-only and only persists in BBR while the
    backup domain stays powered - with no battery, a USB-unplug reset cold-starts
    the module at 9600 every time. Try high FIRST (the warm/BBR case), and only
    send the raise-baud command once the module is actually found at 9600 -
    sending it at the wrong baud (host@9600 to module@115200) injects line noise
    that breaks the next detection. _gps_alive accepts UBX or NMEA. Reused on a
    read-stall (see gps_task) to self-heal a boot race where the module was not
    streaming yet when first probed, or a baud revert."""
    rate = config.GPS_NAV_RATE_HZ
    board.gps_reopen(config.GPS_BAUD_HIGH)
    if not _gps_alive(board.gps_uart):               # not already at high baud
        board.gps_reopen(config.GPS_BAUD)            # try the 9600 cold default
        if _gps_alive(board.gps_uart):               # found at 9600 -> raise it
            gps_set_baud(board.gps_uart, config.GPS_BAUD_HIGH)
            board.gps_reopen(config.GPS_BAUD_HIGH)
            if not _gps_alive(board.gps_uart):        # raise didn't confirm -> stay
                board.gps_reopen(config.GPS_BAUD)
                rate = min(rate, 5)                  # 9600 can't carry high rate
    return rate


async def gps_task(state):
    global _pps_rtc
    rate = _gps_detect_baud()
    gps_configure(board.gps_uart, rate)              # configure at the live baud
    _assistnow_capture_identity()            # UBX identity for ZTP (before read loop)
    _assistnow_feed_stored(state)            # warm the GPS with any cached orbits
    reader = asyncio.StreamReader(board.gps_uart)
    rtc = RTC()
    _pps_rtc = rtc
    Pin(config.GPS_PPS, Pin.IN).irq(trigger=Pin.IRQ_RISING, handler=_on_pps)
    last_fix_save = None
    while True:
        try:
            cls, mid, payload = await asyncio.wait_for(read_ubx(reader), 8)
        except asyncio.TimeoutError:
            # No UBX for 8 s: the module may not have been streaming yet when
            # first probed (boot race), or its baud reverted (BBR lost on a
            # no-battery USB-unplug reset). Re-detect, reconfigure, carry on.
            print("GPS: no data for 8s, re-detecting baud")
            rate = _gps_detect_baud()
            gps_configure(board.gps_uart, rate)
            reader = asyncio.StreamReader(board.gps_uart)
            continue
        if cls != 0x01 or mid != 0x07:        # only UBX-NAV-PVT
            continue
        update = parse_nav_pvt(payload)
        if not update:
            continue
        state.gps_fix = update["fix"]
        state.gps_sats = update["sats"]
        state.gps_hdop = update["pdop"]       # pDOP, kept for reference/logging
        state.gps_hacc_m = update["hacc_m"]   # gate + dashboard use this now
        if state.gps_fix:
            state.lat = update["lat"]
            state.lon = update["lon"]
            state.alt_m = update["alt_m"]
            state.ground_speed_ms = update["gspeed_ms"]
            state.gps_climb_ms = update["climb_ms"]
        else:
            state.ground_speed_ms = 0.0       # no fix -> no speed (don't latch)
        state.gps_ts = time.ticks_ms()
        # Winch-relative geometry, once the winch has sent its position and we
        # have our own fix. Updates at the GPS rate (cheap trig); the 50 Hz
        # imu_task just logs the latest values. Reel-in rate is the offline
        # derivative of winch_dist_m, so it needs nothing here.
        if state.winch_pos_ts and state.gps_fix:
            rel = winch_relative(
                state.winch_lat, state.winch_lon, state.winch_alt_m,
                state.lat, state.lon, state.alt_m, GLIDER_HOOK_DIST_M)
            state.cable_length_m = rel["cable_length_m"]
            state.winch_dist_m = rel["slant_m"]
            state.elevation_deg = rel["elevation_deg"]
        # Persist the field position/elevation (throttled, IDLE, confident 3D
        # fix) so the next boot can seed QNH from the known elevation.
        if (state.phase == protocol.PHASE_IDLE and state.gps_fix
                and state.gps_sats >= LAST_FIX_MIN_SATS
                and (last_fix_save is None or time.ticks_diff(
                    time.ticks_ms(), last_fix_save) >= LAST_FIX_SAVE_S * 1000)):
            _save_last_fix(state.lat, state.lon, state.alt_m)
            last_fix_save = time.ticks_ms()
        # RTC: rough set at once (so we have time), PPS refines the second edge.
        if update["datetime"]:
            y, mo, d, h, mi, s = update["datetime"]
            # GPS time wins over an earlier NTP fallback (more accurate, and
            # PPS-disciplined below). Set the RTC unless GPS already owns it.
            if state.time_source != "gps":
                rtc.datetime((y, mo, d, 0, h, mi, s, 0))
                state.time_synced = True
                state.time_source = "gps"
                print("RTC synced from GPS: %04d-%02d-%02d %02d:%02d:%02dZ"
                      % (y, mo, d, h, mi, s))
            _pps_arm_next(y, mo, d, h, mi, s)


def _ntp_time_aid(state):
    # GPS time is preferred (more accurate + PPS-disciplined), so this only acts
    # before GPS has synced. When WiFi is up it uses NTP to (a) set the clock as
    # a fallback, so logs are timestamped even with no GPS fix, and (b) inject
    # coarse time into the GPS (UBX-MGA-INI-TIME) so it acquires a fix FASTER.
    # GPS later overrides both (gps_task sets time_source="gps"). Blocks briefly
    # (a UDP round-trip); callers gate it to IDLE, like the GitHub uploads.
    if state.time_source == "gps":
        return
    try:
        import ntptime
        ntptime.settime()                 # set the RTC to UTC from NTP
    except Exception as e:
        print("NTP time failed:", e)
        return
    tm = time.gmtime()
    if tm[0] < 2024:                       # sanity: reject a bogus result
        return
    if state.time_source != "gps":
        state.time_synced = True
        state.time_source = "ntp"
        print("RTC synced from NTP: %04d-%02d-%02d %02d:%02d:%02dZ" % tm[:6])
    try:                                   # speed up the GPS fix with coarse time
        gps_ini_time(board.gps_uart, tm[:6])
        print("GPS time-aided from NTP")
    except Exception as e:
        print("GPS time-aid failed:", e)


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


async def button_task(pmu):
    # Long-press the AXP2101 power key -> turn the rope off. Handled in software
    # (polling the latched PKEY IRQs) so the charging LED is switched off *before*
    # shutdown - LED on = rope on, off = rope off.
    #
    # Power off only AFTER the key is RELEASED. The AXP2101 powers ON whenever the
    # key is held low for setPowerKeyPressOnTime (2 s, board.py), so calling
    # shutdown() while the key is still held lets that press immediately re-power
    # the system and it restarts. So on the long-press we switch the LED off
    # (impending shutdown) and then wait for the positive (release) edge before
    # shutting down - by then the key is high and the unit stays off.
    while True:
        await asyncio.sleep_ms(BUTTON_POLL_MS)
        try:
            pmu.getIrqStatus()              # refresh the latched IRQ flags
            if pmu.isPekeyLongPressIrq():
                pmu.setChargingLedMode(pmu.XPOWERS_CHG_LED_OFF)  # LED off = powering down
                pmu.clearIrqStatus()
                while True:                 # defer shutdown until the key is up
                    await asyncio.sleep_ms(BUTTON_POLL_MS)
                    pmu.getIrqStatus()
                    if pmu.isPekeyPositiveIrq():   # key released
                        pmu.clearIrqStatus()
                        pmu.shutdown()      # key is high now -> stays off
                        break
        except Exception:
            pass


def _log_stamp():
    # "yyyymmdd-hhmm" from the UTC RTC, or None if the clock isn't set yet.
    t = time.localtime()
    if t[0] < 2024:
        return None
    return "%04d%02d%02d-%02d%02d" % (t[0], t[1], t[2], t[3], t[4])


def _gzip_file(path):
    # Return (data, ext, content_type). Gzip the file if this build's deflate
    # supports compression; otherwise upload the raw CSV. This ESP32-S3
    # MicroPython is decompress-only: `deflate` imports fine but DeflateIO has
    # no .write, so the compress path raises AttributeError - fall back to raw
    # (the original behaviour; the rope uploads 4 MB this way fine).
    try:
        import deflate
        import io
        buf = io.BytesIO()
        comp = deflate.DeflateIO(buf, deflate.GZIP)
        comp.write            # AttributeError if compression isn't compiled in
    except (ImportError, AttributeError):
        return open(path, "rb").read(), "", "text/csv"
    f = open(path, "rb")
    try:
        while True:
            chunk = f.read(4096)
            if not chunk:
                break
            comp.write(chunk)
        comp.close()          # flush the gzip trailer
    finally:
        f.close()
    return buf.getvalue(), ".gz", "application/gzip"


def _github_upload_raw(asset):
    # Replace the rolling rope-log asset on the 'logs' release: look up the
    # release, delete any existing asset of the same name, then upload the log
    # (gzip-compressed). Blocks the asyncio loop for the round-trips (~secs) -
    # only called while IDLE, so the launch path is never affected.
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
        gc.collect()
        body, ext, ctype = _gzip_file(RAW_LOG_PATH)
        name = asset + ext
        for a in rel.get("assets", ()):
            if a.get("name") == name:
                urequests.delete(
                    "https://api.github.com/repos/%s/releases/assets/%d"
                    % (GITHUB_REPO, a["id"]), headers=hdr).close()
        gc.collect()
        h2 = dict(hdr)
        h2["Content-Type"] = ctype
        u = urequests.post(
            "https://uploads.github.com/repos/%s/releases/%d/assets?name=%s"
            % (GITHUB_REPO, rid, name), data=body, headers=h2)
        ok = 200 <= u.status_code < 300
        print("GitHub upload %s: HTTP %d (%d B)" % (name, u.status_code, len(body)))
        u.close()
    except Exception as e:
        print("GitHub upload failed:", e)
    gc.collect()
    return ok


def _github_announce_ip(ip, ssid, tag="rope"):
    # Publish a TAPPABLE dashboard link in the winchy-logs release description.
    # The release page renders the body as markdown, so on the phone you open
    # the release and tap the IP -> http://<ip>/ opens the dashboard. Each
    # segment owns one line ("rope: ..."/"winch: ..."); read-merge-write so we
    # don't clobber the other's line. Called on join / IP change (rare).
    import urequests
    import json as _json
    hdr = {"Authorization": "Bearer " + GITHUB_TOKEN, "User-Agent": "winchy",
           "Accept": "application/vnd.github+json"}
    line = "%s: [http://%s/](http://%s/) (%s, %s)" % (
        tag, ip, ip, ssid, _log_stamp() or "?")
    ok = False
    try:
        r = urequests.get("https://api.github.com/repos/%s/releases/tags/%s"
                          % (GITHUB_REPO, GITHUB_RELEASE_TAG), headers=hdr)
        rel = r.json()
        r.close()
        rid = rel["id"]
        body = rel.get("body") or ""
        keep = [ln for ln in body.split("\n")
                if ln.strip() and not ln.startswith(tag + ":")]
        keep.append(line)
        gc.collect()
        u = urequests.request(
            "PATCH", "https://api.github.com/repos/%s/releases/%d"
            % (GITHUB_REPO, rid),
            data=_json.dumps({"body": "\n".join(sorted(keep))}), headers=hdr)
        ok = 200 <= u.status_code < 300
        print("IP announce %s (%s): HTTP %d" % (ip, ssid, u.status_code))
        u.close()
    except Exception as e:
        print("IP announce failed:", e)
    gc.collect()
    return ok


_HTTP_MONTHS = ("Jan", "Feb", "Mar", "Apr", "May", "Jun",
                "Jul", "Aug", "Sep", "Oct", "Nov", "Dec")


def _parse_http_date(s):
    # "Mon, 22 Jun 2026 20:43:21 GMT" -> (y, mo, d, h, mi, s) UTC, or None.
    try:
        p = s.split()
        d, y = int(p[1]), int(p[3])
        mo = _HTTP_MONTHS.index(p[2]) + 1
        hh, mm, ss = (int(x) for x in p[4].split(":"))
        return (y, mo, d, hh, mm, ss)
    except (ValueError, IndexError, AttributeError):
        return None


def _assistnow_capture_identity():
    # Read the receiver's UBX-SEC-UNIQID + UBX-MON-VER (full frames, as hex) for
    # the one-time ZTP registration. Only needed before we have a chipcode and
    # only when a token is configured; done here (before the NMEA read loop) so
    # the binary replies aren't eaten by the line reader.
    global _gps_uniqid_hex, _gps_monver_hex
    if not UBLOX_ZTP_TOKEN:
        return
    try:
        os.stat(ASSISTNOW_CHIP_PATH)
        return                           # already registered -> no identity needed
    except OSError:
        pass
    import binascii
    uid = gps_poll_ubx(board.gps_uart, 0x27, 0x03)   # UBX-SEC-UNIQID
    ver = gps_poll_ubx(board.gps_uart, 0x0A, 0x04)   # UBX-MON-VER
    if uid:
        _gps_uniqid_hex = binascii.hexlify(uid).decode()
    if ver:
        _gps_monver_hex = binascii.hexlify(ver).decode()
    print("AssistNow ZTP: identity %s/%s"
          % ("ok" if uid else "?", "ok" if ver else "?"))


def _assistnow_feed_stored(state):
    # Feed the cached AssistNow blob (if any) to the GPS at boot so a cold start
    # uses predicted orbits even with no internet. Time is injected only if the
    # RTC is already set (usually not on a cold boot - then just the orbits go,
    # still used once the receiver self-acquires time-of-week).
    try:
        blob = open(ASSISTNOW_PATH, "rb").read()
    except OSError:
        return
    if not blob:
        return
    gc.collect()
    t = time.localtime()
    if state.time_synced and t[0] >= 2024:
        gps_ini_time(board.gps_uart, t[:6])
    gps_feed_mga(board.gps_uart, blob)
    print("AssistNow: fed %d B of cached predicted orbits to GPS" % len(blob))
    gc.collect()


def _assistnow_should_download(state):
    # Fetch if there is no cache, or (once we know the time) the cache is stale.
    try:
        os.stat(ASSISTNOW_PATH)
    except OSError:
        return True                      # no cache -> fetch
    if not state.time_synced:
        return False                     # have a cache, can't judge age yet
    try:
        ts = int(open(ASSISTNOW_TS_PATH).read())
    except (OSError, ValueError):
        return True                      # age unknown but we have time -> refresh
    return time.time() - ts > ASSISTNOW_MAX_AGE_S


def _assistnow_register():
    # One-time ZTP registration: POST the receiver identity + Device-Profile
    # token, cache the returned chipcode + data serviceUrl. Returns True on
    # success (chipcode cached). Token never logged.
    import urequests
    if not (_gps_uniqid_hex and _gps_monver_hex):
        print("AssistNow ZTP: no UBX identity captured, cannot register")
        return False
    body = ('{"token":"%s","messages":{"UBX-SEC-UNIQID":"%s",'
            '"UBX-MON-VER":"%s"}}'
            % (UBLOX_ZTP_TOKEN, _gps_uniqid_hex, _gps_monver_hex))
    try:
        gc.collect()
        r = urequests.post(ASSISTNOW_ZTP_URL, data=body,
                           headers={"Content-Type": "application/json"})
        code = r.status_code
        d = r.json() if 200 <= code < 300 else None
        r.close()
        if d and d.get("chipcode") and d.get("serviceUrl"):
            with open(ASSISTNOW_CHIP_PATH, "w") as f:
                f.write('{"chipcode":"%s","serviceUrl":"%s"}'
                        % (d["chipcode"], d["serviceUrl"]))
            print("AssistNow ZTP: registered, allowedData=%s"
                  % d.get("allowedData"))
            return True
        print("AssistNow ZTP: register HTTP %d" % code)
    except Exception as e:
        print("AssistNow ZTP register failed:", e)
    gc.collect()
    return False


def _assistnow_download(state):
    # Fetch predicted-orbit data over WiFi (registering first if needed), cache
    # it on flash, and feed it to the GPS (+ coarse time if the RTC is set).
    # Blocks the loop for the round-trip (~secs) - only called while IDLE.
    import urequests
    import json
    try:
        meta = json.loads(open(ASSISTNOW_CHIP_PATH).read())
    except (OSError, ValueError):
        if not _assistnow_register():
            return False
        try:
            meta = json.loads(open(ASSISTNOW_CHIP_PATH).read())
        except (OSError, ValueError):
            return False
    url = ("%s?chipcode=%s&gnss=%s&data=%s"
           % (meta["serviceUrl"], meta["chipcode"], ASSISTNOW_GNSS,
              ASSISTNOW_DATA))
    ok = False
    try:
        gc.collect()
        r = urequests.get(url)
        if r.status_code == 200:
            blob = r.content
            hdrs = getattr(r, "headers", None)            # server "Date" = trusted now
            srv = _parse_http_date(hdrs.get("Date")) if hdrs else None
            r.close()
            if blob:
                with open(ASSISTNOW_PATH, "wb") as f:
                    f.write(blob)
                # Coarse time aid + cache age-stamp. Prefer the server Date: it
                # works even with no GPS fix yet, lets the predicted orbits be
                # used immediately, and gives _assistnow_should_download a real
                # 'now' so a fresh cache isn't needlessly re-downloaded. Fall
                # back to the RTC if the header was missing.
                src = None
                if srv:
                    gps_ini_time(board.gps_uart, srv, acc_s=4)
                    try:
                        with open(ASSISTNOW_TS_PATH, "w") as f:
                            f.write(str(time.mktime(srv + (0, 0))))
                        src = "srv"
                    except (OSError, OverflowError):
                        pass
                elif state.time_synced and time.localtime()[0] >= 2024:
                    gps_ini_time(board.gps_uart, time.localtime()[:6])
                    try:
                        with open(ASSISTNOW_TS_PATH, "w") as f:
                            f.write(str(time.time()))
                        src = "rtc"
                    except OSError:
                        pass
                gc.collect()
                gps_feed_mga(board.gps_uart, blob)
                print("AssistNow: downloaded + fed %d B to GPS (%s time)"
                      % (len(blob), src or "no"))
                ok = True
        else:
            print("AssistNow: data HTTP %d" % r.status_code)
            r.close()
    except Exception as e:
        print("AssistNow download failed:", e)
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
                if state.time_source is None:   # NTP fallback + GPS time-aiding
                    _ntp_time_aid(state)
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


async def dashboard_task(state):
    # Continuous-WiFi status dashboard for walk/ground tests (the rope OLED is
    # disabled). Unlike wifi_task it keeps WiFi UP (no duty-cycling), serves the
    # page (winchy/dashboard.py), rejoins if dropped, and uploads raw.csv
    # opportunistically while IDLE. Battery cost is accepted for now; restore
    # power saving for flight by setting config.ROPE_DASHBOARD = False.
    global _assistnow_done
    import network
    dashboard.state = state
    try:
        network.hostname("winchy-rope")   # mDNS may still advertise the default
    except Exception:
        pass
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    server_started = False
    announced_ip = None
    n = 0
    while True:
        if not wlan.isconnected() and n % 30 == 0:   # (re)join when down
            joined = await wifi.connect_any(wlan, WIFI_NETWORKS,
                                            WIFI_JOIN_TIMEOUT_S)
            if joined:
                print("Rope dashboard WiFi '%s' joined - http://%s/"
                      % (joined, wlan.ifconfig()[0]))
                if not server_started:
                    await asyncio.start_server(dashboard.handle, "0.0.0.0", 80)
                    server_started = True
        # Announce our IP as a tappable dashboard link in the winchy-logs
        # release body, so it's findable on any subnet. On IP change only.
        if (GITHUB_TOKEN and wlan.isconnected()
                and state.phase == protocol.PHASE_IDLE):
            cur = wlan.ifconfig()[0]
            if cur and cur != "0.0.0.0" and cur != announced_ip:
                try:
                    ssid = wlan.config("essid")
                except Exception:
                    ssid = "?"
                if _github_announce_ip(cur, ssid, "rope"):
                    announced_ip = cur
        # NTP time fallback + GPS time-aiding: while WiFi is up and neither GPS
        # nor NTP has set the clock yet, sync from NTP (so logs are timestamped
        # even without a GPS fix) and inject coarse time into the GPS to speed up
        # its fix. GPS overrides later (gps_task). IDLE only; retried until set.
        if (wlan.isconnected() and state.time_source is None
                and state.phase == protocol.PHASE_IDLE and n % 10 == 0):
            _ntp_time_aid(state)
        # Opportunistic AssistNow Predictive Orbits: while IDLE + online, fetch
        # predicted orbits and feed them to the GPS (cached for the next cold
        # start). First call also does the one-time ZTP registration.
        if (UBLOX_ZTP_TOKEN and not _assistnow_done
                and wlan.isconnected() and state.phase == protocol.PHASE_IDLE):
            if _assistnow_should_download(state):
                _assistnow_download(state)
                _assistnow_done = True       # one attempt per boot
            elif state.time_synced:
                _assistnow_done = True        # cache confirmed fresh
            # else: have a cache but no time yet -> re-check on a later loop
        # GitHub upload of raw.csv (WiFi up, IDLE): the periodic auto-offload
        # plus a manual trigger from the dashboard "Upload log" button.
        if (GITHUB_TOKEN and wlan.isconnected()
                and state.phase == protocol.PHASE_IDLE
                and (state.upload_request or (n and n % WIFI_PERIOD_S == 0))):
            try:
                size = os.stat(RAW_LOG_PATH)[6]
            except OSError:
                size = 0
            if size > len(RAW_LOG_HEADER):
                stamp = state.log_start or _log_stamp()
                asset = (stamp + "_" if stamp else "") + GITHUB_ASSET
                if _github_upload_raw(asset):
                    state.raw_uploaded_bytes = size
            state.upload_request = False   # clear the manual request once handled
        n += 1
        await asyncio.sleep_ms(1000)


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
        button_task(pmu),
    ]
    if display:
        tasks.append(display_task(display, state))
    if WIFI_ENABLED and WIFI_NETWORKS:
        if config.ROPE_DASHBOARD:
            tasks.append(dashboard_task(state))   # continuous WiFi + status page
        elif GITHUB_TOKEN:
            tasks.append(wifi_task(state))        # duty-cycled upload-only
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
                  gain1=config.ADS_GAIN1, gain=config.ADS_GAIN,
                  speed=config.ADS_SPEED)
    if config.ADS_SPEED is not None:
        adc.set_speed(config.ADS_SPEED_HZ)
        print("Force ADC data rate %d SPS" % config.ADS_SPEED_HZ)
    try:
        adc.calibrate_offset()   # zero the ADC internal offset before taring
        print("Force ADC offset-calibrated")
    except OSError as e:
        print("Force ADC offset cal skipped:", e)
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
