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
import machine
import math
import micropython
import os
import time

from machine import Pin, RTC, SPI
import sh1106
# Radio: the official micropython-lib lora driver (lora-sx126x + lora-async).
# Replaced micropySX126X (2026-07-06): that driver ran its ENTIRE IRQ handling
# (getIrqStatus + startReceive re-arm + user callback, each an SPI transaction
# with sleep_ms busy-waits) inside the scheduled DIO1 pin interrupt, while
# telemetry_task did SPI from the main context - an unsynchronized dual-context
# bus with a driver-made preemption window (sleep_ms services the scheduler
# MID-TRANSACTION, CS low). Bisect-proven source of the stochastic WDT/PANIC
# (MTBF ~15-20 min, needed only TX + TX_DONE IRQ). The official driver's ISR
# only sets a flag; ALL SPI happens in asyncio context (single-context radio).
from lora import AsyncSX1262

import adr
import config
import crossupload
import eventlog
import imubias
import logtail
import protocol
import wifi
import gpstime
from winchy import board, dashboard
from winchy.fusion import altitude
from winchy.fusion.attitude import rope_angle_above_ground
from winchy.fusion.geometry import winch_relative
from winchy.fusion.kalman import GravityKalman, VerticalKalman
from winchy.fusion.speed import glider_speed
from winchy.sensors.ads1232 import ADS1232
from winchy.sensors.barometer import Barometer
from winchy.sensors.gps import (GPS, parse_nav_pvt, read_ubx, has_gps_frame,
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
# Live gyro-bias learning moved to shared/imubias.BiasTracker (2026-07-09,
# field-test-#2 fix): the old accel-norm gate (|a|-1 < 0.05) never opened in
# some orientations (per-axis accel scale error puts the rest norm at up to
# ~1.06 g), so a poisoned boot bias could never heal. The tracker gates on
# raw-gyro VARIANCE instead - bias-independent, heals within seconds of rest.
GYRO_CAL_PATH = "gyro.cal"    # last good boot bias (JSON [x, y, z] dps)
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
TELEMETRY_PRINT_EVERY = 10  # verbose status prints every Nth frame (~5 s):
                            # USB-CDC console writes BLOCK the loop, and six
                            # lines per 500 ms measurably stole IMU cadence
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
# each small chunk still blocks briefly, not one ~330 ms batch.)
#
# DURING a motion episode the writer STREAMS in extra-small chunks instead of
# deferring everything to RAM: field tests 2026-07-03/04 lost entire rides
# because the RAM-held episode died with the power (contact bounce / power-off
# before the 5 s rest drain), and RAW_Q_MAX silently truncated episodes >80 s.
# A 2-row pass blocks ~6 ms - under the 20 ms sample period - and the Kalman
# filters use the real per-sample dt, so sampling quality is preserved. Set
# RAW_STREAM_WHILE_RECORDING = False for the legacy full deferral (zero writes
# while recording; episode is lost on power failure).
RAW_STREAM_WHILE_RECORDING = True
RAW_WRITE_CHUNK = 8         # rows written per writer pass before it yields (idle)
RAW_WRITE_CHUNK_REC = 2     # rows per pass while an episode records (~6 ms block)
RAW_FLUSH_EVERY_REC = 50    # tighter flush cadence while recording: power loss
                            # costs <= ~1-2 s of rows instead of the episode
RAW_WRITE_IDLE_MS = 50      # writer poll interval while idle / deferring
RAW_GC_EVERY = 20           # gc.collect() every N flushes (skipped mid-episode:
                            # a GC pause of 10-50 ms is the biggest single stall)
RAW_Q_MAX = 4000            # safety cap on the pending-write queue; with
                            # streaming it is a net, not a limit (was ~80 s cap)
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
WIFI_ROAM_PERIOD_S = 60     # check this often for a higher-priority network
                            # (field bug 2026-07-07: latched onto the Airbus
                            # site AP, never switched to the phone hotspot)
# Liveness: wlan.isconnected() can LIE (zombie state, bench 2026-07-07:
# ESSID/IP/RSSI healthy, no packet moves, rejoin never fires). Served
# requests and HTTPS responses refresh state.net_alive_ms; once that goes
# stale a tiny DNS probe to the gateway checks the truth, and repeated
# failures force a full interface reset + rejoin.
WIFI_LIVENESS_STALE_S = 60  # no proven traffic this long -> start probing
WIFI_PROBE_RETRY_S = 10     # spacing between probes while stale
WIFI_PROBE_FAILS_ZOMBIE = 2  # consecutive probe failures -> interface reset

# Persistent event log (events.log, bounded, served at /events): WiFi joins/
# drops/roams/zombies, upload results, boot reasons. Field test #2 was
# diagnosed nearly blind because all of this lived only on the console.
_events = eventlog.EventLog()
_event = _events.log        # never raises (guarded inside EventLog)
GITHUB_REPO = "dspreitz/winchy-logs"
GITHUB_RELEASE_TAG = "logs"
GITHUB_ASSET = "rope_rawlog.csv"
GITHUB_TIMEOUT_S = 30   # socket timeout on the (blocking) upload round-trips, so
                        # a stalled connection can't freeze the asyncio loop.
                        # NOTE: does NOT cover DNS - getaddrinfo runs before the
                        # socket exists. A dead DNS still stalls the loop, but
                        # lwIP bounds it (~14 s with retries), so no forever-hang.
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
ASSISTNOW_TIMEOUT_S = 20    # socket timeout on the (blocking) AssistNow round-trips,
                            # so a stalled server can't freeze the asyncio loop
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
# Re-enabled 2026-07-07 after the WDT-panic hunt concluded: the panics came
# from the old radio driver's IRQ architecture (SPI inside the scheduled DIO1
# interrupt), not from ADR itself. With the official single-context driver
# the power change is a plain standby() + configure() from asyncio context -
# no IRQ collision possible. Distance-aware hardening (shared/adr.py) caps
# power on the bench so the RF-saturation latch-up cannot recur.
ADR_ENABLED = True
ADR_TX_POWER_MIN_DBM = -9     # SX1262 floor (driver clamps below this)
ADR_TX_POWER_MAX_DBM = 22     # SX1262 max (+22 dBm / 158 mW), within g3's 500 mW
                              # ERP cap. OCP: the SX1262 reset default is
                              # already 140 mA, enough for +22 dBm.
                              # PA can't deliver it. Battery cost on the rope is
                              # ~7 mA average (118 mA x ~6% TX duty).
ADR_RSSI_LOW_DBM = -90        # winch RSSI below this -> raise (keeps ~20 dB above
                              # the ~-110 dBm SF7/BW500 sensitivity floor)
ADR_RSSI_HIGH_DBM = -70       # winch RSSI above this -> trim (margin to spare)
ADR_STEP_UP_DB = 4            # raise quickly to restore margin
ADR_STEP_DOWN_DB = 1          # trim gently
ADR_LOSS_HIGH_PCT = 20        # loss above this -> jump to max
ADR_REPORT_TIMEOUT_MS = 4000  # no fresh report this long -> jump to max
# Distance-aware hardening (bench night 2026-07-06: blind fail-loud latched
# both ends at +22 dBm in mutual RX saturation; see shared/adr.py header).
ADR_NEAR_M = 50               # closer than this + lossy -> assume saturation
ADR_TARGET_RSSI_DBM = -80     # distance cap aims for this at the winch
ADR_CAP_MARGIN_DB = 15        # fading/obstruction margin on top of FSPL
ADR_PROBE_START = 24          # stale decision cycles before the probe ladder
ADR_PROBE_HOLD = 16           # cycles per probe step (max -> mid -> min)


def _adr_next_power(state):
    """Next TX power (dBm). The decision itself lives in shared/adr.py (pure,
    host-tested - a regression here means a dead link at range); this wrapper
    supplies report freshness, the winch-distance prior and the stale-cycle
    count. winch_dist_m keeps its LAST value when the link is down - that is
    intended (the winch is parked), it is exactly what breaks the saturation
    latch-up."""
    fresh = (state.link_report_ts != 0 and time.ticks_diff(
        time.ticks_ms(), state.link_report_ts) <= ADR_REPORT_TIMEOUT_MS)
    if fresh:
        state.adr_stale_cycles = 0
    else:
        state.adr_stale_cycles += 1
    dist = (state.winch_dist_m
            if state.winch_pos_ts and state.winch_dist_m > 0 else None)
    return adr.next_tx_power(
        state.tx_power_dbm, fresh, state.link_loss_pct, state.link_rssi_dbm,
        power_min=ADR_TX_POWER_MIN_DBM, power_max=ADR_TX_POWER_MAX_DBM,
        rssi_low=ADR_RSSI_LOW_DBM, rssi_high=ADR_RSSI_HIGH_DBM,
        step_up=ADR_STEP_UP_DB, step_down=ADR_STEP_DOWN_DB,
        loss_high_pct=ADR_LOSS_HIGH_PCT,
        distance_m=dist, near_m=ADR_NEAR_M,
        target_rssi=ADR_TARGET_RSSI_DBM, margin_db=ADR_CAP_MARGIN_DB,
        stale_count=state.adr_stale_cycles,
        probe_start=ADR_PROBE_START, probe_hold=ADR_PROBE_HOLD)

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
# GPS time is only trusted when the receiver's own accuracy estimate (NAV-PVT
# tAcc) is under this. Before a real fix the M10 emits an AIDED time with a huge
# tAcc; ignoring it keeps the (correct) NTP clock and avoids per-frame churn.
GPS_TIME_MAX_TACC_NS = 1000000000   # 1 s
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
    # Defense in depth (all sensor tasks): one bad iteration must never kill
    # the task - under asyncio.gather a dead task takes the WHOLE app down
    # (crash-guard reboot), which mid-launch means ~30 s of telemetry loss.
    # Drop the iteration, keep sampling.
    last_recal = time.ticks_ms()
    while True:
        try:
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
        except Exception as e:
            state.force_errors += 1
            print("force task iteration error:", e)
            await asyncio.sleep_ms(100)


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
    tracker = imubias.BiasTracker(gyro_bias)
    bias = tracker.bias     # live list - the tracker updates it in place
    last = time.ticks_ms()
    ring = []              # rolling pre-roll: (t_ms, rowstr), idle only
    recording = False
    last_motion_ts = 0
    prev_angle = None      # for the rope-angle rate (glider speed)
    budget = getattr(imu, "DRAIN", 1)   # samples processed per wake (C ring: 6)
    while True:
        # Defense in depth: an SPI hiccup in one read must not kill the task
        # (under asyncio.gather that reboots the whole app - see force_task).
        t0 = time.ticks_ms()
        try:
          # Drain up to `budget` finished samples. With the winchy_fast C
          # sampler the timestamps come from the sampler itself, so the SAMPLE
          # SPACING stays hardware-exact even when this task wakes up late
          # behind other tasks' blocks; the backlog is caught up here.
          for _ in range(budget):
            smp = imu.next_sample()
            if smp is None:
                break                      # ring drained
            now, accel, gyro = smp
            dt = time.ticks_diff(now, last) / 1000.0
            if dt <= 0:                    # first sample / ticks quirk
                dt = IMU_PERIOD_MS / 1000.0
            last = now

            norm = math.sqrt(accel[0] ** 2 + accel[1] ** 2 + accel[2] ** 2)
            if state.phase == protocol.PHASE_IDLE:
                tracker.update(gyro)   # variance-gated, self-healing learner
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
            else:                          # segment 3-D speed; rotation-immune
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
        except Exception as e:
            print("imu task iteration error:", e)
            await asyncio.sleep_ms(IMU_PERIOD_MS)
            continue
        # Deadline cadence: sleep only the REMAINDER of the period (a fixed
        # sleep stacked on the work time stretched the cadence to ~30 ms).
        # With the C sampler this only paces the DRAIN - the sampling itself
        # is exact regardless. sleep_ms(0) still yields (no starvation).
        spent = time.ticks_diff(time.ticks_ms(), t0)
        await asyncio.sleep_ms(IMU_PERIOD_MS - spent if spent < IMU_PERIOD_MS else 0)


# machine.reset_cause() -> label. THE discriminator for unexplained reboots
# (crash.log only catches Python exceptions): PWRON also covers BROWNOUT
# (power dip / battery contact bounce), WDT covers C-level panics ("Guru
# Meditation") and hardware watchdogs, HARD is the EN/RST pin (button or a
# glitched line), SOFT is machine.reset() (crash guard / deploy).
_RESET_CAUSES = {machine.PWRON_RESET: "PWRON/BROWNOUT",
                 machine.HARD_RESET: "HARD(EN/RST pin)",
                 machine.WDT_RESET: "WDT/PANIC",
                 machine.DEEPSLEEP_RESET: "DEEPSLEEP",
                 machine.SOFT_RESET: "SOFT(machine.reset)"}


def reset_cause_str():
    c = machine.reset_cause()
    return "%s(%d)" % (_RESET_CAUSES.get(c, "?"), c)


def _fw_line(role, app_path):
    # One-time firmware fingerprint written to the log at boot, so a later
    # debugger can tell which build produced a log: MicroPython version + build
    # date, whether this build has deflate compression (only the custom Winchy
    # builds do), whether the app is frozen into the image (app source is
    # absent from the filesystem), and WHY the chip (re)booted (rst= - tells a
    # power dip from a panic from a pin glitch on the next field-test log).
    # Written only at boot, not on the cap/offload rotation, so the
    # "header-only = no episodes" checks stay valid.
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
    return "# fw: %s | %s | deflate=%s frozen=%s rst=%s\n" % (
        role, sys.version, comp, frozen, reset_cause_str())


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
    # A failed open/header write (flash full of OTHER files - the 4 MB cap only
    # bounds raw.csv itself) must disable raw logging, not kill the whole app
    # (under asyncio.gather a dead task = crash-guard reboot LOOP, since every
    # boot would retry the same failing write).
    try:
        rawf = open(RAW_LOG_PATH, "a")
        try:
            raw_bytes = os.stat(RAW_LOG_PATH)[6]   # cap across reboots (append)
        except OSError:
            raw_bytes = 0
        rawf.write(RAW_LOG_HEADER)
        rawf.write(_fw_line("rope", "winchy/app.py"))   # one-time fw fingerprint
        rawf.flush()
    except OSError as e:
        print("raw log DISABLED (open/header failed):", e)
        return
    unflushed = 0
    flushes = 0
    while True:
        # Same defense in depth as the sensor tasks: a flash-write error drops
        # the chunk (data loss) instead of killing the writer (reboot loop).
        try:
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
                    state.last_episode_start = None   # offsets now invalid
                    print("raw.csv offloaded; reset")
                state.raw_uploaded_bytes = 0

            # While a motion episode records, STREAM in extra-small chunks so
            # the rows reach flash continuously (a mid-episode power loss used
            # to erase the whole RAM-held episode - field tests 2026-07-03/04).
            # The legacy full deferral (zero writes while recording) is kept
            # behind RAW_STREAM_WHILE_RECORDING = False for launch captures
            # that must have absolute write silence.
            recording = RAW_LOG_MOTION_GATED and state.raw_recording
            if recording and not RAW_STREAM_WHILE_RECORDING:
                await asyncio.sleep_ms(RAW_WRITE_IDLE_MS)
                continue

            q = state.raw_q
            if not q:
                if unflushed:
                    rawf.flush()
                    unflushed = 0
                await asyncio.sleep_ms(RAW_WRITE_IDLE_MS)
                continue
            chunk_n = RAW_WRITE_CHUNK_REC if recording else RAW_WRITE_CHUNK
            chunk = q[:chunk_n]            # atomic slice+del (no await between)
            del q[:chunk_n]
            for r in chunk:
                if raw_bytes >= RAW_LOG_MAX_BYTES:
                    # Rolling cap: rotate (drop the oldest) instead of halting, so a
                    # stuck/failed upload can never silently stop recording. Keeps
                    # the most recent ~RAW_LOG_MAX_BYTES.
                    rawf = _reset_raw_file(rawf)
                    raw_bytes = len(RAW_LOG_HEADER)
                    unflushed = 0
                    state.last_episode_start = None   # offsets now invalid
                    print("raw.csv hit cap; rotated (keeping recent data)")
                if r.startswith("# motion-start"):
                    # Remember where the newest episode begins - the manual
                    # "Upload last ride" sends the file from here.
                    state.last_episode_start = raw_bytes
                rawf.write(r)
                raw_bytes += len(r)
                unflushed += 1
            if unflushed >= (RAW_FLUSH_EVERY_REC if recording
                             else RAW_LOG_FLUSH_EVERY):
                rawf.flush()
                unflushed = 0
                flushes += 1
                # GC only while idle: its 10-50 ms pause is the biggest single
                # stall and must not land inside a recorded episode.
                if flushes % RAW_GC_EVERY == 0 and not recording:
                    gc.collect()
        except Exception as e:
            print("raw writer iteration error:", e)
            await asyncio.sleep_ms(1000)
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
        # Defense in depth: the BMP280 status reads inside apressure_hpa are
        # raw I2C - one bus hiccup must not kill the task (see force_task).
        try:
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
        except Exception as e:
            print("baro task iteration error:", e)
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
    try:
        micropython.schedule(_pps_apply, 0)
    except RuntimeError:
        pass          # schedule queue full - skip this edge (PPS retries at 1 Hz)


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


async def _gps_alive(uart, timeout_ms=1500):
    """True if recognisable, CHECKSUM-VALID GPS framing is arriving at the
    current baud - used to confirm a baud before configuring. Accepts EITHER a
    valid UBX frame (the NAV-PVT we set, retained in BBR) OR a valid NMEA
    sentence (the factory default) - whichever the module currently emits.
    A bare 2-byte sync match is NOT enough: it turns up ~10% of the time in the
    garbage read at the WRONG baud, which used to false-positive the high-baud
    probe and leave the app/module baud mismatched (no sats). See
    gps.has_gps_frame; the check runs BEFORE configure (see gps_task).
    ASYNC (was time.sleep_ms): the re-detect runs mid-flight after a read
    stall, and a blocking 0.6-1.5 s probe froze telemetry + dashboard."""
    t0 = time.ticks_ms()
    buf = b""
    while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
        n = uart.any()
        if n:
            buf += uart.read(n)
            if len(buf) > 600:
                buf = buf[-600:]            # bound the scan; a frame is <=~100 B
            if has_gps_frame(buf):
                return True
        await asyncio.sleep_ms(10)
    return False


async def _gps_detect_baud():
    """Probe the module's actual baud, bring it to high baud, and return the nav
    rate. PATIENT: right after power-up the module isn't streaming yet (and with
    no battery a USB-unplug reset cold-starts it at 9600 every time), so the
    probe is retried while it boots - awaiting between tries so the rest of the
    app keeps running. Try high FIRST (the warm/BBR case); only send the raise
    command once the module is actually found at 9600 (sending it at the wrong
    baud injects line noise that breaks the next detection). After a raise the
    next pass re-probes high, so a switched-but-not-yet-confirmed module is still
    caught. _gps_alive accepts UBX or NMEA. Reused by gps_task on a read-stall."""
    rate = config.GPS_NAV_RATE_HZ
    for _ in range(12):                              # retry while the GPS powers up
        board.gps_reopen(config.GPS_BAUD_HIGH)
        if await _gps_alive(board.gps_uart, 600):
            return rate                              # already at high baud
        board.gps_reopen(config.GPS_BAUD)
        if await _gps_alive(board.gps_uart, 600):    # found at 9600 -> raise it
            gps_set_baud(board.gps_uart, config.GPS_BAUD_HIGH)
            await asyncio.sleep_ms(400)              # let the baud switch settle
            board.gps_reopen(config.GPS_BAUD_HIGH)
            if await _gps_alive(board.gps_uart, 900):
                return rate                          # raised + confirmed
            # not confirmed yet -> loop; the next pass probes high first
        await asyncio.sleep_ms(500)                  # GPS not responding yet -> wait
    board.gps_reopen(config.GPS_BAUD_HIGH)           # gave up -> assume high baud
    print("GPS: baud probe gave up; assuming %d" % config.GPS_BAUD_HIGH)
    return rate


async def gps_task(state):
    global _pps_rtc
    rate = await _gps_detect_baud()
    gps_configure(board.gps_uart, rate)              # configure at the live baud
    _assistnow_capture_identity()            # UBX identity for ZTP (before read loop)
    _assistnow_feed_stored(state)            # warm the GPS with any cached orbits
    reader = asyncio.StreamReader(board.gps_uart)
    rtc = RTC()
    _pps_rtc = rtc
    Pin(config.GPS_PPS, Pin.IN).irq(trigger=Pin.IRQ_RISING, handler=_on_pps)
    last_fix_save = None
    while True:
        # Defense in depth: only read_ubx's timeout was guarded before; an
        # unexpected exception anywhere else (parse, QNH save, geometry, time)
        # killed the task and rebooted the whole app (see force_task).
        try:
            try:
                cls, mid, payload = await asyncio.wait_for(read_ubx(reader), 8)
            except asyncio.TimeoutError:
                # No UBX for 8 s: the module may not have been streaming yet when
                # first probed (boot race), or its baud reverted (BBR lost on a
                # no-battery USB-unplug reset). Re-detect, reconfigure, carry on.
                print("GPS: no data for 8s, re-detecting baud")
                rate = await _gps_detect_baud()
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
            # RTC from GPS, but DEFENSIVELY (gpstime.time_fix_decision): a glitched
            # "fullyResolved" NAV-PVT can be minutes wrong (seen: ~37 min off, then
            # latched for the session, overriding correct NTP). Only CONFIDENT times
            # are considered (small tAcc - the M10 emits an aided time with a huge
            # tAcc before a real fix); then cross-check vs an NTP-set clock, require a
            # consistent 2nd frame with no clock, and re-sync on drift. PPS refines.
            if update["datetime"] and update.get("t_acc_ns", 0) <= GPS_TIME_MAX_TACC_NS:
                y, mo, d, h, mi, s = update["datetime"]
                gps_epoch = time.mktime((y, mo, d, h, mi, s, 0, 0))
                action, state.gps_time_cand = gpstime.time_fix_decision(
                    state.time_source, gps_epoch, time.time(), update["t_acc_ns"],
                    state.gps_time_cand, GPS_TIME_MAX_TACC_NS)
                if action == "set":
                    rtc.datetime((y, mo, d, 0, h, mi, s, 0))
                    print("RTC %s from GPS: %04d-%02d-%02d %02d:%02d:%02dZ"
                          % ("re-synced" if state.time_source == "gps" else "synced",
                             y, mo, d, h, mi, s))
                    state.time_synced = True
                    state.time_source = "gps"
                    _pps_arm_next(y, mo, d, h, mi, s)
                elif action == "arm":
                    _pps_arm_next(y, mo, d, h, mi, s)   # agrees -> PPS stays disciplined
                elif (action == "reject" and state.time_source == "ntp"
                        and time.ticks_diff(time.ticks_ms(),
                                            state.gps_time_warn_ms) > 30000):
                    state.gps_time_warn_ms = time.ticks_ms()   # rate-limited: rare
                    print("GPS time rejected: %+ds vs NTP (confident but disagrees)"
                          % (gps_epoch - time.time()))
        except Exception as e:
            print("gps task iteration error:", e)
            await asyncio.sleep_ms(200)


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


def _cross_tx_frame(state, seq):
    # One cross-upload radio frame to send INSTEAD of telemetry this cycle (rare;
    # one TX/cycle keeps the half-duplex sequence clean, a dropped telemetry
    # frame is harmless). The decision lives in shared/crossupload.py (pure,
    # host-tested). Returns a frame or None.
    now = time.ticks_ms()
    kind, nonce, tries, done = crossupload.tx_plan(
        state.cross_ack_nonce, state.cross_cmd_nonce, state.cross_cmd_tries,
        time.ticks_diff(now, state.cross_cmd_ts))
    if kind == "ack":
        state.cross_ack_nonce = None
        return protocol.encode_upload_ack(seq, nonce)
    if kind == "cmd":
        state.cross_cmd_ts = now
        state.cross_cmd_tries = tries
        if done:
            state.cross_cmd_nonce = None       # gave up after this final send
        return protocol.encode_upload_cmd(seq, nonce)
    return None


async def radio_task(sx, state):
    """All radio RX handling, in plain asyncio context (single-context radio:
    the driver's DIO1 ISR only sets a ThreadSafeFlag; every SPI byte moves
    from here or telemetry_task, and asyncio tasks only yield at await points,
    so driver calls are atomic - no locks needed). recv() keeps the modem
    listening; a send() from telemetry_task suspends RX and the driver
    resumes it automatically after TX_DONE."""
    while True:
        try:
            rx = await sx.recv(None)     # continuous listen, IRQ-woken
        except Exception as e:
            print("radio recv error:", e)
            await asyncio.sleep_ms(250)
            continue
        if not rx:
            continue
        try:
            msg = protocol.decode(bytes(rx))
        except Exception:
            msg = None
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
        elif msg and msg["type"] == protocol.UPLOAD_CMD:
            # Winch asked us to upload too. ACK every copy (telemetry_task
            # sends it); trigger our upload once per nonce - the dedup
            # EXPIRES (crossupload.accept_cmd) so a rebooted winch's
            # reused nonce still triggers.
            now = time.ticks_ms()
            if crossupload.accept_cmd(
                    msg["nonce"], state.cross_last_cmd,
                    time.ticks_diff(now, state.cross_last_cmd_ts)):
                state.cross_last_cmd = msg["nonce"]
                state.cross_last_cmd_ts = now
                state.upload_request = True
                print("Cross-upload requested by winch (nonce %d)"
                      % msg["nonce"])
            state.cross_ack_nonce = msg["nonce"]
        elif msg and msg["type"] == protocol.UPLOAD_ACK:
            if msg["nonce"] == state.cross_cmd_nonce:
                state.cross_cmd_nonce = None      # winch got it; stop retry
                print("Cross-upload ACKed by winch (nonce %d)"
                      % msg["nonce"])
        else:
            print("RX ?: %s (rssi=%s)" % (bytes(rx), rx.rssi))


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
        # Verbose status only every Nth frame: console writes block the loop
        # and were part of what kept the IMU cadence above its 20 ms target.
        verbose = frame_count % TELEMETRY_PRINT_EVERY == 0
        if verbose:
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
                # configure() refuses while receiving: stop RX first, the
                # radio_task's recv() then returns None and re-arms listen.
                # No IRQ-collision guard needed anymore (single-context radio).
                try:
                    sx.standby()
                    sx.configure({"output_power": new_power})
                    print("ADR: tx power %d -> %d dBm (snr=%d loss=%d%%)"
                          % (state.tx_power_dbm, new_power, state.link_snr_db,
                             state.link_loss_pct))
                    state.tx_power_dbm = new_power
                except Exception as e:
                    print("ADR power change deferred:", e)
        # Cross-upload: a pending UPLOAD_ACK / retry UPLOAD_CMD preempts this
        # telemetry frame (one TX per cycle; a dropped telemetry frame is fine).
        cross = _cross_tx_frame(state, seq)
        if cross is not None:
            frame = cross
        # await returns once TX_DONE fired; the driver then resumes the
        # radio_task's suspended RX automatically, so the winch's reply is
        # caught with no explicit listen window; we just dwell longer on
        # report cycles so it isn't clobbered.
        try:
            await sx.send(frame)
        except Exception as e:
            print("TX error:", e)   # drop this frame; next cycle retries
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
        if verbose:
            print("Sent packet (binary):", frame)
        await asyncio.sleep_ms(TELEMETRY_PERIOD_MS
                               + (REPORT_WINDOW_MS if report_requested else 0))


async def display_task(display, state):
    while True:
        try:                       # an I2C hiccup must not kill the task
            display.fill(0)
            display.text("F: {}".format(state.force_raw - state.force_offset), 0, 0)
            display.text("Seilwinkel:", 0, 16)
            display.text("{:.1f} deg".format(state.angle_deg), 0, 26)
            display.text("{:.0f}hPa {:.0f}m".format(state.pressure_hpa,
                                                    state.baro_alt_m), 0, 42)
            display.text("Sat:{} {}mV".format(state.gps_sats, state.system_mv),
                         0, 54)
            display.show()
        except Exception as e:
            print("display task iteration error:", e)
        await asyncio.sleep_ms(DISPLAY_PERIOD_MS)


async def supervisor_task(pmu, state):
    while True:
        try:      # the AXP2101 reads are SoftI2C - guard like the sensor tasks
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
        except Exception as e:
            print("supervisor task iteration error:", e)
        await asyncio.sleep_ms(SUPERVISOR_PERIOD_MS)


# NOTE: power-off is pure HARDWARE now (board.py enableLongPressShutdown):
# hold the key 6 s -> the AXP2101 powers off, even if the app is crashed or
# hung. The old software path (button_task: PKEY IRQ poll -> LED cue -> wait
# for release -> shutdown()) proved unreliable on battery-only power in the
# 2026-07-03 field test and was removed.


def _log_stamp():
    # "yyyymmdd-hhmm" from the UTC RTC, or None if the clock isn't set yet.
    t = time.localtime()
    if t[0] < 2024:
        return None
    return "%04d%02d%02d-%02d%02d" % (t[0], t[1], t[2], t[3], t[4])


def _upload_suffix():
    # Per-upload unique tag so each offloaded chunk becomes a DISTINCT asset
    # (never overwriting an earlier one). Seconds-resolution wall time when the
    # clock is set (uploads are always >1 s apart); a ticks-based tag otherwise.
    t = time.localtime()
    if t[0] >= 2024:
        return "%02d%02d%02d" % (t[3], t[4], t[5])
    return "x%06x" % (time.ticks_ms() & 0xFFFFFF)


# Upload plumbing, reworked after field test #2 (2026-07-07): the old path
# gzipped the WHOLE file into RAM and pushed it through blocking urequests -
# at ~4 MB over a phone hotspot that froze the asyncio loop (no TX, no
# dashboard) for MINUTES and then failed. Now the body is staged to a temp
# file and POSTed in small chunks with awaits between them, so telemetry and
# the dashboard stay live and the operator sees progress. Only DNS + the TLS
# handshake + the response read still block (seconds, socket-timeout-bounded).
UPLOAD_TMP = "upload.tmp"        # staged (gzipped) POST body; deleted after
UPLOAD_CHUNK = 2048              # stage/POST chunk size (yield between chunks)
UPLOAD_TAIL_CAP = 2 * 1024 * 1024   # ride upload: max bytes searched/sent

_release_id = None               # cached GitHub release id (also in a file, so
                                 # later boots skip the huge release JSON GET)
RELEASE_ID_PATH = "relid.txt"


def _github_release_id(hdr):
    """The 'logs' release id. Fetched at most once (the release JSON carries
    EVERY asset - a big parse on this chip), then cached in RAM + file."""
    global _release_id
    if _release_id is not None:
        return _release_id
    try:
        with open(RELEASE_ID_PATH) as f:
            _release_id = int(f.read().strip())
        return _release_id
    except (OSError, ValueError):
        pass
    import urequests
    r = urequests.get("https://api.github.com/repos/%s/releases/tags/%s"
                      % (GITHUB_REPO, GITHUB_RELEASE_TAG), headers=hdr,
                      timeout=GITHUB_TIMEOUT_S)
    rid = r.json()["id"]
    r.close()
    gc.collect()
    _release_id = rid
    try:
        with open(RELEASE_ID_PATH, "w") as f:
            f.write(str(rid))
    except OSError:
        pass
    return rid


def _drop_release_id():
    """Invalidate the cached id (a POST 404 means the release was recreated)."""
    global _release_id
    _release_id = None
    try:
        os.remove(RELEASE_ID_PATH)
    except OSError:
        pass


async def _stage_upload(path, start, size_cap):
    """Stage file[start : start+size_cap] into UPLOAD_TMP, gzipped when this
    build's deflate can compress (custom builds can; stock is decompress-only
    -> raw CSV fallback). Chunked with yields. Returns (bytes, ext, ctype).
    size_cap is the caller's size snapshot, so rows appended (or a rotation)
    during staging never bleed into the body."""
    src = open(path, "rb")
    dst = open(UPLOAD_TMP, "wb")
    comp = None
    ext, ctype = "", "text/csv"
    try:
        try:
            import deflate
            comp = deflate.DeflateIO(dst, deflate.GZIP)
            comp.write        # AttributeError when compression isn't built in
            ext, ctype = ".gz", "application/gzip"
        except (ImportError, AttributeError):
            comp = None
        src.seek(start)
        left = size_cap
        out = comp or dst
        while left > 0:
            chunk = src.read(UPLOAD_CHUNK if left > UPLOAD_CHUNK else left)
            if not chunk:
                break
            out.write(chunk)
            left -= len(chunk)
            await asyncio.sleep_ms(0)
        if comp:
            comp.close()      # flush the gzip trailer (dst closed below)
    finally:
        src.close()
        try:
            dst.close()
        except Exception:
            pass
    return os.stat(UPLOAD_TMP)[6], ext, ctype


def _resp_asset_size(body):
    """The 'size' field from GitHub's asset-created JSON, parsed with a plain
    byte scan - tolerant of chunked framing that would break json.loads."""
    i = body.find(b'"size"')
    if i < 0:
        return -1
    k = body.find(b":", i) + 1
    while k < len(body) and body[k:k + 1] in b" \t":
        k += 1
    n = -1
    while k < len(body) and 48 <= body[k] <= 57:
        n = (0 if n < 0 else n) * 10 + (body[k] - 48)
        k += 1
    return n


async def _https_post_file(host, path, ctype, body_path, body_len, state):
    """Streaming HTTPS POST of a staged file. Yields between chunks and
    publishes progress to state.upload_status. Returns (status, resp_body)."""
    import socket
    import tls
    ai = socket.getaddrinfo(host, 443)[0][-1]   # blocking DNS (no timeout!)
    s = socket.socket()
    s.settimeout(GITHUB_TIMEOUT_S)
    resp = b""
    try:
        s.connect(ai)
        ctx = tls.SSLContext(tls.PROTOCOL_TLS_CLIENT)
        ctx.verify_mode = tls.CERT_NONE          # same trust model as urequests
        s = ctx.wrap_socket(s, server_hostname=host)
        s.write(("POST %s HTTP/1.1\r\nHost: %s\r\n"
                 "Authorization: Bearer %s\r\nUser-Agent: winchy\r\n"
                 "Accept: application/vnd.github+json\r\n"
                 "Content-Type: %s\r\nContent-Length: %d\r\n"
                 "Connection: close\r\n\r\n"
                 % (path, host, GITHUB_TOKEN, ctype, body_len)).encode())
        sent = 0
        with open(body_path, "rb") as f:
            while True:
                chunk = f.read(UPLOAD_CHUNK)
                if not chunk:
                    break
                mv = memoryview(chunk)
                while len(mv):               # ssl write may be partial
                    n = s.write(mv)
                    mv = mv[len(mv) if n is None else n:]
                sent += len(chunk)
                state.upload_status = "up %d%%" % (100 * sent // body_len)
                await asyncio.sleep_ms(0)    # telemetry/dashboard slot
        while len(resp) < 8192:              # status + headers + small JSON
            b = s.read(1024)
            if not b:
                break
            resp += b
            await asyncio.sleep_ms(0)
    finally:
        try:
            s.close()
        except Exception:
            pass
    try:
        status = int(resp.split(b"\r\n", 1)[0].split(b" ")[1])
    except (IndexError, ValueError):
        status = 0
    parts = resp.split(b"\r\n\r\n", 1)
    return status, parts[1] if len(parts) > 1 else b""


async def _github_upload_raw(asset, start, size_cap, state):
    # Upload raw.csv[start : start+size_cap] as a (unique-named) asset on the
    # 'logs' release. Verification now comes from the POST RESPONSE itself
    # (GitHub returns the created asset incl. its stored size) - the old
    # read-back GET of the whole release JSON is gone.
    # Returns (ok, verified).
    hdr = {"Authorization": "Bearer " + GITHUB_TOKEN, "User-Agent": "winchy",
           "Accept": "application/vnd.github+json"}
    ok = False
    verified = False
    try:
        rid = _github_release_id(hdr)      # cached: blocks only the first time
        staged, ext, ctype = await _stage_upload(RAW_LOG_PATH, start, size_cap)
        name = asset + ext
        status, body = await _https_post_file(
            "uploads.github.com",
            "/repos/%s/releases/%d/assets?name=%s" % (GITHUB_REPO, rid, name),
            ctype, UPLOAD_TMP, staged, state)
        ok = 200 <= status < 300
        verified = ok and _resp_asset_size(body) == staged
        if status:                         # any response = proven traffic
            state.net_alive_ms = time.ticks_ms()
        print("GitHub upload %s: HTTP %d (%d B) %s"
              % (name, status, staged, "verified" if verified else ""))
        if status == 404:                  # release recreated -> refetch id
            _drop_release_id()
    except Exception as e:
        print("GitHub upload failed:", e)
    try:
        os.remove(UPLOAD_TMP)
    except OSError:
        pass
    gc.collect()
    return ok, verified


async def _run_upload(state, ride=False):
    # Drive one raw.csv upload + the dashboard status. Each upload goes to a
    # UNIQUE asset (so chunks are never overwritten), and the device flash is
    # only reclaimed (raw_uploaded_bytes) once the server copy is verified.
    # ride=True (dashboard "Upload last ride"): send only the LAST recorded
    # episode - the natural unit for a quick post-ride look in the field,
    # where the full file is minutes of hotspot uplink (field test #2).
    if state.uploading:                    # one already in flight -> ignore
        return
    try:
        size = os.stat(RAW_LOG_PATH)[6]
    except OSError:
        size = 0
    if size <= len(RAW_LOG_HEADER):        # header only - nothing to offload
        state.upload_status = "nodata"
        return
    start = 0
    base = GITHUB_ASSET
    if ride:
        base = "rope_ride.csv"
        # Writer-tracked offset (this boot); after a reboot fall back to a
        # backward scan for the last "# motion-start" marker, and with no
        # marker at all (continuous mode) to a line-aligned tail window.
        start = state.last_episode_start
        if start is None or start >= size:
            try:
                with open(RAW_LOG_PATH, "rb") as f:
                    start = logtail.last_marker_offset(
                        f, cap_bytes=UPLOAD_TAIL_CAP)
                    if start is None:
                        start = logtail.align_to_line(
                            f, size - UPLOAD_TAIL_CAP if
                            size > UPLOAD_TAIL_CAP else 0)
            except OSError:
                start = 0
    state.uploading = True
    state.upload_status = "uploading"
    mode = "ride" if ride else "full"
    _event("upload %s start (%d B)" % (mode, size - start))
    try:
        await asyncio.sleep_ms(600)        # let the dashboard push "uploading"
        stamp = state.log_start or _log_stamp() or "nogps"
        asset = stamp + "_" + _upload_suffix() + "_" + base
        ok, verified = await _github_upload_raw(asset, start, size - start,
                                                state)
        if ok and verified:
            if not ride:                   # a partial upload reclaims nothing
                state.raw_uploaded_bytes = size
            state.upload_status = "ok"
        elif ok:
            state.upload_status = "unverified"
        else:
            state.upload_status = "fail"
        _event("upload %s -> %s" % (mode, state.upload_status))
    finally:
        state.uploading = False


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
                          % (GITHUB_REPO, GITHUB_RELEASE_TAG), headers=hdr, timeout=GITHUB_TIMEOUT_S)
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
            data=_json.dumps({"body": "\n".join(sorted(keep))}), headers=hdr, timeout=GITHUB_TIMEOUT_S)
        ok = 200 <= u.status_code < 300
        print("IP announce %s (%s): HTTP %d" % (ip, ssid, u.status_code))
        u.close()
    except Exception as e:
        print("IP announce failed:", e)
    gc.collect()
    return ok


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
                           headers={"Content-Type": "application/json"},
                           timeout=ASSISTNOW_TIMEOUT_S)
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
        r = urequests.get(url, timeout=ASSISTNOW_TIMEOUT_S)
        if r.status_code == 200:
            blob = r.content
            hdrs = getattr(r, "headers", None)            # server "Date" = trusted now
            srv = gpstime.parse_http_date(hdrs.get("Date")) if hdrs else None
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
                print("WiFi '%s' joined (%s); uploading"
                      % (joined, wlan.ifconfig()[0]))
                await _run_upload(state)
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
    wifi.tune(wlan)          # pm=PM_NONE (server role) + driver auto-reconnect
    server_started = False
    announced_ip = None
    n = 0
    join_fails = 0           # consecutive failed joins -> interface reset at 3
    retry_at = None          # ticks_ms of the next join attempt; None = due now
    was_connected = False    # for the one-shot "wifi lost" event
    probe_at = None          # next liveness probe (while traffic-stale)
    probe_fails = 0
    while True:
        conn = wlan.isconnected()
        if was_connected and not conn:
            _event("wifi lost")
        was_connected = conn
        # (Re)join when down: FIRST attempt immediately, then backoff
        # 5 s -> 10 s -> 30 s (the old fixed ~30 s gate made hotspot handovers
        # take a minute). After 3 consecutive failures the interface is
        # power-cycled to clear a stuck supplicant.
        if not conn and (
                retry_at is None
                or time.ticks_diff(time.ticks_ms(), retry_at) >= 0):
            probe_fails = 0
            if join_fails >= 3:
                print("WiFi: interface reset after repeated join failures")
                _event("wifi iface reset (join failures)")
                await wifi.reset_iface(wlan)
                join_fails = 0
            joined = await wifi.connect_any(wlan, WIFI_NETWORKS,
                                            WIFI_JOIN_TIMEOUT_S)
            if joined:
                print("Rope dashboard WiFi '%s' joined - http://%s/"
                      % (joined, wlan.ifconfig()[0]))
                _event("wifi joined %s %s" % (joined, wlan.ifconfig()[0]))
                state.net_alive_ms = time.ticks_ms()
                was_connected = True
                join_fails = 0
                retry_at = None
                if not server_started:
                    await asyncio.start_server(dashboard.handle, "0.0.0.0", 80)
                    server_started = True
            else:
                join_fails += 1
                delay_ms = (5000, 10000, 30000)[min(join_fails - 1, 2)]
                retry_at = time.ticks_add(time.ticks_ms(), delay_ms)
                print("WiFi: join failed (%d), retry in %d s"
                      % (join_fails, delay_ms // 1000))
        # LIVENESS (see the constants above): probe the gateway once the
        # traffic proof goes stale; a repeatedly dead probe while
        # "connected" is the zombie -> full interface reset, then the
        # rejoin block above brings the link back for real.
        elif conn and (time.ticks_diff(time.ticks_ms(), state.net_alive_ms)
                       > WIFI_LIVENESS_STALE_S * 1000):
            if (probe_at is None
                    or time.ticks_diff(time.ticks_ms(), probe_at) >= 0):
                probe_at = time.ticks_add(time.ticks_ms(),
                                          WIFI_PROBE_RETRY_S * 1000)
                if wifi.probe_gateway(wlan):
                    state.net_alive_ms = time.ticks_ms()
                    probe_fails = 0
                    print("WiFi liveness probe ok")
                else:
                    probe_fails += 1
                    print("WiFi liveness probe FAILED (%d)" % probe_fails)
                    if probe_fails >= WIFI_PROBE_FAILS_ZOMBIE:
                        _event("wifi ZOMBIE (connected but dead) - reset")
                        await wifi.reset_iface(wlan)
                        probe_fails = 0
                        retry_at = None    # rejoin on the next iteration
        # HOLD OFF all blocking GitHub/NTP round-trips while the unit is MOVING
        # (a motion episode is recording). With a phone-hotspot riding along
        # (field tests) WiFi stays up during motion, and phase is still always
        # IDLE - so a periodic upload / IP announce used to fire MID-RIDE and
        # freeze the dashboard + stall the 50 Hz sampling for seconds. A manual
        # "Upload log" click still runs immediately (explicit user intent).
        still = not state.raw_recording
        # Roam UP the priority list when a better network comes in range
        # (the operator's hotspot is often enabled only after we already
        # joined a site AP - see roam_to_preferred). The scan stutters the
        # link for ~2 s, so: only while still + IDLE, and never when already
        # on the top-priority network (then it returns without scanning).
        if (wlan.isconnected() and still and state.phase == protocol.PHASE_IDLE
                and n and n % WIFI_ROAM_PERIOD_S == 0):
            roamed = await wifi.roam_to_preferred(wlan, WIFI_NETWORKS)
            if roamed:
                print("Rope dashboard WiFi roamed to '%s' - http://%s/"
                      % (roamed, wlan.ifconfig()[0]))
                _event("wifi roamed to %s %s" % (roamed, wlan.ifconfig()[0]))
                state.net_alive_ms = time.ticks_ms()
                # announced_ip is stale now; the announce below re-fires.
        # Announce our IP as a tappable dashboard link in the winchy-logs
        # release body, so it's findable on any subnet. On IP change only.
        if (GITHUB_TOKEN and wlan.isconnected() and still
                and state.phase == protocol.PHASE_IDLE):
            cur = wlan.ifconfig()[0]
            if cur and cur != "0.0.0.0" and cur != announced_ip:
                try:
                    ssid = wlan.config("essid")
                except Exception:
                    ssid = "?"
                if _github_announce_ip(cur, ssid, "rope"):
                    announced_ip = cur
                    _event("announced %s (%s)" % (cur, ssid))
                    state.net_alive_ms = time.ticks_ms()   # full round-trip
        # NTP time fallback + GPS time-aiding: while WiFi is up and neither GPS
        # nor NTP has set the clock yet, sync from NTP (so logs are timestamped
        # even without a GPS fix) and inject coarse time into the GPS to speed up
        # its fix. GPS overrides later (gps_task). IDLE only; retried until set.
        if (wlan.isconnected() and state.time_source is None and still
                and state.phase == protocol.PHASE_IDLE and n % 10 == 0):
            _ntp_time_aid(state)
        # Opportunistic AssistNow Predictive Orbits: while IDLE + online, fetch
        # predicted orbits and feed them to the GPS (cached for the next cold
        # start). First call also does the one-time ZTP registration.
        if (UBLOX_ZTP_TOKEN and not _assistnow_done and still
                and wlan.isconnected() and state.phase == protocol.PHASE_IDLE):
            if _assistnow_should_download(state):
                _assistnow_download(state)
                _assistnow_done = True       # one attempt per boot
            elif state.time_synced:
                _assistnow_done = True        # cache confirmed fresh
            # else: have a cache but no time yet -> re-check on a later loop
        # GitHub upload of raw.csv (WiFi up, IDLE): the periodic auto-offload
        # plus a manual trigger from the dashboard "Upload log" button. The
        # PERIODIC trigger waits for stillness; a manual click runs regardless.
        if (GITHUB_TOKEN and wlan.isconnected()
                and state.phase == protocol.PHASE_IDLE
                and (state.upload_request
                     or (still and n and n % WIFI_PERIOD_S == 0))):
            await _run_upload(state, ride=(state.upload_request == "ride"))
            state.upload_request = False   # clear the manual request once handled
        n += 1
        await asyncio.sleep_ms(1000)


def _load_gyro_cal():
    """Last persisted good boot bias; zeros if none. Zeros are a SAFE
    fallback: a real QMI8658 bias is a few dps, far below the 10 dps motion
    threshold, and the BiasTracker converges on the exact value once the
    unit rests."""
    try:
        import json
        with open(GYRO_CAL_PATH) as f:
            b = json.load(f)
        return (float(b[0]), float(b[1]), float(b[2]))
    except Exception:
        return (0.0, 0.0, 0.0)


def _save_gyro_cal(bias):
    """Persist a good boot estimate for the moving-boot fallback. Skipped
    when within 0.3 dps of the stored value (flash wear - this runs every
    boot)."""
    old = _load_gyro_cal()
    if max(abs(bias[i] - old[i]) for i in range(3)) < 0.3:
        return
    try:
        import json
        with open(GYRO_CAL_PATH, "w") as f:
            json.dump(list(bias), f)
    except Exception as e:
        print("gyro.cal save failed:", e)


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
        supervisor_task(pmu, state),
    ]
    if sx is not None:
        tasks.append(telemetry_task(sx, state))
        tasks.append(radio_task(sx, state))
    if display:
        tasks.append(display_task(display, state))
    if WIFI_ENABLED and WIFI_NETWORKS:
        if config.ROPE_DASHBOARD:
            tasks.append(dashboard_task(state))   # continuous WiFi + status page
        elif GITHUB_TOKEN:
            tasks.append(wifi_task(state))        # duty-cycled upload-only
    await asyncio.gather(*tasks)


def run():
    print("Reset cause:", reset_cause_str())   # first thing: why did we boot?
    _event("boot " + reset_cause_str())
    pmu = board.init_power()
    state = State()

    # --- Sensors (with boot-time liveness output)
    baro = Barometer(board.i2c0)
    print(baro.pressure_hpa(), "hPa")

    gps = GPS(board.gps_uart)
    gps.dump(10)

    # IMU path is chosen by config.IMU_FAST (see there - stability bisect):
    # True = winchy_fast C sampler (esp_timer task, hardware-exact 50 Hz),
    # False = legacy Python driver on machine.SPI(2).
    if config.IMU_FAST:
        from winchy.sensors.imu_fast import FastIMU
        imu = FastIMU(config.QMI_SCK, config.QMI_MOSI, config.QMI_MISO,
                      config.QMI_CS, 1000 // IMU_PERIOD_MS)
    else:
        imu = QMI8658(board.qmi_spi, board.qmi_cs)
        print("IMU: legacy python driver (IMU_FAST disabled)")
    print("Accelerations:", imu.read_accel_avg(IMU_WINDOW))
    # Initial gyro bias: only trust a STILL sample window. Field test #2
    # (2026-07-07): powering on while the unit was handled averaged motion
    # into the bias, the corrected gyro then exceeded the motion gate
    # forever -> continuous 50 Hz recording rotated the ride data away and
    # still=False blocked announce/roam/upload for the whole session.
    # Reject moving windows (spread check, shared/imubias.py), retry
    # briefly; if the unit never rests, fall back to the last persisted
    # good bias - the imu_task BiasTracker heals it once the unit rests.
    gyro_bias = None
    for attempt in range(4):
        win = []
        for _ in range(50):
            win.append(imu.read_gyro())
            time.sleep_ms(5)
        mean, still_ok = imubias.window_bias(win)
        if still_ok:
            gyro_bias = mean
            break
        print("Gyro bias: window %d rejected (moving)" % (attempt + 1))
    if gyro_bias is not None:
        _save_gyro_cal(gyro_bias)
        print("Gyro bias (dps): (%.2f, %.2f, %.2f)" % gyro_bias)
    else:
        gyro_bias = _load_gyro_cal()
        print("Gyro bias: no still window - fallback (%.2f, %.2f, %.2f)"
              % gyro_bias)
        _event("gyro bias: MOVING boot, fallback used")

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

    # --- LoRa (official lora-sx126x driver; RX handling lives in radio_task)
    if getattr(config, "RADIO_ENABLED", True):
        sx = AsyncSX1262(
            spi=SPI(config.LORA_SPI_BUS, baudrate=2000000,
                    sck=Pin(config.LORA_CLK), mosi=Pin(config.LORA_MOSI),
                    miso=Pin(config.LORA_MISO)),
            cs=Pin(config.LORA_CS),
            busy=Pin(config.LORA_BUSY),
            dio1=Pin(config.LORA_IRQ),
            reset=Pin(config.LORA_RST),
            dio2_rf_sw=True,                 # DIO2 drives the RF switch
            dio3_tcxo_millivolts=1700,       # DIO3-powered TCXO (as before)
            dio3_tcxo_start_time_us=5000,    # old driver's TCXO settle time
            lora_cfg={
                "freq_khz": int(config.LORA_FREQ_MHZ * 1000),
                "bw": int(config.LORA_BW_KHZ),
                "sf": config.LORA_SF,
                "coding_rate": config.LORA_CR,
                "preamble_len": 8,
                "output_power": config.LORA_TX_POWER_DBM,
                "syncword": config.LORA_SYNC_WORD,  # writes 0x1424, matches
                "crc_en": True,                     # the winch's old driver
                "implicit_header": False,
            })
        # The official driver never calls its calibrate methods itself. On
        # this board the power-on auto-calibration ran WITHOUT the TCXO
        # (DIO3-powered, enabled only later in the constructor), so
        # recalibrate with the TCXO running + image-calibrate the 869 MHz
        # band - the old driver did both; without them TX was inaudible on
        # the first migration attempt (2026-07-06).
        sx.calibrate()
        sx.calibrate_image()
        state.tx_power_dbm = config.LORA_TX_POWER_DBM  # ADR adjusts from here
    else:
        # Soak diagnostic (panic hunt 2026-07-06): run everything EXCEPT the
        # radio - the SX1262 chip stays in reset, no soft-IRQ callback, no
        # telemetry task. A clean multi-hour soak in this mode implicates
        # the radio path; a crash exonerates it.
        sx = None
        print("RADIO DISABLED (config.RADIO_ENABLED=False) - soak diagnostic")

    print("Starting asyncio runtime")
    asyncio.run(_main(pmu, adc, imu, baro, sx, display, state, gyro_bias, mag))
