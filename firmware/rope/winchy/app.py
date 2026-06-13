# Rope segment application: asyncio runtime (migration step 4).
#
# Each measurement source runs as its own task at its own rate and
# publishes into the shared State; telemetry and display sample the latest
# values. The force ADC is never stalled by the slow barometer read or the
# radio anymore. See docs/rope_segment_architecture.md.

import asyncio
import math
import micropython
import time

from machine import Pin, RTC
import sh1106
from sx1262 import SX1262

import config
import protocol
from winchy import board
from winchy.fusion import altitude
from winchy.fusion.attitude import rope_angle_above_ground
from winchy.fusion.kalman import GravityKalman, VerticalKalman
from winchy.sensors.ads1232 import ADS1232
from winchy.sensors.barometer import Barometer
from winchy.sensors.gps import GPS, parse_nmea
from winchy.sensors.magnetometer import Magnetometer
from winchy.sensors.qmi8658 import QMI8658
from winchy.state import State

FORCE_TIMEOUT_MS = 1000
IMU_PERIOD_MS = 20          # 50 Hz sampling
IMU_WINDOW = 20             # samples for the boot-time accel print
BARO_PERIOD_MS = 1000

# Gyro bias: learned slowly while IDLE and still, frozen during the tow so
# it cannot chase real rotation.
GYRO_BIAS_ALPHA = 0.01
GYRO_STILL_TOLERANCE_G = 0.05
TELEMETRY_PERIOD_MS = 500   # 2 Hz. At SF7/BW500 airtime is ~15 ms so there is
                            # rate headroom, but ~2% duty already nears the
                            # 868.0-868.6 MHz 1% limit - raise rate with care.
REPORT_REQUEST_EVERY = 2    # ask the winch for a LINK_REPORT every Nth frame
                            # (~1 Hz at 2 Hz telemetry); 0 disables feedback
REPORT_WINDOW_MS = 150      # extra RX dwell after a request so the winch's
                            # half-duplex reply lands before the next TX; must
                            # exceed reply airtime + winch processing (~60 ms
                            # at SF7, incl. the winch OLED update)
DISPLAY_PERIOD_MS = 500
SUPERVISOR_PERIOD_MS = 5000
TIME_SYNC_RESEND_MS = 30000  # re-announce GPS time so a late/NTP-less winch syncs

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

# Closed-loop TX-power control (ADR). Drives output power from the winch's
# reported downlink SNR margin: back off when the link is strong (save the
# battery / reduce interference), push up when it thins. Only power is
# adapted here - changing SF/BW would need both ends retuned in lockstep, so
# that stays a deliberate, handshaked step. Set ADR_ENABLED = False to pin
# power at config.LORA_TX_POWER_DBM.
ADR_ENABLED = True          # set False for a range test (fixes TX power so
                            # RSSI maps directly to path loss)
ADR_TX_POWER_MIN_DBM = -9   # SX1262 floor (driver clamps below this)
ADR_TX_POWER_MAX_DBM = 14   # EU 868.0-868.6 MHz ERP cap (25 mW); raise only if
                            # antenna gain / regulatory domain allows
ADR_STEP_DB = 2             # per-adjustment increment, keeps changes gentle
ADR_MARGIN_DB = 12          # desired SNR headroom above the SF's demod floor
ADR_HYSTERESIS_DB = 4       # deadband around the target to avoid oscillation
ADR_LOSS_HIGH_PCT = 50      # sustained loss above this forces power up
ADR_REPORT_TIMEOUT_MS = 4000  # no fresh report this long -> ramp up (fail safe)

# Approx LoRa demodulator SNR floor (dB) per spreading factor.
_LORA_SNR_FLOOR_DB = {7: -7, 8: -10, 9: -12, 10: -15, 11: -17, 12: -20}


def _adr_next_power(state):
    """Return the next TX power (dBm) one step toward the SNR-margin target,
    clamped. Ramps up when feedback is stale or loss is high (fail safe)."""
    power = state.tx_power_dbm
    fresh = (state.link_report_ts != 0 and time.ticks_diff(
        time.ticks_ms(), state.link_report_ts) <= ADR_REPORT_TIMEOUT_MS)
    if not fresh or state.link_loss_pct >= ADR_LOSS_HIGH_PCT:
        return min(ADR_TX_POWER_MAX_DBM, power + ADR_STEP_DB)
    floor = _LORA_SNR_FLOOR_DB.get(config.LORA_SF, -20)
    margin = state.link_snr_db - (floor + ADR_MARGIN_DB)
    if margin < -ADR_HYSTERESIS_DB:
        return min(ADR_TX_POWER_MAX_DBM, power + ADR_STEP_DB)
    if margin > ADR_HYSTERESIS_DB:
        return max(ADR_TX_POWER_MIN_DBM, power - ADR_STEP_DB)
    return power  # inside the deadband: hold

# MicroPython's time.time() counts from 2000-01-01; the protocol carries
# unix epoch seconds.
_UNIX_EPOCH_OFFSET = 946684800

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


async def imu_task(imu, state, filt, gyro_bias):
    bias = list(gyro_bias)
    last = time.ticks_ms()
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
        await asyncio.sleep_ms(IMU_PERIOD_MS)


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


async def gps_task(state):
    global _pps_rtc
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
        elif update["type"] == "RMC":
            if update["datetime"]:
                y, mo, d, h, mi, s = update["datetime"]
                if not state.time_synced:
                    # Rough initial set (NMEA-latency) so we have time at once;
                    # PPS then refines the second boundary on each edge.
                    rtc.datetime((y, mo, d, 0, h, mi, s, 0))
                    state.time_synced = True
                    state.pending_time_sync = True
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
        if state.pending_time_sync:
            state.pending_time_sync = False
            frame = protocol.encode_time_sync(
                seq, time.time() + _UNIX_EPOCH_OFFSET)
            sx.send(frame)
            seq = (seq + 1) & 0xFFFF
            state.tx_count += 1
            print("Sent TIME_SYNC:", frame)

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
            state.batt_pct)
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
    ticks = 0
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
        # Re-announce GPS time periodically so a winch that powered up after
        # the rope (and has no NTP) still gets synced.
        ticks += 1
        if (state.time_synced
                and ticks % (TIME_SYNC_RESEND_MS // SUPERVISOR_PERIOD_MS) == 0):
            state.pending_time_sync = True
        await asyncio.sleep_ms(SUPERVISOR_PERIOD_MS)


async def _main(pmu, adc, imu, baro, sx, display, state, gyro_bias):
    gravity_filter = GravityKalman()
    vertical_filter = VerticalKalman()
    tasks = [
        force_task(adc, state),
        imu_task(imu, state, gravity_filter, gyro_bias),
        baro_task(baro, state, vertical_filter),
        gps_task(state),
        telemetry_task(sx, state),
        supervisor_task(pmu, state),
    ]
    if display:
        tasks.append(display_task(display, state))
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
             power=config.LORA_TX_POWER_DBM, currentLimit=60.0,
             preambleLength=8, implicit=False, implicitLen=0xFF,
             crcOn=True, txIq=False, rxIq=False,
             tcxoVoltage=1.7, useRegulatorLDO=False, blocking=True)
    sx.setBlockingCallback(False, on_radio)
    state.tx_power_dbm = config.LORA_TX_POWER_DBM  # ADR adjusts from here

    print("Starting asyncio runtime")
    asyncio.run(_main(pmu, adc, imu, baro, sx, display, state, gyro_bias))
