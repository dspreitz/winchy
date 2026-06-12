# Rope segment application: asyncio runtime (migration step 4).
#
# Each measurement source runs as its own task at its own rate and
# publishes into the shared State; telemetry and display sample the latest
# values. The force ADC is never stalled by the slow barometer read or the
# radio anymore. See docs/rope_segment_architecture.md.

import asyncio
import math
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
TELEMETRY_PERIOD_MS = 500   # 2 Hz, the old loop cadence; the SF12/BW500
                            # airtime (~130 ms/packet) caps what is sane here
DISPLAY_PERIOD_MS = 500
SUPERVISOR_PERIOD_MS = 5000

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


async def gps_task(state):
    reader = asyncio.StreamReader(board.gps_uart)
    rtc = RTC()
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
            if update["datetime"] and not state.time_synced:
                y, mo, d, h, mi, s = update["datetime"]
                rtc.datetime((y, mo, d, 0, h, mi, s, 0))
                state.time_synced = True
                state.pending_time_sync = True
                print("RTC synced from GPS: %04d-%02d-%02d %02d:%02d:%02dZ"
                      % (y, mo, d, h, mi, s))


async def telemetry_task(sx, state):
    seq = 0
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
        # Tow phase detection arrives with the state machine (step 6).
        frame = protocol.encode_telemetry(
            seq, protocol.PHASE_IDLE, force, state.angle_deg,
            state.pressure_hpa, state.system_mv, flags)
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
        sx.send(frame)  # non-blocking; TX_DONE arrives via radio callback
        seq = (seq + 1) & 0xFFFF
        state.tx_count += 1
        print("Sent packet (binary):", frame)
        await asyncio.sleep_ms(TELEMETRY_PERIOD_MS)


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
        if events & SX1262.RX_DONE:
            msg, err = sx.recv()
            print("Receive: {}, {}".format(msg, SX1262.STATUS[err]))
        elif events & SX1262.TX_DONE:
            print("TX done.")

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

    print("Starting asyncio runtime")
    asyncio.run(_main(pmu, adc, imu, baro, sx, display, state, gyro_bias))
