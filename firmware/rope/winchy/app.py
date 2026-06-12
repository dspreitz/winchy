# Rope segment application: asyncio runtime (migration step 4).
#
# Each measurement source runs as its own task at its own rate and
# publishes into the shared State; telemetry and display sample the latest
# values. The force ADC is never stalled by the slow barometer read or the
# radio anymore. See docs/rope_segment_architecture.md.

import asyncio
import struct
import time

from machine import Pin, RTC
import sh1106
from sx1262 import SX1262

import config
from winchy import board
from winchy.fusion.attitude import rope_angle_above_ground
from winchy.sensors.ads1232 import ADS1232
from winchy.sensors.barometer import Barometer
from winchy.sensors.gps import GPS, parse_nmea
from winchy.sensors.magnetometer import Magnetometer
from winchy.sensors.qmi8658 import QMI8658
from winchy.state import State

FORCE_TIMEOUT_MS = 1000
IMU_PERIOD_MS = 20          # 50 Hz sampling
IMU_WINDOW = 20             # moving-average window (matches old smoothing)
BARO_PERIOD_MS = 1000
TELEMETRY_PERIOD_MS = 500   # 2 Hz, the old loop cadence; the SF12/BW500
                            # airtime (~130 ms/packet) caps what is sane here
DISPLAY_PERIOD_MS = 500
SUPERVISOR_PERIOD_MS = 5000


async def force_task(adc, state):
    # Free-running at the ADC conversion rate (10 SPS at gain 128).
    while True:
        try:
            state.force_raw = await adc.aread_raw(FORCE_TIMEOUT_MS)
            state.force_ts = time.ticks_ms()
        except asyncio.TimeoutError:
            state.force_errors += 1


async def imu_task(imu, state):
    window = []
    while True:
        window.append(imu.read_accel())
        if len(window) > IMU_WINDOW:
            window.pop(0)
        n = len(window)
        ax = sum(s[0] for s in window) / n
        ay = sum(s[1] for s in window) / n
        az = sum(s[2] for s in window) / n
        state.accel = (ax, ay, az)
        state.angle_deg = rope_angle_above_ground(ax, ay, az)
        state.accel_ts = time.ticks_ms()
        await asyncio.sleep_ms(IMU_PERIOD_MS)


async def baro_task(baro, state):
    while True:
        state.pressure_hpa = await baro.apressure_hpa()
        state.baro_ts = time.ticks_ms()
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
                print("RTC synced from GPS: %04d-%02d-%02d %02d:%02d:%02dZ"
                      % (y, mo, d, h, mi, s))


async def telemetry_task(sx, state):
    while True:
        adc_val = state.force_raw
        print("ADC value:", adc_val - state.force_offset)
        print("Seilwinkel:", state.angle_deg)
        # Pack: 4-byte signed ADC + 2-byte unsigned pressure (x10) + 1-byte
        # unsigned angle (1 deg resolution). NOTE: still the untared raw ADC
        # value, as before. Replaced by versioned frames in step 5.
        pressure_scaled = int(state.pressure_hpa * 10)
        angle_uint8 = max(0, min(255, int(round(state.angle_deg))))
        packet = struct.pack(">iHB", adc_val, pressure_scaled, angle_uint8)
        sx.send(packet)  # non-blocking; TX_DONE arrives via radio callback
        state.tx_count += 1
        print("Sent packet (binary):", packet)
        await asyncio.sleep_ms(TELEMETRY_PERIOD_MS)


async def display_task(display, state):
    while True:
        display.fill(0)
        display.text("F: {}".format(state.force_raw - state.force_offset), 0, 0)
        display.text("Seilwinkel:", 0, 16)
        display.text("{:.1f} deg".format(state.angle_deg), 0, 26)
        display.text("{:.1f} hPa".format(state.pressure_hpa), 0, 42)
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


async def _main(pmu, adc, imu, baro, sx, display, state):
    await asyncio.gather(
        force_task(adc, state),
        imu_task(imu, state),
        baro_task(baro, state),
        gps_task(state),
        telemetry_task(sx, state),
        display_task(display, state),
        supervisor_task(pmu, state),
    )


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
    display.sleep(False)
    display.fill(0)
    display.text("Winchy rope unit", 0, 0)
    display.show()

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
    asyncio.run(_main(pmu, adc, imu, baro, sx, display, state))
