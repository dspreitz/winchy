# ===================
# Schematics of T-Beam-S3-Supreme
# https://github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series/blob/master/schematic/LilyGo_T-BeamS3Supreme.pdf
# https://github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series/blob/e865cad8243db266d7759faf724719dd53a6be2a/examples/Sensor/QMI8658_BlockExample/utilities.h#L378

# Rope segment application.
#
# Migration step 3 (docs/rope_segment_architecture.md): hardware access
# lives in winchy/board.py and winchy/sensors/; this file retains only the
# application flow (init, measure/transmit loop) until the asyncio runtime
# replaces it in step 4.

import struct
import time

from machine import Pin
import sh1106  # Display library
from sx1262 import SX1262  # LoRa library

import config
from winchy import board
from winchy.fusion.attitude import rope_angle_above_ground
from winchy.sensors.ads1232 import ADS1232
from winchy.sensors.barometer import Barometer
from winchy.sensors.gps import GPS
from winchy.sensors.magnetometer import Magnetometer
from winchy.sensors.qmi8658 import QMI8658

PMU = board.init_power()

# ============ Sensors ============
baro = Barometer(board.i2c0)
print(baro.pressure_hpa(), "hPa")

gps = GPS(board.gps_uart)
gps.dump(10)  # boot-time liveness check

imu = QMI8658(board.qmi_spi, board.qmi_cs)
print("Accelerations:", imu.read_accel_avg(20))

mag = Magnetometer()  # loads calibration.cal

adc = ADS1232(pdwn=config.ADS_PDWN, sclk=config.ADS_SCLK, dout=config.ADS_DOUT,
              gain0=config.ADS_GAIN0, gain1=config.ADS_GAIN1,
              gain=config.ADS_GAIN)

# ============ Display ============
display = sh1106.SH1106_I2C(config.OLED_WIDTH, config.OLED_HEIGHT, board.i2c0,
                            Pin(config.OLED_RST), config.OLED_ADDR)


def printdisplay(text):
    display.sleep(False)
    display.fill(0)
    display.text(str(text), 0, 0, 1)
    display.show()


printdisplay("Test")
time.sleep(1)

# ============ LoRa ============


def cb(events):
    if events & SX1262.RX_DONE:
        msg, err = sx.recv()
        error = SX1262.STATUS[err]
        print('Receive: {}, {}'.format(msg, error))
    elif events & SX1262.TX_DONE:
        print('TX done.')


sx = SX1262(spi_bus=config.LORA_SPI_BUS, clk=config.LORA_CLK,
            mosi=config.LORA_MOSI, miso=config.LORA_MISO, cs=config.LORA_CS,
            irq=config.LORA_IRQ, rst=config.LORA_RST, gpio=config.LORA_BUSY)

sx.begin(freq=config.LORA_FREQ_MHZ, bw=config.LORA_BW_KHZ, sf=config.LORA_SF,
         cr=config.LORA_CR, syncWord=config.LORA_SYNC_WORD,
         power=config.LORA_TX_POWER_DBM, currentLimit=60.0, preambleLength=8,
         implicit=False, implicitLen=0xFF,
         crcOn=True, txIq=False, rxIq=False,
         tcxoVoltage=1.7, useRegulatorLDO=False, blocking=True)

sx.setBlockingCallback(False, cb)


def send_adc_and_pressure_over_lora(adc_value, pressure_hpa, angle_deg):
    # Pack: 4-byte signed ADC + 2-byte unsigned pressure (x10) + 1-byte
    # unsigned angle (1 deg resolution). Replaced by the versioned frames of
    # shared/protocol.py in migration step 5.
    pressure_scaled = int(pressure_hpa * 10)
    angle_uint8 = max(0, min(255, int(round(angle_deg))))
    packet = struct.pack('>iHB', adc_value, pressure_scaled, angle_uint8)
    sx.send(packet)
    print("Sent packet (binary):", packet)


# ============ Main loop ============
adc.tare()

while True:
    adc_val = adc.read_raw()
    print("ADC value:", adc_val - adc.offset)
    druck = baro.pressure_hpa()
    angle = rope_angle_above_ground(*imu.read_accel_avg(20))
    print("Seilwinkel:", angle)
    # NOTE: transmits the untared raw ADC value (original behavior).
    send_adc_and_pressure_over_lora(adc_val, druck, angle)
    display.fill(0)
    display.text("Seilwinkel:", 0, 0)
    display.text("{:.1f} deg".format(angle), 0, 10)
    display.show()
    time.sleep(0.5)
