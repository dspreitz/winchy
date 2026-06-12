# ===================
# Schematics of T-Beam-S3-Supreme
# https://github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series/blob/master/schematic/LilyGo_T-BeamS3Supreme.pdf
# https://github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series/blob/e865cad8243db266d7759faf724719dd53a6be2a/examples/Sensor/QMI8658_BlockExample/utilities.h#L378

# XPowersLib for AXP2101 Chip
# https://github.com/lewisxhe/XPowersLib/tree/master/Micropython

# BMP280 Lib
# https://github.com/dafvid/micropython-bmp280/blob/master/bmp280.py
# I2C Adress: 0x77

# QMI8658 Datasheet
# https://www.waveshare.com/w/upload/5/5f/QMI8658C.pdf

# QMC6310U - Hall Sensor
# I2C Adress: 0x1c

# OLED SH1106
# I2C Adress: 0x3c


# Board wiring and AXP2101 power-rail setup live in config.py and
# winchy/board.py (migration step 2, docs/rope_segment_architecture.md).
# The rest of this file is the original application, pending step 3+.

import math
import struct
import time

from machine import Pin
from bmp280 import *
import sh1106  # Display library
from sx1262 import SX1262  # LoRa library

import config
from winchy import board

PMU = board.init_power()
i2c = board.i2c0


# ============ Begin of BMP2800 Initialization ============
# I2CBMP = SoftI2C(scl=Pin(18), sda=Pin(17), freq=1000000)
# Scan I2C bus
print('Scan i2c bus...')
# devices = I2CBMP.scan()
devices = i2c.scan()

if len(devices) == 0:
  print("No i2c device !")
else:
  print('I2CBMP devices found:',len(devices))

  for device in devices:  
    print("Decimal address: ",device," | Hexa address: ",hex(device))

bmp = BMP280(i2c)

# Initialize BMP280
bmp.use_case(BMP280_CASE_WEATHER)
bmp.oversample(BMP280_OS_HIGH)

bmp.temp_os = BMP280_TEMP_OS_8
bmp.press_os = BMP280_PRES_OS_4

bmp.standby = BMP280_STANDBY_250
bmp.iir = BMP280_IIR_FILTER_2

# bmp.spi3w = BMP280_SPI3W_ON

bmp.power_mode = BMP280_POWER_FORCED


def pressure():
        try:
            bmp.force_measure()
        except:
            print("BMP Force measure not working.")
            
        while bmp.is_measuring:
            time.sleep(0.1)
            
        while bmp.is_updating:
            time.sleep(0.1)
            
        # print(bmp.temperature)
        # print(bmp.pressure / 100 + (360/10*1.22))
        # print(bmp.pressure / 100)

        bmp.sleep()
        # return bmp.pressure / 100 + (360/10*1.22) # QNH Wert
        return bmp.pressure / 100

print(pressure(), "hPa")

# ============ Begin of UBlox M10S-00B Initialization ============
uart = board.gps_uart
n=0
while True:
    if uart.any():
        print(uart.readline())
        n=n+1
    if n >= 10:
        break

# ============ Begin of QMI8658 Initialization ============

# QMI is connected via SPI
# SCLK = IO36
# MISO = IO37
# MOSI = IO35
# CS = IO34
# INT = IO33

spi = board.qmi_spi
cs = board.qmi_cs  # initialised high

# Register addresses
REG_WHO_AM_I = 0x00		# Default: 0x05
REVISION_ID = 0x01		# Default: 0x7C
REG_CTRL1 = 0x02
REG_CTRL2 = 0x03		# Accelerometer Settings
REG_CTRL3 = 0x04		# Gyro Settings
REG_CTRL5 = 0x06
REG_CTRL7 = 0x08
REG_RESET = 0x60

REG_ACC_X_L = 0x35  # X-axis Acceleration
REG_ACC_Y_L = 0x37  # Y-axis Acceleration
REG_ACC_Z_L = 0x39  # Z-axis Acceleration

# QMI8658A register addresses
ACCEL_XOUT_L = 0x35
ACCEL_YOUT_L = 0x37
ACCEL_ZOUT_L = 0x39

# Scale factor (depends on the range, assuming ±4g for this example)
# Check the sensor's datasheet for the exact value based on your configuration
scale_factor = 4.0 / 32768.0

def read_register(register, length=1):
    cs.value(0)  # Select the device by pulling CS low
    spi.write(bytearray([register | 0x80]))  # Send register address with read bit set
    data = spi.read(length)  # Read the data from the register
    cs.value(1)  # Deselect the device by pulling CS high
    return data

def register_to_binary(register):
    data = read_register(register, 1)  # Read 1 byte from the register
    binary_value = f'{data[0]:08b}'  # Convert to 8-bit binary string with leading zeros
    return binary_value

def write_to_register(register, data):
    cs.value(0)  # Set CS low to start communication
    spi.write(bytes([register & 0x7F]))  # Write the register address with the write bit set to 0
    spi.write(bytes([data]))  # Write the data to the register
    cs.value(1)  # Set CS high to end communication
    return
    
# Function to convert two bytes into a 16-bit signed integer
def bytes_to_int16(data):
    return struct.unpack('<h', data)[0]  # Little-endian format

def read_acc_register(register):
    combined_value = (read_register(register+1)[0]<<8) |(read_register(register)[0])
    if combined_value & 0x8000:  # If the MSB is set (sign bit is 1)
        # The number is negative
        signed_value = combined_value - 65536  # Convert to signed 16-bit value
    else:
        # The number is positive
        signed_value = combined_value  # No change needed
    return signed_value
 

def read_accelerations():
    acc_x = read_acc_register(ACCEL_XOUT_L)
    # print(f"{read_register(ACCEL_XOUT_L+1)[0]:08b}")
    # print(f"{read_register(ACCEL_XOUT_L)[0]:08b}")
    # print(f"Acc_x: 0b{test:08b}")
    # print(int(test))
    # print(int(test)/16384.0)
        # print(f"Acc_x: 0d{test:08d}")
    # print(f"Acc_x: 0d{(test/16384):08f}")
    acc_x = read_acc_register(ACCEL_XOUT_L)
    acc_y = read_acc_register(ACCEL_YOUT_L)
    acc_z = read_acc_register(ACCEL_ZOUT_L)
    
    # check the Sensitivity per Axis
        
    return (acc_x/16384, acc_y/16384, acc_z/16384)

def moving_average(samples, window_size):
    return sum(samples[-window_size:]) / window_size

def read_average_acceleration(window_size=10):
    acc_x_samples = []
    acc_y_samples = []
    acc_z_samples = []
    
    for _ in range(window_size):
        acc_x, acc_y, acc_z = read_accelerations()
        acc_x_samples.append(acc_x)
        acc_y_samples.append(acc_y)
        acc_z_samples.append(acc_z)
        
        time.sleep_ms(10)  # Small delay between readings
    
    avg_acc_x = moving_average(acc_x_samples, window_size)
    avg_acc_y = moving_average(acc_y_samples, window_size)
    avg_acc_z = moving_average(acc_z_samples, window_size)
    
    return avg_acc_x, avg_acc_y, avg_acc_z



# Reset chip at beginning
def reset_QMI():
    register_address = REG_RESET
    data = 0x0B
    write_to_register(register_address,data)
    time.sleep_ms(20)
    binary_value = register_to_binary(0x4D)
    print(f"Binary value of register 0x{register_address:02X}: 0x{binary_value}")

def read_QMI_temp():
    register_address = 0x33
    Temp = (read_register(0x34)[0]<<8) |(read_register(0x33)[0])
    return Temp / 256.0

# Example usage
register_address = 0x00  # Replace with your register address
binary_value = register_to_binary(register_address)
print(f"Binary value of register 0x{register_address:02X}: {binary_value}")

register_address = 0x01  # Replace with your register address
binary_value = register_to_binary(register_address)
print(f"Binary value of register 0x{register_address:02X}: {binary_value}")

# Serial Interface and Sensor Enable
register_address = REG_CTRL1  # Replace with your register address
data = 0b00100000
write_to_register(register_address,data)
binary_value = register_to_binary(register_address)
print(f"Binary value of register 0x{register_address:02X} - Serial Interface and Sensor Enable: {binary_value}")


register_address = REG_CTRL2  # Replace with your register address
data = 0b10001000
write_to_register(register_address,data)
binary_value = register_to_binary(register_address)
print(f"Binary value of register 0x{register_address:02X} - Accelerometer Settings: {binary_value}")

register_address = REG_CTRL3  # Replace with your register address
binary_value = register_to_binary(register_address)
print(f"Binary value of register 0x{register_address:02X} - Gyroscope Settings: {binary_value}")
data = 0b11001000
write_to_register(REG_CTRL3,data)




register_address = 0x33  # Replace with your register address
binary_value = register_to_binary(register_address)
print(f"Binary value of register 0x{register_address:02X} - Temp Sensor Output Low: {binary_value}")

register_address = 0x34  # Replace with your register address
binary_value = register_to_binary(register_address)
print(f"Binary value of register 0x{register_address:02X} - Temp Sensor Output High: {binary_value}")

# Enable Sensors and Configure Data Reads
register_address = REG_CTRL7  # Replace with your register address
binary_value = register_to_binary(register_address)
print(f"Binary value of register 0x{register_address:02X} - Enable Sensors and Configure Data Reads: {binary_value}")
data = 0b10000011
write_to_register(REG_CTRL7,data)

# Sensor Data Processing
register_address = REG_CTRL5  # Replace with your register address
binary_value = register_to_binary(register_address)
print(f"Binary value of register 0x{register_address:02X} - Sensor Data Processing Settings: {binary_value}")
data = 0b00011001
write_to_register(REG_CTRL5,data)

# read_accelerations()

while True:
    # Get the accelerations
    acc_x, acc_y, acc_z = read_average_acceleration(20)
    print("Accelerations:", acc_x , acc_y, acc_z)
    # time.sleep_ms(100)
    break


# ============ Begin of QMI8658 Initialization ============

# Read the magnetic field strength and determine if a magnet is nearby

from PiicoDev_QMC6310 import PiicoDev_QMC6310
from PiicoDev_Unified import sleep_ms

# NOTE: PiicoDev re-creates I2C bus 0 at 1 MHz; to be consolidated into
# board.i2c0 in migration step 3.
magSensor = PiicoDev_QMC6310(bus=0, freq=1000000, scl=Pin(config.I2C0_SCL),
                             sda=Pin(config.I2C0_SDA), range=3000)
# magSensor.calibrate()
 
threshold = 120 # microTesla or 'uT'.

def printdisplay(text):    
    display.sleep(False)
    display.fill(0)
    display.text(str(text), 0, 0, 1)
    display.show()

def displaymag():    
    reading = magSensor.read()
    display.sleep(False)
    display.fill(0)
    display.text("x= " + str(reading["x"]), 0, 0, 1)
    display.text("y= " + str(reading["y"]), 0, 9, 1)
    display.text("z= " + str(reading["z"]), 0, 18, 1)
    display.show()


# ============ Begin of Display Initialization ============
display = sh1106.SH1106_I2C(config.OLED_WIDTH, config.OLED_HEIGHT, board.i2c0,
                            Pin(config.OLED_RST), config.OLED_ADDR)
printdisplay("Test")
sleep_ms(1000)

# ============ Begin LoRa ============


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
# print("2")
# LoRa
sx.begin(freq=config.LORA_FREQ_MHZ, bw=config.LORA_BW_KHZ, sf=config.LORA_SF,
         cr=config.LORA_CR, syncWord=config.LORA_SYNC_WORD,
         power=config.LORA_TX_POWER_DBM, currentLimit=60.0, preambleLength=8,
         implicit=False, implicitLen=0xFF,
         crcOn=True, txIq=False, rxIq=False,
         tcxoVoltage=1.7, useRegulatorLDO=False, blocking=True)
# print("3")
# FSK
##sx.beginFSK(freq=923, br=48.0, freqDev=50.0, rxBw=156.2, power=-5, currentLimit=60.0,
##            preambleLength=16, dataShaping=0.5, syncWord=[0x2D, 0x01], syncBitsLength=16,
##            addrFilter=SX126X_GFSK_ADDRESS_FILT_OFF, addr=0x00, crcLength=2, crcInitial=0x1D0F, crcPolynomial=0x1021,
##            crcInverted=True, whiteningOn=True, whiteningInitial=0x0100,
##            fixedPacketLength=False, packetLength=0xFF, preambleDetectorLength=SX126X_GFSK_PREAMBLE_DETECT_16,
##            tcxoVoltage=1.6, useRegulatorLDO=False,
##            blocking=True)

sx.setBlockingCallback(False, cb)
 
# ============ Initialize ADS1232  ============

# Pin assignments
PDWN = Pin(config.ADS_PDWN, Pin.OUT)   # Power down / reset
SCLK = Pin(config.ADS_SCLK, Pin.OUT)   # Serial clock
DOUT = Pin(config.ADS_DOUT, Pin.IN)    # Data out / data ready (DRDY)
GAIN0 = Pin(config.ADS_GAIN0, Pin.OUT)
GAIN1 = Pin(config.ADS_GAIN1, Pin.OUT)

# Wake up and initialize ADS1232
PDWN.value(1)
SCLK.value(0)

def read_adc():
    # Wait for DRDY signal (goes LOW)
    while DOUT.value() == 1:
        pass

    result = 0
    for _ in range(24):
        SCLK.value(1)
        result = (result << 1) | DOUT.value()
        SCLK.value(0)

    # Sign-extend 24-bit to 32-bit
    if result & 0x800000:
        result |= ~0xFFFFFF

    # One extra clock pulse to complete read cycle
    SCLK.value(1)
    SCLK.value(0)

    return result

# === Gain selection mapping ===
GAIN_SETTINGS = {
    1: (0, 0),
    2: (1, 0),
    64: (0, 1),
    128: (1, 1)
}

def set_gain(gain):
    """Set ADS1232 gain via GAIN0 and GAIN1"""
    if gain not in GAIN_SETTINGS:
        raise ValueError("Invalid gain: choose 1, 2, 64, or 128")
    g0, g1 = GAIN_SETTINGS[gain]
    GAIN0.value(g0)
    GAIN1.value(g1)
    print(f"Gain set to {gain}x")
    
def send_adc_and_pressure_over_lora(adc_value, pressure_hpa, angle_deg):
    # Scale pressure (float in hPa) to int (×10)
    pressure_scaled = int(pressure_hpa * 10)

    # Limit angle to 0-255, round to nearest int for 1° resolution
    angle_uint8 = max(0, min(255, int(round(angle_deg))))

    # Pack: 4-byte signed ADC + 2-byte unsigned pressure + 1-byte unsigned angle
    packet = struct.pack('>iHB', adc_value, pressure_scaled, angle_uint8)
    sx.send(packet)
    print("Sent packet (binary):", packet)

# ============ Begin of Loop  for Mag Sensor ============
while False:

    strength = magSensor.readMagnitude()   # Reads the magnetic-field strength in microTesla (uT)
    myString = str(strength) + ' uT'       # create a string with the field-strength and the unit
    print(magSensor.read())                        # Print the field strength
    # printdisplay(magSensor.read())                        # Print the field strength
    displaymag()
    # sx.send(b'Ping')
    sx.send(magSensor.read())
    if strength > threshold:               # Check if the magnetic field is strong
        print('Strong Magnet!')

    sleep_ms(1000)

def compute_rope_inclination(ax, ay, az):
    """
    Computes the angle between the rope (Y-axis of device) and gravity,
    immune to rope spinning around its axis.
    Returns angle in degrees: 0° = vertical, 90° = horizontal.
    """
    norm = math.sqrt(ax**2 + ay**2 + az**2)
    ay /= norm
    angle_rad = math.acos(abs(ay))  # Use abs to always get smallest angle to gravity
    return math.degrees(angle_rad)

# ============ Begin of Loop for ADS1232 ============
# Example loop
gain = config.ADS_GAIN
set_gain(gain)

def tare(samples=10):
    return sum(read_adc() for _ in range(samples)) // samples

offset = tare()

while True:

    adc_val = read_adc()
    print("ADC value:", adc_val - offset)
    druck = pressure()
    angle = 90 - compute_rope_inclination(*read_average_acceleration(20))
    print("Seilwinkel:", angle)
    send_adc_and_pressure_over_lora(adc_val, druck, angle)
    display.fill(0)
    display.text("Seilwinkel:", 0, 0)
    display.text("{:.1f} deg".format(angle), 0, 10)
    display.show()
    time.sleep(0.5)
    

# Try each gain and print readings
#    for gain in [1, 2, 64, 128]:
#        set_gain(gain)
#        time.sleep(0.1)
#        for _ in range(5):
#            print(f"ADC (gain {gain}):", read_adc())
#            time.sleep(0.5)
