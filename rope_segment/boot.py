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


# This file is executed on every boot (including wake-boot from deepsleep)
import esp
#esp.osdebug(None)
import webrepl
#webrepl.start()

import math

from AXP2101 import *
import time
# import machine
from machine import I2C, Pin, SPI, UART, SoftI2C
from bmp280 import *
import struct
import sh1106 # Display library

from sx1262 import SX1262 # LoRa library

i2c = I2C(0, scl=Pin(18), sda=Pin(17), freq=400000)

# ============ Begin of AXP2101 ============
SDA = None
SCL = None
IRQ = None
I2CBUS = None

if implementation.name == 'micropython':
    from machine import Pin, SoftI2C
    SDA = 42
    SCL = 41
    IRQ = 40
    I2CBUS = SoftI2C(scl=Pin(SCL), sda=Pin(SDA))
pmu_flag = False
irq = None


def __callback(args):
    global pmu_flag
    pmu_flag = True
    # print('callback')


PMU = AXP2101(I2CBUS, addr=AXP2101_SLAVE_ADDRESS)

id = PMU.getChipID()
if id != XPOWERS_AXP2101_CHIP_ID:
    print("PMU is not online...")
    while True:
        pass

print('getID:%s' % hex(PMU.getChipID()))

#  Set the minimum common working voltage of the PMU VBUS input,
#  below this value will turn off the PMU
PMU.setVbusVoltageLimit(PMU.XPOWERS_AXP2101_VBUS_VOL_LIM_4V36)

#  Set the maximum current of the PMU VBUS input,
#  higher than this value will turn off the PMU
PMU.setVbusCurrentLimit(PMU.XPOWERS_AXP2101_VBUS_CUR_LIM_1500MA)

#  Get the VSYS shutdown voltage
vol = PMU.getSysPowerDownVoltage()
print('->  getSysPowerDownVoltage:%u' % vol)

#  Set VSY off voltage as 2600mV, Adjustment range 2600mV ~ 3300mV
PMU.setSysPowerDownVoltage(2600)

vol = PMU.getSysPowerDownVoltage()
print('->  getSysPowerDownVoltage:%u' % vol)

#  DC1 IMAX = 2A
#  1500~3400mV, 100mV/step, 20steps
PMU.setDC1Voltage(3300)
# print('DC1  : %s   Voltage:%u mV ' % PMU.isEnableDC1()  ? '+': '-', PMU.getDC1Voltage())
print('DC1  : {0}   Voltage:{1} mV '.format(
    ('-', '+')[PMU.isEnableDC1()], PMU.getDC1Voltage()))


#  DC2 IMAX = 2A
#  500~1200mV  10mV/step, 71steps
#  1220~1540mV 20mV/step, 17steps
PMU.setDC2Voltage(1000)
print(PMU.isEnableDC2())
print('DC2  : {0}   Voltage:{1} mV '.format(
    ('-', '+')[PMU.isEnableDC2()], PMU.getDC2Voltage()))

#  DC3 IMAX = 2A
#  500~1200mV, 10mV/step, 71steps
#  1220~1540mV, 20mV/step, 17steps
#  1600~3400mV, 100mV/step, 19steps
PMU.setDC3Voltage(3300)
print('DC3  : {0}   Voltage:{1} mV '.format(
    ('-', '+')[PMU.isEnableDC3()], PMU.getDC3Voltage()))

#  DCDC4 IMAX = 1.5A
#  500~1200mV, 10mV/step, 71steps
#  1220~1840mV, 20mV/step, 32steps
PMU.setDC4Voltage(1000)
print('DC4  : {0}   Voltage:{1} mV '.format(
    ('-', '+')[PMU.isEnableDC4()], PMU.getDC4Voltage()))

#  DC5 IMAX = 2A
#  1200mV
#  1400~3700mV, 100mV/step, 24steps
PMU.setDC5Voltage(3300)
print('DC5  : {0}   Voltage:{1} mV '.format(
    ('-', '+')[PMU.isEnableDC5()], PMU.getDC5Voltage()))

# ALDO1 IMAX = 300mA
# 500~3500mV, 100mV/step, 31steps
PMU.setALDO1Voltage(3300)

# ALDO2 IMAX = 300mA
# 500~3500mV, 100mV/step, 31steps
PMU.setALDO2Voltage(3300)

# ALDO3 IMAX = 300mA
# 500~3500mV, 100mV/step, 31steps
PMU.setALDO3Voltage(3300)

# ALDO4 IMAX = 300mA
# 500~3500mV, 100mV/step, 31steps
PMU.setALDO4Voltage(3300)

# BLDO1 IMAX = 300mA
# 500~3500mV, 100mV/step, 31steps
PMU.setBLDO1Voltage(3300)

# BLDO2 IMAX = 300mA - This supports the ADS1232 with 5V
# 500~3500mV, 100mV/step, 31steps
PMU.setBLDO2Voltage(3300)

# CPUSLDO IMAX = 30mA
# 500~1400mV, 50mV/step, 19steps
PMU.setCPUSLDOVoltage(1000)

# DLDO1 IMAX = 300mA
# 500~3400mV, 100mV/step, 29steps
PMU.setDLDO1Voltage(3300)

# DLDO2 IMAX = 300mA
# 500~1400mV, 50mV/step, 2steps
PMU.setDLDO2Voltage(3300)

#  PMU.enableDC1()
PMU.enableDC2()
PMU.enableDC3()
PMU.enableDC4()
PMU.enableDC5()
PMU.enableALDO1()
PMU.enableALDO2()
PMU.enableALDO3()
PMU.enableALDO4()
PMU.enableBLDO1()
PMU.enableBLDO2()
PMU.enableCPUSLDO()
PMU.enableDLDO1()
PMU.enableDLDO2()

print('===================================')
print('DC1    : {0}   Voltage:{1} mV '.format(
    ('-', '+')[PMU.isEnableDC1()], PMU.getDC1Voltage()))
print('DC2    : {0}   Voltage:{1} mV '.format(
    ('-', '+')[PMU.isEnableDC2()], PMU.getDC2Voltage()))
print('DC3    : {0}   Voltage:{1} mV '.format(
    ('-', '+')[PMU.isEnableDC3()], PMU.getDC3Voltage()))
print('DC4    : {0}   Voltage:{1} mV '.format(
    ('-', '+')[PMU.isEnableDC4()], PMU.getDC4Voltage()))
print('DC5    : {0}   Voltage:{1} mV '.format(
    ('-', '+')[PMU.isEnableDC5()], PMU.getDC5Voltage()))
print('===================================')
print('ALDO1  : {0}   Voltage:{1} mV '.format(
    ('-', '+')[PMU.isEnableALDO1()], PMU.getALDO1Voltage()))
print('ALDO2  : {0}   Voltage:{1} mV '.format(
    ('-', '+')[PMU.isEnableALDO2()], PMU.getALDO2Voltage()))
print('ALDO3  : {0}   Voltage:{1} mV '.format(
    ('-', '+')[PMU.isEnableALDO3()], PMU.getALDO3Voltage()))
print('ALDO4  : {0}   Voltage:{1} mV '.format(
    ('-', '+')[PMU.isEnableALDO4()], PMU.getALDO4Voltage()))
print('===================================')
print('BLDO1  : {0}   Voltage:{1} mV '.format(
    ('-', '+')[PMU.isEnableBLDO1()], PMU.getBLDO1Voltage()))
print('BLDO2  : {0}   Voltage:{1} mV '.format(
    ('-', '+')[PMU.isEnableBLDO2()], PMU.getBLDO2Voltage()))
print('===================================')
print('CPUSLDO: {0}   Voltage:{1} mV '.format(
    ('-', '+')[PMU.isEnableCPUSLDO()], PMU.getCPUSLDOVoltage()))
print('===================================')
print('DLDO1  : {0}   Voltage:{1} mV '.format(
    ('-', '+')[PMU.isEnableDLDO1()], PMU.getDLDO1Voltage()))
print('DLDO2  : {0}   Voltage:{1} mV '.format(
    ('-', '+')[PMU.isEnableDLDO2()], PMU.getDLDO2Voltage()))
print('===================================')

#  Set the time of pressing the button to turn off
powerOff = ['4', '6', '8', '10']
PMU.setPowerKeyPressOffTime(PMU.XPOWERS_POWEROFF_6S)
opt = PMU.getPowerKeyPressOffTime()
print('PowerKeyPressOffTime: %s Sceond' % powerOff[opt])


#  Set the button power-on press time
powerOn = ['128ms', '512ms', '1000ms', '2000ms']
PMU.setPowerKeyPressOnTime(PMU.XPOWERS_POWERON_2S)
opt = PMU.getPowerKeyPressOnTime()
print('PowerKeyPressOnTime: %s ' % powerOn[opt])


print('===================================')

#  DCDC 120 % (130 %) high voltage turn off PMIC function
en = PMU.getDCHighVoltagePowerDowmEn()
print('getDCHighVoltagePowerDowmEn:%s' % ('DISABLE', 'ENABLE')[en])

#  DCDC1 85 % low voltage turn off PMIC function
en = PMU.getDC1LowVoltagePowerDowmEn()
print('getDC1LowVoltagePowerDowmEn:%s' % ('DISABLE', 'ENABLE')[en])

#  DCDC2 85 % low voltage turn off PMIC function
en = PMU.getDC2LowVoltagePowerDowmEn()
print('getDC2LowVoltagePowerDowmEn:%s' % ('DISABLE', 'ENABLE')[en])

#  DCDC3 85 % low voltage turn off PMIC function
en = PMU.getDC3LowVoltagePowerDowmEn()
print('getDC3LowVoltagePowerDowmEn:%s' % ('DISABLE', 'ENABLE')[en])

#  DCDC4 85 % low voltage turn off PMIC function
en = PMU.getDC4LowVoltagePowerDowmEn()
print('getDC4LowVoltagePowerDowmEn:%s' % ('DISABLE', 'ENABLE')[en])

#  DCDC5 85 % low voltage turn off PMIC function
en = PMU.getDC5LowVoltagePowerDowmEn()
print('getDC5LowVoltagePowerDowmEn:%s' % ('DISABLE', 'ENABLE')[en])


#  PMU.setDCHighVoltagePowerDowm(true)
#  PMU.setDC1LowVoltagePowerDowm(true)
#  PMU.setDC2LowVoltagePowerDowm(true)
#  PMU.setDC3LowVoltagePowerDowm(true)
#  PMU.setDC4LowVoltagePowerDowm(true)
#  PMU.setDC5LowVoltagePowerDowm(true)

#  It is necessary to disable the detection function of the TS pin on the board
#  without the battery temperature detection function, otherwise it will cause abnormal charging
PMU.disableTSPinMeasure()

#  PMU.enableTemperatureMeasure()

#  Enable internal ADC detection
PMU.enableBattDetection()
PMU.enableVbusVoltageMeasure()
PMU.enableBattVoltageMeasure()
PMU.enableSystemVoltageMeasure()

'''
The default setting is CHGLED is automatically controlled by the PMU.
- XPOWERS_CHG_LED_OFF,
- XPOWERS_CHG_LED_BLINK_1HZ,
- XPOWERS_CHG_LED_BLINK_4HZ,
- XPOWERS_CHG_LED_ON,
- XPOWERS_CHG_LED_CTRL_CHG,
'''
PMU.setChargingLedMode(PMU.XPOWERS_CHG_LED_OFF)


#  Disable all interrupts
PMU.disableIRQ(PMU.XPOWERS_AXP2101_ALL_IRQ)
#  Clear all interrupt flags
PMU.clearIrqStatus()
#  Enable the required interrupt function
PMU.enableIRQ(
    PMU.XPOWERS_AXP2101_BAT_INSERT_IRQ | PMU.XPOWERS_AXP2101_BAT_REMOVE_IRQ |  # BATTERY
    PMU.XPOWERS_AXP2101_VBUS_INSERT_IRQ | PMU.XPOWERS_AXP2101_VBUS_REMOVE_IRQ |  # VBUS
    PMU.XPOWERS_AXP2101_PKEY_SHORT_IRQ | PMU.XPOWERS_AXP2101_PKEY_LONG_IRQ |  # POWER KEY
    PMU.XPOWERS_AXP2101_BAT_CHG_DONE_IRQ | PMU.XPOWERS_AXP2101_BAT_CHG_START_IRQ  # CHARGE
    #  PMU.XPOWERS_AXP2101_PKEY_NEGATIVE_IRQ | PMU.XPOWERS_AXP2101_PKEY_POSITIVE_IRQ | # POWER KEY
)

#  Set the precharge charging current
PMU.setPrechargeCurr(PMU.XPOWERS_AXP2101_PRECHARGE_50MA)
#  Set constant current charge current limit
PMU.setChargerConstantCurr(PMU.XPOWERS_AXP2101_CHG_CUR_200MA)
#  Set stop charging termination current
PMU.setChargerTerminationCurr(PMU.XPOWERS_AXP2101_CHG_ITERM_25MA)

#  Set charge cut-off voltage
PMU.setChargeTargetVoltage(PMU.XPOWERS_AXP2101_CHG_VOL_4V1)

#  Set the watchdog trigger event type
PMU.setWatchdogConfig(PMU.XPOWERS_AXP2101_WDT_IRQ_TO_PIN)
#  Set watchdog timeout
PMU.setWatchdogTimeout(PMU.XPOWERS_AXP2101_WDT_TIMEOUT_4S)
#  Enable watchdog to trigger interrupt event
#  PMU.enableWatchdog()

PMU.disableWatchdog()

PMU.clearIrqStatus()


data = [1, 2, 3, 4]
print('Write buffer to pmu')
PMU.writeDataBuffer(data, 4)
print('Read buffer from pmu')
tmp = PMU.readDataBuffer(4)
print(tmp)


if implementation.name == 'micropython':
    irq = Pin(IRQ, Pin.IN, Pin.PULL_UP)
    irq.irq(trigger=Pin.IRQ_FALLING, handler=__callback)


while True:

    if pmu_flag:
        pmu_flag = False
        mask = PMU.getIrqStatus()
        print('pmu_flag:', end='')
        print(bin(mask))

        if PMU.isPekeyShortPressIrq():
            print("IRQ ---> isPekeyShortPress")
        if PMU.isPekeyLongPressIrq():
            print("IRQ ---> isPekeyLongPress")
        if PMU.isPekeyNegativeIrq():
            print("IRQ ---> isPekeyNegative")
        if PMU.isPekeyPositiveIrq():
            print("IRQ ---> isPekeyPositive")
        if PMU.isWdtExpireIrq():
            print("IRQ ---> isWdtExpire")

        PMU.clearIrqStatus()

    PMU.setChargingLedMode((PMU.XPOWERS_CHG_LED_OFF, PMU.XPOWERS_CHG_LED_ON)[
                           PMU.getChargingLedMode() == PMU.XPOWERS_CHG_LED_OFF])
    print("getBattVoltage:{0}mV".format(PMU.getBattVoltage()))
    print("getSystemVoltage:{0}mV".format(PMU.getSystemVoltage()))
    print("getBatteryPercent:{0}%".format(PMU.getBatteryPercent()))

    print("isCharging:{0}".format(PMU.isCharging()))
    print("isDischarge:{0}".format(PMU.isDischarge()))
    print("isStandby:{0}".format(PMU.isStandby()))
    break
    # time.sleep(0.8)

# ============ End of PMU Initialization ============

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
uart = UART(1,baudrate = 9600, bits=8, parity=None, stop=1, tx = Pin(8),rx = Pin(9), timeout=300)
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

spi = SPI(2, baudrate=1000000, polarity=1, phase=1, sck=Pin(36), mosi=Pin(35), miso=Pin(37))
cs = Pin(34,Pin.OUT)
cs.value(1) # Set CS high to start

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

magSensor = PiicoDev_QMC6310(bus=0, freq=1000000, scl=Pin(18), sda=Pin(17), range=3000) # initialise the magnetometer
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
display = sh1106.SH1106_I2C(128, 64, i2c, Pin(16), 0x3c)
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
        
sx = SX1262(spi_bus=1, clk=12, mosi=11, miso=13, cs=10, irq=1, rst=5, gpio=4)
# print("2")
# LoRa
sx.begin(freq=868.0, bw=500.0, sf=12, cr=8, syncWord=0x12,
         power=-5, currentLimit=60.0, preambleLength=8,
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
PDWN = Pin(39, Pin.OUT)  # Power down / reset 39
SCLK = Pin(45, Pin.OUT)  # Serial clock 45
DOUT = Pin(46, Pin.IN)   # Data out / data ready (DRDY) 46
GAIN0 = Pin(38, Pin.OUT)    # GAIN0 connected to IO38
GAIN1 = Pin(2, Pin.OUT)     # GAIN1 connected to IO2

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
gain = 128
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
