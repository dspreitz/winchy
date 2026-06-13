# T-Beam S3 Supreme board definition: power rails and bus singletons.
#
# Only this module and config.py know how the board is wired. Drivers and
# the application get their buses from here instead of constructing their
# own (the old monolith created three different setups of the same I2C bus).

from machine import I2C, Pin, SPI, SoftI2C, UART

import config
from AXP2101 import AXP2101, AXP2101_SLAVE_ADDRESS, XPOWERS_AXP2101_CHIP_ID

# Bus singletons. Constructing these does not touch the attached chips, so
# it is safe before init_power() has enabled the sensor rails.
i2c0 = I2C(0, scl=Pin(config.I2C0_SCL), sda=Pin(config.I2C0_SDA),
           freq=config.I2C0_FREQ)
gps_uart = UART(1, baudrate=config.GPS_BAUD, bits=8, parity=None, stop=1,
                tx=Pin(config.GPS_TX), rx=Pin(config.GPS_RX), timeout=300)
qmi_spi = SPI(2, baudrate=1000000, polarity=1, phase=1,
              sck=Pin(config.QMI_SCK), mosi=Pin(config.QMI_MOSI),
              miso=Pin(config.QMI_MISO))
qmi_cs = Pin(config.QMI_CS, Pin.OUT, value=1)

# AXP2101 rail plan: (rail, millivolts, enable).
# DC1 powers the ESP32-S3 itself - set but never toggled here.
# BLDO2 supplies the ADS1232 breakout.
_RAILS = (
    ("DC1", 3300, False),
    ("DC2", 1000, True),
    ("DC3", 3300, True),
    ("DC4", 1000, True),
    ("DC5", 3300, True),
    ("ALDO1", 3300, True),
    ("ALDO2", 3300, True),
    ("ALDO3", 3300, True),
    ("ALDO4", 3300, True),
    ("BLDO1", 3300, True),
    ("BLDO2", 3300, True),
    ("CPUSLDO", 1000, True),
    ("DLDO1", 3300, True),
    ("DLDO2", 3300, True),
)


def init_power():
    """Configure the AXP2101 PMU and enable all peripheral rails.

    Returns the PMU object. Settings replicate the original monolith:
    same rail voltages, charger profile, IRQ mask and watchdog state.
    """
    pmu_i2c = SoftI2C(scl=Pin(config.PMU_SCL), sda=Pin(config.PMU_SDA))
    pmu = AXP2101(pmu_i2c, addr=AXP2101_SLAVE_ADDRESS)
    if pmu.getChipID() != XPOWERS_AXP2101_CHIP_ID:
        raise RuntimeError("AXP2101 PMU not found")

    pmu.setVbusVoltageLimit(pmu.XPOWERS_AXP2101_VBUS_VOL_LIM_4V36)
    pmu.setVbusCurrentLimit(pmu.XPOWERS_AXP2101_VBUS_CUR_LIM_1500MA)
    pmu.setSysPowerDownVoltage(2600)

    for rail, millivolts, enable in _RAILS:
        getattr(pmu, "set" + rail + "Voltage")(millivolts)
        if enable:
            getattr(pmu, "enable" + rail)()

    pmu.setPowerKeyPressOffTime(pmu.XPOWERS_POWEROFF_6S)
    pmu.setPowerKeyPressOnTime(pmu.XPOWERS_POWERON_2S)

    # No battery temperature sensor fitted; leaving TS measurement on
    # disturbs charging.
    pmu.disableTSPinMeasure()
    pmu.enableBattDetection()
    pmu.enableVbusVoltageMeasure()
    pmu.enableBattVoltageMeasure()
    pmu.enableSystemVoltageMeasure()
    pmu.setChargingLedMode(pmu.XPOWERS_CHG_LED_OFF)

    pmu.disableIRQ(pmu.XPOWERS_AXP2101_ALL_IRQ)
    pmu.clearIrqStatus()
    pmu.enableIRQ(
        pmu.XPOWERS_AXP2101_BAT_INSERT_IRQ | pmu.XPOWERS_AXP2101_BAT_REMOVE_IRQ |
        pmu.XPOWERS_AXP2101_VBUS_INSERT_IRQ | pmu.XPOWERS_AXP2101_VBUS_REMOVE_IRQ |
        pmu.XPOWERS_AXP2101_PKEY_SHORT_IRQ | pmu.XPOWERS_AXP2101_PKEY_LONG_IRQ |
        pmu.XPOWERS_AXP2101_BAT_CHG_DONE_IRQ | pmu.XPOWERS_AXP2101_BAT_CHG_START_IRQ
    )

    pmu.setPrechargeCurr(pmu.XPOWERS_AXP2101_PRECHARGE_50MA)
    # 1 A ~= 0.29C for the Samsung INR18650-35E (3500 mAh); needs a >=1.3 A
    # USB source to actually reach it (a weak laptop port will throttle).
    pmu.setChargerConstantCurr(pmu.XPOWERS_AXP2101_CHG_CUR_1000MA)
    pmu.setChargerTerminationCurr(pmu.XPOWERS_AXP2101_CHG_ITERM_25MA)
    pmu.setChargeTargetVoltage(pmu.XPOWERS_AXP2101_CHG_VOL_4V1)

    pmu.setWatchdogConfig(pmu.XPOWERS_AXP2101_WDT_IRQ_TO_PIN)
    pmu.setWatchdogTimeout(pmu.XPOWERS_AXP2101_WDT_TIMEOUT_4S)
    pmu.disableWatchdog()
    pmu.clearIrqStatus()

    print("PMU: system %dmV, battery %dmV (%d%%)" % (
        pmu.getSystemVoltage(), pmu.getBattVoltage(), pmu.getBatteryPercent()))
    return pmu
