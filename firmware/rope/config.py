# Rope segment configuration: pin map (LilyGo T-Beam S3 Supreme) and
# tunables. Schematic:
# https://github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series/blob/master/schematic/LilyGo_T-BeamS3Supreme.pdf

# I2C bus 0: BMP280 (0x77), QMC6310 magnetometer (0x1c), SH1106 OLED (0x3c)
I2C0_SCL = 18
I2C0_SDA = 17
I2C0_FREQ = 400000

# AXP2101 PMU on its own SoftI2C bus
PMU_SCL = 41
PMU_SDA = 42
PMU_IRQ = 40

# u-blox M10S GPS
GPS_TX = 8
GPS_RX = 9
GPS_BAUD = 9600
GPS_PPS = 6   # u-blox 1PPS (TIMEPULSE) -> ESP32 GPIO6, disciplines the RTC
# Nav/output rate. At 9600 baud only GGA+RMC fit at 5 Hz (~770 B/s); higher
# needs a faster baud. gps.configure() trims to GGA+RMC and applies this.
GPS_NAV_RATE_HZ = 5

# QMI8658 IMU on SPI2
QMI_SCK = 36
QMI_MOSI = 35
QMI_MISO = 37
QMI_CS = 34
QMI_INT = 33

# SX1262 LoRa radio
LORA_SPI_BUS = 1
LORA_CLK = 12
LORA_MOSI = 11
LORA_MISO = 13
LORA_CS = 10
LORA_IRQ = 1
LORA_RST = 5
LORA_BUSY = 4  # 'gpio' parameter of the sx1262 driver

# LoRa link parameters - SF/BW/CR/sync/freq must match the winch segment.
# SF7 (not 12): range to come is short and line-of-sight, and SF7/BW500 still
# closes a multi-km airborne link while cutting airtime ~25x (~15 ms/frame),
# which keeps the duty cycle sane and leaves room for the link back-channel.
LORA_FREQ_MHZ = 868.0
LORA_BW_KHZ = 500.0
LORA_SF = 7
LORA_CR = 8
LORA_SYNC_WORD = 0x12
LORA_TX_POWER_DBM = 14  # start at the EU ERP cap; ADR adapts down from here

# ADS1232 force ADC
ADS_PDWN = 39
ADS_SCLK = 45
ADS_DOUT = 46
ADS_GAIN0 = 38
ADS_GAIN1 = 2
ADS_GAIN = 128

# SH1106 OLED. Disabled: the panel is physically blocked by the ADS1232
# breakout PCB, so it only wastes power. Set True to re-enable the status
# page and the display task.
DISPLAY_ENABLED = False
OLED_RST = 16
OLED_ADDR = 0x3C
OLED_WIDTH = 128
OLED_HEIGHT = 64
