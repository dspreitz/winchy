from sx1262 import SX1262
from machine import Pin, I2C
import time
import struct
import ssd1306
from _sx126x import ERR_NONE, ERR_CRC_MISMATCH, SX126X_IRQ_RX_DONE

# =================== LoRa Configuration (T3S3 V1.2 Pinout)
lora = SX1262(spi_bus=1, clk=5, mosi=6, miso=3, cs=7, irq=33, rst=8, gpio=34)

lora.begin(
    freq=868.0, bw=500.0, sf=12, cr=8, syncWord=0x12,
    power=10, currentLimit=60.0, preambleLength=8,
    implicit=False, crcOn=True,
    tcxoVoltage=1.7, useRegulatorLDO=False,
    blocking=True
)

# =================== OLED Setup
i2c = I2C(sda=Pin(18), scl=Pin(17))
display = ssd1306.SSD1306_I2C(128, 64, i2c)
display.fill(0)
display.text('Waiting for data...', 0, 0)
display.show()

# =================== OLED Display Helpers
def show_on_display(adc_val, pressure_hpa, angle):
    display.fill(0)
    display.text("LoRa Receiver", 0, 0)
    display.text(f"ADC: {adc_val}", 0, 16)
    display.text(f"Pressure:", 0, 32)
    display.text(f"{pressure_hpa:.1f} hPa", 0, 42)
    display.text(f"{angle_deg:.1f} hPa", 0, 52)
    display.show()

# =================== Packet Decoder
def decode_packet_old(packet):
    if len(packet) != 6:
        print("Invalid packet length:", len(packet))
        return
    adc_val, pressure_raw = struct.unpack('>iH', packet)
    pressure_hpa = pressure_raw / 10.0
    print(f"[RECEIVED] ADC: {adc_val}, Pressure: {pressure_hpa:.1f} hPa")
    show_on_display(adc_val, pressure_hpa)

def decode_packet(packet):
    if len(packet) != 7:
        print("Invalid packet length:", len(packet))
        return

    try:
        adc_val, pressure_raw, angle_deg = struct.unpack('>iHB', packet)
        pressure_hpa = pressure_raw / 10.0

        print(f"[RECEIVED] ADC: {adc_val}, Pressure: {pressure_hpa:.1f} hPa, Angle: {angle_deg}°")
        
        # Optional: display on OLED
        show_on_display(adc_val, pressure_hpa, angle)
        
    except Exception as e:
        print("Error decoding packet:", e)

# =================== LoRa Receive Callback
def on_receive(events):
    if events & SX1262.RX_DONE:
        packet, err = lora.recv()
        if err == ERR_NONE:
            decode_packet(packet)
            display.text('Received something...', 0, 0)
            display.show()
        else:
            print("Receive error:", lora.STATUS[err])

lora.setBlockingCallback(False, on_receive)

# =================== Main Loop
print("LoRa receiver with OLED display ready")
while True:
    time.sleep(1)
