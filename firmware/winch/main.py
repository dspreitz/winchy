# Winch segment (ground station) - LilyGo T3S3 V1.2.
#
# Receives Winchy protocol frames from the rope unit and shows them to the
# winch operator. Tracks sequence gaps as a link quality measure.
#
# NOTE: rewritten against shared/protocol.py but NOT yet hardware-verified
# (no T3S3 available). Pinout carried over from the original receiver
# draft. Deploy protocol.py (from firmware/shared/) alongside this file.

import time

from machine import I2C, Pin
import ssd1306
from sx1262 import SX1262
from _sx126x import ERR_NONE

import protocol

# Radio parameters: must match firmware/rope/config.py.
LORA_FREQ_MHZ = 868.0
LORA_BW_KHZ = 500.0
LORA_SF = 12
LORA_CR = 8
LORA_SYNC_WORD = 0x12

lora = SX1262(spi_bus=1, clk=5, mosi=6, miso=3, cs=7, irq=33, rst=8, gpio=34)
lora.begin(freq=LORA_FREQ_MHZ, bw=LORA_BW_KHZ, sf=LORA_SF, cr=LORA_CR,
           syncWord=LORA_SYNC_WORD, power=10, currentLimit=60.0,
           preambleLength=8, implicit=False, crcOn=True,
           tcxoVoltage=1.7, useRegulatorLDO=False, blocking=True)

i2c = I2C(0, sda=Pin(18), scl=Pin(17))
display = ssd1306.SSD1306_I2C(128, 64, i2c)

last_seq = None
received = 0
lost = 0


def show_telemetry(msg):
    display.fill(0)
    display.text("Winchy " + protocol.PHASE_NAMES.get(msg["phase"], "?"),
                 0, 0)
    unit = "cnt" if msg["flags"] & protocol.FLAG_FORCE_UNCALIBRATED else "N"
    display.text("F: {} {}".format(msg["force"], unit), 0, 14)
    display.text("Angle: {:.1f}".format(msg["angle_deg"]), 0, 26)
    display.text("P: {:.1f} hPa".format(msg["pressure_hpa"]), 0, 38)
    display.text("rx{} lost{}".format(received, lost), 0, 54)
    display.show()


def on_receive(events):
    global last_seq, received, lost
    if not (events & SX1262.RX_DONE):
        return
    frame, err = lora.recv()
    if err != ERR_NONE:
        print("Receive error:", lora.STATUS[err])
        return
    msg = protocol.decode(frame)
    if msg is None:
        print("Ignoring unknown frame:", frame)
        return
    received += 1
    seq = msg["seq"]
    if last_seq is not None:
        gap = (seq - last_seq) & 0xFFFF
        if 1 < gap < 0x8000:  # forward gap = lost frames; backward = restart
            lost += gap - 1
    last_seq = seq

    if msg["type"] == protocol.TELEMETRY:
        print("[RX]", msg)
        show_telemetry(msg)
    elif msg["type"] == protocol.TIME_SYNC:
        # Rope unit announces GPS time at startup; RTC sync goes here.
        print("[RX] time sync, unix epoch", msg["epoch_s"])
    else:
        print("[RX]", msg)


lora.setBlockingCallback(False, on_receive)

display.fill(0)
display.text("Waiting for data", 0, 0)
display.show()
print("Winch receiver ready")

while True:
    time.sleep(1)
