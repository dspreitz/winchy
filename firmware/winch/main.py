# Winch segment (ground station) - LilyGo T3S3 V1.2.
#
# Receives Winchy protocol frames from the rope unit and shows them to the
# winch operator. Tracks sequence gaps as a link quality measure, and when a
# TELEMETRY frame requests it, transmits a LINK_REPORT back (measured RSSI,
# SNR and recent loss) so the rope can adapt its radio settings. The radio
# stays in non-blocking mode, which auto-returns to RX after each TX, so the
# reply needs no explicit mode juggling here.
#
# Hardware-verified on a T3S3 V1.2 (2026-06-13): boots, SX1262 inits at the
# pinout below, decodes live TELEMETRY frames from the rope unit, OLED works.
# Deploy protocol.py (from firmware/shared/) alongside this file.

import time

from machine import I2C, Pin
import ssd1306
from sx1262 import SX1262
from _sx126x import ERR_NONE

import protocol

# Radio parameters: SF/BW/CR/sync/freq must match firmware/rope/config.py.
LORA_FREQ_MHZ = 868.0
LORA_BW_KHZ = 500.0
LORA_SF = 7
LORA_CR = 8
LORA_SYNC_WORD = 0x12
LORA_TX_POWER_DBM = 14  # uplink (LINK_REPORT) power; fixed, no ADR on the winch

lora = SX1262(spi_bus=1, clk=5, mosi=6, miso=3, cs=7, irq=33, rst=8, gpio=34)
lora.begin(freq=LORA_FREQ_MHZ, bw=LORA_BW_KHZ, sf=LORA_SF, cr=LORA_CR,
           syncWord=LORA_SYNC_WORD, power=LORA_TX_POWER_DBM, currentLimit=60.0,
           preambleLength=8, implicit=False, crcOn=True,
           tcxoVoltage=1.7, useRegulatorLDO=False, blocking=True)

i2c = I2C(0, sda=Pin(18), scl=Pin(17))
display = ssd1306.SSD1306_I2C(128, 64, i2c)

last_seq = None
received = 0
lost = 0
tx_seq = 0          # our own (winch) transmit sequence, for LINK_REPORTs
last_rssi = 0       # dBm of the most recent frame
last_snr = 0        # dB of the most recent frame
recv_window = 0     # frames received since the last report
lost_window = 0     # frames lost since the last report
loss_ema = 0.0      # smoothed loss %, so a single gap in the tiny per-report
                    # window doesn't swing the figure to 50/100%
LOSS_EMA_ALPHA = 0.3
blink = 0           # render counter; alternates the bottom line when warning
WARN_BLINK_FRAMES = 4   # frames per state (~2 s at 2 Hz) when battery is low

# Flash logging of received frames, for range tests when the winch is
# untethered (so we see the downlink directly instead of inferring it from
# the back-channel). Records are buffered in RAM by the RX callback and
# written in the main loop - never do flash I/O in the IRQ or we'd stall the
# radio and drop the frames we're trying to count. Disable for routine use to
# avoid flash wear; clear winch_rxlog.csv before each run for a fresh log.
LOG_TO_FLASH = True
LOG_PATH = "winch_rxlog.csv"
log_buf = []        # pending "t_ms,seq,rssi,snr,flags" lines


def show_telemetry(msg):
    global blink
    blink += 1
    display.fill(0)
    display.text("Winchy " + protocol.PHASE_NAMES.get(msg["phase"], "?"),
                 0, 0)
    unit = "cnt" if msg["flags"] & protocol.FLAG_FORCE_UNCALIBRATED else "N"
    display.text("F: {} {}".format(msg["force"], unit), 0, 14)
    display.text("Angle: {:.1f}".format(msg["angle_deg"]), 0, 26)
    display.text("Alt: {} m".format(msg["altitude_m"]), 0, 38)
    # Bottom line shows link info, but while the rope reports a low battery it
    # alternates with a warning so the operator can't miss it.
    if (msg["flags"] & protocol.FLAG_BATTERY_LOW
            and (blink // WARN_BLINK_FRAMES) % 2 == 0):
        bottom = "!BATT LOW {:.1f}V".format(msg["batt_v"])
    else:
        bottom = "rx{} l{} {}dBm".format(received, lost, last_rssi)
    display.text(bottom, 0, 54)
    display.show()


def send_link_report():
    # EMA-smooth the per-report-window loss so a single gap in the (tiny)
    # window doesn't swing the reported figure. Sending auto-returns to RX.
    global tx_seq, recv_window, lost_window, loss_ema
    total = recv_window + lost_window
    window_loss = (100.0 * lost_window / total) if total else 0.0
    loss_ema = LOSS_EMA_ALPHA * window_loss + (1 - LOSS_EMA_ALPHA) * loss_ema
    loss_pct = int(round(loss_ema))
    lora.send(protocol.encode_link_report(tx_seq, last_rssi, last_snr,
                                          loss_pct))
    tx_seq = (tx_seq + 1) & 0xFFFF
    recv_window = 0
    lost_window = 0
    print("[TX] link report rssi={} snr={} loss={}%".format(
        last_rssi, last_snr, loss_pct))


def on_receive(events):
    global last_seq, received, lost, last_rssi, last_snr
    global recv_window, lost_window
    if not (events & SX1262.RX_DONE):
        return
    frame, err = lora.recv()
    if err != ERR_NONE:
        print("Receive error:", lora.STATUS[err])
        return
    last_rssi = int(lora.getRSSI())
    last_snr = int(lora.getSNR())
    msg = protocol.decode(frame)
    if msg is None:
        print("Ignoring unknown frame:", frame)
        return
    received += 1
    recv_window += 1
    seq = msg["seq"]
    if last_seq is not None:
        gap = (seq - last_seq) & 0xFFFF
        if 1 < gap < 0x8000:  # forward gap = lost frames; backward = restart
            lost += gap - 1
            lost_window += gap - 1
    last_seq = seq

    if msg["type"] == protocol.TELEMETRY:
        if LOG_TO_FLASH:  # buffer only; the main loop does the flash write
            log_buf.append("%d,%d,%d,%d,%d\n" % (
                time.ticks_ms(), seq, last_rssi, last_snr, msg["flags"]))
        print("[RX]", msg)
        show_telemetry(msg)
        if msg["flags"] & protocol.FLAG_REQUEST_REPORT:
            send_link_report()
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

if LOG_TO_FLASH:
    logf = open(LOG_PATH, "a")  # append so a reboot mid-walk keeps prior data
    logf.write("# boot ticks_ms=%d\n" % time.ticks_ms())
    logf.flush()

while True:
    time.sleep(1)
    if LOG_TO_FLASH and log_buf:
        # Write the snapshot length, then drop exactly those; anything the IRQ
        # appends meanwhile stays for the next pass.
        n = len(log_buf)
        for i in range(n):
            logf.write(log_buf[i])
        del log_buf[0:n]
        logf.flush()
