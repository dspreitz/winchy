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

import os
import time

from machine import I2C, Pin, RTC
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
LOG_TO_FLASH = True         # log every RX frame to flash; also served at /log
LOG_PATH = "winch_rxlog.csv"
# Written at open and after an offload-reset; a file of exactly this size has
# no data rows, so it isn't uploaded.
LOG_HEADER = ("# boot\n# utc,seq,phase,force,angle_deg,alt_m,batt_v,batt_pct,"
              "flags,rssi,snr,speed_ms\n")
log_buf = []        # pending CSV rows: utc,seq,phase,force,angle_deg,alt_m,
                    # batt_v,batt_pct,flags,rssi,snr  (utc = ISO8601 Z)

# Optional WiFi dashboard: the winch joins an existing WiFi network and serves
# a live telemetry page (HTTP poll, ~2 Hz) to any phone/laptop on that network
# - a real operator display alongside the small OLED. WiFi is winch-only and
# never touches the LoRa link. Credentials live in secrets.py (gitignored, on
# the device only) so they stay out of the repo:
#     WIFI_SSID = "spreitz intern"
#     WIFI_PASSWORD = "..."
# Browse to the IP printed at boot. Set WIFI_ENABLED = False for OLED-only.
WIFI_ENABLED = True
WIFI_HOSTNAME = "winchy"    # reachable as winchy.local (mDNS) + via router DNS
try:
    from secrets import WIFI_SSID, WIFI_PASSWORD
except ImportError:
    WIFI_SSID = WIFI_PASSWORD = None   # no secrets.py -> dashboard stays off

# Winch-direct GitHub upload of the received log (no PC in the loop). Needs a
# fine-grained PAT scoped to GITHUB_REPO ("Contents: read and write") stored in
# secrets.py as GITHUB_TOKEN. No token -> uploads disabled (log still kept and
# served at /log). The asset is replaced each upload (one rolling file).
GITHUB_REPO = "dspreitz/winchy-logs"
GITHUB_RELEASE_TAG = "logs"
GITHUB_ASSET = "winch_rxlog.csv"
UPLOAD_PERIOD_S = 600       # interim: re-upload every 10 min while on WiFi
                            # (later: trigger once per launch via phase detection)
try:
    from secrets import GITHUB_TOKEN
except ImportError:
    GITHUB_TOKEN = None

# Latest decoded telemetry, served as JSON. Rebuilt (not mutated) in the RX
# callback so the server always reads a complete snapshot.
latest = {"phase": "--", "force": 0, "uncal": True, "angle": 0.0, "alt": 0,
          "speed": 0.0, "rssi": 0, "batt": 0.0, "battlow": False,
          "battpct": 255, "charging": False, "tsync": False,
          "time": "--:--:--", "tsrc": "--", "rx": 0, "lost": 0}

# Winch RTC sync: NTP (via WiFi) is preferred; the rope's GPS time over the
# radio (TIME_SYNC frame) is the fallback when there's no internet/WiFi.
clock_set = False           # is the winch RTC set?
clock_src = "--"            # "NTP" / "GPS" / "--"
log_start = None            # "yyyymmdd-hhmm" session start, latched once synced


def show_telemetry(msg):
    global blink
    blink += 1
    display.fill(0)
    # Top line: phase left, glider speed (km/h) right-aligned.
    display.text(protocol.PHASE_NAMES.get(msg["phase"], "?"), 0, 0)
    spd = "%.0f" % (msg["glider_speed_ms"] * 3.6)
    display.text(spd, 128 - len(spd) * 8, 0)   # 8 px per char, 128 px wide
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


def _set_rtc_unix(epoch_s):
    # epoch_s is Unix UTC seconds; gmtime wants seconds since 2000-01-01.
    tm = time.gmtime(epoch_s - 946684800)
    RTC().datetime((tm[0], tm[1], tm[2], tm[6], tm[3], tm[4], tm[5], 0))


def _stamp():
    # "yyyymmdd-hhmm" from the UTC RTC, or None if it isn't set yet.
    if not clock_set:
        return None
    t = time.localtime()
    return "%04d%02d%02d-%02d%02d" % (t[0], t[1], t[2], t[3], t[4])


def _ntp_sync():
    global clock_set, clock_src
    try:
        import ntptime
        ntptime.settime()       # sets the RTC to UTC from pool.ntp.org
        clock_set = True
        clock_src = "NTP"
        print("RTC set from NTP")
    except Exception as e:
        print("NTP sync failed:", e)


def _github_upload():
    # Replace the rolling log asset on the 'logs' release: look up the release,
    # delete any existing asset of the same name, then upload the current log.
    # Blocks the asyncio loop for the round-trips (~secs); RX stays IRQ-driven.
    # Returns the byte count offloaded (0 on failure) so the caller can reset.
    if not GITHUB_TOKEN:
        return 0
    import urequests
    import gc
    uploaded = 0
    # Prepend the session start (first synced time) so each session is a
    # distinct, dated file on the release.
    stamp = log_start or _stamp()
    asset = (stamp + "_" if stamp else "") + GITHUB_ASSET
    hdr = {"Authorization": "Bearer " + GITHUB_TOKEN, "User-Agent": "winchy",
           "Accept": "application/vnd.github+json"}
    try:
        r = urequests.get("https://api.github.com/repos/%s/releases/tags/%s"
                          % (GITHUB_REPO, GITHUB_RELEASE_TAG), headers=hdr)
        rel = r.json()
        r.close()
        rid = rel["id"]
        for a in rel.get("assets", ()):
            if a.get("name") == asset:
                urequests.delete(
                    "https://api.github.com/repos/%s/releases/assets/%d"
                    % (GITHUB_REPO, a["id"]), headers=hdr).close()
        gc.collect()
        body = open(LOG_PATH, "rb").read()
        h2 = dict(hdr)
        h2["Content-Type"] = "text/csv"
        u = urequests.post(
            "https://uploads.github.com/repos/%s/releases/%d/assets?name=%s"
            % (GITHUB_REPO, rid, asset), data=body, headers=h2)
        print("GitHub upload %s: HTTP %d" % (asset, u.status_code))
        if 200 <= u.status_code < 300:
            uploaded = len(body)
        u.close()
    except Exception as e:
        print("GitHub upload failed:", e)
    gc.collect()
    return uploaded


def _reset_log(uploaded):
    # Reclaim flash after a successful offload, but only if nothing new was
    # flushed since (else keep the rows for the next upload). IRQ-buffered rows
    # in log_buf are in RAM and flush into the fresh file afterwards.
    global logf
    if logf is None:
        return
    try:
        if os.stat(LOG_PATH)[6] > uploaded:
            return
    except OSError:
        return
    logf.close()
    try:
        os.remove(LOG_PATH)
    except OSError:
        pass
    logf = open(LOG_PATH, "a")
    logf.write(LOG_HEADER)
    logf.flush()
    print("winch_rxlog.csv offloaded; reset")


def on_receive(events):
    global last_seq, received, lost, last_rssi, last_snr
    global recv_window, lost_window, latest, clock_set, clock_src
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
        tm = time.localtime()           # RTC is UTC (NTP/GPS-synced)
        if LOG_TO_FLASH:  # buffer only; the main loop does the flash write
            log_buf.append(
                "%04d-%02d-%02dT%02d:%02d:%02dZ,%d,%d,%d,%.1f,%d,%.1f,%d,%d,%d,%d,%.1f\n"
                % (tm[0], tm[1], tm[2], tm[3], tm[4], tm[5], seq, msg["phase"],
                   msg["force"], msg["angle_deg"], msg["altitude_m"],
                   msg["batt_v"], msg["batt_pct"], msg["flags"],
                   last_rssi, last_snr, msg["glider_speed_ms"]))
        latest = {"phase": protocol.PHASE_NAMES.get(msg["phase"], "?"),
                  "force": msg["force"],
                  "uncal": bool(msg["flags"] & protocol.FLAG_FORCE_UNCALIBRATED),
                  "angle": msg["angle_deg"], "alt": msg["altitude_m"],
                  "speed": msg["glider_speed_ms"],
                  "rssi": last_rssi, "batt": msg["batt_v"],
                  "battlow": bool(msg["flags"] & protocol.FLAG_BATTERY_LOW),
                  "battpct": msg["batt_pct"],
                  "charging": bool(msg["flags"] & protocol.FLAG_CHARGING),
                  "gps": bool(msg["flags"] & protocol.FLAG_GPS_FIX),
                  "tsync": clock_set,   # winch RTC set (NTP or GPS)?
                  "time": ("%02d:%02d:%02d" % (tm[3], tm[4], tm[5])
                           if clock_set else "--:--:--"),
                  "tsrc": clock_src,
                  "rx": received, "lost": lost}
        print("[RX]", msg)
        show_telemetry(msg)
        if msg["flags"] & protocol.FLAG_REQUEST_REPORT:
            send_link_report()
    elif msg["type"] == protocol.TIME_SYNC:
        # Rope's GPS time. NTP (over WiFi) has priority, so only use this as a
        # fallback when the clock isn't already NTP-synced.
        if clock_src != "NTP":
            try:
                _set_rtc_unix(msg["epoch_s"])
                clock_set = True
                clock_src = "GPS"
            except Exception as e:
                print("RTC from GPS failed:", e)
        print("[RX] time sync, unix epoch", msg["epoch_s"])
    else:
        print("[RX]", msg)


PAGE = """<!DOCTYPE html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>Winchy</title><style>
body{font-family:sans-serif;background:#111;color:#eee;margin:0;text-align:center}
#phase{font-size:13vw;font-weight:bold;padding:8px;background:#223}
.big{font-size:10vw;font-weight:bold}.lbl{font-size:4vw;color:#8ac}
.row{display:flex}.cell{flex:1;padding:8px}
#warn{background:#c00;color:#fff;font-size:6vw;padding:10px;display:none}
.stale{opacity:.35}
.dot{display:inline-block;width:7vw;height:7vw;border-radius:50%;background:#555;vertical-align:middle;margin-right:2vw}
</style></head><body>
<div id=warn>! ROPE BATTERY LOW</div>
<div id=phase>--</div>
<div class=row><div class=cell><span id=gpsdot class=dot></span><span class=lbl>GPS</span></div>
<div class=cell><span id=tdot class=dot></span><span class=lbl>TIME</span></div></div>
<div id=clock class=lbl>--:--:--</div>
<div class=cell><div class=lbl>SPEED km/h</div><div id=speed class=big>--</div></div>
<div class=row><div class=cell><div class=lbl>FORCE</div><div id=force class=big>--</div></div>
<div class=cell><div class=lbl>ANGLE</div><div id=angle class=big>--</div></div></div>
<div class=row><div class=cell><div class=lbl>ALT m</div><div id=alt class=big>--</div></div>
<div class=cell><div class=lbl>BATT</div><div id=batt class=big>--</div><div id=battsub class=lbl>--</div></div></div>
<div id=link class=lbl>link --</div>
<script>
var last=Date.now();
function tick(){
 fetch('/data').then(function(r){return r.json()}).then(function(d){
  last=Date.now();document.body.className='';
  phase.textContent=d.phase;
  speed.textContent=(d.speed*3.6).toFixed(0);
  force.textContent=d.force+(d.uncal?' c':' N');
  angle.textContent=d.angle.toFixed(1);
  alt.textContent=d.alt;
  batt.textContent=(d.battpct>100?'--':d.battpct+'%')+(d.charging?' CHG':'');
  battsub.textContent=d.batt.toFixed(1)+' V';
  link.textContent='link '+d.rssi+' dBm   rx'+d.rx+' lost'+d.lost;
  warn.style.display=d.battlow?'block':'none';
  gpsdot.style.background=d.gps?'#1c1':'#c33';
  tdot.style.background=d.tsync?'#1c1':'#c33';
  clock.textContent=d.time+' UTC'+(d.tsrc!='--'?' ('+d.tsrc+')':'');
 }).catch(function(e){});
 if(Date.now()-last>3000){document.body.className='stale';
  gpsdot.style.background='#555';tdot.style.background='#555';}
}
setInterval(tick,500);tick();
</script></body></html>"""


async def handle(reader, writer):
    try:
        req = await reader.readline()
        while True:                       # drain request headers
            h = await reader.readline()
            if not h or h == b"\r\n":
                break
        path = req.split(b" ")[1] if b" " in req else b"/"
        if path.startswith(b"/data"):
            writer.write(b"HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
                         b"Connection: close\r\n\r\n")
            writer.write(json.dumps(latest).encode())
        elif path.startswith(b"/log"):   # serve the flash log for offload
            try:
                body = open(LOG_PATH, "rb").read()
            except OSError:
                body = b""
            writer.write(b"HTTP/1.1 200 OK\r\nContent-Type: text/csv\r\n"
                         b"Connection: close\r\n\r\n")
            writer.write(body)
        else:
            writer.write(b"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n"
                         b"Connection: close\r\n\r\n")
            writer.write(PAGE.encode())
        await writer.drain()
    except Exception as e:
        print("http err:", e)
    try:
        writer.close()
        await writer.wait_closed()
    except Exception:
        pass


def _flush_log():
    if logf is None or not log_buf:
        return
    n = len(log_buf)                      # write snapshot, drop exactly those;
    for i in range(n):                    # anything the IRQ adds meanwhile waits
        logf.write(log_buf[i])
    del log_buf[0:n]
    logf.flush()


async def _serve():
    if WIFI_SSID:
        try:                              # MUST precede active(True) so mDNS
            network.hostname(WIFI_HOSTNAME)   # advertises <name>.local
        except Exception:
            pass
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        if not wlan.isconnected():
            wlan.connect(WIFI_SSID, WIFI_PASSWORD)
            for _ in range(40):           # wait up to ~20 s for join + DHCP
                if wlan.isconnected():
                    break
                await asyncio.sleep_ms(500)
        if wlan.isconnected():
            print("WiFi '%s' joined - http://%s/  or  http://%s.local/"
                  % (WIFI_SSID, wlan.ifconfig()[0], WIFI_HOSTNAME))
            _ntp_sync()                   # NTP has priority over rope GPS time
            await asyncio.start_server(handle, "0.0.0.0", 80)
        else:
            print("WiFi: could not join '%s'; dashboard off" % WIFI_SSID)
    else:
        print("WiFi: no secrets.py; dashboard off")
    global log_start
    n = 0
    while True:                           # keep-alive + periodic flash flush
        _flush_log()
        if clock_set and log_start is None:   # latch session start once synced
            log_start = _stamp()
        n += 1
        if n % 3600 == 0:                 # re-sync NTP hourly (corrects drift)
            _ntp_sync()
        if GITHUB_TOKEN and n % UPLOAD_PERIOD_S == 0:
            try:                          # only if there are rows to offload
                has_data = os.stat(LOG_PATH)[6] > len(LOG_HEADER)
            except OSError:
                has_data = False
            if has_data:
                up = _github_upload()     # interim periodic; later: per launch
                if up:
                    _reset_log(up)        # reclaim flash after a good upload
        await asyncio.sleep_ms(1000)


lora.setBlockingCallback(False, on_receive)

display.fill(0)
display.text("Waiting for data", 0, 0)
display.show()
print("Winch receiver ready")

logf = None
if LOG_TO_FLASH:
    logf = open(LOG_PATH, "a")  # append so a reboot mid-run keeps prior data
    logf.write(LOG_HEADER)      # delimiter; data rows carry UTC once synced
    logf.flush()

if WIFI_ENABLED:
    import asyncio
    import network
    import json
    # RX stays IRQ-driven; this runs WiFi + dashboard + periodic flush.
    asyncio.run(_serve())
else:
    while True:
        time.sleep(1)
        _flush_log()
