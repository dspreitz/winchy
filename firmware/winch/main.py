# Winchy - glider winch rope force & advice system
# Copyright (C) 2026 Dominic Spreitz
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version. Distributed WITHOUT ANY WARRANTY.
# See the GNU General Public License for more details, and the LICENSE
# file or <https://www.gnu.org/licenses/> for the full text.
#
# SPDX-License-Identifier: GPL-3.0-or-later

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
import struct
import time

import micropython
from machine import I2C, Pin, RTC, UART
import ssd1306
from sx1262 import SX1262
from _sx126x import ERR_NONE

import protocol
import nmea
import wifi
from survey import SurveyIn

# Radio parameters: SF/BW/CR/sync/freq must match firmware/rope/config.py.
# Band g3 (869.525 MHz, BW 250 kHz): 10% duty cycle, power/range headroom, and
# clear of the FLARM band (868.2-868.4 MHz). g3 is 250 kHz wide, so BW <= 250.
LORA_FREQ_MHZ = 869.525
LORA_BW_KHZ = 250.0
LORA_SF = 7
LORA_CR = 8
LORA_SYNC_WORD = 0x12
LORA_TX_POWER_DBM = 22  # +22 dBm (SX1262 max), within g3's 500 mW ERP; the winch
                        # is mains-powered (no battery cost) and fixed (no ADR).
                        # Keeps the link symmetric so the back-channel reaches the
                        # rope at range. Needs OCP >= 140 mA (set in begin()).

lora = SX1262(spi_bus=1, clk=5, mosi=6, miso=3, cs=7, irq=33, rst=8, gpio=34)
lora.begin(freq=LORA_FREQ_MHZ, bw=LORA_BW_KHZ, sf=LORA_SF, cr=LORA_CR,
           syncWord=LORA_SYNC_WORD, power=LORA_TX_POWER_DBM, currentLimit=140.0,
           preambleLength=8, implicit=False, crcOn=True,
           tcxoVoltage=1.7, useRegulatorLDO=False, blocking=True)

i2c = I2C(0, sda=Pin(18), scl=Pin(17))
display = ssd1306.SSD1306_I2C(128, 64, i2c)

# GPS (interim u-blox 7 on UART1; see tools/gps_uart_probe.py). We survey-in the
# parked winch's position and send it to the rope as WINCH_POS. The survey is
# receiver-agnostic, so this same code runs on the Supreme's M10 later - only
# these pins change. Verified on the T3S3: rx=42, tx=41, 9600 baud NMEA.
GPS_UART_ID = 1
GPS_RX_PIN = 42
GPS_TX_PIN = 41
GPS_BAUD = 9600
GPS_PPS_PIN = 40            # GPS 1PPS -> exact UTC second boundary (RTC sync)
WINCH_POS_PERIOD_S = 15      # how often to (re)send the surveyed position

gps_uart = UART(GPS_UART_ID, baudrate=GPS_BAUD, rx=GPS_RX_PIN, tx=GPS_TX_PIN,
                timeout=50)
survey = SurveyIn()
gps_sats = 0
gps_has_fix = False
_gps_buf = b""

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
last_rx_ms = 0      # ticks_ms of last telemetry RX; GPS owns the OLED when idle

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
              "flags,rssi,snr,speed_ms,winch_lat,winch_lon\n")
log_buf = []        # pending CSV rows: utc,seq,phase,force,angle_deg,alt_m,
                    # batt_v,batt_pct,flags,rssi,snr  (utc = ISO8601 Z)

# Optional WiFi dashboard: the winch joins an existing WiFi network and serves
# a live telemetry page (HTTP poll, ~2 Hz) to any phone/laptop on that network
# - a real operator display alongside the small OLED. WiFi is winch-only and
# never touches the LoRa link. Credentials live in secrets.py (gitignored, on
# the device only) so they stay out of the repo:
#     WIFI_NETWORKS = [("ssid1", "pass1"), ("ssid2", "pass2"), ...]
# It joins the strongest in-range network from the list (see wifi.py). Browse
# to the IP printed at boot. Set WIFI_ENABLED = False for OLED-only.
WIFI_ENABLED = True
WIFI_HOSTNAME = "winchy"    # reachable as winchy.local (mDNS) + via router DNS
WIFI_RETRY_S = 30           # when WiFi is down, retry the join this often (s)
WIFI_PROBE_S = 30           # how often to probe winchy-logs reachability (s)
try:
    from secrets import WIFI_NETWORKS              # [(ssid, password), ...]
except ImportError:
    try:                                           # back-compat: single AP
        from secrets import WIFI_SSID, WIFI_PASSWORD
        WIFI_NETWORKS = [(WIFI_SSID, WIFI_PASSWORD)]
    except ImportError:
        WIFI_NETWORKS = []

# Winch-direct GitHub upload of the received log (no PC in the loop). Needs a
# fine-grained PAT scoped to GITHUB_REPO ("Contents: read and write") stored in
# secrets.py as GITHUB_TOKEN. No token -> uploads disabled (log still kept and
# served at /log). The asset is replaced each upload (one rolling file).
GITHUB_REPO = "dspreitz/winchy-logs"
GITHUB_RELEASE_TAG = "logs"
GITHUB_ASSET = "winch_rxlog.csv"
UPLOAD_PERIOD_S = 600       # interim: re-upload every 10 min while on WiFi
                            # (later: trigger once per launch via phase detection)
# Blocking GitHub round-trips (log upload, IP announce) stall the single-thread
# asyncio loop for seconds; never run them while a launch is active or the live
# OLED/dashboard would freeze mid-launch (roadmap #12). "Active" = recent
# telemetry with the glider moving faster than ground-handling. The phase stays
# IDLE until the phase machine exists, so received glider speed is the signal.
UPLOAD_PAUSE_SPEED_MS = 8   # >~29 km/h received glider speed = launch -> hold off
try:
    from secrets import GITHUB_TOKEN
except ImportError:
    GITHUB_TOKEN = None

# Latest decoded telemetry, served as JSON. Rebuilt (not mutated) in the RX
# callback so the server always reads a complete snapshot.
latest = {"phase": "--", "force": 0, "uncal": True, "angle": 0.0, "alt": 0,
          "speed": 0.0, "rssi": 0, "batt": 0.0, "battlow": False,
          "battpct": 255, "charging": False, "tsync": False,
          "time": "--:--:--", "rx": 0, "lost": 0,
          "wsats": 0, "wfix": False, "wlat": None, "wlon": None,
          "wacc": None, "wconv": False}


def _survey_fields():
    # Winch GPS / survey-in status for the OLED and dashboard.
    acc = survey.accuracy_m
    return {"wsats": gps_sats, "wfix": gps_has_fix, "wconv": survey.converged,
            "wlat": round(survey.lat, 6) if survey.lat is not None else None,
            "wlon": round(survey.lon, 6) if survey.lon is not None else None,
            "wacc": None if acc == float("inf") else round(acc, 1)}

# Winch RTC: set from the winch's own GPS+PPS (the exact UTC second boundary).
# No NTP and no time-over-radio any more - both segments keep their own GPS time.
clock_set = False           # is the winch RTC set (from its own GPS+PPS)?
log_start = None            # "yyyymmdd-hhmm" session start, latched once synced
online = False              # winchy-logs (GitHub) currently reachable? -> OLED "I"


uploading = False   # True while a blocking GitHub round-trip runs: the OLED
                    # shows an upload notice and the IRQ telemetry redraw is
                    # suppressed, so the notice stays put through the freeze and
                    # the operator knows the pause is expected (not a crash).
_upload_request = False   # dashboard "Upload log" button -> picked up by _serve


def _begin_upload(what):
    global uploading
    uploading = True
    try:
        display.fill(0)
        display.text("WiFi upload:", 0, 8)
        display.text(what, 0, 24)
        display.text("display pauses", 0, 44)
        display.text("back shortly...", 0, 54)
        display.show()
    except Exception:
        pass


def _end_upload():
    # Normal screen behaviour resumes on the next telemetry / survey redraw.
    global uploading
    uploading = False


def show_telemetry(msg):
    global blink
    blink += 1
    display.fill(0)
    # Top line: phase left, glider speed (km/h) right-aligned.
    display.text(protocol.PHASE_NAMES.get(msg["phase"], "?"), 0, 0)
    spd = "%.0f" % (msg["glider_speed_ms"] * 3.6)
    right = 120 if online else 128             # leave the last cell for the "I"
    display.text(spd, right - len(spd) * 8, 0)  # 8 px/char, 128 px wide
    if online:                                  # winchy-logs reachable
        display.text("I", 120, 0)
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


def _send_winch_pos():
    # Send the surveyed winch position to the rope (low rate). Like the
    # LINK_REPORT, a send auto-returns the radio to RX. accuracy_m may be inf
    # (n<2) or large; the byte field saturates at 25.5 m.
    global tx_seq
    acc = survey.accuracy_m
    if acc == float("inf") or acc > 25.5:
        acc = 25.5
    status = 0
    if gps_has_fix:
        status |= protocol.WINCH_FIX
    if survey.converged:
        status |= protocol.WINCH_SURVEY_DONE
    lora.send(protocol.encode_winch_pos(tx_seq, survey.lat, survey.lon,
                                        survey.alt or 0.0, acc, status))
    tx_seq = (tx_seq + 1) & 0xFFFF
    print("[TX] winch_pos lat=%.6f lon=%.6f acc=%.1f conv=%s"
          % (survey.lat, survey.lon, acc, survey.converged))


def _show_survey():
    # OLED screen shown while no telemetry is arriving, so the operator can
    # watch the GPS lock and the survey converge.
    display.fill(0)
    display.text("WINCH GPS", 0, 0)
    if online:                                  # winchy-logs reachable
        display.text("I", 120, 0)
    display.text("sat %d  %s" % (gps_sats, "FIX" if gps_has_fix else "no fix"),
                 0, 14)
    if survey.lat is None:
        display.text("waiting for fix", 0, 34)
    else:
        acc = survey.accuracy_m
        display.text("n=%d" % survey.n, 0, 26)
        display.text("acc %s m" % ("--" if acc == float("inf")
                                   else "%.1f" % acc), 0, 38)
        display.text("SURVEYED" if survey.converged else "surveying...", 0, 54)
    display.show()


# --- GPS cold-start aiding (UBX-AID-INI) -----------------------------------
# Feed the u-blox 7 its last-known position (cached on flash) at boot, and -
# once WiFi gives us the time - the time, so a cold start fixes in seconds
# instead of ~30 s. The cache is for AIDING ONLY: WINCH_POS is still sent solely
# from the live, converged survey, never from the cache. (AID-INI is the
# u-blox 6/7 path - HW 00070000 / PROTVER 14 here; the M10 winch will use MGA.)
AID_CACHE_PATH = "winch_aid.json"
AID_SAVE_S = 300              # re-cache the converged position at most this often
AID_POS_ACC_M = 100           # confidence the winch sits at the cached spot
AID_TIME_ACC_MS = 5000        # coarse time (HTTP Date + latency + leap margin)
_GPS_LEAP_S = 18              # GPS-UTC offset (18 s since 2017)
_HTTP_MONTHS = ("Jan", "Feb", "Mar", "Apr", "May", "Jun",
                "Jul", "Aug", "Sep", "Oct", "Nov", "Dec")
_aid_utc = None               # (y,mo,d,h,mi,s) from the internet; set by _serve
_aided_time = False           # time aiding already injected this boot
_aid_save_ms = 0


def _aid_ubx(cls, mid, payload):
    body = bytes((cls, mid)) + len(payload).to_bytes(2, "little") + payload
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return b"\xb5\x62" + body + bytes((a, b))


def _aid_ini(lat=None, lon=None, alt_m=0.0, week=None, tow_ms=0):
    # UBX-AID-INI (0x0B 0x01): position (lat/lon 1e-7 deg, alt cm) and/or time
    # (GPS week + TOW ms). flags: 0x01 pos, 0x20 lla(geodetic), 0x02 time.
    flags = 0
    ex = ey = ez = pacc = 0
    if lat is not None:
        ex = int(round(lat * 1e7)); ey = int(round(lon * 1e7))
        ez = int(round(alt_m * 100)); pacc = int(AID_POS_ACC_M * 100)
        flags |= 0x21
    wn = tow = tacc = 0
    if week is not None:
        wn = week & 0xFFFF; tow = tow_ms & 0xFFFFFFFF; tacc = AID_TIME_ACC_MS
        flags |= 0x02
    payload = struct.pack("<iiiIHHIiIIiII",
                          ex, ey, ez, pacc, 0, wn, tow, 0, tacc, 0, 0, 0, flags)
    return _aid_ubx(0x0B, 0x01, payload)


def _utc_to_gps(y, mo, d, h, mi, s):
    # UTC -> (GPS week, time-of-week ms). mktime is 2000-epoch on the ESP32;
    # +630720000 shifts to the 1980-01-06 GPS epoch, + leap seconds to GPS time.
    g = time.mktime((y, mo, d, h, mi, s, 0, 0)) + 630720000 + _GPS_LEAP_S
    return g // 604800, (g % 604800) * 1000


def _save_aid(lat, lon, alt):
    try:
        with open(AID_CACHE_PATH, "w") as f:
            f.write('{"lat":%.7f,"lon":%.7f,"alt":%.1f}' % (lat, lon, alt))
    except OSError:
        pass


def _load_aid():
    import json
    try:
        d = json.loads(open(AID_CACHE_PATH).read())
        return d["lat"], d["lon"], d["alt"]
    except (OSError, ValueError, KeyError):
        return None


def _parse_http_date(s):
    # "Mon, 22 Jun 2026 20:43:21 GMT" -> (y, mo, d, h, mi, s) UTC, or None.
    try:
        p = s.split()
        return (int(p[3]), _HTTP_MONTHS.index(p[2]) + 1, int(p[1]),
                int(p[4][0:2]), int(p[4][3:5]), int(p[4][6:8]))
    except (ValueError, IndexError, AttributeError):
        return None


def _aid_fetch_time():
    # One-shot: UTC for GPS time aiding (the winch RTC is cold until a fix, so
    # the internet is the only pre-fix clock). NTP first (accurate, no auth);
    # HTTP Date as a fallback if NTP/UDP is blocked. Does NOT set the RTC - that
    # stays GPS+PPS-disciplined; this only seeds the receiver's time aid.
    global _aid_utc
    if _aid_utc is not None:
        return
    try:
        import ntptime
        _aid_utc = time.gmtime(ntptime.time())[:6]
        print("GPS aiding: NTP UTC", _aid_utc)
        return
    except Exception as e:
        print("aid NTP failed, trying HTTP Date:", e)
    import urequests
    try:
        # User-Agent required or GitHub 403s (and the 403 carries no Date).
        r = urequests.get("https://api.github.com",
                          headers={"User-Agent": "winchy"})
        d = r.headers.get("Date") if getattr(r, "headers", None) else None
        r.close()
        t = _parse_http_date(d)
        if t:
            _aid_utc = t
            print("GPS aiding: HTTP UTC", t)
    except Exception as e:
        print("aid HTTP date failed:", e)


async def _gps_task():
    # Drain NMEA, feed the survey-in, periodically send WINCH_POS, and own the
    # OLED when the rope link is idle. RX/TX of the LoRa link is unaffected
    # (it is IRQ-driven); this only shares the radio for the low-rate send.
    global gps_sats, gps_has_fix, _gps_buf, _pps_rtc, clock_set
    global _aided_time, _aid_save_ms
    _pps_rtc = RTC()
    Pin(GPS_PPS_PIN, Pin.IN).irq(trigger=Pin.IRQ_RISING, handler=_on_pps)
    last_send = time.ticks_ms()
    pos = _load_aid()                         # cached last-known position
    if pos is not None:                       # inject it now for a fast fix
        gps_uart.write(_aid_ini(lat=pos[0], lon=pos[1], alt_m=pos[2]))
        print("GPS aiding: position injected (%.6f, %.6f, %.0f m)"
              % (pos[0], pos[1], pos[2]))
    while True:
        if gps_uart.any():
            _gps_buf += gps_uart.read() or b""
            if len(_gps_buf) > 1024:           # guard a wedged/garbage stream
                _gps_buf = _gps_buf[-256:]
            while b"\n" in _gps_buf:
                line, _gps_buf = _gps_buf.split(b"\n", 1)
                m = nmea.parse_nmea(line)
                if not m:
                    continue
                if m["type"] == "GGA":
                    gps_sats = m["sats"]
                    gps_has_fix = m["fix"] > 0
                    if (gps_has_fix and m["lat"] is not None
                            and m["lon"] is not None):
                        survey.add(m["lat"], m["lon"], m["alt_m"] or 0.0)
                elif m["type"] == "RMC" and m["datetime"]:
                    y, mo, d, h, mi, s = m["datetime"]
                    if not clock_set:    # rough set now; PPS refines on the edge
                        RTC().datetime((y, mo, d, 0, h, mi, s, 0))
                        clock_set = True
                        print("RTC set from GPS RMC; PPS disciplining on GPIO %d"
                              % GPS_PPS_PIN)
                    _pps_arm_next(y, mo, d, h, mi, s)
        now = time.ticks_ms()
        if (survey.lat is not None
                and time.ticks_diff(now, last_send) >= WINCH_POS_PERIOD_S * 1000):
            _send_winch_pos()
            last_send = now
        # Inject time aiding once the internet has given us UTC (via _serve).
        if _aid_utc is not None and not _aided_time:
            wk, tw = _utc_to_gps(*_aid_utc)
            gps_uart.write(_aid_ini(week=wk, tow_ms=tw))
            _aided_time = True
            print("GPS aiding: time injected (GPS week %d)" % wk)
        # Cache the converged position for the next boot's aiding (throttled,
        # flash-wear-bounded). AIDING ONLY - WINCH_POS stays live-survey-sourced.
        if (survey.converged and survey.lat is not None
                and time.ticks_diff(now, _aid_save_ms) >= AID_SAVE_S * 1000):
            _save_aid(survey.lat, survey.lon, survey.alt or 0.0)
            _aid_save_ms = now
        latest.update(_survey_fields())
        if time.ticks_diff(now, last_rx_ms) > 3000:   # link idle -> show GPS
            _show_survey()
        await asyncio.sleep_ms(200)


# GPS 1PPS RTC discipline (same approach as the rope): NMEA RMC gives the time
# but arrives with latency; the PPS rising edge is the exact UTC second
# boundary. So from each RMC we pre-arm the RTC for the NEXT whole second and
# write that on the edge. The hard IRQ only schedules the apply (no allocation
# in IRQ context). This is the winch's only time source now (no NTP, no
# time-over-radio); the RTC stays unset until the GPS gets a fix.
_pps_rtc = None         # RTC() handle
_pps_armed = None       # datetime tuple to write on the next edge, or None
_pps_count = 0          # rising edges seen (liveness / debug)


def _pps_apply(_):
    global _pps_armed, clock_set
    a = _pps_armed
    if a is not None and _pps_rtc is not None:
        _pps_rtc.datetime(a)
        _pps_armed = None
        clock_set = True


def _on_pps(pin):
    global _pps_count
    _pps_count += 1
    micropython.schedule(_pps_apply, 0)


def _pps_arm_next(y, mo, d, h, mi, s):
    # Arm the RTC for the NEXT whole second (the upcoming PPS edge), UTC.
    # Guard a glitched GPS date: time.mktime() overflows a 32-bit machine word
    # for years beyond ~2068, which crash-looped the unit (crash-guard reset).
    # Reject implausible dates and never let a bad fix take the unit down.
    global _pps_armed
    if not (2024 <= y <= 2050 and 1 <= mo <= 12 and 1 <= d <= 31):
        return
    try:
        nxt = time.gmtime(time.mktime((y, mo, d, h, mi, s, 0, 0)) + 1)
    except (OverflowError, ValueError):
        return
    _pps_armed = (nxt[0], nxt[1], nxt[2], nxt[6], nxt[3], nxt[4], nxt[5], 0)


def _stamp():
    # "yyyymmdd-hhmm" from the UTC RTC, or None if it isn't set yet.
    if not clock_set:
        return None
    t = time.localtime()
    return "%04d%02d%02d-%02d%02d" % (t[0], t[1], t[2], t[3], t[4])


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


def _github_announce_ip(ip, ssid):
    # Publish a TAPPABLE dashboard link in the winchy-logs release description.
    # The release page renders the body as markdown, so on the phone you open
    # the release and tap the IP -> http://<ip>/ opens the winch dashboard. Each
    # segment owns one line ("rope:"/"winch:"); read-merge-write so we don't
    # clobber the rope's line. Called on join / IP change (rare).
    if not GITHUB_TOKEN:
        return False
    import urequests
    import gc
    hdr = {"Authorization": "Bearer " + GITHUB_TOKEN, "User-Agent": "winchy",
           "Accept": "application/vnd.github+json"}
    line = "winch: [http://%s/](http://%s/) (%s, %s)" % (
        ip, ip, ssid, _stamp() or "?")
    ok = False
    try:
        r = urequests.get("https://api.github.com/repos/%s/releases/tags/%s"
                          % (GITHUB_REPO, GITHUB_RELEASE_TAG), headers=hdr)
        rel = r.json()
        r.close()
        rid = rel["id"]
        body = rel.get("body") or ""
        keep = [ln for ln in body.split("\n")
                if ln.strip() and not ln.startswith("winch:")]
        keep.append(line)
        gc.collect()
        u = urequests.request(
            "PATCH", "https://api.github.com/repos/%s/releases/%d"
            % (GITHUB_REPO, rid),
            data=json.dumps({"body": "\n".join(sorted(keep))}), headers=hdr)
        ok = 200 <= u.status_code < 300
        print("IP announce %s (%s): HTTP %d" % (ip, ssid, u.status_code))
        u.close()
    except Exception as e:
        print("IP announce failed:", e)
    gc.collect()
    return ok


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
    global recv_window, lost_window, latest, clock_set, last_rx_ms
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
        last_rx_ms = time.ticks_ms()    # telemetry owns the OLED over the GPS
        tm = time.localtime()           # RTC is UTC (GPS+PPS); for the dash clock
        if LOG_TO_FLASH:  # buffer only; the main loop does the flash write
            # ms-precision UTC stamp, PPS-disciplined; one ns read keeps the
            # seconds and the ms consistent (no second-boundary race).
            ns = time.time_ns()
            g = time.gmtime(ns // 1000000000)
            ms = (ns // 1000000) % 1000
            # the winch's own surveyed position, so a walk/launch is mappable
            # (and any survey drift/reposition is captured per frame)
            wlat = survey.lat if survey.lat is not None else 0.0
            wlon = survey.lon if survey.lon is not None else 0.0
            log_buf.append(
                "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ,%d,%d,%d,%.1f,%d,%.1f,%d,%d,%d,%d,%.1f,%.6f,%.6f\n"
                % (g[0], g[1], g[2], g[3], g[4], g[5], ms, seq, msg["phase"],
                   msg["force"], msg["angle_deg"], msg["altitude_m"],
                   msg["batt_v"], msg["batt_pct"], msg["flags"],
                   last_rssi, last_snr, msg["glider_speed_ms"], wlat, wlon))
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
                  "tsync": clock_set,   # winch RTC set from GPS+PPS?
                  "time": ("%02d:%02d:%02d" % (tm[3], tm[4], tm[5])
                           if clock_set else "--:--:--"),
                  "rx": received, "lost": lost}
        latest.update(_survey_fields())
        print("[RX]", msg)
        if not uploading:           # keep the upload notice on screen if busy
            show_telemetry(msg)
        if msg["flags"] & protocol.FLAG_REQUEST_REPORT:
            send_link_report()
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
<div id=winch class=lbl>winch GPS --</div>
<button id=ulbtn onclick=ul() style="font-size:5vw;padding:12px;margin:8px 2px;width:96%;background:#235;color:#eee;border:1px solid #8ac;border-radius:6px">Upload log to GitHub</button>
<div id=ulmsg class=lbl>&nbsp;</div>
<script>
var last=Date.now();var wasup=false;
function ul(){ulmsg.textContent='requested...';
 fetch('/upload',{method:'POST'}).then(function(r){return r.text()}).then(function(t){ulmsg.textContent=t;}).catch(function(e){ulmsg.textContent='error';});}
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
  winch.textContent='winch '+(d.wfix?'fix':'no fix')+' '+(d.wsats||0)+'sat'+(d.wacc!=null?'  '+d.wacc+'m':'')+(d.wconv?'  SURVEYED':'');
  warn.style.display=d.battlow?'block':'none';
  gpsdot.style.background=d.gps?'#1c1':'#c33';
  tdot.style.background=d.tsync?'#1c1':'#c33';
  clock.textContent=d.time+' UTC';
  if(d.upreq){ulmsg.textContent='uploading...';wasup=true;}
  else if(wasup){ulmsg.textContent='upload done';wasup=false;}
 }).catch(function(e){});
 if(Date.now()-last>3000){document.body.className='stale';
  gpsdot.style.background='#555';tdot.style.background='#555';}
}
setInterval(tick,500);tick();
</script></body></html>"""


async def handle(reader, writer):
    global _upload_request
    try:
        req = await reader.readline()
        while True:                       # drain request headers
            h = await reader.readline()
            if not h or h == b"\r\n":
                break
        path = req.split(b" ")[1] if b" " in req else b"/"
        if path.startswith(b"/data"):
            d = dict(latest)
            d["upreq"] = _upload_request
            writer.write(b"HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
                         b"Connection: close\r\n\r\n")
            writer.write(json.dumps(d).encode())
        elif path.startswith(b"/upload"):  # manual log upload (picked up by _serve)
            _upload_request = True
            writer.write(b"HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n"
                         b"Connection: close\r\n\r\nupload queued")
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


async def _internet_ok():
    # Lightweight reachability check for winchy-logs: a TCP connect (no TLS) to
    # GitHub's API host. True only if it actually connects, so a WiFi link with
    # no real internet (e.g. a dataless hotspot) does NOT light the OLED "I".
    try:
        r, w = await asyncio.wait_for(
            asyncio.open_connection("api.github.com", 443), 3)
        w.close()
        try:
            await w.wait_closed()
        except Exception:
            pass
        return True
    except Exception:
        return False


async def _serve():
    # Bring up the STA interface; the actual (re)join happens in the loop so a
    # dropped link recovers without a reboot. The HTTP server binds 0.0.0.0 and
    # is started once, on the first successful join (the binding survives later
    # rejoins / IP changes).
    wlan = None
    server_started = False
    announced_ip = None
    if WIFI_ENABLED and WIFI_NETWORKS:
        try:                              # MUST precede active(True) so mDNS
            network.hostname(WIFI_HOSTNAME)   # advertises <name>.local
        except Exception:
            pass
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
    else:
        print("WiFi: disabled or no secrets.py; dashboard off")
    global log_start, online, _upload_request
    n = 0
    while True:                           # keep-alive + WiFi upkeep + flush
        _flush_log()
        if clock_set and log_start is None:   # latch session start once synced
            log_start = _stamp()
        # (Re)join WiFi while it is down: at boot (n==0) and every WIFI_RETRY_S.
        if (wlan is not None and not wlan.isconnected()
                and n % WIFI_RETRY_S == 0):
            joined = await wifi.connect_any(wlan, WIFI_NETWORKS, 15)
            if joined:
                print("WiFi '%s' joined - http://%s/  or  http://%s.local/"
                      % (joined, wlan.ifconfig()[0], WIFI_HOSTNAME))
                if not server_started:    # bind once; survives later rejoins
                    await asyncio.start_server(handle, "0.0.0.0", 80)
                    server_started = True
                _begin_upload("time sync")
                _aid_fetch_time()         # one-shot UTC for GPS time aiding
                _end_upload()
            else:
                print("WiFi: no known network in range; will retry")
        # Hold off blocking GitHub round-trips while a launch is active, so the
        # live OLED/dashboard never freezes mid-launch (roadmap #12).
        busy = (time.ticks_diff(time.ticks_ms(), last_rx_ms) < 3000
                and latest.get("speed", 0) >= UPLOAD_PAUSE_SPEED_MS)
        # Announce our IP as a tappable dashboard link in the release body, so
        # the winch is findable on any subnet. On IP change only (no API spam).
        if (GITHUB_TOKEN and not busy
                and wlan is not None and wlan.isconnected()):
            cur = wlan.ifconfig()[0]
            if cur and cur != "0.0.0.0" and cur != announced_ip:
                try:
                    ssid = wlan.config("essid")
                except Exception:
                    ssid = "?"
                _begin_upload("IP announce")
                if _github_announce_ip(cur, ssid):
                    announced_ip = cur
                _end_upload()
        # winchy-logs reachability for the OLED "I": only when WiFi is up and a
        # token exists, probed against GitHub every WIFI_PROBE_S (so a hotspot
        # with no real internet shows no "I").
        if wlan is not None and wlan.isconnected() and GITHUB_TOKEN:
            if n % WIFI_PROBE_S == 0:
                online = await _internet_ok()
        else:
            online = False
        # GitHub offload: periodic auto + manual dashboard "Upload log" trigger.
        # Only while connected, with data, and not mid-launch (held off if busy;
        # a manual request stays pending until the launch ends).
        if (GITHUB_TOKEN and not busy and wlan is not None and wlan.isconnected()
                and (_upload_request or (n and n % UPLOAD_PERIOD_S == 0))):
            try:                          # only if there are rows to offload
                has_data = os.stat(LOG_PATH)[6] > len(LOG_HEADER)
            except OSError:
                has_data = False
            if has_data:
                _begin_upload("log upload")
                up = _github_upload()     # interim periodic; later: per launch
                if up:
                    _reset_log(up)        # reclaim flash after a good upload
                _end_upload()
            _upload_request = False       # clear the manual request once handled
        n += 1
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

import asyncio
if WIFI_ENABLED:
    import network
    import json


async def _main():
    # RX stays IRQ-driven. Run the GPS survey-in + WINCH_POS alongside the
    # WiFi dashboard and periodic flush.
    asyncio.create_task(_gps_task())
    await _serve()


# Crash guard (mirrors the rope's main.py): on an unexpected crash, log the
# traceback to crash.log and self-heal by resetting after an interruptible
# countdown, so a fielded winch can't be left dead - and so the traceback is
# captured for a post-mortem instead of scrolling off the serial. A deliberate
# Ctrl-C drops to the REPL.
import sys
import machine

CRASH_RESET_DELAY_S = 10

try:
    asyncio.run(_main())
except KeyboardInterrupt:
    print("Application interrupted, dropping to REPL")
except Exception as e:
    sys.print_exception(e)
    try:
        with open("crash.log", "w") as f:
            sys.print_exception(e, f)
    except OSError:
        pass
    try:
        print("Application crashed (traceback in crash.log). Resetting in "
              "%ds - Ctrl-C for REPL." % CRASH_RESET_DELAY_S)
        for _ in range(CRASH_RESET_DELAY_S):
            time.sleep(1)
        machine.reset()
    except KeyboardInterrupt:
        print("Auto-reset cancelled, dropping to REPL")
