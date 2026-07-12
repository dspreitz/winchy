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

# Rope-segment WiFi status dashboard for walk/ground tests. The rope's OLED is
# physically blocked by the ADS1232 PCB (config.DISPLAY_ENABLED=False), so this
# small web page is the only live view of the rope's status. All values come
# from the shared State; mirrors the winch dashboard in firmware/winch/main.py.
# app.py sets `state` before starting the HTTP server.

import asyncio
import binascii
import hashlib
import json
import time

state = None   # the shared State object, set by app.py's dashboard task


def _hms():
    t = time.localtime()            # RTC is UTC (GPS+PPS, or NTP fallback)
    return "%02d:%02d:%02d" % (t[3], t[4], t[5])


def _data(s):
    # Age of the last WINCH_POS in seconds (-1 = never received). Drives the
    # survey banner: the operator must SEE whether the winch has a converged
    # position before ride 1 (field test #2: ride flown 2 min after winch
    # boot -> survey not converged -> no distance the whole ride).
    wage = -1
    if s.winch_pos_ts:
        wage = time.ticks_diff(time.ticks_ms(), s.winch_pos_ts) // 1000
    return {
        "wacc": s.winch_acc_m, "wage": wage,
        "fix": s.gps_fix, "sats": s.gps_sats, "hacc": s.gps_hacc_m,
        "lat": s.lat, "lon": s.lon,
        "gspeed": s.ground_speed_ms, "baro_alt": s.baro_alt_m,
        "climb": s.climb_rate_ms, "force": s.force_raw - s.force_offset,
        "angle": s.angle_deg, "speed": s.glider_speed_ms,
        "cable": s.cable_length_m, "wdist": s.winch_dist_m,
        "elev": s.elevation_deg, "wfix": bool(s.winch_status & 0x01),
        "wsurv": bool(s.winch_status & 0x02), "battmv": s.batt_mv,
        "battpct": s.batt_pct, "charging": s.charging, "battlow": s.batt_low,
        "txdbm": s.tx_power_dbm, "rssi": s.link_rssi_dbm,
        "snr": s.link_snr_db, "loss": s.link_loss_pct,
        "tsync": s.time_synced, "rec": s.raw_recording,
        "upstatus": s.upload_status,
        "time": _hms() if s.time_synced else "--:--:--",
    }


PAGE = """<!DOCTYPE html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>Winchy rope</title><style>
body{font-family:sans-serif;background:#111;color:#eee;margin:0;padding:6px}
h1{font-size:5vw;margin:4px;color:#8ac}
table{width:100%;border-collapse:collapse;font-size:4.6vw}
td{padding:4px 6px;border-bottom:1px solid #333}
td.l{color:#8ac;width:46%}
.stale{opacity:.4}.g{color:#1c1}.r{color:#c33}
</style></head><body>
<h1>Winchy rope</h1>
<div id=wbanner style="font-size:4.8vw;font-weight:bold;text-align:center;padding:10px 6px;margin:2px 0 6px;border-radius:6px;background:#333">--</div>
<table>
<tr><td class=l>Time</td><td id=clock>--</td></tr>
<tr><td class=l>GPS</td><td id=gps>--</td></tr>
<tr><td class=l>Lat/Lon</td><td id=ll>--</td></tr>
<tr><td class=l>Baro alt / climb</td><td id=alt>--</td></tr>
<tr><td class=l>Ground speed</td><td id=gs>--</td></tr>
<tr><td class=l>Glider speed</td><td id=spd>--</td></tr>
<tr><td class=l>Force</td><td id=force>--</td></tr>
<tr><td class=l>Rope angle</td><td id=ang>--</td></tr>
<tr><td class=l>Winch dist</td><td id=wd>--</td></tr>
<tr><td class=l>Cable len</td><td id=cl>--</td></tr>
<tr><td class=l>Elevation</td><td id=el>--</td></tr>
<tr><td class=l>Winch pos</td><td id=wpos>--</td></tr>
<tr><td class=l>Battery</td><td id=batt>--</td></tr>
<tr><td class=l>Radio TX</td><td id=tx>--</td></tr>
<tr><td class=l>Link (winch)</td><td id=link>--</td></tr>
<tr><td class=l>Logging</td><td id=rec>--</td></tr>
</table>
<button onclick=ul(1) style="font-size:5vw;padding:12px;margin:8px 1px 2px;width:48%;background:#253;color:#eee;border:1px solid #8ca;border-radius:6px">Upload last ride</button>
<button onclick=ul(0) style="font-size:5vw;padding:12px;margin:8px 1px 2px;width:48%;background:#235;color:#eee;border:1px solid #8ac;border-radius:6px">Upload full log</button>
<div id=ulmsg style="font-size:4vw;color:#8ac;padding:2px 6px">&nbsp;</div>
<script>
var last=Date.now();var ws;
var UPLBL={uploading:'uploading…',ok:'✓ upload verified',unverified:'⚠ uploaded, NOT verified',fail:'✗ upload failed',nodata:'nothing new to upload'};
function ul(r){ulmsg.textContent='uploading…';
 fetch('/upload'+(r?'?mode=ride':''),{method:'POST'}).catch(function(e){ulmsg.textContent='✗ upload error';});}
function f(x,n){return (x==null)?'--':x.toFixed(n);}
function render(d){
  last=Date.now();document.body.className='';
  clock.textContent=d.time+' UTC';
  gps.innerHTML=(d.fix?'<span class=g>FIX</span>':'<span class=r>no fix</span>')+' '+d.sats+' sat'+(d.hacc<100?'  ±'+f(d.hacc,1)+'m':'');
  ll.textContent=f(d.lat,6)+', '+f(d.lon,6);
  alt.textContent=f(d.baro_alt,1)+' m   '+f(d.climb,2)+' m/s';
  gs.textContent=f(d.gspeed*3.6,1)+' km/h';
  spd.textContent=f(d.speed*3.6,1)+' km/h';
  force.textContent=d.force+' cnt';
  ang.textContent=f(d.angle,1)+' deg';
  var gstale=(!d.fix)||d.wage<0||d.wage>60;  /* geometry holds its LAST value
     (intended for ADR) - without a rope fix + fresh WINCH_POS it can be very
     wrong (field test #4: 26 km = home-to-field from the persisted position) */
  wd.textContent=f(d.wdist,1)+' m'+(gstale?' (stale)':'');
  cl.textContent=f(d.cable,1)+' m'+(gstale?' (stale)':'');
  el.textContent=f(d.elev,1)+' deg'+(gstale?' (stale)':'');
  wd.style.opacity=cl.style.opacity=el.style.opacity=gstale?0.4:1;
  wpos.textContent=(d.wfix?'fix':'no fix')+(d.wsurv?' SURVEYED':'')+(d.wage>=0?'  ('+d.wage+'s ago)':'');
  if(d.wage<0||d.wage>60){wbanner.textContent='WINCH: no data'+(d.wage>60?' ('+d.wage+'s)':'');wbanner.style.background='#622';}
  else if(!d.wsurv){wbanner.textContent='WINCH surveying… ±'+f(d.wacc,1)+' m — WAIT';wbanner.style.background='#653';}
  else{wbanner.textContent='WINCH position ready  ±'+f(d.wacc,1)+' m';wbanner.style.background='#252';}
  batt.textContent=(d.battpct>100?'--':d.battpct+'%')+'  '+f(d.battmv/1000,2)+'V'+(d.charging?' CHG':'')+(d.battlow?' LOW':'');
  tx.textContent=d.txdbm+' dBm';
  link.textContent=d.rssi+' dBm  snr '+d.snr+'  loss '+d.loss+'%';
  rec.textContent=(d.rec?'recording':'idle')+(d.tsync?'':'  (no time)');
  if(d.upstatus){ulmsg.textContent=UPLBL[d.upstatus]||d.upstatus;}
}
function connect(){
 ws=new WebSocket('ws://'+location.host+'/ws');
 ws.onmessage=function(ev){try{render(JSON.parse(ev.data));}catch(e){}};
 ws.onclose=function(){setTimeout(connect,1000);};   // auto-reconnect
 ws.onerror=function(){try{ws.close();}catch(e){}};
}
connect();
setInterval(function(){if(Date.now()-last>3000) document.body.className='stale';},1000);
</script></body></html>"""


# --- WebSocket push --------------------------------------------------------
# The dashboard opens a WebSocket to /ws and the rope PUSHES the latest status
# (JSON) every WS_PUSH_MS instead of the browser polling /data. SHA1 for the
# handshake is available because the build has SSL (HASHLIB_SHA1 = PY_SSL).
WS_PUSH_MS = 500
_WS_GUID = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11"   # RFC 6455 magic


def _ws_accept(key):
    return binascii.b2a_base64(hashlib.sha1(key + _WS_GUID).digest()).strip()


def _ws_text_frame(payload):
    # Server->client text frame: FIN + opcode 0x1, unmasked.
    n = len(payload)
    if n < 126:
        hdr = bytes((0x81, n))
    elif n < 65536:
        hdr = bytes((0x81, 126, n >> 8, n & 0xFF))
    else:
        hdr = bytes((0x81, 127)) + n.to_bytes(8, "big")
    return hdr + payload


async def _ws_push(writer, key):
    # Finish the handshake, then push status until the client leaves (a write to
    # a closed socket raises -> we stop quietly).
    writer.write(b"HTTP/1.1 101 Switching Protocols\r\n"
                 b"Upgrade: websocket\r\nConnection: Upgrade\r\n"
                 b"Sec-WebSocket-Accept: " + _ws_accept(key) + b"\r\n\r\n")
    await writer.drain()
    try:
        while True:
            writer.write(_ws_text_frame(json.dumps(_data(state)).encode()))
            await writer.drain()
            await asyncio.sleep_ms(WS_PUSH_MS)
    except Exception:
        pass                          # client disconnected


async def handle(reader, writer):
    try:
        req = await reader.readline()
        if state is not None:            # an inbound request = the link WORKS
            state.net_alive_ms = time.ticks_ms()
        headers = {}
        while True:                       # read request headers
            h = await reader.readline()
            if not h or h == b"\r\n":
                break
            i = h.find(b":")
            if i > 0:
                headers[h[:i].strip().lower()] = h[i + 1:].strip()
        path = req.split(b" ")[1] if b" " in req else b"/"
        if (path == b"/ws"
                and headers.get(b"upgrade", b"").lower() == b"websocket"
                and headers.get(b"sec-websocket-key")):
            await _ws_push(writer, headers[b"sec-websocket-key"])
        elif path.startswith(b"/data"):   # kept for curl/debug; the UI uses /ws
            writer.write(b"HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
                         b"Connection: close\r\n\r\n")
            writer.write(json.dumps(_data(state)).encode())
            await writer.drain()
        elif path.startswith(b"/upload"):  # manual log upload (picked up by app)
            if state is not None and not state.uploading:   # ignore while busy
                # ?mode=ride = last episode only (field-sized); default = full
                state.upload_request = "ride" if b"mode=ride" in path else True
                # Also ask the winch to upload, over the radio (cross-upload).
                # telemetry_task sends UPLOAD_CMD(nonce) up to 5x until ACKed.
                state.cross_nonce_ctr = (state.cross_nonce_ctr + 1) & 0xFF
                state.cross_cmd_nonce = state.cross_nonce_ctr
                state.cross_cmd_tries = 5
                state.cross_cmd_ts = 0          # send on the next telemetry cycle
            writer.write(b"HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n"
                         b"Connection: close\r\n\r\nupload queued")
            await writer.drain()
        elif path.startswith(b"/events"):  # persistent event log (small)
            writer.write(b"HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n"
                         b"Connection: close\r\n\r\n")
            await writer.drain()
            try:
                with open("events.log", "rb") as f:
                    while True:
                        chunk = f.read(1024)
                        if not chunk:
                            break
                        writer.write(chunk)
                        await writer.drain()
            except OSError:
                writer.write(b"(no events yet)")
                await writer.drain()
        elif path.startswith(b"/raw"):    # download the raw log
            # STREAM in chunks: raw.csv can be 4 MB, and reading it whole
            # allocated one huge buffer that OOMed the ESP32 (MemoryError).
            writer.write(b"HTTP/1.1 200 OK\r\nContent-Type: text/csv\r\n"
                         b"Connection: close\r\n\r\n")
            await writer.drain()
            try:
                with open("raw.csv", "rb") as f:
                    while True:
                        chunk = f.read(2048)
                        if not chunk:
                            break
                        writer.write(chunk)
                        await writer.drain()
            except OSError:
                pass
        else:
            writer.write(b"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n"
                         b"Connection: close\r\n\r\n")
            writer.write(PAGE.encode())
            await writer.drain()
    except Exception as e:
        print("rope http err:", e)
    try:
        writer.close()
        await writer.wait_closed()
    except Exception:
        pass
