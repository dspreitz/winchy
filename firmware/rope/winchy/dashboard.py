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

import json

state = None   # the shared State object, set by app.py's dashboard task


def _data(s):
    return {
        "fix": s.gps_fix, "sats": s.gps_sats, "lat": s.lat, "lon": s.lon,
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
<table>
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
<script>
var last=Date.now();
function f(x,n){return (x==null)?'--':x.toFixed(n);}
function tick(){
 fetch('/data').then(function(r){return r.json()}).then(function(d){
  last=Date.now();document.body.className='';
  gps.innerHTML=(d.fix?'<span class=g>FIX</span>':'<span class=r>no fix</span>')+' '+d.sats+' sat';
  ll.textContent=f(d.lat,6)+', '+f(d.lon,6);
  alt.textContent=f(d.baro_alt,1)+' m   '+f(d.climb,2)+' m/s';
  gs.textContent=f(d.gspeed,2)+' m/s';
  spd.textContent=f(d.speed*3.6,1)+' km/h';
  force.textContent=d.force+' cnt';
  ang.textContent=f(d.angle,1)+' deg';
  wd.textContent=f(d.wdist,1)+' m';
  cl.textContent=f(d.cable,1)+' m';
  el.textContent=f(d.elev,1)+' deg';
  wpos.textContent=(d.wfix?'fix':'no fix')+(d.wsurv?' SURVEYED':'');
  batt.textContent=(d.battpct>100?'--':d.battpct+'%')+'  '+f(d.battmv/1000,2)+'V'+(d.charging?' CHG':'')+(d.battlow?' LOW':'');
  tx.textContent=d.txdbm+' dBm';
  link.textContent=d.rssi+' dBm  snr '+d.snr+'  loss '+d.loss+'%';
  rec.textContent=(d.rec?'recording':'idle')+(d.tsync?'':'  (no time)');
 }).catch(function(e){});
 if(Date.now()-last>3000) document.body.className='stale';
}
setInterval(tick,1000);tick();
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
            writer.write(json.dumps(_data(state)).encode())
        elif path.startswith(b"/raw"):    # download the raw log
            try:
                body = open("raw.csv", "rb").read()
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
        print("rope http err:", e)
    try:
        writer.close()
        await writer.wait_closed()
    except Exception:
        pass
