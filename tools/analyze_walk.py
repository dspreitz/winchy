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

"""Join the rope GPS log and the winch RSSI log on telemetry seq, and report
link quality vs. actual distance from base.

  python tools/analyze_walk.py rope_gpslog_walk.csv winch_rxlog_walk.csv
"""
import math
import sys

rope_path, winch_path = sys.argv[1], sys.argv[2]


def haversine_m(a, b):
    R = 6371000.0
    lat1, lon1, lat2, lon2 = map(math.radians, (a[0], a[1], b[0], b[1]))
    dlat, dlon = lat2 - lat1, lon2 - lon1
    h = (math.sin(dlat / 2) ** 2
         + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2)
    return 2 * R * math.asin(math.sqrt(h))


# rope: seq -> (lat, lon, alt, tx_dbm) for rows with a real fix
gps = {}
for line in open(rope_path):
    if line[0] in "#s":   # comment or header
        continue
    f = line.strip().split(",")
    if len(f) < 7 or f[6] != "1":   # fix != 1
        continue
    lat, lon = float(f[2]), float(f[3])
    if lat == 0.0 and lon == 0.0:
        continue
    tx = int(f[7]) if len(f) > 7 else None   # tx_dbm (added later)
    gps[int(f[0])] = (lat, lon, float(f[4]), tx)

# winch: seq -> (rssi, snr). Columns:
# utc,seq,phase,force,angle_deg,alt_m,batt_v,batt_pct,flags,rssi,snr
rssi = {}
for line in open(winch_path):
    if line[0] == "#":
        continue
    f = line.strip().split(",")
    if len(f) < 11:
        continue
    rssi[int(f[1])] = (int(f[9]), int(f[10]))

joined = sorted(set(gps) & set(rssi))
if not joined:
    print("no overlapping seq with fix"); sys.exit()

base = gps[joined[0]]
print("base (first joined fix): seq %d  %.6f,%.6f" % (joined[0], base[0], base[1]))
print("rope fix acquired at seq %d; winch RX seq %d..%d"
      % (min(gps), min(rssi), max(rssi)))
print()
print(" seq   dist_m   rssi  snr   tx  pathloss   margin(vs -111)")
maxd = (0, None)
for s in joined:
    d = haversine_m(base, gps[s])
    r, n = rssi[s]
    tx = gps[s][3]
    pl = "%d" % (tx - r) if tx is not None else "  -"
    if d > maxd[0]:
        maxd = (d, s, r, n)
    # print every 10th row + the last few, to keep it readable
    if s % 10 == 0 or s >= joined[-1] - 4:
        print("%4d   %6.0f   %4d  %3d  %3s   %5s     %3d"
              % (s, d, r, n, "%d" % tx if tx is not None else "-", pl,
                 r - (-111)))

print()
last = joined[-1]
print("max distance with link: %.0f m at seq %d (rssi %d dBm, snr %d dB)"
      % (maxd[0], maxd[1], maxd[2], maxd[3]))
print("last frame winch received: seq %d at %.0f m (rssi %d, snr %d)"
      % (last, haversine_m(base, gps[last]), rssi[last][0], rssi[last][1]))
# how much further did the rope go (GPS only, link already dead)?
beyond = [haversine_m(base, gps[s]) for s in gps if s > max(rssi)]
if beyond:
    print("rope walked to %.0f m beyond the link (GPS continued past seq %d)"
          % (max(beyond), max(rssi)))
