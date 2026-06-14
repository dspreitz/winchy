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

# Bench probe: try each WIFI_NETWORKS entry from secrets.py individually and
# report whether it is visible, joins, and what IP it gets. Verifies every
# configured AP's credentials + reachability independent of the scan-strongest
# logic. Run with:  mpremote connect COMx run tools/wifi_probe.py
# (Interrupts the app; reset/power-cycle afterwards to resume.)

import network
import sys
import time

sys.modules.pop("secrets", None)   # read the freshly-deployed secrets.py
try:
    from secrets import WIFI_NETWORKS
except ImportError:
    try:
        from secrets import WIFI_SSID, WIFI_PASSWORD
        WIFI_NETWORKS = [(WIFI_SSID, WIFI_PASSWORD)]
    except ImportError:
        WIFI_NETWORKS = []

w = network.WLAN(network.STA_IF)
w.active(True)

print("scan:")
seen = {}
for ap in w.scan():
    s = ap[0].decode() if isinstance(ap[0], bytes) else ap[0]
    if s and (s not in seen or ap[3] > seen[s]):
        seen[s] = ap[3]
for s in sorted(seen, key=lambda k: -seen[k]):
    print("  %-26s %d dBm" % (s, seen[s]))

for ssid, pw in WIFI_NETWORKS:
    vis = ("%d dBm" % seen[ssid]) if ssid in seen else "NOT VISIBLE"
    print("try '%s' (%s)" % (ssid, vis))
    if w.isconnected():
        w.disconnect()
        time.sleep_ms(300)
    try:
        w.connect(ssid, pw)
    except Exception as e:
        print("  connect error:", e)
        continue
    ok = False
    for _ in range(30):            # up to ~15 s
        if w.isconnected():
            ok = True
            break
        time.sleep_ms(500)
    print("  JOINED ip=%s" % w.ifconfig()[0] if ok else "  FAILED (timeout)")
    w.disconnect()
    time.sleep_ms(300)

w.active(False)
print("done")
