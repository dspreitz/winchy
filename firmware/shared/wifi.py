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

# Multi-AP WiFi join, shared by both segments. Given a list of known networks
# it scans, then connects to the strongest one that is actually in range -
# falling back through the rest - so a device roams between sites (home bench,
# field router, phone hotspot) without reconfiguration. Async so it never
# blocks the asyncio loop. The caller owns the WLAN (active(True) first) and
# the power policy (the rope duty-cycles WiFi off between attempts).

import asyncio


async def connect_any(wlan, networks, timeout_s=15):
    """Connect to the best in-range network from `networks` (list of
    (ssid, password)). Returns the joined SSID, or None if none joined."""
    if wlan.isconnected():
        try:
            return wlan.config("essid")
        except Exception:
            return "?"
    if not networks:
        return None

    # A dropped AP can leave the STA mid-(auto)reconnect, which makes scan()
    # come back empty and connect() raise "Wifi Internal Error". Force a clean
    # disconnect and let the driver settle before scanning/connecting.
    try:
        wlan.disconnect()
    except Exception:
        pass
    await asyncio.sleep_ms(700)

    # Scan once; map ssid -> best RSSI seen.
    seen = {}
    try:
        for ap in wlan.scan():
            ssid = ap[0].decode() if isinstance(ap[0], bytes) else ap[0]
            rssi = ap[3]
            if ssid not in seen or rssi > seen[ssid]:
                seen[ssid] = rssi
    except Exception as e:
        print("WiFi scan failed:", e)

    # Configured networks that are visible, strongest first; then any unseen
    # (in case the scan missed a hidden/edge AP) as a last-ditch try.
    visible = sorted(((seen[s], s, p) for (s, p) in networks if s in seen),
                     reverse=True)
    ordered = [(s, p) for (_, s, p) in visible]
    ordered += [(s, p) for (s, p) in networks if s not in seen]

    for ssid, pw in ordered:
        print("WiFi: trying '%s'%s" % (
            ssid, "" if ssid in seen else " (not in scan)"))
        try:
            wlan.connect(ssid, pw)
        except Exception as e:
            print("WiFi connect error:", e)
            continue
        for _ in range(timeout_s * 2):
            if wlan.isconnected():
                return ssid
            await asyncio.sleep_ms(500)
        try:
            wlan.disconnect()
        except Exception:
            pass
    return None
