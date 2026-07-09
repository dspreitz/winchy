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
# it scans, then connects to the highest-PRIORITY one that is actually in
# range - falling back through the rest - so a device roams between sites
# (home bench, field router, phone hotspot) without reconfiguration. Async so
# it never blocks the asyncio loop. The caller owns the WLAN (active(True)
# first) and the power policy (the rope duty-cycles WiFi off between attempts).
#
# PRIORITY IS THE LIST ORDER of `networks`, NOT signal strength. Field test
# 2026-07-07 (Airbus site): selection by RSSI made the rope join the site's
# strong corporate AP while the winch joined the operator's phone hotspot -
# different subnets, so the rope's dashboard was unreachable from the phone
# and the test had to be aborted. The operator expresses intent by ordering
# WIFI_NETWORKS; the strongest signal is NOT the most useful network.
# roam_to_preferred() completes the contract: once connected, periodically
# switch UP the priority list when a better network comes in range (a hotspot
# is often enabled only after the device has already latched onto a site AP).

import asyncio


def tune(wlan):
    """Apply the server-role WiFi tuning (call after every active(True)):
    - pm=PM_NONE: modem power-save OFF. The units SERVE HTTP/WebSocket;
      with power-save the modem sleeps between DTIM beacons and inbound
      packets stall or drop - the main cause of 'the dashboard is flaky'.
    - reconnects=5: the driver heals short drops itself in ~1-2 s instead
      of waiting for the application-level rejoin loop.
    Both settings are best-effort (ports/builds without them just skip)."""
    try:
        wlan.config(pm=wlan.PM_NONE)
    except Exception:
        pass
    try:
        wlan.config(reconnects=5)
    except Exception:
        pass


async def reset_iface(wlan):
    """Power-cycle the STA interface: clears a stuck supplicant ('Wifi
    Internal Error', endless empty scans). Re-applies tune()."""
    try:
        wlan.active(False)
    except Exception:
        pass
    await asyncio.sleep_ms(1000)
    wlan.active(True)
    tune(wlan)


async def scan_rssi(wlan):
    """Scan and return {ssid: best_rssi}. An EMPTY result is a known driver
    glitch right after a drop, so retry once after a short settle before
    concluding "nothing in range"."""
    seen = {}
    for attempt in range(2):
        try:
            for ap in wlan.scan():
                ssid = ap[0].decode() if isinstance(ap[0], bytes) else ap[0]
                rssi = ap[3]
                if ssid not in seen or rssi > seen[ssid]:
                    seen[ssid] = rssi
        except Exception as e:
            print("WiFi scan failed:", e)
        if seen:
            break
        await asyncio.sleep_ms(500)
    return seen


def order_candidates(networks, seen):
    """Join order for `networks` [(ssid, password), ...] given a scan result
    {ssid: rssi}: list order IS the priority - visible networks first (in
    list order, NOT by RSSI - see the module header for the field test this
    cost), then the unseen ones (scan can miss a hidden/edge AP) as a
    last-ditch try, also in list order."""
    visible = [(s, p) for (s, p) in networks if s in seen]
    return visible + [(s, p) for (s, p) in networks if s not in seen]


def preferred_ssid(networks, current, seen, min_rssi=-75):
    """The SSID the device SHOULD be on instead of `current`: the first
    (= highest-priority) configured network that is visible with a usable
    signal (>= min_rssi, so a barely-audible AP never steals a working
    link) and ranks ABOVE current in the list. None = stay put. An unknown
    `current` (not in the list) ranks below everything."""
    for ssid, _ in networks:
        if ssid == current:
            return None
        if ssid in seen and seen[ssid] >= min_rssi:
            return ssid
    return None


async def connect_any(wlan, networks, timeout_s=15):
    """Connect to the highest-priority in-range network from `networks`
    (list of (ssid, password), LIST ORDER = PRIORITY). Returns the joined
    SSID, or None if none joined."""
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

    seen = await scan_rssi(wlan)
    for ssid, pw in order_candidates(networks, seen):
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


def probe_gateway(wlan, timeout_ms=2000):
    """Cheap two-way-traffic proof: one DNS query to the gateway (phones and
    routers all run a DNS forwarder); ANY reply counts. Needed because
    wlan.isconnected() can LIE - the zombie state seen on the bench
    2026-07-07: ESSID/IP/RSSI all look healthy while no packet moves, and
    the rejoin loop never fires because it only checks isconnected().
    Blocking up to timeout_ms - callers gate it to idle moments."""
    import socket
    try:
        gw = wlan.ifconfig()[2]
        if not gw or gw == "0.0.0.0":
            return False
        # minimal A query for github.com (we only care that ANYTHING answers)
        q = (b"\x12\x34\x01\x00\x00\x01\x00\x00\x00\x00\x00\x00"
             b"\x06github\x03com\x00\x00\x01\x00\x01")
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(timeout_ms / 1000)
        try:
            s.sendto(q, (gw, 53))
            s.recvfrom(256)
            return True
        finally:
            s.close()
    except Exception:
        return False


async def roam_to_preferred(wlan, networks, min_rssi=-75):
    """While connected to a lower-priority network, switch to a
    higher-priority one that has come in range (field bug 2026-07-07: the
    rope latched onto the site AP and never reconsidered, so the phone
    hotspot - enabled after the join - was never picked up). Costs one scan
    (a ~2 s off-channel trip that stutters the served link), so the caller
    should gate the cadence (and skip while a launch/motion is active); on
    the top-priority network it returns immediately without scanning.
    Returns the newly joined SSID, or None (= still on the old network).
    If the switch attempt fails, connect_any falls back down the ordered
    list - the old network is still in it, so the device rejoins rather
    than being left offline."""
    if not networks or not wlan.isconnected():
        return None
    try:
        current = wlan.config("essid")
    except Exception:
        return None
    if current == networks[0][0]:
        return None                     # already best - no scan cost
    seen = await scan_rssi(wlan)
    target = preferred_ssid(networks, current, seen, min_rssi)
    if target is None:
        return None
    print("WiFi roam: '%s' (%d dBm) outranks '%s'"
          % (target, seen[target], current))
    try:
        wlan.disconnect()               # else connect_any early-returns
    except Exception:
        pass
    return await connect_any(wlan, networks)
