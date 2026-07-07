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

# Host tests for the multi-AP WiFi selection (firmware/shared/wifi.py).
# Regression anchor: field test 2026-07-07 (Airbus site) - selection by RSSI
# made the rope join the site's strong corporate AP while the operator's
# phone hotspot (the network the dashboard must be reachable on) was weaker;
# the winch happened to pick the hotspot, the segments landed on different
# subnets, and the field test had to be aborted. Contract under test:
#   * WIFI_NETWORKS list order IS the priority; RSSI never overrides it.
#   * Once connected, roam_to_preferred() switches UP the list when a
#     higher-priority network comes in range (a hotspot is often enabled
#     only after the device already latched onto a site AP) - and never
#     strands the device offline if the switch fails.

import asyncio
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "firmware",
                                "shared"))

# MicroPython asyncio shim: sleep_ms does not exist on CPython. Instant sleep
# keeps the join/poll loops fast (the tests exercise selection, not timing).
if not hasattr(asyncio, "sleep_ms"):
    asyncio.sleep_ms = lambda ms: asyncio.sleep(0)

import wifi

NETWORKS = [("Spreitz_intern", "pw-home"),
            ("SpreitzMobile", "pw-hotspot"),
            ("Airbus", "pw-site")]


class FakeWLAN:
    """Just enough of network.WLAN(STA_IF) for connect_any/roam_to_preferred:
    a canned scan result, the set of SSIDs a connect() succeeds on, and the
    current association. scan_allowed=False asserts the no-scan fast paths."""

    def __init__(self, scan=(), joinable=None, connected_to=None,
                 scan_allowed=True):
        # wlan.scan() tuple layout: (ssid, bssid, channel, RSSI, security, hidden)
        self._scan = [(ssid.encode(), b"\x00" * 6, 1, rssi, 3, False)
                      for ssid, rssi in scan]
        self._joinable = set(joinable if joinable is not None
                             else [ssid for ssid, _ in scan])
        self._essid = connected_to
        self._scan_allowed = scan_allowed
        self.join_attempts = []

    def isconnected(self):
        return self._essid is not None

    def config(self, key):
        assert key == "essid"
        return self._essid

    def disconnect(self):
        self._essid = None

    def scan(self):
        assert self._scan_allowed, "scan() called on a no-scan fast path"
        return list(self._scan)

    def connect(self, ssid, pw):
        self.join_attempts.append(ssid)
        if ssid in self._joinable:
            self._essid = ssid


def run(coro):
    return asyncio.run(coro)


# --- connect_any: priority beats RSSI ---------------------------------------

def test_field_regression_hotspot_beats_stronger_site_ap():
    # THE 2026-07-07 failure, replayed: at the field the site AP is much
    # stronger than the phone hotspot. RSSI-based selection joins 'Airbus'
    # (unreachable from the operator's phone); priority order must join the
    # hotspot. This test fails on the pre-fix strongest-first code.
    w = FakeWLAN(scan=[("Airbus", -48), ("SpreitzMobile", -77)])
    assert run(wifi.connect_any(w, NETWORKS, timeout_s=1)) == "SpreitzMobile"


def test_top_priority_wins_even_when_weaker():
    # Home bench with the phone hotspot accidentally on and much louder:
    # the home network still outranks it by list order.
    w = FakeWLAN(scan=[("SpreitzMobile", -40), ("Spreitz_intern", -68)])
    assert run(wifi.connect_any(w, NETWORKS, timeout_s=1)) == "Spreitz_intern"


def test_falls_back_down_the_list_when_preferred_join_fails():
    # The hotspot is visible but the join fails (wrong password, DHCP dead):
    # fall through to the next visible network instead of ending offline.
    w = FakeWLAN(scan=[("SpreitzMobile", -70), ("Airbus", -50)],
                 joinable=["Airbus"])
    assert run(wifi.connect_any(w, NETWORKS, timeout_s=1)) == "Airbus"
    assert w.join_attempts[0] == "SpreitzMobile"   # tried in priority order


def test_visible_networks_tried_before_unseen_ones():
    # Only the site AP shows up in the scan: join it first; the configured
    # but unseen networks are last-ditch attempts after it.
    w = FakeWLAN(scan=[("Airbus", -50)])
    assert run(wifi.connect_any(w, NETWORKS, timeout_s=1)) == "Airbus"
    assert w.join_attempts == ["Airbus"]


def test_already_connected_returns_current_without_scanning():
    w = FakeWLAN(connected_to="Airbus", scan_allowed=False)
    assert run(wifi.connect_any(w, NETWORKS, timeout_s=1)) == "Airbus"


# --- order_candidates / preferred_ssid (pure) --------------------------------

def test_order_candidates_is_list_order_not_rssi():
    seen = {"Airbus": -40, "SpreitzMobile": -80}
    assert [s for s, _ in wifi.order_candidates(NETWORKS, seen)] == \
        ["SpreitzMobile", "Airbus", "Spreitz_intern"]


def test_preferred_ssid_flags_the_hotspot_over_the_site_ap():
    seen = {"SpreitzMobile": -66, "Airbus": -50}
    assert wifi.preferred_ssid(NETWORKS, "Airbus", seen) == "SpreitzMobile"


def test_preferred_ssid_weak_ap_never_steals_a_working_link():
    # -85 dBm is below min_rssi: switching would trade a working link for
    # an edge-of-range one. Stay put.
    seen = {"SpreitzMobile": -85}
    assert wifi.preferred_ssid(NETWORKS, "Airbus", seen) is None


def test_preferred_ssid_none_on_top_priority():
    seen = {"SpreitzMobile": -40, "Airbus": -40}
    assert wifi.preferred_ssid(NETWORKS, "Spreitz_intern", seen) is None


def test_preferred_ssid_unknown_current_ranks_below_everything():
    seen = {"Airbus": -60}
    assert wifi.preferred_ssid(NETWORKS, "NeighborsAP", seen) == "Airbus"


# --- roam_to_preferred: re-evaluate while connected --------------------------

def test_roam_switches_to_hotspot_that_came_in_range():
    # Second half of the field bug: the rope joined 'Airbus' BEFORE the
    # hotspot was enabled and then never reconsidered. The roam check must
    # move it once the hotspot shows up in a scan.
    w = FakeWLAN(scan=[("SpreitzMobile", -66), ("Airbus", -50)],
                 connected_to="Airbus")
    assert run(wifi.roam_to_preferred(w, NETWORKS)) == "SpreitzMobile"
    assert w.config("essid") == "SpreitzMobile"


def test_roam_on_top_priority_returns_without_scanning():
    # The common case (home bench on the home AP) must cost nothing: no
    # scan, no link stutter.
    w = FakeWLAN(connected_to="Spreitz_intern", scan_allowed=False)
    assert run(wifi.roam_to_preferred(w, NETWORKS)) is None


def test_roam_stays_when_nothing_better_is_visible():
    w = FakeWLAN(scan=[("Airbus", -50)], connected_to="Airbus")
    assert run(wifi.roam_to_preferred(w, NETWORKS)) is None
    assert w.config("essid") == "Airbus"          # link untouched


def test_roam_failed_switch_falls_back_to_the_old_network():
    # The hotspot is visible but the join fails: the device must end up
    # back on the old network, not offline (connect_any keeps the old
    # network in its ordered candidate list).
    w = FakeWLAN(scan=[("SpreitzMobile", -60), ("Airbus", -50)],
                 joinable=["Airbus"], connected_to="Airbus")
    assert run(wifi.roam_to_preferred(w, NETWORKS)) == "Airbus"
    assert w.config("essid") == "Airbus"


def test_roam_disconnected_is_a_noop():
    w = FakeWLAN(scan=[("SpreitzMobile", -60)], connected_to=None,
                 scan_allowed=False)
    assert run(wifi.roam_to_preferred(w, NETWORKS)) is None
