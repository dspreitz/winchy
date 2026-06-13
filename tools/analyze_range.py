"""Summarise a range_log_*.txt from a walk test.

Correlates each LINK_REPORT (winch-measured downlink quality) with the rope's
TX power at that moment, so we can read path loss = tx_power - rssi, which is
the distance-dependent part independent of ADR. Also flags the link-down gaps
(no reports), which on this run are mostly the power-bank cutting out.
"""
import re
import sys

path = sys.argv[1]
SENS_SF7_BW500 = -111  # approx SX1262 sensitivity floor, dBm

# Sequence of (seconds, kind, payload). Track ADR power over time.
rep = re.compile(r"(\d\d):(\d\d):(\d\d)\s+Link report: rssi=(-?\d+) dBm "
                 r"snr=(-?\d+) dB loss=(\d+)%")
adr = re.compile(r"(\d\d):(\d\d):(\d\d)\s+ADR: tx power (-?\d+) -> (-?\d+)")


def secs(h, m, s):
    return int(h) * 3600 + int(m) * 60 + int(s)


reports, power_events = [], []
with open(path) as f:
    for line in f:
        m = rep.search(line)
        if m:
            h, mi, s, rssi, snr, loss = m.groups()
            reports.append((secs(h, mi, s), int(rssi), int(snr), int(loss)))
            continue
        m = adr.search(line)
        if m:
            h, mi, s, _old, new = m.groups()
            power_events.append((secs(h, mi, s), int(new)))


def power_at(t):
    """Rope TX power in effect at time t (last ADR change at or before t)."""
    p = 14  # boot default / config start
    for et, ep in power_events:
        if et <= t:
            p = ep
        else:
            break
    return p


t0 = reports[0][0] if reports else 0
print("LINK REPORTS (winch-measured downlink):")
print(" t+s  rssi  snr loss  txpwr  pathloss  margin(vs %ddBm)" % SENS_SF7_BW500)
prev = None
for t, rssi, snr, loss in reports:
    pwr = power_at(t)
    pathloss = pwr - rssi
    margin = rssi - SENS_SF7_BW500
    gap = "" if prev is None else "   <-- %ds gap (link down)" % (t - prev)
    print("%4d  %4d  %3d  %3d%%  %+4d   %4d dB    %3d dB%s"
          % (t - t0, rssi, snr, loss, pwr, pathloss, margin, gap))
    prev = t

if reports:
    rssis = [r[1] for r in reports]
    print("\nRSSI range: %d .. %d dBm   |   worst margin above floor: %d dB"
          % (min(rssis), max(rssis), min(rssis) - SENS_SF7_BW500))
    print("SNR at every live point: %d..%d dB   loss: %d..%d%%"
          % (min(r[2] for r in reports), max(r[2] for r in reports),
             min(r[3] for r in reports), max(r[3] for r in reports)))
    print("ADR power changes: %d (oscillation = reports coming and going)"
          % len(power_events))
