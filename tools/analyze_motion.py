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

"""Analyze a timestamped motion-test capture from the rope unit.

Buckets the telemetry lines of tools/motion_test.log into the guided test
phases and summarizes angle, acceleration, gyro, altitude and force per
phase.
"""

import re

LOG = r"C:\Users\dom14\vs_code\Winchy\tools\motion_test.log"

PHASES = [
    ("P0 boot + rest flat", 0, 35),
    ("P1 tilt 45 deg", 35, 60),
    ("P2 long axis vertical", 60, 85),
    ("P3 back flat", 85, 110),
    ("P4 on long edge", 110, 135),
    ("P5 flat spin (record)", 135, 160),
    ("P6 horizontal shake", 160, 185),
    ("P7 held high", 185, 210),
    ("P8 on floor", 210, 235),
    ("P9 back on table", 235, 300),
]

# Trim the first seconds of each phase: human reaction + transition motion.
SETTLE_S = 8

re_t = re.compile(r"^\[ *([\d.]+)\] ?(.*)")
re_angle = re.compile(r"Seilwinkel: ([-\d.]+)")
re_motion = re.compile(
    r"Motion: \|a\|=([-\d.]+) g a=\(([-\d.,]+)\) gyro=\(([-\d.,]+)\) dps")
re_alt = re.compile(r"Baro alt: ([-\d.]+) m \(GPS ([-\d.]+) m, ([-+\d.]+) m/s\)")
re_adc = re.compile(r"ADC value: ([-\d]+)")

samples = []  # (t, kind, values)
for line in open(LOG, encoding="utf-8", errors="replace"):
    m = re_t.match(line)
    if not m:
        continue
    t = float(m.group(1))
    rest = m.group(2)
    a = re_angle.search(rest)
    if a:
        samples.append((t, "angle", float(a.group(1))))
    mo = re_motion.search(rest)
    if mo:
        accel = [float(x) for x in mo.group(2).split(",")]
        gyro = [float(x) for x in mo.group(3).split(",")]
        samples.append((t, "motion", (float(mo.group(1)), accel, gyro)))
    al = re_alt.search(rest)
    if al:
        samples.append((t, "alt", (float(al.group(1)), float(al.group(2)),
                                   float(al.group(3)))))
    ad = re_adc.search(rest)
    if ad:
        samples.append((t, "adc", int(ad.group(1))))


def stats(vals):
    if not vals:
        return "no data"
    return "mean %7.2f  min %7.2f  max %7.2f  (n=%d)" % (
        sum(vals) / len(vals), min(vals), max(vals), len(vals))


for name, t0, t1 in PHASES:
    held0 = t0 + SETTLE_S
    angles = [v for t, k, v in samples if k == "angle" and held0 <= t < t1]
    angles_full = [v for t, k, v in samples if k == "angle" and t0 <= t < t1]
    motion = [v for t, k, v in samples if k == "motion" and t0 <= t < t1]
    alts = [v for t, k, v in samples if k == "alt" and held0 <= t < t1]
    adcs = [v for t, k, v in samples if k == "adc" and t0 <= t < t1]
    print("=" * 72)
    print(name)
    print("  angle (held)   :", stats(angles))
    if angles_full and angles:
        print("  angle (full)   : min %7.2f  max %7.2f" %
              (min(angles_full), max(angles_full)))
    if motion:
        norms = [m[0] for m in motion]
        gyro_peak = max(max(abs(g) for g in m[2]) for m in motion)
        ay = [m[1][1] for m in motion]
        print("  |a| g          :", stats(norms))
        print("  a_y g          :", stats(ay))
        print("  gyro peak dps  : %7.1f" % gyro_peak)
    if alts:
        print("  baro alt m     :", stats([a[0] for a in alts]))
        print("  gps alt m      :", stats([a[1] for a in alts]))
        print("  climb m/s      :", stats([a[2] for a in alts]))
    if adcs:
        print("  force counts   :", stats(adcs))
