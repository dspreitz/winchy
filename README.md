# Winchy

Winchy is a glider winch rope force and advice system. It measures rope
tension and rope angle during a winch launch and gives the winch operator
real-time guidance on whether to increase or decrease throttle —
effectively retrofitting the sensing half of an automatic-tension-control
winch onto a manually driven one.

## System overview

Two ESP32-S3 units running MicroPython, linked by LoRa radio (amateur-band
868 MHz):

- **Rope unit** — a LilyGo T-Beam S3 Supreme attached to the rope near the
  glider, just before the weak link. An ADS1232 bridge ADC reads a
  strain-gauge-instrumented weak link for rope tension; a Kalman filter
  fuses accelerometer and gyroscope for the rope angle; the barometer
  (GPS-calibrated while idle) provides altitude and climb rate; the RTC is
  disciplined by GPS + 1PPS. During the initial acceleration the glider mass
  is estimated from force and acceleration (planned).
- **Winch unit** — currently a LilyGo T3S3 at the winch (moving to a second
  T-Beam S3 Supreme so both segments are identical hardware). An OLED and a
  WiFi dashboard show the operator glider speed, rope tension, rope angle,
  altitude and link quality. Its own GPS surveys-in the fixed winch position
  (sent to the rope as a WINCH_POS frame) and disciplines its RTC by 1PPS.
  Estimated glider mass and throttle advice over the tow phases are planned.

Wind is ignored for now; wind compensation is a future revision.
Measurement data is logged on the units (millisecond-stamped UTC) and
uploaded over WiFi to a GitHub release for offline analysis.

## Repository layout

```
firmware/
  shared/              # used by BOTH units: protocol (radio frames), nmea,
                       #   survey (GPS survey-in), wifi (multi-AP join)
  rope/                # rope unit (deploy this directory + shared/)
    boot.py, main.py   # minimal boot, crash-guarded entry (BOOT btn = safe mode)
    config.py          # full pin map + radio/ADC tunables
    winchy/            # application package
      app.py           #   asyncio runtime: one task per measurement rate
      state.py         #   shared latest-value state object
      board.py         #   T-Beam S3 Supreme wiring + AXP2101 power rails
      sensors/         #   ADS1232, QMI8658, BMP280, GPS/NMEA, QMC6310
      fusion/          #   rope angle, altitude, Kalman, glider speed, geometry
    lib/               # vendored third-party drivers (never edited)
  winch/main.py        # winch receiver + GPS survey-in + WiFi dashboard/upload
docs/
  rope_segment_architecture.md  # target design + migration plan
  winch_launch_physics.md       # phase-by-phase launch physics, with sources
tests/                 # desktop pytest suite (protocol, altitude, Kalman)
tools/                 # pull_device.py rescue, motion test capture/analysis
```

## Status

Done — migration steps 1–5 of the architecture plan, all verified on the
rope hardware:

- Crash-safe boot structure (unit always reachable over USB)
- Board/config extraction, sensor driver classes
- asyncio runtime: force sampling at ADC rate, IMU 50 Hz, baro 1 Hz,
  GPS event-driven, telemetry 2 Hz
- Versioned radio protocol with sequence numbers (engineering units;
  force still in raw counts until the gauge is calibrated)
- Per-segment GPS + 1PPS RTC discipline (no NTP, no time-over-radio)
- Barometric altitude self-calibrated against GPS while idle (±1 m on
  the bench); vertical Kalman for climb rate
- Gravity-vector Kalman (gyro + accelerometer, adaptive accel trust) for
  the rope angle: survives launch acceleration and rope spin — confirmed
  by simulation and a guided motion test on hardware
- Winch hardware-verified; its GPS surveys-in the winch position (WINCH_POS),
  and the rope folds winch-relative geometry (cable length, elevation) into
  the raw log
- Motion-gated raw logging on the rope; multi-AP WiFi with auto-rejoin;
  duty-cycled WiFi→GitHub upload of both logs, with millisecond UTC timestamps

Next — tow-phase state machine (IDLE → SLACK_OUT → GROUND_ROLL → ROTATION
→ CLIMB → TOP → RELEASED, plus LINK_BREAK), auto-tare in IDLE, force
calibration in newtons, mass estimation, and validating the glider-speed and
winch-relative geometry against a real launch before surfacing it live.

## Development

- Deploy: copy `firmware/rope/` contents plus the `firmware/shared/` modules
  to the device (`mpremote connect COMx resume fs cp ...` — keep `resume`,
  otherwise mpremote soft-resets into the running app).
- Tests: `python -m pytest tests` on the desktop — protocol round-trips,
  ISA altitude, and Kalman filter simulations run without hardware.
- Rescue: `tools/pull_device.py` recovers files from a wedged unit via
  USB replug (should no longer be needed with the current boot structure).
- The OLED is disabled (`config.DISPLAY_ENABLED`): the panel sits hidden
  behind the ADS1232 breakout PCB.

## License

Winchy is free software under the GNU General Public License v3.0 or later
(`GPL-3.0-or-later`) — see [LICENSE](LICENSE). First-party source files carry
an SPDX license header; the vendored drivers in `firmware/rope/lib/` keep
their own upstream licenses.
