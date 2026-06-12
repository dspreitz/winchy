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
  (GPS-calibrated while idle) provides altitude and climb rate; system
  time comes from GPS at startup and is announced over the radio link.
  During the initial acceleration the glider mass is estimated from force
  and acceleration (planned).
- **Winch unit** — a LilyGo T3S3 at the winch with a display for the
  operator: rope tension, estimated glider mass, throttle advice changing
  over the phases of the tow, and link quality.

Wind is ignored for now; wind compensation is a future revision.
Measurement data is logged on the units and (future) uploaded over WiFi
for analysis.

## Repository layout

```
firmware/
  shared/protocol.py   # versioned binary radio frames, used by BOTH units
  rope/                # rope unit (deploy this directory + shared/)
    boot.py, main.py   # minimal boot, crash-guarded entry (BOOT btn = safe mode)
    config.py          # full pin map + radio/ADC tunables
    winchy/            # application package
      app.py           #   asyncio runtime: one task per measurement rate
      state.py         #   shared latest-value state object
      board.py         #   T-Beam S3 Supreme wiring + AXP2101 power rails
      sensors/         #   ADS1232, QMI8658, BMP280, GPS/NMEA, QMC6310
      fusion/          #   rope angle, ISA altitude, Kalman filters
    lib/               # vendored third-party drivers (never edited)
  winch/main.py        # winch-side receiver (not yet hardware-verified)
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
- GPS→RTC time sync + TIME_SYNC broadcast
- Barometric altitude self-calibrated against GPS while idle (±1 m on
  the bench); vertical Kalman for climb rate
- Gravity-vector Kalman (gyro + accelerometer, adaptive accel trust) for
  the rope angle: survives launch acceleration and rope spin — confirmed
  by simulation and a guided motion test on hardware

Next — tow-phase state machine (IDLE → SLACK_OUT → GROUND_ROLL → ROTATION
→ CLIMB → TOP → RELEASED, plus LINK_BREAK), auto-tare in IDLE, per-tow
flash logging, force calibration in newtons, mass estimation. The winch
receiver needs its T3S3 board located for hardware verification.

## Development

- Deploy: copy `firmware/rope/` contents plus `firmware/shared/protocol.py`
  to the device (`mpremote connect COMx resume fs cp ...` — keep `resume`,
  otherwise mpremote soft-resets into the running app).
- Tests: `python -m pytest tests` on the desktop — protocol round-trips,
  ISA altitude, and Kalman filter simulations run without hardware.
- Rescue: `tools/pull_device.py` recovers files from a wedged unit via
  USB replug (should no longer be needed with the current boot structure).
- The OLED is disabled (`config.DISPLAY_ENABLED`): the panel sits hidden
  behind the ADS1232 breakout PCB.
