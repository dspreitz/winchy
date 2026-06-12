# Rope Segment — Software Architecture

Target architecture for the rope unit firmware (LilyGo T-Beam S3 Supreme,
MicroPython). This replaces the current single-file `boot.py` monolith that
was pulled from the device.

## Problems with the current code

The snapshot in `rope_segment/` (commit `3a0159a`) is one 800-line `boot.py`
that runs everything at import time, plus a `main.py` that never executes
(it is actually winch-side receiver code for a different board). Concrete
issues the new structure must solve:

1. **Everything runs in `boot.py`.** MicroPython runs `boot.py` before
   `main.py` and before the USB console is fully serviced. A crash or hang
   here makes the device hard to reach (we needed a power cycle and a timed
   Ctrl-C to pull the code off). `boot.py` must become minimal.
2. **No interruptible idle points.** `read_adc()` busy-waits on the DRDY
   pin with a tight `while` loop; LoRa send blocks. The scheduler must wait
   on events/IRQs so the REPL stays reachable and other work can proceed.
3. **Sequencing by accident.** Sensor init, debug register dumps, dead
   `while False:` blocks and the application loop are interleaved. There is
   no place to add the tow-phase state machine, mass estimation or the
   Kalman filter without making it worse.
4. **No separation of measurement rates.** The loop reads ADC, then a
   ~200 ms forced BMP280 measurement, then 20 averaged IMU samples, then
   transmits — serially, at ~2 Hz overall. Rope force needs to be sampled
   much faster than pressure during a tow.
5. **Protocol is implicit.** The 7-byte packet format lives in one function
   here and is hand-decoded on the winch side. Both segments must share one
   protocol definition.
6. **Configuration and calibration are scattered** (pin numbers inline,
   gain constant inline, `calibration.cal` in a homegrown text format).

## Constraints

- MicroPython v1.23 on ESP32-S3 (8 MB flash, no PSRAM assumed). RAM is the
  scarce resource; the AXP2101 driver alone is 95 KB of source. Vendored
  drivers stay as-is; precompiling to `.mpy` is an option later.
- Single-core cooperative scheduling is sufficient: use `asyncio`
  (uasyncio). No threads.
- The device is sensor → radio, one direction, plus local logging. It must
  keep working (and logging) when the radio link or GPS is unavailable.
- Radio regulations: amateur-band LoRa, parameters must live in config.

## Target source tree

```
firmware/
├── shared/                  # deployed to BOTH devices
│   └── protocol.py          # packet formats, versioning, encode/decode
├── rope/
│   ├── boot.py              # minimal: nothing but safe-mode check
│   ├── main.py              # entry: build app from config, run scheduler
│   ├── config.py            # pins, buses, radio params, rates (constants)
│   ├── winchy/
│   │   ├── board.py         # T-Beam S3 Supreme: buses, pins, PMU rails
│   │   ├── sensors/
│   │   │   ├── ads1232.py   # force ADC driver (IRQ-driven DRDY)
│   │   │   ├── qmi8658.py   # IMU driver class (replaces inline pokes)
│   │   │   ├── magnetometer.py  # QMC6310 wrapper + calibration
│   │   │   ├── barometer.py # BMP280 wrapper (async, non-blocking)
│   │   │   └── gps.py       # NMEA parse, fix status, RTC time sync
│   │   ├── fusion/
│   │   │   ├── attitude.py  # rope angle from accel (today's math)
│   │   │   ├── kalman.py    # sensor fusion (future, README goal)
│   │   │   └── mass.py      # glider mass estimate from F and a (future)
│   │   ├── comm/
│   │   │   └── radio.py     # LoRa link: TX queue, retries, stats
│   │   ├── logger.py        # measurement + debug logging to flash
│   │   ├── display.py       # SH1106 status pages
│   │   └── app.py           # tow state machine + asyncio tasks
│   └── lib/                 # vendored third-party drivers, unmodified
│       ├── AXP2101.py  sx1262.py  sx126x.py  _sx126x.py
│       ├── bmp280.py   sh1106.py  PiicoDev_QMC6310.py  PiicoDev_Unified.py
│       └── I2CInterface.py
└── winch/                   # ground segment (separate effort; the current
    └── ...                  #   rope_segment/main.py draft moves here)
```

`rope_segment/` (the device snapshot) stays in git history as the
reference; it is deleted from the tree once `firmware/rope/` reaches
feature parity.

## Layering rules

```
app.py ──► fusion, comm, logger, display, sensors   (orchestrates)
sensors/, comm/, display ──► board.py, lib/          (hardware access)
fusion/, shared/protocol.py ──► nothing hardware     (pure Python, testable
                                                      on desktop CPython)
```

- Only `board.py` knows pin numbers and PMU rail setup; it owns the I2C/SPI
  bus singletons and hands them to drivers. (Today three different I2C
  configurations of the same bus are created in different places.)
- `fusion/` and `shared/protocol.py` import neither `machine` nor drivers,
  so the angle math, Kalman filter, mass estimator and packet round-trip
  get plain pytest tests on the desktop.
- `lib/` is vendored upstream code: never edited, only replaced.

## Boot and startup sequence

`boot.py` does almost nothing:

```python
# boot.py — keep tiny; runs before USB console is reliable
import machine, os
# Safe mode: hold BOOT button (IO0) at reset → skip application
if machine.Pin(0, machine.Pin.IN, machine.Pin.PULL_UP).value() == 0:
    raise SystemExit
```

`main.py` builds and runs the application, with a top-level guard so a
crash drops to the REPL with a logged traceback instead of a wedged unit:

1. `board.init_power()` — AXP2101 rails (the current 250-line rail setup
   collapses to a table of `(rail, mV, on/off)` tuples).
2. Construct drivers, load calibration (`calibration.json`).
3. Sync time: wait up to N s for GPS fix → set RTC; else continue without.
4. `asyncio.run(app.main())`.

## Runtime model: asyncio tasks

| Task        | Rate          | What it does |
|-------------|---------------|--------------|
| `force`     | 10–80 Hz      | ADS1232 samples; DRDY pin IRQ sets an `asyncio.ThreadSafeFlag` — no busy-wait |
| `imu`       | 50 Hz         | accel read, feeds attitude/Kalman |
| `baro`      | 1 Hz          | non-blocking forced measurement (kick, return, collect when ready) |
| `gps`       | as data arrives | `asyncio.StreamReader` on UART; updates position/time |
| `telemetry` | 4 Hz          | latest fused state → `protocol.encode()` → radio TX queue |
| `logger`    | continuous    | ring-buffers samples, flushes to flash files |
| `display`   | 2 Hz          | status page (force, angle, link, GPS, battery) |
| `supervisor`| 1 Hz          | PMU IRQs, battery, link health, watchdog feed |

Tasks communicate through one shared `State` object (latest values +
timestamps), not by calling each other. The telemetry task samples it; the
logger drains queues. Backpressure rule: radio TX queue keeps only the
newest telemetry frame — stale force data is worthless to the operator.

## Tow state machine (in `app.py`)

The README requires phase-dependent behavior (mass estimate during initial
acceleration, throttle advice changing over the tow). The rope segment owns
phase detection; the winch segment only renders advice. Phases follow the
real launch profile (see `winch_launch_physics.md` for the physics and the
detection signals per phase):

```
IDLE ─tension rises─► SLACK_OUT ─accel spike─► GROUND_ROLL ─angle rises─►
ROTATION ─angle > ~35°─► CLIMB ─angle > ~60°, alt rate decays─► TOP
TOP ─tension ≈ 0─► RELEASED ─► IDLE          any phase ─tension step to 0─►
                                              LINK_BREAK ─► IDLE
```

- `IDLE`: slow sampling, slow telemetry; auto-tare of the force ADC.
- `SLACK_OUT`: small steady tension; timestamp launch, reset per-tow log.
- `GROUND_ROLL`: max-rate force+IMU sampling; mass estimator fits T = m·a
  while drag is negligible.
- `ROTATION`: tension-smoothness monitoring (oscillation/overshoot warnings
  matter most here — minimum stall margin).
- `CLIMB`: constant-tension advice (k·W target, anticipating power taper as
  rope speed falls).
- `TOP`: "reduce/cut power" advice ahead of back-release.
- `RELEASED / LINK_BREAK`: flush logs, transmit tow summary; a tension step
  to zero before `TOP` is signalled to the winch display immediately.

Phase transitions are part of the telemetry packet so the winch display
can switch modes without re-deriving them.

## Radio protocol (`shared/protocol.py`)

One module, imported by both segments, defines all frames. Binary structs,
each frame starting with `(version, type)`:

| Frame       | Contents |
|-------------|----------|
| `TELEMETRY` | seq, phase, force (N), rope angle (0.5°), pressure/altitude, battery, flags |
| `TIME_SYNC` | GPS epoch at startup (README requirement) |
| `MASS`      | estimated glider mass + confidence, sent once per tow |
| `SUMMARY`   | per-tow stats after release |

Rules: explicit versioning (receiver tolerates unknown frame types), engi-
neering units fixed in the protocol (force in N — not raw ADC counts like
today; the rope unit owns its calibration), every frame carries a sequence
number so the winch side can show link quality.

## Calibration and configuration

- `config.py`: frozen constants (pins via `board.py`, radio params, rates,
  state-machine thresholds). Code, reviewed in git.
- `calibration.json` on the device: force scale/offset, magnetometer
  offsets (replaces `calibration.cal`), written by on-device calibration
  routines, never by deploy.

## Logging

- `logger.py` writes binary measurement records (same structs as the radio
  protocol — one definition, two sinks) to per-tow files, plus a small
  rotating text log for debug/tracebacks.
- Flash wear: buffer in RAM, flush on phase change and every few seconds;
  cap total log size with oldest-first deletion.
- WiFi upload (README goal) is a later `comm/upload.py` that drains the
  same files; the on-disk format is designed for that from day one.

## Testing and tooling

- `fusion/`, `shared/protocol.py`, state-machine transitions: pure-Python
  unit tests run by pytest on the desktop (no MicroPython needed).
- `tools/deploy.ps1`: `mpremote fs cp` of `shared/` + `rope/` to the
  device; `tools/pull_device.py` remains for rescue.
- During development, `mpremote mount` lets the device run code straight
  from the workstation without flashing.

## Migration plan (each step leaves the device working)

1. **Scaffold** `firmware/` tree; move vendored drivers to `lib/`; minimal
   `boot.py` + `main.py` that reproduce today's behavior by importing one
   `legacy.py` (the old boot.py body). Flash, verify, commit.
2. **Extract `board.py`** (PMU table, bus singletons, pins) and
   `config.py`; delete the duplicated I2C setups. Verify on hardware.
3. **Driver classes**: `ads1232.py` (IRQ-driven), `qmi8658.py`,
   `barometer.py`, `gps.py` — each replacing its block in `legacy.py`.
4. **Asyncio loop + State object** replacing the final `while True`;
   `legacy.py` disappears. From here the REPL stays reachable.
5. **`shared/protocol.py`** with versioned frames; winch receiver draft
   moves to `firmware/winch/` and uses it.
6. **State machine, logger, calibration JSON** — new functionality.
7. **Fusion upgrades**: Kalman filter, mass estimator, with desktop tests.

Steps 1–4 are restructuring with unchanged behavior; 5+ change the system.
