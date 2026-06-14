# Rope Segment — Software Architecture

Architecture of the rope unit firmware (LilyGo T-Beam S3 Supreme,
MicroPython). The asyncio restructure described here is **implemented and
running on hardware** (it replaced the original single-file `boot.py`
monolith). The tow-phase state machine and the fusion/mass upgrades are still
open work — see the Status note at the end.

## Problems the restructure solved

The original snapshot in `rope_segment/` (commit `3a0159a`) was one 800-line
`boot.py` that ran everything at import time, plus a `main.py` that never
executed (it was actually winch-side receiver code for a different board).
Concrete issues the new structure solved:

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

- MicroPython on ESP32-S3 with 8 MB PSRAM, which is used to buffer the raw
  log in RAM during a launch (flash writes are deferred to idle). The AXP2101
  driver alone is 95 KB of source. Vendored drivers stay as-is; precompiling
  to `.mpy` is an option later.
- Single-core cooperative scheduling is sufficient: use `asyncio`
  (uasyncio). No threads.
- The device is sensor → radio, one direction, plus local logging. It must
  keep working (and logging) when the radio link or GPS is unavailable.
- Radio regulations: amateur-band LoRa, parameters must live in config.

## Source tree (as built)

```
firmware/
├── shared/                  # deployed to BOTH devices
│   ├── protocol.py          # versioned binary frames, encode/decode
│   ├── nmea.py              # GGA/RMC NMEA parsing (pure)
│   ├── survey.py            # GPS survey-in averager (pure)
│   └── wifi.py              # multi-AP join: scan + strongest-in-range
├── rope/
│   ├── boot.py              # minimal: safe-mode (BOOT button) check
│   ├── main.py              # crash-guarded entry; kbd_intr boot guard
│   ├── config.py            # pins, buses, radio params, rates (constants)
│   ├── winchy/
│   │   ├── board.py         # T-Beam S3 Supreme: buses, pins, AXP2101 rails
│   │   ├── state.py         # shared latest-value State object
│   │   ├── app.py           # ALL asyncio tasks + radio RX/TX inline
│   │   ├── sensors/
│   │   │   ├── ads1232.py   # force ADC driver
│   │   │   ├── qmi8658.py   # IMU driver (cold-start retry)
│   │   │   ├── magnetometer.py  # QMC6310 wrapper + calibration.cal
│   │   │   ├── barometer.py # BMP280 wrapper (async, non-blocking)
│   │   │   └── gps.py       # NMEA parse + 1PPS RTC discipline
│   │   └── fusion/          # pure, desktop-testable
│   │       ├── attitude.py  # rope angle from the gravity vector
│   │       ├── altitude.py  # ISA pressure -> altitude
│   │       ├── kalman.py    # gravity + vertical Kalman filters
│   │       ├── speed.py     # glider (CG-hook) speed estimate
│   │       └── geometry.py  # winch-relative geometry (cable len, elevation)
│   └── lib/                 # vendored third-party drivers, unmodified
└── winch/
    └── main.py              # winch receiver + GPS survey-in + WiFi dashboard
```

Note the radio link, flash logging and the display were **not** split into
`comm/`, `logger.py`, `display.py` packages as originally sketched — they live
as tasks inside `app.py` (a `mass.py` estimator is not written yet either).
The `rope_segment/` device snapshot has been removed from the tree; it remains
in git history as the reference.

## Layering rules

```
app.py ──► fusion, sensors, shared    (orchestrates; also holds the radio,
                                        logging & display tasks inline)
sensors/ ──► board.py, lib/            (hardware access)
fusion/, shared/ ──► nothing hardware  (pure Python, testable on desktop
                                        CPython)
```

- Only `board.py` knows pin numbers and PMU rail setup; it owns the I2C/SPI
  bus singletons and hands them to drivers.
- `fusion/` and `shared/` import neither `machine` nor drivers, so the angle
  math, the Kalman filters, the glider-speed and winch-relative geometry, the
  GPS survey-in, the NMEA parsing and the packet round-trip all get plain
  pytest tests on the desktop.
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

`main.py` builds and runs the application behind a crash guard: an unexpected
exception is logged to `crash.log`, then the unit resets after an
interruptible 10 s countdown. During the boot window `micropython.kbd_intr(-1)`
blocks a stray host Ctrl-C from aborting startup.

1. `board.init_power()` — AXP2101 rails.
2. Construct drivers; tare the force ADC; load the magnetometer
   `calibration.cal`; seed the gyro bias from 50 at-rest samples.
3. Init the SX1262 (with the RX IRQ callback) and the display.
4. `asyncio.run(...)`. The GPS task sets the RTC from NMEA, then disciplines
   it on every 1PPS rising edge once locked.

## Runtime model: asyncio tasks

| Task         | Rate            | What it does |
|--------------|-----------------|--------------|
| `force`      | ADC rate (~10 Hz) | ADS1232 conversions; `aread_raw` awaits the DRDY transition — no busy-wait |
| `imu`        | 50 Hz (~21 Hz sustained) | accel/gyro; gravity Kalman → rope angle; glider-speed; queues raw-log rows |
| `mag`        | 20 Hz           | QMC6310 read (slow I2C, kept off the IMU loop) |
| `baro`       | 1 Hz            | non-blocking forced measurement; vertical Kalman climb rate; GPS-cal QNH while idle |
| `gps`        | as data arrives | `StreamReader` on UART; position, RMC ground speed, 1PPS RTC; winch-relative geometry |
| `telemetry`  | 2 Hz            | latest state → `protocol.encode_telemetry` → radio TX; ADR TX-power control |
| `raw_writer` | continuous      | drains the raw-log row queue to `raw.csv` (writes deferred during a motion episode) |
| `supervisor` | 0.2 Hz          | PMU/battery, low-battery warning, session-start latch |
| `display`    | 2 Hz            | SH1106 status page (disabled by default — panel sits behind the ADS1232 PCB) |
| `wifi`       | every 10 min    | duty-cycled (idle + battery OK): multi-AP join → GitHub upload of `raw.csv` |

Radio **RX** is not a task: the SX1262 DIO1 IRQ (`on_radio`) decodes
`LINK_REPORT` (downlink quality → ADR) and `WINCH_POS` (winch position) inline.

Tasks communicate through one shared `State` object (latest values +
timestamps), not by calling each other. The telemetry task samples it; the
raw writer drains the row queue. Backpressure rule: each cycle sends only the
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

| Frame         | Dir        | Contents |
|---------------|------------|----------|
| `TELEMETRY`   | rope→winch | seq, phase, force, rope angle (0.5°), altitude (AMSL), battery (V + %), flags, glider speed |
| `LINK_REPORT` | winch→rope | RSSI / SNR / loss of the downlink — feeds the rope's ADR |
| `WINCH_POS`   | winch→rope | surveyed winch lat/lon/alt + accuracy + fix/survey status |
| `MASS`        | rope→winch | estimated glider mass + confidence, once per tow (planned) |
| `SUMMARY`     | rope→winch | per-tow stats after release (planned) |

Frame type 2 (`TIME_SYNC`) is **retired** — each segment now keeps its own
GPS + 1PPS time, so time is never sent over the radio. Rules: explicit
versioning (currently **v6**; receivers ignore unknown version/type/length),
every frame carries a sequence number so the receiver can measure link
quality across frame types. Force is still raw ADC counts with
`FLAG_FORCE_UNCALIBRATED` set until the load cell is calibrated to newtons.

## Calibration and configuration

- `config.py`: frozen constants (pins via `board.py`, radio params, rates,
  state-machine thresholds). Code, reviewed in git.
- `calibration.cal` on the device: magnetometer offsets, loaded at boot. The
  force ADC is tared at boot (no stored scale yet — calibrating it to newtons
  is pending); a richer on-device calibration file is a later option.

## Logging

- The raw logger (`raw_writer` task in `app.py`) writes **CSV** rows to
  `raw.csv`: every filter input (raw accel/gyro/mag, pressure, force, GPS)
  plus on-device outputs (baro alt/climb, angle, glider speed, winch geometry)
  at the IMU cadence, for offline Kalman validation. Each row carries a
  monotonic `t_ms` and an absolute `utc_ms`.
- **Motion-gated**: a 5 s ring-buffer pre-roll captures the start of a launch;
  recording stops after 5 s at rest. Writes are deferred (buffered in PSRAM)
  during an episode so sampling never stalls on flash, then drained while idle.
  `RAW_LOG_MAX_BYTES` caps the file.
- WiFi upload is implemented (the `wifi` task): while idle it joins a known
  network and pushes `raw.csv` to the `winchy-logs` GitHub release as a dated
  asset, deleting the local file after a good upload. The winch uploads its RX
  log the same way.

## Testing and tooling

- `fusion/`, `shared/protocol.py`, state-machine transitions: pure-Python
  unit tests run by pytest on the desktop (no MicroPython needed).
- Deploy with `mpremote fs cp` of `firmware/shared/` + `firmware/rope/` to
  the device; `tools/pull_device.py` remains for rescue.
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

**Status (2026-06):** steps 1–5 are complete and hardware-verified. Step 6's
logging is done (motion-gated CSV + WiFi→GitHub upload) and the GPS+1PPS clock
landed, but the **tow-phase state machine** (still hardcoded `IDLE`) and force
calibration to newtons remain — the state machine is the current keystone.
Step 7: the gravity and vertical Kalman filters and the glider-speed / winch
geometry are in; mass estimation is not. Added since the original plan: winch
GPS survey-in (`WINCH_POS`) + rope-side geometry, multi-AP WiFi with auto-
rejoin, and removal of `TIME_SYNC` in favour of per-segment GPS+PPS time.
