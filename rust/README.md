# Winchy — Rust firmware

Rewrite of the MicroPython firmware in Rust (branch `rust`). Two workspaces:

| Path | Toolchain | Contents |
|------|-----------|----------|
| `rust/` | stable | Pure `no_std` logic crates + host tests |
| `rust/firmware/` | `esp` (Xtensa) | ESP-IDF firmware binaries (`apps/*`) |

## Crates

| Crate | What it does | Depends on |
|-------|--------------|------------|
| `winchy-protocol` | Versioned LoRa frames (encode/decode, fixed-point), `decode_any`; `v6` = codec for MicroPython units (both directions) | — |
| `winchy-geo` | Winch↔rope geometry on the local tangent plane | — |
| `winchy-gnss` | UBX framing/NAV-PVT/CFG-VALSET, u-blox 7 CFG messages, NMEA, survey-in, GNSS time trust, PPS edge labelling + clock maths, baud discovery state machine | geo |
| `winchy-fusion` | Gravity + vertical Kalman, rope angle, ISA altitude, glider speed, gyro bias | — |
| `winchy-link` | Adaptive TX power (ADR) | — |
| `winchy-axp2101` | AXP2101 PMU driver: rails, charger, gauge, IRQ mask | embedded-hal |
| `winchy-bmp280` | BMP280/BME280 barometer, bit-exact integer compensation | embedded-hal |
| `winchy-qmi8658` | QMI8658 IMU over SPI, burst read (FIFO supported, lossy) | embedded-hal |
| `winchy-ads1232` | ADS1232 force ADC, bit-banged | embedded-hal |
| `winchy-sx126x` | SX1262 LoRa radio: init/calibration, TX, continuous RX, IRQ, packet status | embedded-hal |
| `winchy-tbeam` | T-Beam S3 Supreme (rope) pin map + power profile | axp2101 |
| `winchy-t3s3` | T3S3 V1.2 (winch) pin map | — |
| `winchy-rope-logic` | Rope app decisions: shared state, telemetry, battery warning, QNH, ADR feedback, radio cycle, erase gate | protocol, link, geo, gnss, fusion |
| `winchy-winch-logic` | Winch app decisions: link statistics/report, reply planning, winch position, OLED screen model, dashboard page + JSON | protocol, gnss, geo |
| `winchy-winch-ui` | Draws the winch screens on a 128x64 monochrome display | embedded-graphics |
| `winchy-datalog` | Flash data-log format: timestamped records, CRC'd 4 KiB blocks with UTC offset, ring resume | — |
| `winchy-provision` | Settings/secret key registry + USB console commands (secrets never echoed) | — |
| `winchy-testkit` | Seeded PRNG for simulation tests (dev only) | — |

Firmware workspace: `winchy-settings` stores provisioned values in NVS and
runs the USB console; `winchy-board` turns the board crates' pin numbers into
ESP-IDF drivers (`RopeBoard`, `WinchBoard`); `winchy-rt` holds clocks, task
spawning and the 1PPS edge capture; `winchy-net-esp` does WiFi/HTTPS/SNTP;
`winchy-storage` keeps the AssistNow blob on a raw partition.

`apps/rope` runs one task per sensor rate, all writing one shared
`RopeState` (decisions live in `winchy-rope-logic`):

| Task | Rate | Prio | Does |
|------|------|------|------|
| `imu` | 50 Hz | 10 | gyro bias, gravity Kalman, rope angle + rate, glider speed |
| `radio` | 2 Hz | 9 | ADR, telemetry, listens for link reports / winch position |
| `force` | 10 SPS | 8 | offset calibration + tare, periodic recalibration while idle |
| `logger` | — | 6 | packs every measurement into the flash data log (`storage` partition) |
| `gnss` | 10 Hz | 7 | baud discovery, NAV-PVT, GNSS time, 1PPS clock discipline, winch geometry |
| `baro` | 1 Hz | 5 | QNH (seeded from stored field elevation, then GNSS), vertical Kalman |
| `power` | 0.2 Hz | 4 | battery/charging, low-battery warning, status log line |
| `net` | 1/min | 3 | WiFi join/roam/zombie check, SNTP fallback, AssistNow refresh |

### Data log

Every measurement (IMU, force, baro, GNSS, power, link report, events) is
stored with the monotonic time of the measurement (µs since boot); each 4 KiB
block header holds the UTC offset of the PPS-disciplined system clock, so every
sample has an exact UTC time. The 4.7 MB `storage` partition is a ring
(~1 h at ~1.3 kB/s); the oldest blocks are overwritten. Read it out with

```sh
espflash read-flash --port /dev/cu.usbmodemXXXX 0x350000 0x4B0000 dump.bin
cd rust && cargo run -p winchy-logdump -- dump.bin out/   # imu.csv, force.csv, gnss.csv, ...
```

`winchy-logdump` gives samples from before the clock was set the UTC time of
their boot (preferring the offset after the 1PPS lock).

A flash sector erase stalls all tasks for up to ~80 ms (the IMU loses 2-3
samples; timestamps stay exact). The logger therefore keeps a reserve of 64
erased sectors (~3 min) and refills it only while the unit lies still and no
tow runs (`rope-logic::erase_gate`), so towing writes without erasing.
Rejected alternatives, measured on the rope: the QMI8658A FIFO drops ~0.2
samples per read, unreported (see `winchy-qmi8658` docs); ESP-IDF flash
auto-suspend made the stalls longer (erases up to ~1 s, 5x more IMU gaps).

The main thread serves the USB console. Telemetry goes out in the current
protocol (`RadioConfig::wire`; `Wire::LegacyV6` only for a winch reflashed to
MicroPython); received frames are accepted in both formats.

### Winch (`apps/winch`, LilyGo T3S3, 4 MB flash)

| Task | Rate | Prio | Does |
|------|------|------|------|
| `radio` | on RX | 9 | continuous receive; answers each telemetry frame (20 ms guard) with a link report when asked, else the winch position every 15 s; replies at 22 dBm |
| `gnss` | 1 Hz | 7 | u-blox 7 NMEA: Stationary + SBAS setup, survey-in from GGA, clock from RMC, 1PPS discipline |
| `display` | 2 Hz | 5 | SSD1306: telemetry screen while the rope talks, GPS/survey screen after 3 s silence |
| `ws-push` | 2 Hz | 4 | pushes the dashboard snapshot to every connected WebSocket client |
| `net` | 1/30 s | 3 | WiFi join/roam/zombie check as DHCP host `winchy`, starts the dashboard, SNTP until GNSS time, internet check (OLED "I") |

Dashboard: `http://<winch-ip>/` (or `http://winchy.fritz.box/` where the
router resolves DHCP names) — the MicroPython page, fed by a WebSocket at
`/ws`; `/data` returns the same JSON snapshot. ESP-IDF 5.5 does not call the
URI handler for the WebSocket handshake, so the page says "hello" on open and
the winch registers a client on its first frame.

The winch has its own partition table and flash size:

```sh
cd rust/firmware
cargo build --release -p winchy-winch
espflash flash --port /dev/cu.usbmodemXXXX --flash-size 4mb \
    --partition-table partitions-winch.csv target/xtensa-esp32s3-espidf/release/winchy-winch
```

Still on the MicroPython winch only (later stages): RX log + GitHub upload
(and the dashboard's upload button), IP announce, cross-upload, GPS aiding.

Layering: logic crates never touch hardware. Drivers only see
`embedded-hal` traits; only a board crate knows pins and PMU rails; the
firmware apps just wire things together.

## Workflow

Test first: every function gets its failing test before the implementation.

```sh
# host crates
cd rust
cargo test            # all host tests
cargo lint            # clippy, warnings are errors (same gate as CI)
cargo fmt

# firmware (once per shell: . ~/export-esp.sh)
cd rust/firmware
cargo build --release
cargo run --release   # flash + serial monitor via espflash
```

## Provisioning secrets (WiFi, AssistNow token, GitHub token)

Secrets are never compiled into the image. They are typed once into the USB
console and stored in NVS (namespace `winchy`); they survive reflashing the app.

Use the bundled console (it keeps DTR/RTS low - with DTR asserted the
native USB port stays silent, which is why `espflash monitor`'s interactive
mode shows nothing):

```sh
cd rust
cargo run -p winchy-console -- /dev/cu.usbmodemXXXX   # no port: lists Espressif ports
> set wifi1_ssid My Network
> set wifi1_pass                 # no value: asks with hidden input
wifi1_pass (hidden):
> set ztp_token                  # Thingstream AssistNow device-profile token
ztp_token (hidden):
> list                           # secrets show only their length
> exit
```

Keys: `ztp_token`, `ztp_url` (optional override), `github_token`,
`wifi1_ssid`/`wifi1_pass` ... `wifi4_*` (slot number = priority).

Toolchain setup (once): `cargo install espup espflash ldproxy --locked && espup install --targets esp32s3`.
