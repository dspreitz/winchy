# Winchy — Rust firmware

Rewrite of the MicroPython firmware in Rust (branch `rust`). Two workspaces:

| Path | Toolchain | Contents |
|------|-----------|----------|
| `rust/` | stable | Pure `no_std` logic crates + host tests |
| `rust/firmware/` | `esp` (Xtensa) | ESP-IDF firmware binaries (`apps/*`) |

## Crates

| Crate | What it does | Depends on |
|-------|--------------|------------|
| `winchy-protocol` | Versioned LoRa frames (encode/decode, fixed-point); `v6` = transitional codec for the MicroPython winch | — |
| `winchy-geo` | Winch↔rope geometry on the local tangent plane | — |
| `winchy-gnss` | UBX framing/NAV-PVT/CFG-VALSET, NMEA, survey-in, GNSS time trust, PPS edge labelling, baud discovery state machine | geo |
| `winchy-fusion` | Gravity + vertical Kalman, rope angle, ISA altitude, glider speed, gyro bias | — |
| `winchy-link` | Adaptive TX power (ADR) | — |
| `winchy-axp2101` | AXP2101 PMU driver: rails, charger, gauge, IRQ mask | embedded-hal |
| `winchy-bmp280` | BMP280/BME280 barometer, bit-exact integer compensation | embedded-hal |
| `winchy-qmi8658` | QMI8658 IMU over SPI, burst read | embedded-hal |
| `winchy-ads1232` | ADS1232 force ADC, bit-banged | embedded-hal |
| `winchy-sx126x` | SX1262 LoRa radio: init/calibration, TX, continuous RX, IRQ, packet status | embedded-hal |
| `winchy-tbeam` | T-Beam S3 Supreme pin map + power profile (the only place with pin numbers) | axp2101 |
| `winchy-rope-logic` | Rope app decisions: shared state, telemetry, battery warning, QNH, ADR feedback, radio cycle, PPS clock maths | protocol, link, geo, gnss, fusion |
| `winchy-provision` | Settings/secret key registry + USB console commands (secrets never echoed) | — |
| `winchy-testkit` | Seeded PRNG for simulation tests (dev only) | — |

Firmware workspace: `winchy-settings` stores provisioned values in NVS and
runs the USB console; `winchy-board` turns the `winchy-tbeam` pin numbers into
ESP-IDF drivers; `winchy-net-esp` does WiFi/HTTPS/SNTP; `winchy-storage` keeps
the AssistNow blob on a raw partition.

`apps/rope` runs one task per sensor rate, all writing one shared
`RopeState` (decisions live in `winchy-rope-logic`):

| Task | Rate | Prio | Does |
|------|------|------|------|
| `imu` | 50 Hz | 10 | gyro bias, gravity Kalman, rope angle + rate, glider speed |
| `radio` | 2 Hz | 9 | ADR, telemetry, listens for link reports / winch position |
| `force` | 10 SPS | 8 | offset calibration + tare, periodic recalibration while idle |
| `gnss` | 10 Hz | 7 | baud discovery, NAV-PVT, GNSS time, 1PPS clock discipline, winch geometry |
| `baro` | 1 Hz | 5 | QNH (seeded from stored field elevation, then GNSS), vertical Kalman |
| `power` | 0.2 Hz | 4 | battery/charging, low-battery warning, status log line |
| `net` | 1/min | 3 | WiFi join/roam/zombie check, SNTP fallback, AssistNow refresh |

The main thread serves the USB console. Until the winch is ported, telemetry
goes out in the MicroPython v6 format (`RadioConfig::wire`); received frames
are accepted in both formats.

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
