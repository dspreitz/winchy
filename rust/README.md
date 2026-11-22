# Winchy — Rust firmware

Rewrite of the MicroPython firmware in Rust (branch `rust`). Two workspaces:

| Path | Toolchain | Contents |
|------|-----------|----------|
| `rust/` | stable | Pure `no_std` logic crates + host tests |
| `rust/firmware/` | `esp` (Xtensa) | ESP-IDF firmware binaries (`apps/*`) |

## Crates

| Crate | What it does | Depends on |
|-------|--------------|------------|
| `winchy-protocol` | Versioned LoRa frames (encode/decode, fixed-point) | — |
| `winchy-geo` | Winch↔rope geometry on the local tangent plane | — |
| `winchy-gnss` | UBX framing/NAV-PVT/CFG-VALSET, NMEA, survey-in, GNSS time trust, baud detect | geo |
| `winchy-fusion` | Gravity + vertical Kalman, rope angle, ISA altitude, glider speed, gyro bias | — |
| `winchy-link` | Adaptive TX power (ADR) | — |
| `winchy-axp2101` | AXP2101 PMU driver: rails, charger, gauge, IRQ mask | embedded-hal |
| `winchy-bmp280` | BMP280/BME280 barometer, bit-exact integer compensation | embedded-hal |
| `winchy-qmi8658` | QMI8658 IMU over SPI, burst read | embedded-hal |
| `winchy-ads1232` | ADS1232 force ADC, bit-banged | embedded-hal |
| `winchy-tbeam` | T-Beam S3 Supreme pin map + power profile (the only place with pin numbers) | axp2101 |
| `winchy-testkit` | Seeded PRNG for simulation tests (dev only) | — |

Firmware workspace: `winchy-board` turns the `winchy-tbeam` pin numbers into
ESP-IDF drivers; `apps/rope` is currently a sensor bring-up diagnostic.

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

Toolchain setup (once): `cargo install espup espflash ldproxy --locked && espup install --targets esp32s3`.
