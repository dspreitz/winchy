//! Winchy rope unit firmware. Currently a hardware bring-up diagnostic:
//! power profile, then one round of readings from every sensor.

mod assist;

use esp_idf_svc::hal::delay::{Ets, FreeRtos};
use esp_idf_svc::hal::peripherals::Peripherals;
use winchy_ads1232::{Ads1232, Gain};
use winchy_bmp280::Bmp280;
use winchy_board::RopeBoard;
use winchy_fusion::attitude::rope_angle_above_ground_deg;
use winchy_qmi8658::Qmi8658;
use winchy_sx126x::{ChipConfig, IrqFlags, LoraConfig, Sx126x};

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    log::info!("winchy-rope {} bring-up", env!("CARGO_PKG_VERSION"));

    let Some(p) = Peripherals::take().ok() else {
        log::error!("peripherals already taken");
        return;
    };
    let (Ok(sysloop), Ok(nvs_part)) = (
        esp_idf_svc::eventloop::EspSystemEventLoop::take(),
        esp_idf_svc::nvs::EspDefaultNvsPartition::take(),
    ) else {
        log::error!("event loop / NVS partition unavailable");
        return;
    };
    let mut store = match winchy_settings::NvsStore::open(nvs_part.clone()) {
        Ok(s) => s,
        Err(e) => {
            log::error!("NVS: {e:?}");
            return;
        }
    };
    let modem = p.modem;
    report_settings(&mut store);
    let mut board = match RopeBoard::init(p.i2c0, p.i2c1, p.spi2, p.spi3, p.uart1) {
        Ok(b) => b,
        Err(e) => {
            log::error!("board init failed: {e:?}");
            return;
        }
    };

    report_pmu(&mut board);
    let RopeBoard {
        sensor_i2c,
        imu_spi,
        ads,
        lora,
        gnss,
        ..
    } = board;
    report_baro(sensor_i2c);
    report_imu(imu_spi);
    report_force(ads);
    radio_round_trip(lora);
    gnss_bring_up(&gnss);
    network_bring_up(&gnss, &mut store, modem, sysloop, nvs_part.clone());
    log::info!("bring-up done");

    let mut console = winchy_settings::Console::new();
    console.greet();
    loop {
        console.poll(&mut store, 20);
    }
}

fn report_settings(store: &mut winchy_settings::NvsStore) {
    use winchy_provision::{Key, Shown, Store, show};
    let mut buf = [0u8; winchy_provision::MAX_VALUE_LEN];
    let token = match store.get(Key::ZtpToken, &mut buf) {
        Ok(v) => show(Key::ZtpToken, v),
        Err(e) => {
            log::warn!("settings: {e:?}");
            Shown::Unset
        }
    };
    log::info!("settings: AssistNow token {token:?}");
}

fn report_pmu(board: &mut RopeBoard) {
    let pmu = &mut board.pmu;
    log::info!(
        "PMU: vsys {:?} mV, battery {:?} mV ({:?} %), charging {:?}",
        pmu.system_mv(),
        pmu.battery_mv(),
        pmu.battery_percent(),
        pmu.is_charging()
    );
}

fn report_baro(i2c: esp_idf_svc::hal::i2c::I2cDriver<'static>) {
    let mut bmp = match Bmp280::new(
        i2c,
        winchy_bmp280::ADDRESS,
        winchy_bmp280::Config::default(),
    ) {
        Ok(b) => b,
        Err(e) => {
            log::error!("BMP280: {e:?}");
            return;
        }
    };
    for _ in 0..3 {
        if let Err(e) = bmp.start_forced() {
            log::error!("BMP280 start: {e:?}");
            return;
        }
        FreeRtos::delay_ms(5);
        while bmp.is_measuring().unwrap_or(false) {
            FreeRtos::delay_ms(5);
        }
        match bmp.read() {
            Ok(m) => log::info!(
                "BMP280: {:.2} hPa, {:.2} degC, plausible {}",
                m.pressure_hpa(),
                f64::from(m.temperature_cdeg) / 100.0,
                m.is_plausible()
            ),
            Err(e) => log::error!("BMP280 read: {e:?}"),
        }
    }
}

fn report_imu(
    spi: esp_idf_svc::hal::spi::SpiDeviceDriver<'static, esp_idf_svc::hal::spi::SpiDriver<'static>>,
) {
    let mut imu = match Qmi8658::new(spi, &mut FreeRtos) {
        Ok(i) => i,
        Err(e) => {
            log::error!("QMI8658: {e:?}");
            return;
        }
    };
    FreeRtos::delay_ms(50);
    for _ in 0..5 {
        match imu.read_raw() {
            Ok(s) => {
                let a = s.accel_g();
                let norm = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
                log::info!(
                    "QMI8658: raw {:?}/{:?} accel {a:.3?} g |a| {norm:.3} gyro {:.2?} dps rope {:.1} deg",
                    s.accel,
                    s.gyro,
                    s.gyro_dps(),
                    rope_angle_above_ground_deg(a)
                );
            }
            Err(e) => log::error!("QMI8658 read: {e:?}"),
        }
        FreeRtos::delay_ms(20);
    }
    log::info!("QMI8658: temperature {:?} degC", imu.temperature_c());
}

fn report_force(pins: winchy_board::AdsPins) {
    let mut adc = match Ads1232::new(pins, Ets, Gain::X128) {
        Ok(a) => a,
        Err(e) => {
            log::error!("ADS1232: {e:?}");
            return;
        }
    };
    for _ in 0..5 {
        let mut waited = 0;
        while !adc.is_ready().unwrap_or(false) && waited < 500 {
            FreeRtos::delay_ms(1);
            waited += 1;
        }
        if waited >= 500 {
            log::warn!("ADS1232: no DRDY within 500 ms (load cell missing?)");
            return;
        }
        // Keep the 25 SCLK pulses unbroken (standby if SCLK stays high).
        let r = esp_idf_svc::hal::interrupt::free(|| adc.read_raw());
        log::info!("ADS1232: raw {r:?} (after {waited} ms)");
    }
}

/// Legacy (MicroPython protocol v6) TELEMETRY frame with `REQUEST_REPORT`, so
/// the unmodified winch answers with a v6 `LINK_REPORT`. Test-only.
fn legacy_v6_telemetry(seq: u16) -> [u8; 17] {
    let mut f = [0u8; 17];
    f[0] = 6; // version
    f[1] = 1; // TELEMETRY
    f[2..4].copy_from_slice(&seq.to_le_bytes());
    // phase 0, force 0, angle 0, altitude 0, battery 0
    f[13] = 0x08 | 0x01; // flags: REQUEST_REPORT | FORCE_UNCALIBRATED
    f[14] = 255; // battery percent unknown
    f
}

fn radio_round_trip(p: winchy_board::LoraPins) {
    let mut radio = Sx126x::new(p.spi, p.busy, p.reset, Ets);
    if let Err(e) = radio.init(&ChipConfig::LILYGO, &LoraConfig::WINCHY) {
        log::error!("SX1262 init: {e:?}");
        return;
    }
    log::info!("SX1262: initialised (869.525 MHz SF7 BW250 CR4/8 sync 0x12)");
    let mut dio1 = p.dio1;
    let mut buf = [0u8; 255];
    let (mut sent, mut replies) = (0u16, 0u32);
    for seq in 0..6u16 {
        if let Err(e) = radio.transmit(&legacy_v6_telemetry(seq)) {
            log::error!("SX1262 tx: {e:?}");
            return;
        }
        if !wait_irq(&mut radio, &mut dio1, IrqFlags::TX_DONE, 500) {
            log::warn!("SX1262: no TX_DONE");
            continue;
        }
        sent += 1;
        let _ = radio.receive_continuous();
        // Listen ~1.5 s for the winch's answer.
        for _ in 0..300 {
            FreeRtos::delay_ms(5);
            if dio1.is_low() {
                continue;
            }
            let Ok(flags) = radio.irq_status() else {
                continue;
            };
            let _ = radio.clear_irq(flags);
            if !flags.any(IrqFlags::RX_DONE) {
                continue;
            }
            match radio.read_packet(&mut buf) {
                Ok(info) => {
                    let f = &buf[..info.len];
                    let crc_ok = !flags.any(IrqFlags::CRC_ERR);
                    log::info!(
                        "SX1262 rx: {} B, rssi {} dBm, snr {} qdB, crc_ok {crc_ok}, bytes {:02x?}",
                        info.len,
                        info.status.rssi_dbm,
                        info.status.snr_qdb,
                        f
                    );
                    if crc_ok && f.len() == 8 && f[0] == 6 && f[1] == 5 {
                        replies += 1;
                        log::info!(
                            "  winch LINK_REPORT: our frame arrived with rssi {} dBm, snr {} qdB, loss {} %",
                            i16::from_le_bytes([f[4], f[5]]),
                            i8::from_le_bytes([f[6]]),
                            f[7]
                        );
                    }
                }
                Err(e) => log::error!("SX1262 read: {e:?}"),
            }
            break;
        }
    }
    log::info!("SX1262: sent {sent}, winch replies {replies}");
}

fn wait_irq<S, B, R, D, PE>(
    radio: &mut Sx126x<S, B, R, D>,
    dio1: &mut esp_idf_svc::hal::gpio::PinDriver<'static, esp_idf_svc::hal::gpio::Input>,
    want: u16,
    timeout_ms: u32,
) -> bool
where
    S: embedded_hal::spi::SpiDevice,
    B: embedded_hal::digital::InputPin<Error = PE>,
    R: embedded_hal::digital::OutputPin<Error = PE>,
    D: embedded_hal::delay::DelayNs,
{
    for _ in 0..timeout_ms {
        if dio1.is_high()
            && let Ok(f) = radio.irq_status()
        {
            let _ = radio.clear_irq(f);
            if f.any(want) {
                return true;
            }
        }
        FreeRtos::delay_ms(1);
    }
    false
}

fn gnss_listen(uart: &esp_idf_svc::hal::uart::UartDriver<'static>, baud: u32, ms: u32) -> bool {
    use esp_idf_svc::hal::delay::TickType;
    if uart
        .change_baudrate(esp_idf_svc::hal::units::Hertz(baud))
        .is_err()
    {
        return false;
    }
    let _ = uart.clear_rx();
    let mut window = [0u8; 600];
    let mut len = 0;
    let start = std::time::Instant::now();
    while start.elapsed().as_millis() < u128::from(ms) {
        let mut chunk = [0u8; 128];
        let n = uart
            .read(&mut chunk, TickType::new_millis(20).ticks())
            .unwrap_or(0);
        for &b in &chunk[..n] {
            if len == window.len() {
                window.copy_within(1.., 0);
                len -= 1;
            }
            window[len] = b;
            len += 1;
        }
        if winchy_gnss::detect::contains_gnss_frame(&window[..len]) {
            return true;
        }
    }
    false
}

fn gnss_send(
    uart: &esp_idf_svc::hal::uart::UartDriver<'static>,
    items: &[(u32, winchy_gnss::ubx::CfgValue)],
) {
    use winchy_gnss::ubx::{layer, write_valset};
    let mut frame = [0u8; 160];
    match write_valset(layer::RAM | layer::BBR, items, &mut frame) {
        Ok(n) => {
            let _ = uart.write(&frame[..n]);
        }
        Err(e) => log::error!("GNSS VALSET encode: {e:?}"),
    }
}

/// GNSS section duration (raise to ~900 s to wait for UTC parameters + PPS).
const GNSS_TEST_S: u64 = 30;

/// Runs the baud discovery state machine against the real UART.
fn gnss_find_baud(uart: &esp_idf_svc::hal::uart::UartDriver<'static>) -> u32 {
    use winchy_gnss::baud::{BaudProbe, ProbeAction};
    use winchy_gnss::ubx::{CfgValue, key};

    let mut probe = BaudProbe::new(9_600, 115_200, 12);
    let mut outcome = false;
    loop {
        match probe.step(outcome) {
            ProbeAction::Listen { baud, timeout_ms } => {
                outcome = gnss_listen(uart, baud, timeout_ms);
                log::info!(
                    "GNSS: listen {baud} baud -> {}",
                    if outcome { "alive" } else { "silent" }
                );
            }
            ProbeAction::Raise { to } => {
                gnss_send(uart, &[(key::UART1_BAUDRATE, CfgValue::U32(to))]);
                FreeRtos::delay_ms(150);
                outcome = false;
            }
            ProbeAction::Sleep { ms } => {
                FreeRtos::delay_ms(ms);
                outcome = false;
            }
            ProbeAction::Found { baud } => return baud,
            ProbeAction::GaveUp { baud } => {
                log::warn!("GNSS: baud probe gave up, assuming {baud}");
                return baud;
            }
        }
    }
}

fn gnss_bring_up(g: &winchy_board::GnssPins) {
    use esp_idf_svc::hal::delay::TickType;
    use winchy_gnss::time::{ClockSource, PpsDiscipline, TimeAction, TimeFix};
    use winchy_gnss::ubx::{Framer, NavPvt, msg, receiver_config};

    let uart = &g.uart;
    let baud = gnss_find_baud(uart);
    let _ = uart.change_baudrate(esp_idf_svc::hal::units::Hertz(baud));
    gnss_send(uart, &receiver_config(10));
    log::info!("GNSS: running at {baud} baud, NAV-PVT 10 Hz requested");

    let mut framer = Framer::new();
    let mut time_fix = TimeFix::default();
    let mut pps = PpsDiscipline::new();
    let mut clock = ClockSource::Unset;
    let mut clock_s = 0_i64;
    let (mut navs, mut edges, mut pps_prev) = (0u32, 0u32, g.pps.is_high());
    let (mut labelled, mut had_utc, mut had_tacc) = (0u32, false, false);
    let start = std::time::Instant::now();
    let mut buf = [0u8; 256];
    while start.elapsed().as_secs() < GNSS_TEST_S && labelled < 3 {
        let n = uart
            .read(&mut buf, TickType::new_millis(5).ticks())
            .unwrap_or(0);
        let now_ms = u64::try_from(start.elapsed().as_millis()).unwrap_or(u64::MAX);
        let level = g.pps.is_high();
        if level && !pps_prev {
            edges += 1;
            if let Some(s) = pps.on_edge(now_ms) {
                labelled += 1;
                log::info!("GNSS: PPS edge #{edges} marks UTC second {s} (t+{now_ms} ms)");
            } else if edges <= 3 {
                log::info!("GNSS: PPS edge #{edges} (not armed) at t+{now_ms} ms");
            }
        }
        pps_prev = level;
        for &b in &buf[..n] {
            let Some(p) = framer.push(b) else { continue };
            if (p.class, p.id) != msg::NAV_PVT {
                continue;
            }
            let Some(nav) = NavPvt::parse(p.payload) else {
                continue;
            };
            navs += 1;
            if nav.utc.is_some() && !had_utc {
                had_utc = true;
                log::info!("GNSS: UTC fully resolved after {now_ms} ms: {:?}", nav.utc);
            }
            if nav.tacc_ns < 1_000_000_000 && !had_tacc {
                had_tacc = true;
                log::info!("GNSS: tAcc {} ns after {now_ms} ms", nav.tacc_ns);
            }
            if let Some(utc_s) = nav.utc.and_then(|u| u.to_unix()) {
                match time_fix.decide(clock, utc_s, clock_s, nav.tacc_ns) {
                    TimeAction::Set => {
                        clock = ClockSource::Gnss;
                        clock_s = utc_s;
                        log::info!("GNSS: clock set from GNSS: {:?}", nav.utc);
                    }
                    TimeAction::Arm => clock_s = utc_s,
                    TimeAction::Reject | TimeAction::Wait => {}
                }
                if clock == ClockSource::Gnss {
                    pps.arm(utc_s, nav.nano_ns, now_ms);
                }
            }
            if navs % 600 == 1 {
                log::info!(
                    "GNSS: NAV-PVT #{navs} fix {:?} sats {} lat {:.6} lon {:.6} hMSL {:.1} m hAcc {:.1} m utc {:?} tAcc {} ns pps edges {edges}",
                    nav.fix,
                    nav.num_sv,
                    nav.lat_deg(),
                    nav.lon_deg(),
                    f64::from(nav.hmsl_mm) / 1000.0,
                    f64::from(nav.hacc_mm) / 1000.0,
                    nav.utc,
                    nav.tacc_ns
                );
            }
        }
    }
    log::info!("GNSS: {navs} NAV-PVT, {edges} PPS edges, {labelled} labelled");
}

/// Configured WiFi networks from NVS, in priority order (slot 1 first).
fn configured_networks(store: &mut winchy_settings::NvsStore) -> Vec<winchy_net_esp::Network> {
    use winchy_provision::{Key, Store, WIFI_SLOTS};
    (1..=WIFI_SLOTS)
        .filter_map(|slot| {
            let mut sb = [0u8; winchy_provision::MAX_VALUE_LEN];
            let ssid = store
                .get(Key::WifiSsid(slot), &mut sb)
                .ok()
                .flatten()?
                .to_owned();
            let mut pb = [0u8; winchy_provision::MAX_VALUE_LEN];
            let password = store
                .get(Key::WifiPass(slot), &mut pb)
                .ok()
                .flatten()
                .unwrap_or("")
                .to_owned();
            Some(winchy_net_esp::Network { ssid, password })
        })
        .collect()
}

fn network_bring_up(
    gnss: &winchy_board::GnssPins,
    store: &mut winchy_settings::NvsStore,
    modem: esp_idf_svc::hal::modem::Modem<'static>,
    sysloop: esp_idf_svc::eventloop::EspSystemEventLoop,
    nvs: esp_idf_svc::nvs::EspDefaultNvsPartition,
) {
    let networks = configured_networks(store);
    if networks.is_empty() {
        log::warn!("net: no WiFi configured (console: set wifi1_ssid ... / set wifi1_pass ...)");
        return;
    }
    let mut wifi = match winchy_net_esp::Wifi::new(modem, sysloop, nvs) {
        Ok(w) => w,
        Err(e) => {
            log::error!("net: WiFi init: {e:?}");
            return;
        }
    };
    let Some(ssid) = wifi.connect_any(&networks) else {
        log::warn!("net: no configured network could be joined");
        return;
    };
    log::info!(
        "net: joined '{ssid}', ip/gw {:?}, gateway probe {}",
        wifi.ip_and_gateway(),
        if wifi.probe_gateway(std::time::Duration::from_secs(2)) {
            "ok"
        } else {
            "FAILED"
        }
    );
    let now = match winchy_net_esp::sntp_sync(std::time::Duration::from_secs(15)) {
        Ok(t) => {
            log::info!("net: SNTP time {t}");
            Some(t)
        }
        Err(e) => {
            log::warn!("net: SNTP: {e:?}");
            None
        }
    };
    match winchy_storage::MgaCache::open() {
        Ok(cache) => assist::run(&gnss.uart, store, &cache, now),
        Err(e) => log::error!("assistnow: cache partition: {e:?}"),
    }
}
