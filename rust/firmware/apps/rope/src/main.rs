//! Winchy rope unit firmware (LilyGo T-Beam S3 Supreme).
//!
//! `main` brings the board up, hands every peripheral to its task and then
//! serves the USB provisioning console. Task priorities follow the rates:
//! IMU (50 Hz) above radio, force and GNSS; barometer, power and WiFi below.

mod assist;
mod shared;
mod tasks;

use std::sync::{Arc, Mutex};

use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::peripherals::Peripherals;
use winchy_board::RopeBoard;
use winchy_datalog::record::event;
use winchy_rope_logic::state::RopeState;
use winchy_sx126x::LoraConfig;

use shared::{lock, spawn};

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    // SAFETY: esp_reset_reason has no preconditions.
    let reset = unsafe { esp_idf_svc::sys::esp_reset_reason() };
    log::info!(
        "winchy-rope {} (reset reason {reset})",
        env!("CARGO_PKG_VERSION")
    );

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
    let store = match winchy_settings::NvsStore::open(nvs_part.clone()) {
        Ok(s) => Arc::new(Mutex::new(s)),
        Err(e) => {
            log::error!("NVS: {e:?}");
            return;
        }
    };
    let board = match RopeBoard::init(p.i2c0, p.i2c1, p.spi2, p.spi3, p.uart1) {
        Ok(b) => b,
        Err(e) => {
            log::error!("board init failed: {e:?}");
            return;
        }
    };
    let RopeBoard {
        pmu,
        sensor_i2c,
        imu_spi,
        ads,
        lora,
        gnss,
    } = board;
    let state = Arc::new(Mutex::new(RopeState::new(LoraConfig::WINCHY.tx_power_dbm)));
    let uart = Arc::new(Mutex::new(gnss.uart));

    let (logger, sink) = tasks::logger::channel();
    logger.event(event::BOOT, i32::try_from(reset).unwrap_or(-1));

    let (s, l) = (state.clone(), logger.clone());
    spawn(c"imu", 8 * 1024, 10, move || {
        tasks::imu::run(imu_spi, &s, &l);
    });
    let (s, l) = (state.clone(), logger.clone());
    spawn(c"radio", 8 * 1024, 9, move || {
        tasks::radio::run(lora, &s, &l);
    });
    let (s, l) = (state.clone(), logger.clone());
    spawn(c"force", 4 * 1024, 8, move || {
        tasks::force::run(ads, &s, &l);
    });
    let (s, st, u, l) = (state.clone(), store.clone(), uart.clone(), logger.clone());
    let pps = gnss.pps;
    spawn(c"gnss", 10 * 1024, 7, move || {
        tasks::gnss::run(&u, pps, &s, &st, &l);
    });
    let s = state.clone();
    spawn(c"logger", 12 * 1024, 6, move || {
        tasks::logger::run(&sink, &s);
    });
    let (s, st, l) = (state.clone(), store.clone(), logger.clone());
    spawn(c"baro", 6 * 1024, 5, move || {
        tasks::baro::run(sensor_i2c, &s, &st, &l);
    });
    let (s, l) = (state.clone(), logger.clone());
    spawn(c"power", 6 * 1024, 4, move || {
        tasks::power::run(pmu, &s, &l);
    });
    let (s, st, modem) = (state, store.clone(), p.modem);
    spawn(c"net", 20 * 1024, 3, move || {
        tasks::net::run(modem, sysloop, nvs_part, &s, &st, &uart, &logger);
    });

    let mut console = winchy_settings::Console::new();
    console.greet();
    loop {
        console.poll(&mut *lock(&store), 0);
        FreeRtos::delay_ms(20);
    }
}
