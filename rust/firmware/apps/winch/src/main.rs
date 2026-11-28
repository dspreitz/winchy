//! Winchy winch unit firmware (LilyGo T3S3 V1.2).
//!
//! `main` brings the board up, hands every peripheral to its task and then
//! serves the USB provisioning console. The radio task answers the rope
//! within its listen window, so it runs above GNSS and the display.

mod shared;
mod tasks;
mod web;

use std::sync::{Arc, Mutex};

use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::peripherals::Peripherals;
use winchy_board::WinchBoard;
use winchy_winch_logic::state::WinchState;

use shared::{lock, spawn};

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    // SAFETY: esp_reset_reason has no preconditions.
    let reset = unsafe { esp_idf_svc::sys::esp_reset_reason() };
    log::info!(
        "winchy-winch {} (reset reason {reset})",
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
    let WinchBoard { lora, oled, gnss } = match WinchBoard::init(p.i2c0, p.spi2, p.uart1) {
        Ok(b) => b,
        Err(e) => {
            log::error!("board init failed: {e:?}");
            return;
        }
    };
    let state = Arc::new(Mutex::new(WinchState::new()));

    let s = state.clone();
    spawn(c"radio", 8 * 1024, 9, move || tasks::radio::run(lora, &s));
    let s = state.clone();
    spawn(c"gnss", 8 * 1024, 7, move || tasks::gnss::run(gnss, &s));
    let s = state.clone();
    spawn(c"display", 8 * 1024, 5, move || {
        tasks::display::run(oled, &s);
    });
    let (st, modem) = (store.clone(), p.modem);
    spawn(c"net", 16 * 1024, 3, move || {
        tasks::net::run(modem, sysloop, nvs_part, &state, &st);
    });

    let mut console = winchy_settings::Console::new();
    console.greet();
    loop {
        console.poll(&mut *lock(&store), 0);
        FreeRtos::delay_ms(20);
    }
}
