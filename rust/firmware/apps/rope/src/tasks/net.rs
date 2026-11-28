//! WiFi in the background: join the best configured network, SNTP as a time
//! fallback (GNSS wins), AssistNow refresh, then keep the link healthy
//! (rejoin, roam to a preferred network, zombie check).

use std::sync::Mutex;
use std::time::Duration;

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::modem::Modem;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use winchy_gnss::time::ClockSource;
use winchy_net_esp::{Wifi, configured_networks};
use winchy_settings::NvsStore;
use winchy_storage::MgaCache;

use crate::assist;
use crate::shared::{State, lock};
use crate::tasks::gnss::GnssUart;
use crate::tasks::logger::Logger;
use winchy_datalog::record::event;

/// Health check period, s.
const CHECK_S: u32 = 60;
/// AssistNow freshness check every this many health checks (~6 h).
const ASSIST_EVERY: u32 = 360;

/// Everything that needs a fresh connection: clock fallback and AssistNow.
fn on_joined(
    wifi: &Wifi,
    ssid: &str,
    state: &State,
    store: &Mutex<NvsStore>,
    uart: &GnssUart,
    logger: &Logger,
) {
    log::info!("net: joined '{ssid}', ip/gw {:?}", wifi.ip_and_gateway());
    let gnss_clock = lock(state).clock == ClockSource::Gnss;
    let now = if gnss_clock {
        Some(crate::shared::unix_now_us().div_euclid(1_000_000))
    } else {
        match winchy_net_esp::sntp_sync(Duration::from_secs(15)) {
            Ok(t) => {
                let mut s = lock(state);
                if s.clock == ClockSource::Unset {
                    s.clock = ClockSource::Ntp;
                    logger.event(event::CLOCK, event::CLOCK_NTP);
                }
                log::info!("net: SNTP time {t}");
                Some(t)
            }
            Err(e) => {
                log::warn!("net: SNTP {e:?}");
                None
            }
        }
    };
    refresh_assist(state, store, uart, now);
}

fn refresh_assist(_state: &State, store: &Mutex<NvsStore>, uart: &GnssUart, now: Option<i64>) {
    match MgaCache::open() {
        Ok(cache) => {
            if assist::refresh_if_due(uart, store, &cache, now) {
                log::info!("assistnow: refreshed");
            }
        }
        Err(e) => log::warn!("assistnow: cache partition {e:?}"),
    }
}

/// Task body.
pub fn run(
    modem: Modem<'static>,
    sysloop: EspSystemEventLoop,
    nvs: EspDefaultNvsPartition,
    state: &State,
    store: &Mutex<NvsStore>,
    uart: &GnssUart,
    logger: &Logger,
) {
    let networks = configured_networks(&mut *lock(store));
    if networks.is_empty() {
        log::warn!("net: no WiFi configured (console: set wifi1_ssid ... / set wifi1_pass ...)");
        return;
    }
    let mut wifi = match Wifi::new(modem, sysloop, nvs, None) {
        Ok(w) => w,
        Err(e) => {
            log::error!("net: WiFi init {e:?}");
            return;
        }
    };
    let mut fails = 0u32;
    let mut checks = 0u32;
    loop {
        if let Some(ssid) = wifi.maintain(&networks, &mut fails) {
            on_joined(&wifi, &ssid, state, store, uart, logger);
        }
        checks += 1;
        if checks.is_multiple_of(ASSIST_EVERY) && wifi.is_up() {
            let now = (lock(state).clock != ClockSource::Unset)
                .then(|| crate::shared::unix_now_us().div_euclid(1_000_000));
            refresh_assist(state, store, uart, now);
        }
        FreeRtos::delay_ms(CHECK_S * 1_000);
    }
}
