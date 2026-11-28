//! WiFi in the background: join the best configured network (as host
//! `winchy`), start the dashboard on the first join, SNTP while no GNSS time
//! exists yet, keep the link healthy and probe whether the internet (log
//! server) is reachable for the OLED's "I".

use std::net::{TcpStream, ToSocketAddrs};
use std::sync::Mutex;
use std::time::Duration;

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::modem::Modem;
use esp_idf_svc::http::server::EspHttpServer;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use winchy_gnss::time::ClockSource;
use winchy_net_esp::{Wifi, configured_networks};
use winchy_settings::NvsStore;

use crate::shared::{State, lock};
use crate::web;

/// DHCP hostname (reachable as `http://winchy/` where the router resolves it).
const HOSTNAME: &str = "winchy";
/// Upkeep period (join retry, roam, zombie and internet checks), s.
const CHECK_S: u32 = 30;
/// Log server host whose reachability lights the OLED's "I".
const ONLINE_HOST: &str = "api.github.com:443";
/// TCP connect timeout of that check.
const ONLINE_TIMEOUT: Duration = Duration::from_secs(3);

/// True if a TCP connection to the log server host succeeds (a WiFi with no
/// real internet, e.g. a dataless hotspot, must not show "I").
fn internet_ok() -> bool {
    ONLINE_HOST
        .to_socket_addrs()
        .ok()
        .and_then(|mut a| a.next())
        .is_some_and(|addr| TcpStream::connect_timeout(&addr, ONLINE_TIMEOUT).is_ok())
}

fn on_joined(wifi: &Wifi, ssid: &str, state: &State, server: &mut Option<EspHttpServer<'static>>) {
    match wifi.ip_and_gateway() {
        Some((ip, _)) => {
            log::info!("net: joined '{ssid}', dashboard http://{ip}/ (host {HOSTNAME})");
        }
        None => log::info!("net: joined '{ssid}'"),
    }
    if server.is_none() {
        match web::start(state) {
            Ok(s) => *server = Some(s),
            Err(e) => log::error!("net: dashboard {e:?}"),
        }
    }
    if lock(state).clock == ClockSource::Unset {
        match winchy_net_esp::sntp_sync(Duration::from_secs(15)) {
            Ok(t) => {
                let mut s = lock(state);
                if s.clock == ClockSource::Unset {
                    s.clock = ClockSource::Ntp;
                }
                log::info!("net: SNTP time {t}");
            }
            Err(e) => log::warn!("net: SNTP {e:?}"),
        }
    }
}

/// Task body.
pub fn run(
    modem: Modem<'static>,
    sysloop: EspSystemEventLoop,
    nvs: EspDefaultNvsPartition,
    state: &State,
    store: &Mutex<NvsStore>,
) {
    let networks = configured_networks(&mut *lock(store));
    if networks.is_empty() {
        log::warn!("net: no WiFi configured (console: set wifi1_ssid ... / set wifi1_pass ...)");
        return;
    }
    let mut wifi = match Wifi::new(modem, sysloop, nvs, Some(HOSTNAME)) {
        Ok(w) => w,
        Err(e) => {
            log::error!("net: WiFi init {e:?}");
            return;
        }
    };
    let mut server = None;
    let mut fails = 0u32;
    loop {
        if let Some(ssid) = wifi.maintain(&networks, &mut fails) {
            on_joined(&wifi, &ssid, state, &mut server);
        }
        let online = wifi.is_up() && internet_ok();
        let changed = {
            let mut s = lock(state);
            let changed = s.online != online;
            s.online = online;
            changed
        };
        if changed {
            log::info!(
                "net: internet {}",
                if online { "reachable" } else { "unreachable" }
            );
        }
        FreeRtos::delay_ms(CHECK_S * 1_000);
    }
}
