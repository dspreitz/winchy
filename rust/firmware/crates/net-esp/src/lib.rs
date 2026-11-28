//! Networking on ESP-IDF: WiFi with the `winchy-net` join/roam policy, HTTPS
//! with certificate-bundle verification, and SNTP.

use std::net::{SocketAddrV4, UdpSocket};
use std::time::{Duration, Instant};

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::modem::Modem;
use esp_idf_svc::http::Method;
use esp_idf_svc::http::client::{Configuration as HttpConfig, EspHttpConnection};
use esp_idf_svc::ipv4;
use esp_idf_svc::netif::{EspNetif, NetifConfiguration, NetifStack};
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::sntp::{EspSntp, SyncStatus};
use esp_idf_svc::sys::EspError;
use esp_idf_svc::wifi::{
    AuthMethod, BlockingWifi, ClientConfiguration, Configuration, EspWifi, WifiDriver,
};
use winchy_net::wifi::{MIN_ROAM_RSSI, ScanHit, order_candidates, preferred, roam_worth_scanning};
use winchy_provision::{Key, MAX_VALUE_LEN, Store, WIFI_SLOTS};

/// A configured WiFi network (priority = position in the list).
#[derive(Debug, Clone)]
pub struct Network {
    /// SSID.
    pub ssid: String,
    /// Password (empty = open network).
    pub password: String,
}

/// Networking error.
#[derive(Debug)]
pub enum NetError {
    /// ESP-IDF error.
    Esp(EspError),
    /// Response body exceeds the caller's limit.
    BodyTooLarge,
    /// Operation timed out.
    Timeout,
    /// Socket error.
    Io(std::io::ErrorKind),
}

impl From<EspError> for NetError {
    fn from(e: EspError) -> Self {
        Self::Esp(e)
    }
}

/// Per-network join timeout.
pub const JOIN_TIMEOUT: Duration = Duration::from_secs(15);

/// WiFi station.
pub struct Wifi {
    wifi: BlockingWifi<EspWifi<'static>>,
    joined: Option<String>,
}

impl Wifi {
    /// Creates and starts the station interface; `hostname` is announced via
    /// DHCP (so routers resolve it), `None` keeps the ESP-IDF default.
    ///
    /// # Errors
    /// ESP-IDF errors.
    pub fn new(
        modem: Modem<'static>,
        sysloop: EspSystemEventLoop,
        nvs: EspDefaultNvsPartition,
        hostname: Option<&str>,
    ) -> Result<Self, NetError> {
        let driver = WifiDriver::new(modem, sysloop.clone(), Some(nvs))?;
        let sta = match hostname {
            Some(h) => EspNetif::new_with_conf(&NetifConfiguration {
                ip_configuration: Some(ipv4::Configuration::Client(
                    ipv4::ClientConfiguration::DHCP(ipv4::DHCPClientSettings {
                        hostname: h.try_into().ok(),
                    }),
                )),
                ..NetifConfiguration::wifi_default_client()
            })?,
            None => EspNetif::new(NetifStack::Sta)?,
        };
        let esp = EspWifi::wrap_all(driver, sta, EspNetif::new(NetifStack::Ap)?)?;
        let mut wifi = BlockingWifi::wrap(esp, sysloop)?;
        wifi.set_configuration(&Configuration::Client(ClientConfiguration::default()))?;
        wifi.start()?;
        Ok(Self { wifi, joined: None })
    }

    /// Scans, retrying once on an empty result (driver glitch after a drop).
    fn scan(&mut self) -> Vec<(String, i8)> {
        for attempt in 0..2 {
            match self.wifi.scan() {
                Ok(aps) if !aps.is_empty() => {
                    return aps
                        .iter()
                        .map(|a| (a.ssid.to_string(), a.signal_strength))
                        .collect();
                }
                Ok(_) => {}
                Err(e) => log::warn!("wifi: scan failed: {e:?}"),
            }
            if attempt == 0 {
                FreeRtos::delay_ms(500);
            }
        }
        Vec::new()
    }

    /// Joins the highest-priority reachable network. Returns its SSID.
    pub fn connect_any(&mut self, networks: &[Network]) -> Option<String> {
        if self.is_up() {
            return self.joined.clone();
        }
        if networks.is_empty() {
            return None;
        }
        let _ = self.wifi.disconnect();
        let seen = self.scan();
        let hits: Vec<ScanHit<'_>> = seen
            .iter()
            .map(|(s, r)| ScanHit { ssid: s, rssi: *r })
            .collect();
        let ssids: Vec<&str> = networks.iter().map(|n| n.ssid.as_str()).collect();
        let order: Vec<usize> = order_candidates(&ssids, &hits).collect();
        for i in order {
            let net = &networks[i];
            let visible = hits.iter().any(|h| h.ssid == net.ssid);
            log::info!(
                "wifi: trying '{}'{}",
                net.ssid,
                if visible { "" } else { " (not in scan)" }
            );
            if self.try_join(net) {
                self.joined = Some(net.ssid.clone());
                return self.joined.clone();
            }
        }
        None
    }

    fn try_join(&mut self, net: &Network) -> bool {
        let (Ok(ssid), Ok(password)) = (
            net.ssid.as_str().try_into(),
            net.password.as_str().try_into(),
        ) else {
            return false;
        };
        let auth_method = if net.password.is_empty() {
            AuthMethod::None
        } else {
            AuthMethod::WPA2Personal
        };
        let conf = Configuration::Client(ClientConfiguration {
            ssid,
            password,
            auth_method,
            ..Default::default()
        });
        if self.wifi.set_configuration(&conf).is_err() || self.wifi.wifi_mut().connect().is_err() {
            return false;
        }
        let start = Instant::now();
        while start.elapsed() < JOIN_TIMEOUT {
            if self.is_up() {
                return true;
            }
            FreeRtos::delay_ms(250);
        }
        let _ = self.wifi.disconnect();
        false
    }

    /// True when associated and the interface has an IP.
    #[must_use]
    pub fn is_up(&self) -> bool {
        self.wifi.is_up().unwrap_or(false)
    }

    /// Station IP and gateway, when up.
    #[must_use]
    pub fn ip_and_gateway(&self) -> Option<(std::net::Ipv4Addr, std::net::Ipv4Addr)> {
        let info = self.wifi.wifi().sta_netif().get_ip_info().ok()?;
        Some((info.ip, info.subnet.gateway))
    }

    /// Two-way traffic proof: one DNS query to the gateway, any reply counts.
    #[must_use]
    pub fn probe_gateway(&self, timeout: Duration) -> bool {
        let Some((_, gw)) = self.ip_and_gateway() else {
            return false;
        };
        if gw.is_unspecified() {
            return false;
        }
        let Ok(sock) = UdpSocket::bind("0.0.0.0:0") else {
            return false;
        };
        let _ = sock.set_read_timeout(Some(timeout));
        let id = 0x5743; // "WC"
        if sock
            .send_to(&winchy_net::dns::probe_query(id), SocketAddrV4::new(gw, 53))
            .is_err()
        {
            return false;
        }
        let mut buf = [0u8; 512];
        matches!(sock.recv_from(&mut buf), Ok((n, _)) if winchy_net::dns::is_reply(&buf[..n], id))
    }

    /// Drops the association (the next [`Self::connect_any`] rejoins).
    pub fn disconnect(&mut self) {
        let _ = self.wifi.disconnect();
        self.joined = None;
    }

    /// While on a lower-priority network, switches to a better one that came
    /// in range. Returns the new SSID, `None` if it stayed.
    pub fn roam(&mut self, networks: &[Network]) -> Option<String> {
        let current = self.joined.clone()?;
        let ssids: Vec<&str> = networks.iter().map(|n| n.ssid.as_str()).collect();
        if !self.is_up() || !roam_worth_scanning(&ssids, &current) {
            return None;
        }
        let seen = self.scan();
        let hits: Vec<ScanHit<'_>> = seen
            .iter()
            .map(|(s, r)| ScanHit { ssid: s, rssi: *r })
            .collect();
        let target = preferred(&ssids, &current, &hits, MIN_ROAM_RSSI)?;
        log::info!("wifi: roaming '{current}' -> '{}'", networks[target].ssid);
        let _ = self.wifi.disconnect();
        self.joined = None;
        self.connect_any(networks)
    }

    /// One upkeep step: join while down, roam up the priority list while up,
    /// otherwise prove two-way traffic and rejoin a link that is associated
    /// but dead (`fails` counts consecutive failed probes). Returns the SSID
    /// whenever a new association was made.
    pub fn maintain(&mut self, networks: &[Network], fails: &mut u32) -> Option<String> {
        if !self.is_up() {
            let joined = self.connect_any(networks);
            if joined.is_none() {
                log::info!("wifi: no configured network in range");
            }
            return joined;
        }
        if let Some(ssid) = self.roam(networks) {
            return Some(ssid);
        }
        if self.probe_gateway(Duration::from_secs(2)) {
            *fails = 0;
        } else {
            *fails += 1;
            log::warn!("wifi: gateway probe failed ({fails})");
            if *fails >= ZOMBIE_FAILS {
                log::warn!("wifi: link is a zombie, rejoining");
                *fails = 0;
                self.disconnect();
                return self.connect_any(networks);
            }
        }
        None
    }
}

/// Consecutive failed gateway probes before a link counts as a zombie.
pub const ZOMBIE_FAILS: u32 = 2;

/// Networks provisioned in `store` (console `set wifiN_ssid/pass`), in
/// priority order (slot 1 first).
pub fn configured_networks<S: Store>(store: &mut S) -> Vec<Network> {
    (1..=WIFI_SLOTS)
        .filter_map(|slot| {
            let mut sb = [0u8; MAX_VALUE_LEN];
            let ssid = store
                .get(Key::WifiSsid(slot), &mut sb)
                .ok()
                .flatten()?
                .to_owned();
            let mut pb = [0u8; MAX_VALUE_LEN];
            let password = store
                .get(Key::WifiPass(slot), &mut pb)
                .ok()
                .flatten()
                .unwrap_or("")
                .to_owned();
            Some(Network { ssid, password })
        })
        .collect()
}

/// An HTTP response.
#[derive(Debug)]
pub struct HttpResponse {
    /// Status code.
    pub status: u16,
    /// Body.
    pub body: Vec<u8>,
    /// `Date` header, if present.
    pub date: Option<String>,
}

/// Performs an HTTPS request (server certificate verified against the ESP-IDF
/// CA bundle). The body is limited to `max_body` bytes.
///
/// # Errors
/// [`NetError`].
pub fn https(
    method: Method,
    url: &str,
    content_type: Option<&str>,
    body: &[u8],
    max_body: usize,
    timeout: Duration,
) -> Result<HttpResponse, NetError> {
    let mut conn = EspHttpConnection::new(&HttpConfig {
        crt_bundle_attach: Some(esp_idf_svc::sys::esp_crt_bundle_attach),
        timeout: Some(timeout),
        ..Default::default()
    })?;
    let len = body.len().to_string();
    let mut headers = vec![("Content-Length", len.as_str())];
    if let Some(ct) = content_type {
        headers.push(("Content-Type", ct));
    }
    conn.initiate_request(method, url, &headers)?;
    let mut sent = 0;
    while sent < body.len() {
        sent += conn.write(&body[sent..])?;
    }
    conn.initiate_response()?;
    let status = conn.status();
    let date = conn.header("Date").map(str::to_owned);
    let mut out = Vec::new();
    let mut chunk = [0u8; 1024];
    loop {
        let n = conn.read(&mut chunk)?;
        if n == 0 {
            break;
        }
        if out.len() + n > max_body {
            return Err(NetError::BodyTooLarge);
        }
        out.extend_from_slice(&chunk[..n]);
    }
    Ok(HttpResponse {
        status,
        body: out,
        date,
    })
}

/// Sets the system clock via SNTP; returns the Unix time once synced.
///
/// # Errors
/// [`NetError::Timeout`] or ESP-IDF errors.
pub fn sntp_sync(timeout: Duration) -> Result<i64, NetError> {
    let sntp = EspSntp::new_default()?;
    let start = Instant::now();
    while sntp.get_sync_status() != SyncStatus::Completed {
        if start.elapsed() > timeout {
            return Err(NetError::Timeout);
        }
        FreeRtos::delay_ms(200);
    }
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_err(|_| NetError::Timeout)?;
    Ok(i64::try_from(now.as_secs()).unwrap_or(i64::MAX))
}
