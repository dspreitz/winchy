//! u-blox AssistNow: register the receiver once (ZTP), download predicted
//! orbits over HTTPS when the cache is missing or stale, and feed them to the
//! GNSS receiver. The request/response logic lives in `winchy_gnss::assistnow`.

use std::time::{Duration, Instant};

use esp_idf_svc::hal::delay::{FreeRtos, TickType};
use esp_idf_svc::hal::uart::UartDriver;
use esp_idf_svc::http::Method;
use winchy_gnss::assistnow::{
    Cache, FEED_CHUNK, FEED_GAP_MS, ZTP_URL, download_url, json_string_field, parse_http_date,
    register_body, should_download,
};
use winchy_gnss::time::UtcDateTime;
use winchy_gnss::ubx::{Framer, msg, write_frame, write_mga_ini_time_utc};
use winchy_net_esp::https;
use winchy_provision::{Key, Store};
use winchy_settings::NvsStore;
use winchy_storage::MgaCache;

const NVS_CHIPCODE: &str = "mga_chip";
const NVS_SERVICE_URL: &str = "mga_url";
const HTTP_TIMEOUT: Duration = Duration::from_secs(20);
const MAX_BLOB: usize = 200 * 1024;

/// Polls one UBX message (empty payload) and returns the complete reply frame.
fn poll_frame(uart: &UartDriver<'static>, (class, id): (u8, u8)) -> Option<Vec<u8>> {
    let mut poll = [0u8; 8];
    let n = write_frame(class, id, &[], &mut poll).ok()?;
    let _ = uart.clear_rx();
    uart.write(&poll[..n]).ok()?;
    let mut framer = Framer::new();
    let start = Instant::now();
    let mut buf = [0u8; 128];
    while start.elapsed() < Duration::from_millis(1500) {
        let got = uart
            .read(&mut buf, TickType::new_millis(20).ticks())
            .unwrap_or(0);
        for &b in &buf[..got] {
            if let Some(p) = framer.push(b)
                && (p.class, p.id) == (class, id)
            {
                let mut frame = vec![0u8; p.payload.len() + 8];
                let len = write_frame(class, id, p.payload, &mut frame).ok()?;
                frame.truncate(len);
                return Some(frame);
            }
        }
    }
    None
}

/// Streams the MGA blob (optionally preceded by coarse UTC time aiding).
fn feed(uart: &UartDriver<'static>, blob: &[u8], time: Option<UtcDateTime>, acc_s: u16) {
    if let Some(t) = time {
        let mut frame = [0u8; 32];
        if let Ok(n) = write_mga_ini_time_utc(t, acc_s, &mut frame) {
            let _ = uart.write(&frame[..n]);
            FreeRtos::delay_ms(20);
        }
    }
    for chunk in blob.chunks(FEED_CHUNK) {
        let _ = uart.write(chunk);
        FreeRtos::delay_ms(FEED_GAP_MS);
    }
    log::info!(
        "assistnow: fed {} B to the receiver (time aid: {})",
        blob.len(),
        time.is_some()
    );
}

/// One-time ZTP registration; caches chipcode + service URL in NVS.
fn register(uart: &UartDriver<'static>, store: &mut NvsStore, token: &str) -> Option<()> {
    let uniqid = poll_frame(uart, msg::SEC_UNIQID);
    let monver = poll_frame(uart, msg::MON_VER);
    let (Some(uniqid), Some(monver)) = (uniqid, monver) else {
        log::warn!("assistnow: could not read the receiver identity");
        return None;
    };
    let mut body = vec![0u8; 64 + token.len() + 2 * (uniqid.len() + monver.len())];
    let n = register_body(token, &uniqid, &monver, &mut body)
        .map_err(|e| log::warn!("assistnow: register body: {e:?}"))
        .ok()?;
    let mut url_buf = [0u8; winchy_provision::MAX_VALUE_LEN];
    let url = store
        .get(Key::ZtpUrl, &mut url_buf)
        .ok()
        .flatten()
        .unwrap_or(ZTP_URL)
        .to_owned();
    let resp = https(
        Method::Post,
        &url,
        Some("application/json"),
        &body[..n],
        4096,
        HTTP_TIMEOUT,
    )
    .map_err(|e| log::warn!("assistnow: register request: {e:?}"))
    .ok()?;
    if resp.status != 200 {
        log::warn!("assistnow: register HTTP {}", resp.status);
        return None;
    }
    let (mut chip, mut svc) = ([0u8; 256], [0u8; 256]);
    let chipcode = json_string_field(&resp.body, "chipcode", &mut chip)?;
    let service_url = json_string_field(&resp.body, "serviceUrl", &mut svc)?;
    store.set_internal(NVS_CHIPCODE, chipcode).ok()?;
    store.set_internal(NVS_SERVICE_URL, service_url).ok()?;
    log::info!("assistnow: receiver registered");
    Some(())
}

/// Downloads a fresh blob; returns it with the server time.
fn download(store: &NvsStore) -> Option<(Vec<u8>, Option<UtcDateTime>)> {
    let (mut chip, mut svc) = ([0u8; 256], [0u8; 256]);
    let chipcode = store.get_internal(NVS_CHIPCODE, &mut chip).ok().flatten()?;
    let service_url = store
        .get_internal(NVS_SERVICE_URL, &mut svc)
        .ok()
        .flatten()?;
    let mut url = [0u8; 768];
    let url = download_url(service_url, chipcode, &mut url).ok()?;
    let resp = https(Method::Get, url, None, &[], MAX_BLOB, HTTP_TIMEOUT)
        .map_err(|e| log::warn!("assistnow: download: {e:?}"))
        .ok()?;
    if resp.status != 200 || resp.body.is_empty() {
        log::warn!(
            "assistnow: download HTTP {} ({} B)",
            resp.status,
            resp.body.len()
        );
        return None;
    }
    Some((resp.body, resp.date.as_deref().and_then(parse_http_date)))
}

/// Registers if needed, downloads, caches and feeds a fresh blob.
/// Returns false if anything failed (the caller then feeds the old cache).
fn refresh(
    uart: &UartDriver<'static>,
    store: &mut NvsStore,
    cache: &MgaCache,
    now: Option<i64>,
) -> bool {
    let mut tok = [0u8; winchy_provision::MAX_VALUE_LEN];
    let Some(token) = store
        .get(Key::ZtpToken, &mut tok)
        .ok()
        .flatten()
        .map(str::to_owned)
    else {
        log::warn!("assistnow: no ztp_token provisioned (console: set ztp_token ...)");
        return false;
    };
    let mut probe = [0u8; 8];
    let registered = store
        .get_internal(NVS_CHIPCODE, &mut probe)
        .ok()
        .flatten()
        .is_some();
    if !registered && register(uart, store, &token).is_none() {
        return false;
    }
    let Some((blob, server_time)) = download(store) else {
        return false;
    };
    let stamp = server_time.and_then(|t| t.to_unix()).or(now).unwrap_or(0);
    match cache.store(&blob, stamp) {
        Ok(()) => log::info!("assistnow: downloaded and cached {} B", blob.len()),
        Err(e) => log::warn!("assistnow: cache write: {e:?}"),
    }
    feed(uart, &blob, server_time, 4);
    true
}

/// Full AssistNow cycle. `now` is the system time if it is trustworthy.
pub fn run(uart: &UartDriver<'static>, store: &mut NvsStore, cache: &MgaCache, now: Option<i64>) {
    let state = cache.header().map_or(Cache::Missing, |h| Cache::Present {
        downloaded_at_s: (h.downloaded_at_s > 0).then_some(h.downloaded_at_s),
    });
    log::info!("assistnow: cache {state:?}");
    if should_download(state, now) && refresh(uart, store, cache, now) {
        return;
    }
    match cache.load() {
        Some((_, blob)) => feed(uart, &blob, None, 2),
        None => log::info!("assistnow: nothing to feed"),
    }
}
