//! Dashboard server: the page at `/`, the JSON snapshot at `/data` (curl,
//! debugging) and a WebSocket at `/ws` to which the snapshot is pushed every
//! [`PUSH_MS`] ms (content and page in `winch-logic::dashboard`).

use std::sync::{Arc, Mutex};

use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::http::Method;
use esp_idf_svc::http::server::ws::EspHttpWsDetachedSender;
use esp_idf_svc::http::server::{Configuration, EspHttpServer};
use esp_idf_svc::io::{EspIOError, Write as _};
use esp_idf_svc::sys::EspError;
use esp_idf_svc::ws::FrameType;
use winchy_gnss::time::ClockSource;
use winchy_winch_logic::dashboard::{PAGE, PUSH_MS, TimeOfDay, json};

use crate::shared::{State, lock, now_ms, spawn, unix_now_us};

/// Server task stack, bytes (JSON + TLS-free HTTP fits comfortably).
const SERVER_STACK: usize = 10 * 1024;

/// Current snapshot.
fn snapshot(state: &State) -> heapless::String<640> {
    let s = lock(state);
    let time = (s.clock != ClockSource::Unset)
        .then(|| TimeOfDay::from_unix(unix_now_us().div_euclid(1_000_000)));
    json(&s, now_ms(), time)
}

/// Starts the server and the WebSocket pusher.
///
/// # Errors
/// ESP-IDF errors while registering the handlers.
pub fn start(state: &State) -> Result<EspHttpServer<'static>, EspIOError> {
    let mut server = EspHttpServer::new(&Configuration {
        stack_size: SERVER_STACK,
        ..Configuration::default()
    })?;
    server.fn_handler("/", Method::Get, |req| {
        req.into_response(200, None, &[("Content-Type", "text/html")])?
            .write_all(PAGE.as_bytes())
    })?;
    let s = state.clone();
    server.fn_handler("/data", Method::Get, move |req| {
        let j = snapshot(&s);
        req.into_response(200, None, &[("Content-Type", "application/json")])?
            .write_all(j.as_bytes())
    })?;
    let clients: Arc<Mutex<Vec<EspHttpWsDetachedSender>>> = Arc::default();
    let c = clients.clone();
    // ESP-IDF 5.5 does not call this for the handshake, only per received
    // frame: the page sends "hello" on open, which registers it (once per
    // socket); later frames are drained.
    server.ws_handler("/ws", None, move |ws| {
        if ws.is_closed() {
            return Ok::<(), EspError>(());
        }
        let (_, len) = ws.recv(&mut [])?;
        let mut buf = [0u8; 64];
        if len > 0 && len <= buf.len() {
            ws.recv(&mut buf[..len])?;
        }
        let mut list = lock(&c);
        // A closed client's socket number is reused by the next one: drop
        // closed senders first, or the newcomer looks registered already.
        list.retain(|s| !s.is_closed());
        if !list.iter().any(|s| s.session() == ws.session()) {
            list.push(ws.create_detached_sender()?);
            log::info!("web: dashboard client {} connected", ws.session());
        }
        Ok(())
    })?;
    let s = state.clone();
    spawn(c"ws-push", 6 * 1024, 4, move || {
        loop {
            FreeRtos::delay_ms(PUSH_MS);
            let mut list = lock(&clients);
            if list.is_empty() {
                continue;
            }
            let j = snapshot(&s);
            list.retain_mut(|c| {
                !c.is_closed() && c.send(FrameType::Text(false), j.as_bytes()).is_ok()
            });
        }
    });
    log::info!("web: dashboard up");
    Ok(server)
}
