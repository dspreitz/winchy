//! Winchy rope unit firmware entry point (toolchain smoke test for now).

use winchy_fusion::gravity::GravityKalman;
use winchy_link::adr::{AdrConfig, Feedback, next_tx_power};
use winchy_protocol::{Frame, MAX_FRAME_LEN, Message, VERSION};

fn main() {
    // Required once so the ESP-IDF runtime patches are linked in.
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    log::info!(
        "winchy-rope {} booted, protocol {VERSION:#04x}",
        env!("CARGO_PKG_VERSION")
    );

    // Smoke-use the shared crates so they are cross-compiled and linked.
    let mut gravity = GravityKalman::default();
    gravity.update([0.0, 1.0, 0.0]);
    let power = next_tx_power(
        &AdrConfig::default(),
        0,
        Feedback::Stale { cycles: 1 },
        None,
    );
    let mut buf = [0u8; MAX_FRAME_LEN];
    let len = Frame {
        seq: 0,
        message: Message::UploadCmd { nonce: 1 },
    }
    .encode(&mut buf)
    .unwrap_or(0);
    let _ = winchy_gnss::ubx::Framer::new();
    log::info!(
        "gravity {:?}, tx power {power} dBm, frame {len} B",
        gravity.gravity()
    );
}
