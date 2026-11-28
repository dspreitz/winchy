//! LoRa: continuous receive; every telemetry frame is answered at once with
//! at most one frame (link report or winch position), while the rope listens.

use esp_idf_svc::hal::delay::{Ets, FreeRtos};
use esp_idf_svc::hal::gpio::{Input, Output, PinDriver};
use esp_idf_svc::hal::spi::{SpiDeviceDriver, SpiDriver};
use winchy_protocol::{Frame, Message, decode_any};
use winchy_sx126x::{ChipConfig, IrqFlags, LoraConfig, Sx126x};
use winchy_winch_logic::responder::{REPLY_GUARD_MS, Responder};
use winchy_winch_logic::state::RxTelemetry;
use winchy_winch_logic::winch_pos;

use crate::shared::{State, lock, now_ms};

/// TX of a short frame takes ~30 ms at SF7/BW250; give up after this, ms.
const TX_TIMEOUT_MS: u64 = 300;
/// Reply power: the winch is mains-powered and fixed (no ADR); full power
/// keeps the link symmetric so the back channel reaches the rope at range
/// (within band g3's 500 mW ERP).
const TX_POWER_DBM: i8 = 22;
/// Log a reception summary every this many frames.
const LOG_EVERY: u32 = 60;

type Radio = Sx126x<
    SpiDeviceDriver<'static, SpiDriver<'static>>,
    PinDriver<'static, Input>,
    PinDriver<'static, Output>,
    Ets,
>;

/// Reads and clears pending IRQs if DIO1 is up.
fn take_irq(radio: &mut Radio, dio1: &PinDriver<'static, Input>) -> Option<IrqFlags> {
    if dio1.is_low() {
        return None;
    }
    let flags = radio.irq_status().ok()?;
    let _ = radio.clear_irq(flags);
    Some(flags)
}

/// Sends `reply`, waits for `TX_DONE` and returns to continuous receive.
fn send(radio: &mut Radio, dio1: &PinDriver<'static, Input>, reply: &Frame) -> bool {
    let mut out = [0u8; winchy_protocol::MAX_FRAME_LEN];
    let ok = match reply.encode(&mut out) {
        Ok(len) => match radio.transmit(&out[..len]) {
            Ok(()) => {
                let start = now_ms();
                let mut done = false;
                while !done && now_ms() - start < TX_TIMEOUT_MS {
                    done = take_irq(radio, dio1).is_some_and(|f| f.any(IrqFlags::TX_DONE));
                    if !done {
                        FreeRtos::delay_ms(2);
                    }
                }
                done
            }
            Err(e) => {
                log::warn!("radio: tx {e:?}");
                false
            }
        },
        Err(e) => {
            log::error!("radio: encode {e:?}");
            false
        }
    };
    if let Err(e) = radio.receive_continuous() {
        log::warn!("radio: rx restart {e:?}");
    }
    ok
}

/// Task body.
pub fn run(pins: winchy_board::LoraPins, state: &State) {
    let mut radio = Sx126x::new(pins.spi, pins.busy, pins.reset, Ets);
    if let Err(e) = radio.init(&ChipConfig::LILYGO, &LoraConfig::WINCHY) {
        log::error!("radio: SX1262 init {e:?}");
        return;
    }
    let power = radio.set_tx_power(TX_POWER_DBM).unwrap_or_else(|e| {
        log::warn!("radio: tx power {e:?}");
        LoraConfig::WINCHY.tx_power_dbm
    });
    let dio1 = pins.dio1;
    if let Err(e) = radio.receive_continuous() {
        log::error!("radio: rx start {e:?}");
    }
    log::info!("radio: SX1262 up, listening ({power} dBm replies)");
    let mut responder = Responder::new();
    let mut buf = [0u8; 255];
    loop {
        let Some(flags) = take_irq(&mut radio, &dio1) else {
            FreeRtos::delay_ms(2);
            continue;
        };
        if !flags.any(IrqFlags::RX_DONE) || flags.any(IrqFlags::CRC_ERR | IrqFlags::HEADER_ERR) {
            continue;
        }
        let Ok(info) = radio.read_packet(&mut buf) else {
            continue;
        };
        let Some(frame) = decode_any(&buf[..info.len]) else {
            log::debug!("radio: foreign frame {:02x?}", &buf[..info.len]);
            continue;
        };
        let now = now_ms();
        let (rssi, snr) = (info.status.rssi_dbm, info.status.snr_qdb);
        let pos = {
            let s = lock(state);
            winch_pos::build(&s.survey, s.fix)
        };
        let reply = responder.on_frame(&frame, rssi, snr, now, pos);
        {
            let mut s = lock(state);
            if let Message::Telemetry(telemetry) = frame.message {
                s.telemetry = Some(RxTelemetry {
                    telemetry,
                    rssi_dbm: rssi,
                    at_ms: now,
                });
            }
            s.received = responder.stats().received();
            s.lost = responder.stats().lost();
        }
        match frame.message {
            Message::Telemetry(_) => {}
            other => log::info!("radio: {other:?}"),
        }
        if responder.stats().received() % LOG_EVERY == 1 {
            log::info!(
                "radio: rx {} lost {} (last {rssi} dBm, snr {snr} qdB)",
                responder.stats().received(),
                responder.stats().lost()
            );
        }
        if let Some(r) = reply {
            FreeRtos::delay_ms(REPLY_GUARD_MS);
            let ok = send(&mut radio, &dio1, &r);
            responder.sent(&r, ok, now_ms());
            match (ok, r.message) {
                (true, Message::WinchPos(p)) => log::info!(
                    "radio: winch position sent (hAcc {:.1} m, status {:#04x})",
                    f32::from(p.hacc_dm) / 10.0,
                    p.status.0
                ),
                (true, _) => {}
                (false, m) => log::warn!("radio: reply not sent: {m:?}"),
            }
        }
    }
}
