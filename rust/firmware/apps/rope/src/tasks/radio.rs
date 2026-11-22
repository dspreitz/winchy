//! LoRa at 2 Hz: ADR, telemetry, and listening for the winch between frames.

use esp_idf_svc::hal::delay::{Ets, FreeRtos};
use esp_idf_svc::hal::gpio::{Input, PinDriver};
use winchy_protocol::Message;
use winchy_rope_logic::radio::{RadioConfig, RadioLink};
use winchy_sx126x::{ChipConfig, IrqFlags, LoraConfig, Sx126x};

use winchy_datalog::record::Record;

use crate::shared::{State, lock, now_ms, now_us};
use crate::tasks::logger::Logger;

/// TX of a 24-byte frame takes ~40 ms at SF7/BW250; give up after this, ms.
const TX_TIMEOUT_MS: u64 = 300;

type Radio = Sx126x<
    esp_idf_svc::hal::spi::SpiDeviceDriver<'static, esp_idf_svc::hal::spi::SpiDriver<'static>>,
    PinDriver<'static, Input>,
    PinDriver<'static, esp_idf_svc::hal::gpio::Output>,
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

fn wait_tx_done(radio: &mut Radio, dio1: &PinDriver<'static, Input>) -> bool {
    let start = now_ms();
    while now_ms() - start < TX_TIMEOUT_MS {
        if take_irq(radio, dio1).is_some_and(|f| f.any(IrqFlags::TX_DONE)) {
            return true;
        }
        FreeRtos::delay_ms(2);
    }
    false
}

/// Listens until `until_ms`, handing every good frame to the link logic.
fn listen(
    radio: &mut Radio,
    dio1: &PinDriver<'static, Input>,
    link: &mut RadioLink,
    state: &State,
    logger: &Logger,
    until_ms: u64,
) {
    if let Err(e) = radio.receive_continuous() {
        log::warn!("radio: rx start {e:?}");
    }
    let mut buf = [0u8; 255];
    while now_ms() < until_ms {
        let Some(flags) = take_irq(radio, dio1) else {
            FreeRtos::delay_ms(5);
            continue;
        };
        if !flags.any(IrqFlags::RX_DONE) || flags.any(IrqFlags::CRC_ERR | IrqFlags::HEADER_ERR) {
            continue;
        }
        let Ok(info) = radio.read_packet(&mut buf) else {
            continue;
        };
        let frame = &buf[..info.len];
        let t = now_us();
        let (msg, tx_power_dbm) = {
            let mut s = lock(state);
            (link.on_rx(frame, now_ms(), &mut s), s.link.tx_power_dbm)
        };
        if let Some(Message::LinkReport(r)) = msg {
            logger.log(
                t,
                Record::Link {
                    rssi_dbm: r.rssi_dbm,
                    snr_qdb: r.snr_qdb,
                    loss_pct: r.loss_pct,
                    tx_power_dbm,
                },
            );
        }
        match msg {
            Some(Message::LinkReport(r)) => log::debug!(
                "radio: link report rssi {} dBm snr {} qdB loss {} % (rx {} dBm)",
                r.rssi_dbm,
                r.snr_qdb,
                r.loss_pct,
                info.status.rssi_dbm
            ),
            Some(Message::WinchPos(p)) => log::info!(
                "radio: winch position {:.6} {:.6} hAcc {:.1} m status {:#04x}",
                f64::from(p.lat_e7) * 1e-7,
                f64::from(p.lon_e7) * 1e-7,
                f64::from(p.hacc_dm) / 10.0,
                p.status.0
            ),
            Some(other) => log::info!("radio: {other:?}"),
            None => log::debug!("radio: foreign frame {frame:02x?}"),
        }
    }
}

/// Task body.
pub fn run(pins: winchy_board::LoraPins, state: &State, logger: &Logger) {
    let mut radio = Sx126x::new(pins.spi, pins.busy, pins.reset, Ets);
    if let Err(e) = radio.init(&ChipConfig::LILYGO, &LoraConfig::WINCHY) {
        log::error!("radio: SX1262 init {e:?}");
        return;
    }
    let dio1 = pins.dio1;
    let cfg = RadioConfig::default();
    log::info!("radio: SX1262 up, telemetry as {:?}", cfg.wire);
    let mut link = RadioLink::new(cfg);
    let mut out = [0u8; 32];
    loop {
        let start = now_ms();
        let snapshot = *lock(state);
        let plan = match link.plan_tx(&snapshot, start, &mut out) {
            Ok(p) => p,
            Err(e) => {
                log::error!("radio: encode {e:?}");
                FreeRtos::delay_ms(cfg.period_ms);
                continue;
            }
        };
        if plan.tx_power_dbm != snapshot.link.tx_power_dbm {
            match radio.set_tx_power(plan.tx_power_dbm) {
                Ok(p) => {
                    log::info!("radio: ADR {} -> {p} dBm", snapshot.link.tx_power_dbm);
                    lock(state).link.tx_power_dbm = p;
                }
                Err(e) => log::warn!("radio: power change {e:?}"),
            }
        }
        match radio.transmit(&out[..plan.len]) {
            Ok(()) if wait_tx_done(&mut radio, &dio1) => {
                let mut s = lock(state);
                s.link.tx_count = s.link.tx_count.wrapping_add(1);
            }
            Ok(()) => log::warn!("radio: no TX_DONE (seq {})", plan.seq),
            Err(e) => log::warn!("radio: tx {e:?}"),
        }
        listen(
            &mut radio,
            &dio1,
            &mut link,
            state,
            logger,
            start + u64::from(plan.listen_ms),
        );
    }
}
