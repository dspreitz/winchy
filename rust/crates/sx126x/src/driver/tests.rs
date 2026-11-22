use super::*;
use crate::config::{ChipConfig, LoraConfig};
use core::cell::RefCell;
use core::convert::Infallible;
use embedded_hal::digital::ErrorType;
use embedded_hal_mock::eh1::delay::NoopDelay;
use embedded_hal_mock::eh1::spi::{Mock, Transaction as T};
use std::rc::Rc;

/// BUSY pin: low (ready) unless `stuck`.
struct Busy {
    stuck: bool,
}
impl ErrorType for Busy {
    type Error = Infallible;
}
impl InputPin for Busy {
    fn is_high(&mut self) -> Result<bool, Infallible> {
        Ok(self.stuck)
    }
    fn is_low(&mut self) -> Result<bool, Infallible> {
        Ok(!self.stuck)
    }
}

/// RESET pin recording its level history.
#[derive(Clone, Default)]
struct Reset(Rc<RefCell<Vec<bool>>>);
impl ErrorType for Reset {
    type Error = Infallible;
}
impl OutputPin for Reset {
    fn set_low(&mut self) -> Result<(), Infallible> {
        self.0.borrow_mut().push(false);
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Infallible> {
        self.0.borrow_mut().push(true);
        Ok(())
    }
}

fn w(bytes: &[u8]) -> Vec<T<u8>> {
    vec![
        T::transaction_start(),
        T::write_vec(bytes.to_vec()),
        T::transaction_end(),
    ]
}

fn rd(cmd: &[u8], reply: &[u8]) -> Vec<T<u8>> {
    vec![
        T::transaction_start(),
        T::transfer_in_place(cmd.to_vec(), reply.to_vec()),
        T::transaction_end(),
    ]
}

type Radio = Sx126x<Mock<u8>, Busy, Reset, NoopDelay>;

fn radio(t: &[T<u8>]) -> (Radio, Reset) {
    let reset = Reset::default();
    (
        Sx126x::new(
            Mock::new(t),
            Busy { stuck: false },
            reset.clone(),
            NoopDelay::new(),
        ),
        reset,
    )
}

fn done(r: Radio) {
    r.release().0.done();
}

/// The exact init sequence for Winchy on a LilyGo board.
fn init_seq() -> Vec<T<u8>> {
    [
        rd(&[0x12, 0, 0, 0], &[0, 0x22, 0, 0]), // status: STDBY_RC
        w(&[0x9D, 0x01]),                       // DIO2 = RF switch
        w(&[0x97, 0x01, 0x00, 0x01, 0x40]),     // TCXO 1.7 V, 5 ms
        w(&[0x07, 0x00, 0x00]),                 // clear device errors
        rd(&[0x17, 0, 0, 0], &[0, 0x22, 0, 0]), // no device errors
        w(&[0x08, 0x02, 0x43, 0x02, 0x03, 0, 0, 0, 0]), // IRQ + DIO1 masks
        w(&[0x02, 0xFF, 0xFF]),                 // clear IRQs
        w(&[0x8A, 0x01]),                       // LoRa
        w(&[0x86, 0x36, 0x58, 0x66, 0x66]),     // 869.525 MHz
        w(&[0x0D, 0x07, 0x40, 0x14, 0x24]),     // sync word 0x12
        w(&[0x95, 0x02, 0x02, 0x00, 0x01]),     // PA for 14 dBm
        w(&[0x8E, 22, 0x02]),                   // TX params
        w(&[0x8B, 7, 0x05, 4, 0]),              // SF7 BW250 CR4/8 no LDRO
        rd(&[0x1D, 0x08, 0x89, 0, 0], &[0, 0x22, 0x22, 0, 0x91]), // errata reg
        w(&[0x0D, 0x08, 0x89, 0x95]),           // set bit 2 (BW != 500)
        w(&[0x89, 0xFE]),                       // calibrate
        w(&[0x98, 0xD7, 0xDB]),                 // image calibration 863-870 MHz
    ]
    .concat()
}

#[test]
fn packet_status_decoding() {
    assert_eq!(
        PacketStatus::from_reply([0, 181, 0xF6, 0]),
        PacketStatus {
            rssi_dbm: -91,
            snr_qdb: -10
        }
    );
    assert_eq!(
        PacketStatus::from_reply([0, 180, 40, 0]),
        PacketStatus {
            rssi_dbm: -90,
            snr_qdb: 40
        }
    );
    assert_eq!(
        PacketStatus::from_reply([0, 0, 0, 0]),
        PacketStatus {
            rssi_dbm: 0,
            snr_qdb: 0
        }
    );
}

#[test]
fn irq_flag_helper() {
    let f = IrqFlags(IrqFlags::RX_DONE | IrqFlags::CRC_ERR);
    assert!(f.any(IrqFlags::CRC_ERR));
    assert!(!f.any(IrqFlags::TX_DONE | IrqFlags::TIMEOUT));
}

#[test]
fn init_sequence() {
    let (mut r, reset) = radio(&init_seq());
    r.init(&ChipConfig::LILYGO, &LoraConfig::WINCHY).unwrap();
    assert_eq!(
        *reset.0.borrow(),
        vec![false, true],
        "reset pulse low then high"
    );
    done(r);
}

#[test]
fn init_accepts_stale_command_failure_after_reset() {
    // Real rope board, 2026-11-22: status 0xAA = STDBY_RC with command status
    // 5 (failure) left over from before reset. Only the mode is meaningful.
    let mut t = init_seq();
    t.splice(0..3, rd(&[0x12, 0, 0, 0], &[0, 0xAA, 0, 0]));
    let (mut r, _) = radio(&t);
    r.init(&ChipConfig::LILYGO, &LoraConfig::WINCHY).unwrap();
    done(r);
}

#[test]
fn init_rejects_wrong_mode() {
    let (mut r, _) = radio(&rd(&[0x12, 0, 0, 0], &[0, 0x60, 0, 0])); // mode TX
    assert_eq!(
        r.init(&ChipConfig::LILYGO, &LoraConfig::WINCHY),
        Err(Error::BadStatus(0x60))
    );
    done(r);
}

#[test]
fn init_reports_device_errors() {
    let t = [
        rd(&[0x12, 0, 0, 0], &[0, 0x22, 0, 0]),
        w(&[0x9D, 0x01]),
        w(&[0x97, 0x01, 0x00, 0x01, 0x40]),
        w(&[0x07, 0x00, 0x00]),
        rd(&[0x17, 0, 0, 0], &[0, 0x22, 0x00, 0x20]), // XOSC start error
    ]
    .concat();
    let (mut r, _) = radio(&t);
    assert_eq!(
        r.init(&ChipConfig::LILYGO, &LoraConfig::WINCHY),
        Err(Error::Device(0x0020))
    );
    done(r);
}

#[test]
fn init_rejects_bad_config_before_touching_the_chip() {
    let (mut r, reset) = radio(&[]);
    let mut cfg = LoraConfig::WINCHY;
    cfg.sf = 13;
    assert_eq!(
        r.init(&ChipConfig::LILYGO, &cfg),
        Err(Error::Config(ConfigError::SpreadingFactor))
    );
    assert!(reset.0.borrow().is_empty());
    done(r);
}

#[test]
fn busy_timeout_instead_of_hanging() {
    let mut r = Sx126x::new(
        Mock::new(&[]),
        Busy { stuck: true },
        Reset::default(),
        NoopDelay::new(),
    );
    assert_eq!(r.irq_status(), Err(Error::BusyTimeout));
    r.release().0.done();
}

#[test]
fn transmit_sequence() {
    let t = [
        w(&[0x80, 0x01]),
        w(&[0x02, 0xFF, 0xFF]),
        w(&[0x8C, 0x00, 0x08, 0x00, 3, 0x01, 0x00]),
        w(&[0x8F, 0x00, 0xFF]),
        // One CS frame: opcode + offset, then the payload (no copy).
        vec![
            T::transaction_start(),
            T::write_vec(vec![0x0E, 0x00]),
            T::write_vec(vec![0xAA, 0xBB, 0xCC]),
            T::transaction_end(),
        ],
        w(&[0x83, 0x00, 0x00, 0x00]),
    ]
    .concat();
    let (mut r, _) = radio(&t);
    r.transmit(&[0xAA, 0xBB, 0xCC]).unwrap();
    done(r);
}

#[test]
fn transmit_rejects_oversized_payload() {
    let (mut r, _) = radio(&[]);
    assert_eq!(r.transmit(&[0u8; 256]), Err(Error::PayloadTooLong));
    done(r);
}

#[test]
fn receive_continuous_sequence() {
    let t = [
        w(&[0x80, 0x01]),
        w(&[0x02, 0xFF, 0xFF]),
        w(&[0x8F, 0xFF, 0x00]),
        w(&[0x8C, 0x00, 0x08, 0x00, 0xFF, 0x01, 0x00]),
        w(&[0x82, 0xFF, 0xFF, 0xFF]),
    ]
    .concat();
    let (mut r, _) = radio(&t);
    r.receive_continuous().unwrap();
    done(r);
}

#[test]
fn irq_status_and_clear() {
    let t = [
        rd(&[0x12, 0, 0, 0], &[0, 0x52, 0x00, 0x42]),
        w(&[0x02, 0x00, 0x42]),
    ]
    .concat();
    let (mut r, _) = radio(&t);
    let f = r.irq_status().unwrap();
    assert_eq!(f, IrqFlags(IrqFlags::RX_DONE | IrqFlags::CRC_ERR));
    r.clear_irq(f).unwrap();
    done(r);
}

#[test]
fn read_packet_sequence() {
    let t = [
        rd(&[0x13, 0, 0, 0], &[0, 0x52, 4, 0x10]), // len 4 at offset 0x10
        vec![
            T::transaction_start(),
            T::write_vec(vec![0x1E, 0x10, 0x00]),
            T::read_vec(vec![1, 2, 3, 4]),
            T::transaction_end(),
        ],
        rd(&[0x14, 0, 0, 0, 0], &[0, 0x52, 181, 0xF6, 180]),
    ]
    .concat();
    let (mut r, _) = radio(&t);
    let mut buf = [0u8; 255];
    let info = r.read_packet(&mut buf).unwrap();
    assert_eq!(
        info,
        RxInfo {
            len: 4,
            status: PacketStatus {
                rssi_dbm: -91,
                snr_qdb: -10
            }
        }
    );
    assert_eq!(&buf[..4], &[1, 2, 3, 4]);
    done(r);
}

#[test]
fn set_tx_power_uses_pa_table() {
    let t = [
        w(&[0x80, 0x01]),
        w(&[0x02, 0xFF, 0xFF]),
        w(&[0x95, 0x04, 0x07, 0x00, 0x01]),
        w(&[0x8E, 0xF7, 0x02]), // -9 dBm (two's complement)
    ]
    .concat();
    let (mut r, _) = radio(&t);
    assert_eq!(r.set_tx_power(-20), Ok(-9));
    done(r);
}
