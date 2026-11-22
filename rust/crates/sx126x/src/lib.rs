//! Semtech SX1262 LoRa radio driver (`embedded-hal` 1.0, blocking).
//!
//! The sequence and values mirror the field-proven micropython-lib driver
//! used by the MicroPython firmware (TCXO on DIO3, RF switch on DIO2,
//! calibrate 0xFE + image calibration after configuration, errata 0x0889).
//!
//! Interrupt handling is the caller's job: DIO1 rises on TX done, RX done or
//! timeout; the firmware turns that edge into a task notification and then
//! calls [`Sx126x::irq_status`]. The driver itself never spins except on the
//! chip's BUSY line, which is bounded by a timeout.
#![cfg_attr(not(test), no_std)]

pub mod config;
mod driver;

pub use config::{Bandwidth, ChipConfig, CodingRate, ConfigError, LoraConfig, Tcxo};
pub use driver::{Error, IrqFlags, PacketStatus, RxInfo, Sx126x};
