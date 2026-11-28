//! ESP-IDF peripheral bring-up for the Winchy boards: the rope's LilyGo
//! T-Beam S3 Supreme ([`RopeBoard`]) and the winch's LilyGo T3S3
//! ([`WinchBoard`]).
//!
//! Pin numbers come exclusively from the host-tested board crates
//! (`winchy-tbeam`, `winchy-t3s3`); this crate only turns them into ESP-IDF
//! drivers (and, on the rope, applies the PMU power profile first).

mod rope;
mod winch;

pub use rope::{AdsPins, RopeBoard};
pub use winch::WinchBoard;

use esp_idf_svc::hal::gpio::{AnyIOPin, Input, Output, PinDriver};
use esp_idf_svc::hal::spi::{SpiDeviceDriver, SpiDriver};
use esp_idf_svc::hal::uart::UartDriver;
use esp_idf_svc::sys::EspError;

/// Board bring-up error.
#[derive(Debug)]
pub enum BoardError {
    /// ESP-IDF driver error.
    Esp(EspError),
    /// PMU error (chip missing or bus failure).
    Pmu(winchy_axp2101::Error<esp_idf_svc::hal::i2c::I2cError>),
}

impl From<EspError> for BoardError {
    fn from(e: EspError) -> Self {
        Self::Esp(e)
    }
}

/// GNSS receiver UART and 1PPS input.
pub struct GnssPins {
    /// UART1 at the receiver's cold-boot baud (re-tune with `change_baudrate`).
    pub uart: UartDriver<'static>,
    /// 1PPS time-pulse input.
    pub pps: PinDriver<'static, Input>,
}

/// UART RX buffer: >1 s of NAV-PVT at 10 Hz.
const GNSS_RX_BUFFER: usize = 2048;

/// SX1262 SPI device and control pins.
pub struct LoraPins {
    /// SPI device (mode 0, CS handled by the driver).
    pub spi: SpiDeviceDriver<'static, SpiDriver<'static>>,
    /// BUSY input.
    pub busy: PinDriver<'static, Input>,
    /// RESET output (active low).
    pub reset: PinDriver<'static, Output>,
    /// DIO1 interrupt input.
    pub dio1: PinDriver<'static, Input>,
}

/// One GPIO by number.
fn io(n: u8) -> AnyIOPin<'static> {
    // SAFETY: every number comes from one board crate's `pins` module, whose
    // test guarantees no GPIO is assigned twice; a firmware image initialises
    // exactly one board, and `*Board::init` is the only place creating pins
    // (the typed `Peripherals::pins` are never used).
    unsafe { AnyIOPin::steal(n) }
}
