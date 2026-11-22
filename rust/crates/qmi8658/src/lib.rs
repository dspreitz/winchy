//! QST QMI8658 6-axis IMU driver over `embedded-hal` 1.0 `SpiDevice`
//! (SPI mode 3; the device handles chip select).
//!
//! Register setup mirrors the field-proven MicroPython/C driver exactly:
//! accel +/-2 g @ 250 Hz, gyro +/-256 dps @ 250 Hz, low-pass on (~13 Hz).
//! One 12-byte burst reads accel + gyro.
//!
//! `CTRL1` = 0x60 nominally sets the big-endian bit, yet the data decode as
//! little-endian: verified on the rope board 2026-11-22 (|a| = 0.973 g at rest,
//! gyro bias ~2 dps; swapped bytes would give nonsense magnitudes).
#![cfg_attr(not(test), no_std)]

use embedded_hal::delay::DelayNs;
use embedded_hal::spi::{Operation, SpiDevice};

const REG_WHO_AM_I: u8 = 0x00;
const REG_CTRL1: u8 = 0x02;
const REG_CTRL2: u8 = 0x03;
const REG_CTRL3: u8 = 0x04;
const REG_CTRL5: u8 = 0x06;
const REG_CTRL7: u8 = 0x08;
const REG_TEMP_L: u8 = 0x33;
const REG_AX_L: u8 = 0x35;

/// `WHO_AM_I` value.
pub const WHO_AM_I: u8 = 0x05;
/// Accelerometer LSB per g at +/-2 g.
pub const ACCEL_LSB_PER_G: f32 = 16_384.0;
/// Gyroscope LSB per dps at +/-256 dps.
pub const GYRO_LSB_PER_DPS: f32 = 128.0;
/// `WHO_AM_I` attempts at start-up (the IMU reads 0x00 for a few ms after power-up).
pub const PROBE_ATTEMPTS: u32 = 20;

/// Register writes applied at init, in order.
pub const INIT_SEQUENCE: [(u8, u8); 5] = [
    (REG_CTRL1, 0x60), // serial interface: address auto-increment (+ BE bit, see crate doc)
    (REG_CTRL2, 0x05), // accel +/-2 g, 250 Hz
    (REG_CTRL3, 0x45), // gyro +/-256 dps, 250 Hz
    (REG_CTRL7, 0x83), // accel + gyro enabled
    (REG_CTRL5, 0x55), // low-pass filters on, 5.32 % of ODR
];

/// One raw accel + gyro sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RawSample {
    /// Accelerometer counts (x, y, z).
    pub accel: [i16; 3],
    /// Gyroscope counts (x, y, z).
    pub gyro: [i16; 3],
}

impl RawSample {
    /// Decodes the 12-byte burst starting at `AX_L` (little-endian pairs).
    #[must_use]
    pub fn from_burst(b: &[u8; 12]) -> Self {
        let v = |i: usize| i16::from_le_bytes([b[2 * i], b[2 * i + 1]]);
        Self {
            accel: [v(0), v(1), v(2)],
            gyro: [v(3), v(4), v(5)],
        }
    }

    /// Specific force in g.
    #[must_use]
    pub fn accel_g(&self) -> [f32; 3] {
        self.accel.map(|c| f32::from(c) / ACCEL_LSB_PER_G)
    }

    /// Angular rate in dps.
    #[must_use]
    pub fn gyro_dps(&self) -> [f32; 3] {
        self.gyro.map(|c| f32::from(c) / GYRO_LSB_PER_DPS)
    }
}

/// Driver error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    /// Bus error.
    Spi(E),
    /// `WHO_AM_I` never matched (last value read).
    NotFound(u8),
}

/// QMI8658 driver.
#[derive(Debug)]
pub struct Qmi8658<S> {
    spi: S,
}

impl<S: SpiDevice> Qmi8658<S> {
    /// Probes `WHO_AM_I` (up to [`PROBE_ATTEMPTS`], 10 ms apart) and applies
    /// [`INIT_SEQUENCE`].
    ///
    /// # Errors
    /// [`Error::NotFound`] if the id never matches, or bus error.
    pub fn new(spi: S, delay: &mut impl DelayNs) -> Result<Self, Error<S::Error>> {
        let mut imu = Self { spi };
        let mut who = 0;
        for attempt in 0..PROBE_ATTEMPTS {
            if attempt > 0 {
                delay.delay_ms(10);
            }
            who = imu.read_reg::<1>(REG_WHO_AM_I)?[0];
            if who == WHO_AM_I {
                break;
            }
        }
        if who != WHO_AM_I {
            return Err(Error::NotFound(who));
        }
        for (reg, val) in INIT_SEQUENCE {
            imu.spi.write(&[reg & 0x7F, val]).map_err(Error::Spi)?;
        }
        Ok(imu)
    }

    /// Reads one accel + gyro sample in a single burst.
    ///
    /// # Errors
    /// Bus error.
    pub fn read_raw(&mut self) -> Result<RawSample, Error<S::Error>> {
        Ok(RawSample::from_burst(&self.read_reg::<12>(REG_AX_L)?))
    }

    /// Die temperature in degC.
    ///
    /// # Errors
    /// Bus error.
    pub fn temperature_c(&mut self) -> Result<f32, Error<S::Error>> {
        let b = self.read_reg::<2>(REG_TEMP_L)?;
        Ok(f32::from(i16::from_le_bytes(b)) / 256.0)
    }

    /// Releases the bus.
    pub fn release(self) -> S {
        self.spi
    }
}

impl<S: SpiDevice> Qmi8658<S> {
    fn read_reg<const N: usize>(&mut self, reg: u8) -> Result<[u8; N], Error<S::Error>> {
        let mut buf = [0u8; N];
        self.spi
            .transaction(&mut [Operation::Write(&[reg | 0x80]), Operation::Read(&mut buf)])
            .map_err(Error::Spi)?;
        Ok(buf)
    }
}

#[cfg(test)]
mod tests;
