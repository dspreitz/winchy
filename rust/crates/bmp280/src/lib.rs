//! Bosch BMP280 (and pressure/temperature-compatible BME280) barometer driver
//! over `embedded-hal` 1.0 I2C. Uses the datasheet's 64-bit integer
//! compensation, so results are bit-exact and platform independent.
#![cfg_attr(not(test), no_std)]

use embedded_hal::i2c::I2c;

/// Default I2C address (SDO high).
pub const ADDRESS: u8 = 0x77;
/// BMP280 chip id.
pub const CHIP_ID_BMP280: u8 = 0x58;
/// BME280 chip id (same pressure/temperature interface).
pub const CHIP_ID_BME280: u8 = 0x60;

const REG_CALIB: u8 = 0x88;
const REG_ID: u8 = 0xD0;
const REG_STATUS: u8 = 0xF3;
const REG_CTRL_MEAS: u8 = 0xF4;
const REG_CONFIG: u8 = 0xF5;
const REG_DATA: u8 = 0xF7;

/// Factory trimming parameters (registers 0x88..0x9F).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Calibration {
    /// `dig_T1..dig_T3`.
    pub t: (u16, i16, i16),
    /// `dig_P1..dig_P9`.
    pub p: (u16, i16, i16, i16, i16, i16, i16, i16, i16),
}

/// One compensated measurement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Temperature, 0.01 degC.
    pub temperature_cdeg: i32,
    /// Pressure, Pa in Q24.8 fixed point (value / 256 = Pa); 0 if invalid.
    pub pressure_q24_8: u32,
}

impl Measurement {
    /// Pressure in hPa.
    #[must_use]
    pub fn pressure_hpa(&self) -> f32 {
        self.pressure_q24_8 as f32 / 25_600.0
    }

    /// True if the pressure is inside the BMP280 operating range (300..1100 hPa).
    /// The sensor occasionally returns 0, which would become a ~44 km altitude spike.
    #[must_use]
    pub fn is_plausible(&self) -> bool {
        (300.0..=1100.0).contains(&self.pressure_hpa())
    }
}

impl Calibration {
    /// Parses the 24 calibration bytes.
    #[must_use]
    pub fn from_bytes(b: &[u8; 24]) -> Self {
        let u = |i: usize| u16::from_le_bytes([b[i], b[i + 1]]);
        let s = |i: usize| i16::from_le_bytes([b[i], b[i + 1]]);
        Self {
            t: (u(0), s(2), s(4)),
            p: (u(6), s(8), s(10), s(12), s(14), s(16), s(18), s(20), s(22)),
        }
    }

    /// Datasheet integer compensation of raw 20-bit ADC values.
    #[must_use]
    pub fn compensate(&self, adc_t: i32, adc_p: i32) -> Measurement {
        let (t1, t2, t3) = (
            i32::from(self.t.0),
            i32::from(self.t.1),
            i32::from(self.t.2),
        );
        let v1 = (((adc_t >> 3) - (t1 << 1)) * t2) >> 11;
        let v2 = (((((adc_t >> 4) - t1) * ((adc_t >> 4) - t1)) >> 12) * t3) >> 14;
        let t_fine = v1 + v2;
        let temperature_cdeg = (t_fine * 5 + 128) >> 8;

        let (p1, p2, p3, p4, p5, p6, p7, p8, p9) = self.p;
        let mut v1 = i64::from(t_fine) - 128_000;
        let mut v2 = v1 * v1 * i64::from(p6);
        v2 += (v1 * i64::from(p5)) << 17;
        v2 += i64::from(p4) << 35;
        v1 = ((v1 * v1 * i64::from(p3)) >> 8) + ((v1 * i64::from(p2)) << 12);
        v1 = (((1_i64 << 47) + v1) * i64::from(p1)) >> 33;
        if v1 == 0 {
            return Measurement {
                temperature_cdeg,
                pressure_q24_8: 0,
            };
        }
        let mut p = 1_048_576 - i64::from(adc_p);
        p = (((p << 31) - v2) * 3125) / v1;
        let v1 = (i64::from(p9) * (p >> 13) * (p >> 13)) >> 25;
        let v2 = (i64::from(p8) * p) >> 19;
        p = ((p + v1 + v2) >> 8) + (i64::from(p7) << 4);
        Measurement {
            temperature_cdeg,
            pressure_q24_8: p.clamp(0, i64::from(u32::MAX)) as u32,
        }
    }
}

/// Oversampling setting (register code).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Oversampling {
    /// Measurement skipped.
    Skip = 0,
    /// x1.
    X1 = 1,
    /// x2.
    X2 = 2,
    /// x4.
    X4 = 3,
    /// x8.
    X8 = 4,
    /// x16.
    X16 = 5,
}

/// Measurement configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Temperature oversampling.
    pub temperature: Oversampling,
    /// Pressure oversampling.
    pub pressure: Oversampling,
    /// IIR filter code (0 = off, 1 = 2, 2 = 4, 3 = 8, 4 = 16).
    pub iir: u8,
    /// Standby code for normal mode (3 = 250 ms).
    pub standby: u8,
}

impl Default for Config {
    /// Winchy's setting: T x8, P x16, IIR 2, standby 250 ms.
    fn default() -> Self {
        Self {
            temperature: Oversampling::X8,
            pressure: Oversampling::X16,
            iir: 1,
            standby: 3,
        }
    }
}

/// Driver error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    /// Bus error.
    I2c(E),
    /// Unexpected chip id.
    NotFound(u8),
}

/// BMP280 driver (forced-mode use).
#[derive(Debug)]
pub struct Bmp280<I> {
    i2c: I,
    address: u8,
    calibration: Calibration,
    ctrl: u8,
}

impl<I: I2c> Bmp280<I> {
    /// Checks the chip id, reads the calibration and applies `config` (sleep mode).
    ///
    /// # Errors
    /// [`Error::NotFound`] for an unknown chip id, or bus error.
    pub fn new(i2c: I, address: u8, config: Config) -> Result<Self, Error<I::Error>> {
        let mut dev = Self {
            i2c,
            address,
            calibration: Calibration::from_bytes(&[0; 24]),
            ctrl: ((config.temperature as u8) << 5) | ((config.pressure as u8) << 2),
        };
        let mut id = [0u8];
        dev.i2c
            .write_read(address, &[REG_ID], &mut id)
            .map_err(Error::I2c)?;
        if id[0] != CHIP_ID_BMP280 && id[0] != CHIP_ID_BME280 {
            return Err(Error::NotFound(id[0]));
        }
        let mut cal = [0u8; 24];
        dev.i2c
            .write_read(address, &[REG_CALIB], &mut cal)
            .map_err(Error::I2c)?;
        dev.calibration = Calibration::from_bytes(&cal);
        let cfg = ((config.standby & 0x07) << 5) | ((config.iir & 0x07) << 2);
        dev.i2c
            .write(address, &[REG_CONFIG, cfg])
            .map_err(Error::I2c)?;
        dev.i2c
            .write(address, &[REG_CTRL_MEAS, dev.ctrl])
            .map_err(Error::I2c)?; // sleep
        Ok(dev)
    }

    /// Calibration read at construction.
    pub fn calibration(&self) -> Calibration {
        self.calibration
    }

    /// Starts one forced-mode conversion.
    ///
    /// # Errors
    /// Bus error.
    pub fn start_forced(&mut self) -> Result<(), Error<I::Error>> {
        self.i2c
            .write(self.address, &[REG_CTRL_MEAS, self.ctrl | 0x01])
            .map_err(Error::I2c)
    }

    /// True while a conversion is running.
    ///
    /// # Errors
    /// Bus error.
    pub fn is_measuring(&mut self) -> Result<bool, Error<I::Error>> {
        let mut st = [0u8];
        self.i2c
            .write_read(self.address, &[REG_STATUS], &mut st)
            .map_err(Error::I2c)?;
        Ok(st[0] & 0x08 != 0)
    }

    /// Reads and compensates the latest result.
    ///
    /// # Errors
    /// Bus error.
    pub fn read(&mut self) -> Result<Measurement, Error<I::Error>> {
        let mut d = [0u8; 6];
        self.i2c
            .write_read(self.address, &[REG_DATA], &mut d)
            .map_err(Error::I2c)?;
        let raw = |i: usize| {
            (i32::from(d[i]) << 12) | (i32::from(d[i + 1]) << 4) | (i32::from(d[i + 2]) >> 4)
        };
        Ok(self.calibration.compensate(raw(3), raw(0)))
    }

    /// Releases the bus.
    pub fn release(self) -> I {
        self.i2c
    }
}

#[cfg(test)]
mod tests;
