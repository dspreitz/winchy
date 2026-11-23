//! QST QMI8658 6-axis IMU driver over `embedded-hal` 1.0 `SpiDevice`
//! (SPI mode 3; the device handles chip select).
//!
//! Register setup mirrors the field-proven MicroPython/C driver exactly:
//! accel +/-2 g @ 250 Hz, gyro +/-256 dps @ 250 Hz, low-pass on (~13 Hz).
//! One 12-byte burst reads accel + gyro.
//!
//! The 128-frame FIFO is supported too, but measured on the rope board
//! (2026-11-22) every FIFO read costs samples: while `CTRL_CMD_REQ_FIFO` is
//! processed (~1 ms) the QMI8658A drops ~0.2 samples per read on average
//! (215 instead of 224.2 Hz at one read per 20 ms, 222 Hz at one per 100 ms),
//! and nothing reports the loss. So the FIFO cannot provide gap-free,
//! evenly spaced samples; the firmware polls the data registers instead.
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
const REG_CTRL8: u8 = 0x09;
const REG_CTRL9: u8 = 0x0A;
const REG_FIFO_CTRL: u8 = 0x14;
const REG_FIFO_SMPL_CNT: u8 = 0x15;
const REG_FIFO_DATA: u8 = 0x17;
const REG_STATUSINT: u8 = 0x2D;
const REG_TEMP_L: u8 = 0x33;
const REG_AX_L: u8 = 0x35;

/// `CTRL7` value with accel + gyro enabled (bit 7: sync-sample mode).
const CTRL7_ENABLED: u8 = 0x83;
/// `CTRL7` for FIFO operation: accel + gyro, sync-sample OFF. In sync-sample
/// mode the QMI8658A executes `CTRL_CMD_REQ_FIFO` but never reports it done
/// (verified on the rope board 2026-11-22).
const CTRL7_FIFO: u8 = 0x03;
/// `CTRL8`: report `CTRL9` command completion in `STATUSINT` bit 7.
const CTRL8_HANDSHAKE_STATUSINT: u8 = 0x80;
/// `STATUSINT` bit: `CTRL9` command done.
const STATUSINT_CMD_DONE: u8 = 0x80;
/// `CTRL9` commands.
const CMD_ACK: u8 = 0x00;
const CMD_RST_FIFO: u8 = 0x04;
const CMD_REQ_FIFO: u8 = 0x05;
/// `FIFO_CTRL`: 128 samples, stream mode (oldest overwritten); bit 7 (read
/// mode) clear.
pub const FIFO_CTRL_STREAM_128: u8 = 0x0E;
/// `FIFO_STATUS` bits meaning samples may have been lost (full, overflow).
const FIFO_STATUS_LOSS: u8 = 0xA0;
/// `STATUSINT` polls before a `CTRL9` command counts as hung (a command takes
/// ~1 ms, one poll ~5-80 µs).
pub const CMD_POLLS: u32 = 2_000;
/// FIFO depth in accel + gyro frames.
pub const FIFO_FRAMES: usize = 128;
/// Bytes per accel + gyro frame.
const FRAME_BYTES: usize = 12;
/// Output data rate in 6-axis mode for ODR code 5 ("250 Hz"), Hz.
pub const ODR_HZ: f64 = 224.2;

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
    (REG_CTRL7, CTRL7_ENABLED), // accel + gyro enabled
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
    /// A `CTRL9` command never completed.
    Timeout,
}

/// Result of one FIFO read.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FifoBatch<T> {
    /// Frames written to the output, oldest first.
    pub frames: usize,
    /// The FIFO was full or overflowed: samples may be missing before this batch.
    pub overflow: bool,
    /// Value of the stamp closure, taken right after the sample count was latched.
    pub stamp: T,
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
            imu.write_reg(reg, val)?;
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

    /// Switches the FIFO to stream mode (128 frames, accel + gyro) and empties
    /// it; the sensors are off while it is configured.
    ///
    /// # Errors
    /// [`Error::Timeout`] if the reset command hangs, or bus error.
    pub fn enable_fifo(&mut self) -> Result<(), Error<S::Error>> {
        self.write_reg(REG_CTRL8, CTRL8_HANDSHAKE_STATUSINT)?;
        self.write_reg(REG_CTRL7, 0x00)?;
        self.write_reg(REG_FIFO_CTRL, FIFO_CTRL_STREAM_128)?;
        self.command(CMD_RST_FIFO)?;
        self.write_reg(REG_CTRL7, CTRL7_FIFO)
    }

    /// Reads every complete frame in the FIFO into `out` (oldest first).
    /// `stamp` runs right after the sample count is latched, so its value is
    /// an upper bound for the time of the newest frame.
    ///
    /// # Errors
    /// [`Error::Timeout`] if the read request hangs, or bus error.
    pub fn read_fifo<T>(
        &mut self,
        out: &mut [RawSample; FIFO_FRAMES],
        stamp: impl FnOnce() -> T,
    ) -> Result<FifoBatch<T>, Error<S::Error>> {
        self.command(CMD_REQ_FIFO)?;
        let [count_lsb, status] = self.read_reg::<2>(REG_FIFO_SMPL_CNT)?;
        let stamp = stamp();
        // The count is in 16-bit words; a frame is six of them.
        let words = usize::from(status & 0x03) << 8 | usize::from(count_lsb);
        let frames = (words / (FRAME_BYTES / 2)).min(FIFO_FRAMES);
        // One burst with CS held, as the vendor driver does (the HAL splits
        // long transfers itself).
        let mut bytes = [0u8; FIFO_FRAMES * FRAME_BYTES];
        let buf = &mut bytes[..frames * FRAME_BYTES];
        if frames > 0 {
            self.spi
                .transaction(&mut [
                    Operation::Write(&[REG_FIFO_DATA | 0x80]),
                    Operation::Read(buf),
                ])
                .map_err(Error::Spi)?;
        }
        for (sample, raw) in out.iter_mut().zip(buf.chunks_exact(FRAME_BYTES)) {
            let mut frame = [0u8; FRAME_BYTES];
            frame.copy_from_slice(raw);
            *sample = RawSample::from_burst(&frame);
        }
        self.write_reg(REG_FIFO_CTRL, FIFO_CTRL_STREAM_128)?; // leave read mode
        Ok(FifoBatch {
            frames,
            overflow: status & FIFO_STATUS_LOSS != 0,
            stamp,
        })
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
    fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Error<S::Error>> {
        self.spi.write(&[reg & 0x7F, val]).map_err(Error::Spi)
    }

    /// Runs a `CTRL9` command: issue, wait for done, acknowledge, wait for clear.
    fn command(&mut self, cmd: u8) -> Result<(), Error<S::Error>> {
        self.write_reg(REG_CTRL9, cmd)?;
        self.wait_cmd_done(true)?;
        self.write_reg(REG_CTRL9, CMD_ACK)?;
        self.wait_cmd_done(false)
    }

    fn wait_cmd_done(&mut self, done: bool) -> Result<(), Error<S::Error>> {
        for _ in 0..CMD_POLLS {
            let s = self.read_reg::<1>(REG_STATUSINT)?[0];
            if (s & STATUSINT_CMD_DONE != 0) == done {
                return Ok(());
            }
        }
        Err(Error::Timeout)
    }

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
