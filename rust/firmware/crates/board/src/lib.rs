//! ESP-IDF peripheral bring-up for the LilyGo T-Beam S3 Supreme.
//!
//! Pin numbers come exclusively from `winchy-tbeam` (host-tested for
//! uniqueness); this crate only turns them into ESP-IDF drivers and applies
//! the PMU power profile before any sensor is touched.

use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::gpio::{AnyIOPin, Input, Output, PinDriver, Pull};
use esp_idf_svc::hal::i2c::{I2C0, I2C1, I2cConfig, I2cDriver};
use esp_idf_svc::hal::spi::config::{Config as SpiConfig, DriverConfig, MODE_3};
use esp_idf_svc::hal::spi::{SPI2, SpiDeviceDriver, SpiDriver};
use esp_idf_svc::hal::units::Hertz;
use esp_idf_svc::sys::EspError;
use winchy_axp2101::Axp2101;
use winchy_tbeam::pins;

/// Rails need a moment after switching before sensors answer.
const RAIL_SETTLE_MS: u32 = 50;

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

/// ADS1232 pin set as ESP-IDF pin drivers.
pub type AdsPins = winchy_ads1232::Pins<
    PinDriver<'static, Input>,
    PinDriver<'static, Output>,
    PinDriver<'static, Output>,
    PinDriver<'static, Output>,
    PinDriver<'static, Output>,
>;

/// Initialised rope-unit peripherals.
pub struct RopeBoard {
    /// Configured PMU.
    pub pmu: Axp2101<I2cDriver<'static>>,
    /// Sensor I2C bus (BMP280, magnetometer).
    pub sensor_i2c: I2cDriver<'static>,
    /// IMU SPI device (mode 3, CS handled by the driver).
    pub imu_spi: SpiDeviceDriver<'static, SpiDriver<'static>>,
    /// Force ADC pins.
    pub ads: AdsPins,
}

/// One GPIO by number.
fn io(n: u8) -> AnyIOPin<'static> {
    // SAFETY: every number comes from `winchy_tbeam::pins`, whose test
    // guarantees no GPIO is assigned twice, and `RopeBoard::init` is the only
    // place that creates pins; the typed `Peripherals::pins` are never used.
    unsafe { AnyIOPin::steal(n) }
}

impl RopeBoard {
    /// Powers the board (PMU profile) and creates the sensor buses.
    ///
    /// # Errors
    /// [`BoardError`] if a driver cannot be created or the PMU is missing.
    pub fn init(
        i2c0: I2C0<'static>,
        i2c1: I2C1<'static>,
        spi2: SPI2<'static>,
    ) -> Result<Self, BoardError> {
        let pmu_bus = I2cDriver::new(
            i2c1,
            io(pins::pmu::SDA),
            io(pins::pmu::SCL),
            &I2cConfig::new().baudrate(Hertz(pins::pmu::FREQ_HZ)),
        )?;
        let mut pmu = Axp2101::new(pmu_bus);
        winchy_tbeam::apply_power_profile(&mut pmu).map_err(BoardError::Pmu)?;
        FreeRtos::delay_ms(RAIL_SETTLE_MS);
        log::info!("board: PMU profile applied");

        let sensor_i2c = I2cDriver::new(
            i2c0,
            io(pins::i2c0::SDA),
            io(pins::i2c0::SCL),
            &I2cConfig::new().baudrate(Hertz(pins::i2c0::FREQ_HZ)),
        )?;

        let imu_bus = SpiDriver::new(
            spi2,
            io(pins::imu::SCK),
            io(pins::imu::MOSI),
            Some(io(pins::imu::MISO)),
            &DriverConfig::new(),
        )?;
        let imu_spi = SpiDeviceDriver::new(
            imu_bus,
            Some(io(pins::imu::CS)),
            &SpiConfig::new()
                .baudrate(Hertz(pins::imu::FREQ_HZ))
                .data_mode(MODE_3),
        )?;

        let ads = winchy_ads1232::Pins {
            dout: PinDriver::input(io(pins::ads1232::DOUT), Pull::Floating)?,
            sclk: PinDriver::output(io(pins::ads1232::SCLK))?,
            pdwn: PinDriver::output(io(pins::ads1232::PDWN))?,
            gain0: PinDriver::output(io(pins::ads1232::GAIN0))?,
            gain1: PinDriver::output(io(pins::ads1232::GAIN1))?,
        };

        Ok(Self {
            pmu,
            sensor_i2c,
            imu_spi,
            ads,
        })
    }
}
