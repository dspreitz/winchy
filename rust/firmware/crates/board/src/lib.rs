//! ESP-IDF peripheral bring-up for the LilyGo T-Beam S3 Supreme.
//!
//! Pin numbers come exclusively from `winchy-tbeam` (host-tested for
//! uniqueness); this crate only turns them into ESP-IDF drivers and applies
//! the PMU power profile before any sensor is touched.

use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::gpio::{AnyIOPin, Input, Output, PinDriver, Pull};
use esp_idf_svc::hal::i2c::{I2C0, I2C1, I2cConfig, I2cDriver};
use esp_idf_svc::hal::spi::config::{Config as SpiConfig, DriverConfig, MODE_0, MODE_3};
use esp_idf_svc::hal::spi::{SPI2, SPI3, SpiDeviceDriver, SpiDriver};
use esp_idf_svc::hal::uart::{UART1, UartDriver, config::Config as UartConfig};
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
    /// SX1262 radio bus and control pins.
    pub lora: LoraPins,
    /// GNSS receiver.
    pub gnss: GnssPins,
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

/// SX1262 SPI clock (same as the proven MicroPython setup).
const LORA_SPI_HZ: u32 = 2_000_000;

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
        spi3: SPI3<'static>,
        uart1: UART1<'static>,
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

        let lora_bus = SpiDriver::new(
            spi3,
            io(pins::lora::SCK),
            io(pins::lora::MOSI),
            Some(io(pins::lora::MISO)),
            &DriverConfig::new(),
        )?;
        let lora = LoraPins {
            spi: SpiDeviceDriver::new(
                lora_bus,
                Some(io(pins::lora::CS)),
                &SpiConfig::new()
                    .baudrate(Hertz(LORA_SPI_HZ))
                    .data_mode(MODE_0),
            )?,
            busy: PinDriver::input(io(pins::lora::BUSY), Pull::Floating)?,
            reset: PinDriver::output(io(pins::lora::RESET))?,
            dio1: PinDriver::input(io(pins::lora::DIO1), Pull::Floating)?,
        };

        let gnss = GnssPins {
            uart: UartDriver::new(
                uart1,
                io(pins::gnss::TX),
                io(pins::gnss::RX),
                Option::<AnyIOPin>::None,
                Option::<AnyIOPin>::None,
                &UartConfig::new()
                    .baudrate(Hertz(pins::gnss::BAUD_DEFAULT))
                    .rx_fifo_size(GNSS_RX_BUFFER),
            )?,
            pps: PinDriver::input(io(pins::gnss::PPS), Pull::Floating)?,
        };

        Ok(Self {
            pmu,
            sensor_i2c,
            imu_spi,
            ads,
            lora,
            gnss,
        })
    }
}
