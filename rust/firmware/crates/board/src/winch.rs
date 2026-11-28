//! LilyGo T3S3 V1.2 (winch unit).

use esp_idf_svc::hal::gpio::{AnyIOPin, PinDriver, Pull};
use esp_idf_svc::hal::i2c::{I2C0, I2cConfig, I2cDriver};
use esp_idf_svc::hal::spi::config::{Config as SpiConfig, DriverConfig, MODE_0};
use esp_idf_svc::hal::spi::{SPI2, SpiDeviceDriver, SpiDriver};
use esp_idf_svc::hal::uart::{UART1, UartDriver, config::Config as UartConfig};
use esp_idf_svc::hal::units::Hertz;
use winchy_t3s3::pins;

use crate::{BoardError, GNSS_RX_BUFFER, GnssPins, LoraPins, io};

/// Initialised winch-unit peripherals.
pub struct WinchBoard {
    /// SX1262 radio bus and control pins.
    pub lora: LoraPins,
    /// OLED I2C bus.
    pub oled: I2cDriver<'static>,
    /// GNSS receiver (NMEA at its fixed baud) and 1PPS input.
    pub gnss: GnssPins,
}

impl WinchBoard {
    /// Creates the radio, display and GNSS drivers.
    ///
    /// # Errors
    /// [`BoardError`] if a driver cannot be created.
    pub fn init(
        i2c0: I2C0<'static>,
        spi2: SPI2<'static>,
        uart1: UART1<'static>,
    ) -> Result<Self, BoardError> {
        let lora_bus = SpiDriver::new(
            spi2,
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
                    .baudrate(Hertz(pins::lora::FREQ_HZ))
                    .data_mode(MODE_0),
            )?,
            busy: PinDriver::input(io(pins::lora::BUSY), Pull::Floating)?,
            reset: PinDriver::output(io(pins::lora::RESET))?,
            dio1: PinDriver::input(io(pins::lora::DIO1), Pull::Floating)?,
        };
        let oled = I2cDriver::new(
            i2c0,
            io(pins::oled::SDA),
            io(pins::oled::SCL),
            &I2cConfig::new().baudrate(Hertz(pins::oled::FREQ_HZ)),
        )?;
        let gnss = GnssPins {
            uart: UartDriver::new(
                uart1,
                io(pins::gnss::TX),
                io(pins::gnss::RX),
                Option::<AnyIOPin>::None,
                Option::<AnyIOPin>::None,
                &UartConfig::new()
                    .baudrate(Hertz(pins::gnss::BAUD))
                    .rx_fifo_size(GNSS_RX_BUFFER),
            )?,
            pps: PinDriver::input(io(pins::gnss::PPS), Pull::Floating)?,
        };
        Ok(Self { lora, oled, gnss })
    }
}
