//! GPIO assignment (ESP32-S3 GPIO numbers).

/// Sensor I2C bus (BMP280, magnetometer, OLED).
pub mod i2c0 {
    /// SCL.
    pub const SCL: u8 = 18;
    /// SDA.
    pub const SDA: u8 = 17;
    /// Clock, Hz.
    pub const FREQ_HZ: u32 = 400_000;
}

/// PMU I2C bus (AXP2101).
pub mod pmu {
    /// SCL.
    pub const SCL: u8 = 41;
    /// SDA.
    pub const SDA: u8 = 42;
    /// AXP2101 IRQ (active low).
    pub const IRQ: u8 = 40;
    /// Clock, Hz.
    pub const FREQ_HZ: u32 = 100_000;
}

/// u-blox MAX-M10S GNSS.
pub mod gnss {
    /// ESP32 TX -> GNSS RX.
    pub const TX: u8 = 8;
    /// ESP32 RX <- GNSS TX.
    pub const RX: u8 = 9;
    /// 1PPS time pulse.
    pub const PPS: u8 = 6;
    /// Receiver cold-boot baud.
    pub const BAUD_DEFAULT: u32 = 9_600;
    /// Working baud after reconfiguration.
    pub const BAUD_HIGH: u32 = 115_200;
}

/// QMI8658 IMU on its own SPI bus (mode 3).
pub mod imu {
    /// SCK.
    pub const SCK: u8 = 36;
    /// MOSI.
    pub const MOSI: u8 = 35;
    /// MISO.
    pub const MISO: u8 = 37;
    /// Chip select.
    pub const CS: u8 = 34;
    /// Data-ready interrupt.
    pub const INT: u8 = 33;
    /// SPI clock, Hz.
    pub const FREQ_HZ: u32 = 8_000_000;
}

/// SX1262 LoRa radio.
pub mod lora {
    /// SCK.
    pub const SCK: u8 = 12;
    /// MOSI.
    pub const MOSI: u8 = 11;
    /// MISO.
    pub const MISO: u8 = 13;
    /// Chip select.
    pub const CS: u8 = 10;
    /// DIO1 interrupt.
    pub const DIO1: u8 = 1;
    /// Reset (active low).
    pub const RESET: u8 = 5;
    /// BUSY.
    pub const BUSY: u8 = 4;
}

/// ADS1232 force ADC (bit-banged).
pub mod ads1232 {
    /// Power-down (high = running).
    pub const PDWN: u8 = 39;
    /// Serial clock.
    pub const SCLK: u8 = 45;
    /// DRDY/DOUT.
    pub const DOUT: u8 = 46;
    /// GAIN0.
    pub const GAIN0: u8 = 38;
    /// GAIN1.
    pub const GAIN1: u8 = 2;
}

/// SH1106 OLED (hidden behind the ADS1232 breakout; disabled by default).
pub mod oled {
    /// Reset.
    pub const RESET: u8 = 16;
    /// I2C address.
    pub const ADDRESS: u8 = 0x3C;
}

/// Every GPIO used above, for the uniqueness test.
pub const ALL: [u8; 25] = [
    i2c0::SCL,
    i2c0::SDA,
    pmu::SCL,
    pmu::SDA,
    pmu::IRQ,
    gnss::TX,
    gnss::RX,
    gnss::PPS,
    imu::SCK,
    imu::MOSI,
    imu::MISO,
    imu::CS,
    imu::INT,
    lora::SCK,
    lora::MOSI,
    lora::MISO,
    lora::CS,
    lora::DIO1,
    lora::RESET,
    lora::BUSY,
    ads1232::PDWN,
    ads1232::SCLK,
    ads1232::DOUT,
    ads1232::GAIN0,
    ads1232::GAIN1,
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_gpio_is_assigned_twice() {
        let mut v = ALL.to_vec();
        v.sort_unstable();
        v.dedup();
        assert_eq!(v.len(), ALL.len());
    }

    #[test]
    fn avoids_esp32s3_reserved_pins() {
        // 19/20 = native USB (console), 26..=32 = flash/PSRAM.
        for p in ALL {
            assert!(
                !(19..=20).contains(&p) && !(26..=32).contains(&p),
                "GPIO{p}"
            );
            assert!(p <= 48, "GPIO{p}");
        }
    }
}
