//! GPIO assignment (ESP32-S3 GPIO numbers).

/// SX1262 LoRa radio on SPI (mode 0).
pub mod lora {
    /// SCK.
    pub const SCK: u8 = 5;
    /// MOSI.
    pub const MOSI: u8 = 6;
    /// MISO.
    pub const MISO: u8 = 3;
    /// Chip select.
    pub const CS: u8 = 7;
    /// BUSY.
    pub const BUSY: u8 = 34;
    /// DIO1 interrupt.
    pub const DIO1: u8 = 33;
    /// RESET (active low).
    pub const RESET: u8 = 8;
    /// SPI clock, Hz (as the MicroPython winch).
    pub const FREQ_HZ: u32 = 2_000_000;
}

/// SSD1306 128x64 OLED on I2C.
pub mod oled {
    /// SDA.
    pub const SDA: u8 = 18;
    /// SCL.
    pub const SCL: u8 = 17;
    /// 7-bit I2C address.
    pub const ADDRESS: u8 = 0x3C;
    /// Clock, Hz.
    pub const FREQ_HZ: u32 = 400_000;
}

/// GNSS receiver (interim u-blox 7, NMEA) on UART1.
pub mod gnss {
    /// ESP32 RX <- GNSS TX.
    pub const RX: u8 = 42;
    /// ESP32 TX -> GNSS RX.
    pub const TX: u8 = 41;
    /// 1PPS time pulse.
    pub const PPS: u8 = 40;
    /// Receiver baud.
    pub const BAUD: u32 = 9_600;
}

/// Every GPIO used above, for the uniqueness test.
pub const ALL: [u8; 12] = [
    lora::SCK,
    lora::MOSI,
    lora::MISO,
    lora::CS,
    lora::BUSY,
    lora::DIO1,
    lora::RESET,
    oled::SDA,
    oled::SCL,
    gnss::RX,
    gnss::TX,
    gnss::PPS,
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_the_proven_micropython_setup() {
        assert_eq!(
            [
                lora::SCK,
                lora::MOSI,
                lora::MISO,
                lora::CS,
                lora::BUSY,
                lora::DIO1,
                lora::RESET
            ],
            [5, 6, 3, 7, 34, 33, 8]
        );
        assert_eq!([oled::SDA, oled::SCL], [18, 17]);
        assert_eq!(oled::ADDRESS, 0x3C);
        assert_eq!([gnss::RX, gnss::TX, gnss::PPS], [42, 41, 40]);
        assert_eq!(gnss::BAUD, 9_600);
    }

    #[test]
    fn no_gpio_is_assigned_twice() {
        let mut v = ALL.to_vec();
        v.sort_unstable();
        v.dedup();
        assert_eq!(v.len(), ALL.len());
    }

    #[test]
    fn avoids_esp32s3_reserved_pins() {
        // 19/20 = native USB (console), 26..=32 = flash/PSRAM (quad PSRAM on
        // the T3S3's ESP32-S3FH4R2, so 33..=37 stay usable).
        for p in ALL {
            assert!(
                !(19..=20).contains(&p) && !(26..=32).contains(&p),
                "GPIO {p}"
            );
        }
    }
}
