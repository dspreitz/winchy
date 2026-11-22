//! Radio configuration and its pure encodings.

/// LoRa bandwidth.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Bandwidth {
    /// 125 kHz.
    Khz125,
    /// 250 kHz.
    Khz250,
    /// 500 kHz.
    Khz500,
}

impl Bandwidth {
    /// Register code.
    #[must_use]
    pub const fn code(self) -> u8 {
        match self {
            Self::Khz125 => 0x04,
            Self::Khz250 => 0x05,
            Self::Khz500 => 0x06,
        }
    }

    /// Bandwidth in Hz.
    #[must_use]
    pub const fn hz(self) -> u32 {
        match self {
            Self::Khz125 => 125_000,
            Self::Khz250 => 250_000,
            Self::Khz500 => 500_000,
        }
    }
}

/// LoRa coding rate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CodingRate {
    /// 4/5.
    Cr45 = 1,
    /// 4/6.
    Cr46 = 2,
    /// 4/7.
    Cr47 = 3,
    /// 4/8.
    Cr48 = 4,
}

/// TCXO powered from DIO3.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Tcxo {
    /// Supply voltage, 1600..=3300 mV.
    pub millivolts: u16,
    /// Start-up time, us.
    pub start_us: u32,
}

/// Board wiring of the radio.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChipConfig {
    /// DIO3-powered TCXO, if fitted.
    pub tcxo: Option<Tcxo>,
    /// DIO2 drives the RF switch.
    pub dio2_rf_switch: bool,
}

impl ChipConfig {
    /// LilyGo T-Beam S3 Supreme and T3S3: 1.7 V TCXO (5 ms), DIO2 RF switch.
    pub const LILYGO: Self = Self {
        tcxo: Some(Tcxo {
            millivolts: 1700,
            start_us: 5000,
        }),
        dio2_rf_switch: true,
    };
}

/// LoRa modem settings.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LoraConfig {
    /// Carrier, Hz.
    pub freq_hz: u32,
    /// Spreading factor 5..=12.
    pub sf: u8,
    /// Bandwidth.
    pub bw: Bandwidth,
    /// Coding rate.
    pub cr: CodingRate,
    /// Preamble length, symbols.
    pub preamble_len: u16,
    /// TX power, dBm (clamped to -9..=22).
    pub tx_power_dbm: i8,
    /// One-byte LoRa sync word (0x12 = private network).
    pub sync_word: u8,
    /// Payload CRC on.
    pub crc: bool,
}

impl LoraConfig {
    /// Winchy link: 869.525 MHz, SF7, 250 kHz, 4/8, preamble 8, +14 dBm, sync 0x12, CRC.
    pub const WINCHY: Self = Self {
        freq_hz: 869_525_000,
        sf: 7,
        bw: Bandwidth::Khz250,
        cr: CodingRate::Cr48,
        preamble_len: 8,
        tx_power_dbm: 14,
        sync_word: 0x12,
        crc: true,
    };
}

/// Invalid configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigError {
    /// Spreading factor outside 5..=12.
    SpreadingFactor,
    /// No image calibration band for this frequency.
    Frequency,
    /// TCXO voltage or start time out of range.
    Tcxo,
}

/// `SetRfFrequency` argument: `freq * 2^25 / 32 MHz`.
#[must_use]
pub fn rf_freq_code(freq_hz: u32) -> u32 {
    ((u64::from(freq_hz) << 25) / 32_000_000) as u32
}

/// Sync-word register value (0x0740..0x0741) for a one-byte sync word.
#[must_use]
pub fn sync_word_reg(sync: u8) -> u16 {
    0x0404 | (u16::from(sync & 0x0F) << 4) | (u16::from(sync & 0xF0) << 8)
}

/// `SetDIO3AsTcxoCtrl` arguments: trim code (voltage rounded down to the
/// next supported step) + 24-bit start timeout in 15.625 us units (rounded up).
///
/// # Errors
/// [`ConfigError::Tcxo`] if out of range.
pub fn tcxo_args(t: Tcxo) -> Result<[u8; 4], ConfigError> {
    const STEPS_DV: [u16; 8] = [16, 17, 18, 22, 24, 27, 30, 33]; // DS table 13-35
    if !(1600..=3300).contains(&t.millivolts) {
        return Err(ConfigError::Tcxo);
    }
    let dv = t.millivolts / 100;
    let trim = STEPS_DV
        .iter()
        .rposition(|&s| s <= dv)
        .ok_or(ConfigError::Tcxo)? as u8;
    let ticks = (u64::from(t.start_us) * 1000).div_ceil(15_625);
    if ticks >= 1 << 24 {
        return Err(ConfigError::Tcxo);
    }
    let [_, a, b, c] = (ticks as u32).to_be_bytes();
    Ok([trim, a, b, c])
}

/// `SetPaConfig` arguments and the `SetTxParams` power for a requested power.
/// Uses the datasheet's optimal PA settings at 14/17/20/22 dBm.
#[must_use]
pub fn pa_params(dbm: i8) -> ([u8; 4], i8) {
    const DEFAULT: [u8; 4] = [0x04, 0x07, 0x00, 0x01];
    match dbm.clamp(-9, 22) {
        22 => (DEFAULT, 22),
        20 => ([0x03, 0x05, 0x00, 0x01], 22),
        17 => ([0x02, 0x03, 0x00, 0x01], 22),
        14 => ([0x02, 0x02, 0x00, 0x01], 22),
        other => (DEFAULT, other),
    }
}

/// Low-data-rate optimisation needed (symbol time >= 16 ms).
#[must_use]
pub fn ldro(sf: u8, bw: Bandwidth) -> bool {
    // Symbol time 2^sf / bw >= 16 ms  <=>  2^sf * 1000 >= 16 * bw.
    (1_u64 << sf.min(12)) * 1000 >= 16 * u64::from(bw.hz())
}

/// `CalibrateImage` arguments for the band containing `freq_hz`.
///
/// # Errors
/// [`ConfigError::Frequency`] outside the supported bands.
pub fn image_cal_args(freq_hz: u32) -> Result<[u8; 2], ConfigError> {
    Ok(match freq_hz / 1_000_000 {
        430..=440 => [0x6B, 0x6F],
        470..=510 => [0x75, 0x81],
        779..=787 => [0xC1, 0xC5],
        863..=870 => [0xD7, 0xDB],
        902..=928 => [0xE1, 0xE9],
        _ => return Err(ConfigError::Frequency),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bandwidth_codes() {
        assert_eq!(
            (Bandwidth::Khz125.code(), Bandwidth::Khz125.hz()),
            (0x04, 125_000)
        );
        assert_eq!(
            (Bandwidth::Khz250.code(), Bandwidth::Khz250.hz()),
            (0x05, 250_000)
        );
        assert_eq!(
            (Bandwidth::Khz500.code(), Bandwidth::Khz500.hz()),
            (0x06, 500_000)
        );
    }

    #[test]
    fn frequency_code() {
        assert_eq!(rf_freq_code(869_525_000), 0x3658_6666);
        assert_eq!(rf_freq_code(32_000_000), 1 << 25);
    }

    #[test]
    fn sync_word() {
        assert_eq!(sync_word_reg(0x12), 0x1424);
        assert_eq!(sync_word_reg(0x34), 0x3444);
    }

    #[test]
    fn tcxo() {
        let t = |millivolts, start_us| {
            tcxo_args(Tcxo {
                millivolts,
                start_us,
            })
        };
        assert_eq!(t(1700, 5000), Ok([0x01, 0x00, 0x01, 0x40]));
        assert_eq!(t(1600, 1), Ok([0x00, 0x00, 0x00, 0x01]));
        assert_eq!(t(2000, 0), Ok([0x02, 0, 0, 0])); // 2.0 V rounds down to 1.8 V
        assert_eq!(t(3300, 15_625), Ok([0x07, 0, 0x03, 0xE8]));
        assert_eq!(t(1500, 5000), Err(ConfigError::Tcxo));
        assert_eq!(t(3400, 5000), Err(ConfigError::Tcxo));
        assert_eq!(t(1700, 300_000_000), Err(ConfigError::Tcxo));
    }

    #[test]
    fn pa_table() {
        assert_eq!(pa_params(22), ([0x04, 0x07, 0x00, 0x01], 22));
        assert_eq!(pa_params(20), ([0x03, 0x05, 0x00, 0x01], 22));
        assert_eq!(pa_params(17), ([0x02, 0x03, 0x00, 0x01], 22));
        assert_eq!(pa_params(14), ([0x02, 0x02, 0x00, 0x01], 22));
        assert_eq!(pa_params(10), ([0x04, 0x07, 0x00, 0x01], 10));
        assert_eq!(pa_params(30), ([0x04, 0x07, 0x00, 0x01], 22));
        assert_eq!(pa_params(-20), ([0x04, 0x07, 0x00, 0x01], -9));
    }

    #[test]
    fn low_data_rate_optimisation() {
        assert!(!ldro(7, Bandwidth::Khz250));
        assert!(ldro(11, Bandwidth::Khz125));
        assert!(ldro(12, Bandwidth::Khz125));
        assert!(!ldro(11, Bandwidth::Khz250));
    }

    #[test]
    fn image_calibration_bands() {
        assert_eq!(image_cal_args(869_525_000), Ok([0xD7, 0xDB]));
        assert_eq!(image_cal_args(868_000_000), Ok([0xD7, 0xDB]));
        assert_eq!(image_cal_args(915_000_000), Ok([0xE1, 0xE9]));
        assert_eq!(image_cal_args(433_000_000), Ok([0x6B, 0x6F]));
        assert_eq!(image_cal_args(600_000_000), Err(ConfigError::Frequency));
    }
}
