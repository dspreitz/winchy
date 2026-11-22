//! The rope unit's shared state: the latest value of every measured quantity.
//! Each task writes only its own section and reads the others.

use winchy_geo::WinchRelative;
use winchy_gnss::time::ClockSource;
use winchy_protocol::{LinkReport, TowPhase, WinchPos};

/// Load cell (ADS1232).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Force {
    /// Last raw conversion (untared counts).
    pub raw: i32,
    /// Tare offset, counts.
    pub offset: i32,
    /// Uptime of the last conversion, ms.
    pub updated_ms: Option<u64>,
    /// Failed conversions (DRDY timeouts).
    pub errors: u32,
}

impl Force {
    /// Tared force, counts.
    #[must_use]
    pub fn tared(&self) -> i32 {
        self.raw.saturating_sub(self.offset)
    }
}

/// IMU and attitude.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Attitude {
    /// Specific force, g.
    pub accel_g: [f32; 3],
    /// Bias-corrected angular rate, dps.
    pub gyro_dps: [f32; 3],
    /// Rope angle above the horizon, degrees.
    pub angle_deg: f32,
    /// Rope angle rate, dps.
    pub angle_rate_dps: f32,
    /// Uptime of the last sample, ms.
    pub updated_ms: Option<u64>,
}

/// Barometer.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Baro {
    /// Static pressure, hPa.
    pub pressure_hpa: f32,
    /// Sensor temperature, degC.
    pub temperature_c: f32,
    /// Filtered altitude AMSL, m (None until the filter has a sample).
    pub altitude_m: Option<f32>,
    /// Filtered climb rate, m/s.
    pub climb_ms: f32,
    /// Sea-level reference in use, hPa.
    pub qnh_hpa: f32,
    /// True once a GNSS fix calibrated the QNH.
    pub qnh_from_gnss: bool,
    /// Uptime of the last sample, ms.
    pub updated_ms: Option<u64>,
}

/// GNSS receiver.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Gnss {
    /// Valid position fix.
    pub fix: bool,
    /// Satellites used.
    pub sats: u8,
    /// Horizontal accuracy estimate, m.
    pub hacc_m: f32,
    /// Latitude, degrees.
    pub lat_deg: f64,
    /// Longitude, degrees.
    pub lon_deg: f64,
    /// Height above mean sea level, m.
    pub alt_m: f32,
    /// 2-D ground speed, m/s (0 without fix).
    pub ground_speed_ms: f32,
    /// Vertical speed (up positive), m/s.
    pub climb_ms: f32,
    /// Uptime of the last solution, ms.
    pub updated_ms: Option<u64>,
}

/// Power (AXP2101).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Power {
    /// System rail, mV.
    pub system_mv: u16,
    /// Cell voltage, mV (None: no cell).
    pub battery_mv: Option<u16>,
    /// Fuel gauge, percent (None: no cell).
    pub battery_pct: Option<u8>,
    /// The cell is charging.
    pub charging: bool,
    /// Low-battery warning (only raised while idle).
    pub battery_low: bool,
}

/// LoRa link.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Link {
    /// Current TX power, dBm.
    pub tx_power_dbm: i8,
    /// Frames sent.
    pub tx_count: u32,
    /// Valid frames received.
    pub rx_count: u32,
    /// Last downlink report from the winch.
    pub last_report: Option<LinkReport>,
    /// Uptime of that report, ms.
    pub report_ms: Option<u64>,
}

/// Last winch position received over the radio.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Winch {
    /// Surveyed position.
    pub pos: WinchPos,
    /// Uptime of reception, ms.
    pub received_ms: u64,
}

/// Everything the rope unit knows right now.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RopeState {
    /// Tow phase.
    pub phase: TowPhase,
    /// Load cell.
    pub force: Force,
    /// IMU / attitude.
    pub attitude: Attitude,
    /// Barometer.
    pub baro: Baro,
    /// GNSS.
    pub gnss: Gnss,
    /// Glider (CG-hook) speed estimate, m/s.
    pub glider_speed_ms: f32,
    /// Power.
    pub power: Power,
    /// Where the system clock came from.
    pub clock: ClockSource,
    /// LoRa link.
    pub link: Link,
    /// Winch position, once received.
    pub winch: Option<Winch>,
    /// Winch-relative geometry, once both positions are known.
    pub geometry: Option<WinchRelative>,
}

impl RopeState {
    /// Fresh state at boot (`tx_power_dbm` is the radio's initial power).
    #[must_use]
    pub fn new(tx_power_dbm: i8) -> Self {
        Self {
            phase: TowPhase::Idle,
            force: Force::default(),
            attitude: Attitude::default(),
            baro: Baro::default(),
            gnss: Gnss::default(),
            glider_speed_ms: 0.0,
            power: Power::default(),
            clock: ClockSource::Unset,
            link: Link {
                tx_power_dbm,
                ..Link::default()
            },
            winch: None,
            geometry: None,
        }
    }

    /// True while no tow is in progress.
    #[must_use]
    pub fn is_idle(&self) -> bool {
        self.phase == TowPhase::Idle
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fresh_state_is_idle_unsynced_and_empty() {
        let s = RopeState::new(14);
        assert!(s.is_idle());
        assert_eq!(s.clock, ClockSource::Unset);
        assert_eq!(s.link.tx_power_dbm, 14);
        assert_eq!(s.winch, None);
        assert_eq!(s.baro.altitude_m, None);
        assert_eq!(s.power.battery_mv, None);
    }

    #[test]
    fn not_idle_during_a_tow() {
        let s = RopeState {
            phase: TowPhase::Climb,
            ..RopeState::new(14)
        };
        assert!(!s.is_idle());
    }

    #[test]
    fn tared_force_subtracts_offset_saturating() {
        let f = Force {
            raw: 1_000,
            offset: 300,
            ..Force::default()
        };
        assert_eq!(f.tared(), 700);
        let extreme = Force {
            raw: i32::MIN,
            offset: 1,
            ..Force::default()
        };
        assert_eq!(extreme.tared(), i32::MIN);
    }
}
