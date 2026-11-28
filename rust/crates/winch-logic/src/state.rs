//! State shared by the winch tasks (radio, GNSS, display).

use winchy_gnss::survey::{SurveyConfig, SurveyIn};
use winchy_gnss::time::ClockSource;
use winchy_protocol::Telemetry;

/// The latest telemetry frame and how it arrived.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RxTelemetry {
    /// Decoded telemetry.
    pub telemetry: Telemetry,
    /// RSSI of that frame, dBm.
    pub rssi_dbm: i16,
    /// Monotonic receive time, ms.
    pub at_ms: u64,
}

/// Everything the winch knows.
#[derive(Debug, Clone)]
pub struct WinchState {
    /// Latest rope telemetry.
    pub telemetry: Option<RxTelemetry>,
    /// Frames received from the rope since boot.
    pub received: u32,
    /// Frames lost since boot (sequence gaps).
    pub lost: u32,
    /// Satellites in use by the winch receiver.
    pub sats: u8,
    /// Winch receiver has a fix now.
    pub fix: bool,
    /// Survey-in of the parked winch's position.
    pub survey: SurveyIn,
    /// Where the system clock comes from.
    pub clock: ClockSource,
    /// Log server reachable (WiFi stage; false until then).
    pub online: bool,
}

impl WinchState {
    /// State at boot.
    #[must_use]
    pub fn new() -> Self {
        Self {
            telemetry: None,
            received: 0,
            lost: 0,
            sats: 0,
            fix: false,
            survey: SurveyIn::new(SurveyConfig::default()),
            clock: ClockSource::Unset,
            online: false,
        }
    }
}

impl Default for WinchState {
    fn default() -> Self {
        Self::new()
    }
}
