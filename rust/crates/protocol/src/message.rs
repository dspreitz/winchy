//! Message payload types (fixed-point, wire units).

/// Tow phase (see `docs/winch_launch_physics.md`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TowPhase {
    /// Waiting on the ground.
    Idle = 0,
    /// Cable slack being taken up.
    SlackOut = 1,
    /// Ground roll.
    GroundRoll = 2,
    /// Rotation into the climb.
    Rotation = 3,
    /// Full climb.
    Climb = 4,
    /// Top of launch.
    Top = 5,
    /// Released.
    Released = 6,
    /// Cable or weak-link break.
    LinkBreak = 7,
}

impl TryFrom<u8> for TowPhase {
    type Error = u8;

    fn try_from(v: u8) -> Result<Self, u8> {
        Ok(match v {
            0 => Self::Idle,
            1 => Self::SlackOut,
            2 => Self::GroundRoll,
            3 => Self::Rotation,
            4 => Self::Climb,
            5 => Self::Top,
            6 => Self::Released,
            7 => Self::LinkBreak,
            other => return Err(other),
        })
    }
}

/// Telemetry status bits.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TelemetryFlags(pub u8);

impl TelemetryFlags {
    /// `force` is raw ADC counts, not 0.1 N.
    pub const FORCE_UNCALIBRATED: u8 = 0x01;
    /// GNSS has a 3-D fix.
    pub const GNSS_FIX: u8 = 0x02;
    /// Clock is GNSS/PPS disciplined.
    pub const TIME_SYNCED: u8 = 0x04;
    /// Winch should reply with a [`LinkReport`].
    pub const REQUEST_REPORT: u8 = 0x08;
    /// Rope battery low.
    pub const BATTERY_LOW: u8 = 0x10;
    /// Rope battery charging.
    pub const CHARGING: u8 = 0x20;

    /// True if every bit of `mask` is set.
    #[must_use]
    pub const fn contains(self, mask: u8) -> bool {
        self.0 & mask == mask
    }
}

/// Rope -> winch live telemetry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Telemetry {
    /// Current tow phase.
    pub phase: TowPhase,
    /// Status bits.
    pub flags: TelemetryFlags,
    /// Rope force, 0.1 N (raw counts if `FORCE_UNCALIBRATED`).
    pub force: i32,
    /// Rope angle above horizontal, 0.01 deg.
    pub rope_angle_cdeg: i16,
    /// Altitude AMSL, 0.1 m.
    pub altitude_dm: i32,
    /// Climb rate, cm/s.
    pub climb_cms: i16,
    /// Glider (CG-hook) speed estimate, cm/s.
    pub glider_speed_cms: u16,
    /// Battery voltage, mV.
    pub battery_mv: u16,
    /// Battery charge, percent (255 = unknown).
    pub battery_pct: u8,
    /// Current TX power, dBm.
    pub tx_power_dbm: i8,
}

/// Winch -> rope downlink quality report.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LinkReport {
    /// RSSI of the rope's frames at the winch, dBm.
    pub rssi_dbm: i16,
    /// SNR, 0.25 dB.
    pub snr_qdb: i8,
    /// Frame loss over the report window, percent.
    pub loss_pct: u8,
}

/// Winch survey status bits.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct WinchStatus(pub u8);

impl WinchStatus {
    /// Winch GNSS currently has a fix.
    pub const FIX: u8 = 0x01;
    /// Survey-in converged.
    pub const SURVEY_DONE: u8 = 0x02;

    /// True if every bit of `mask` is set.
    #[must_use]
    pub const fn contains(self, mask: u8) -> bool {
        self.0 & mask == mask
    }
}

/// Winch -> rope surveyed winch position.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WinchPos {
    /// Latitude, 1e-7 deg.
    pub lat_e7: i32,
    /// Longitude, 1e-7 deg.
    pub lon_e7: i32,
    /// Altitude AMSL, 0.1 m.
    pub altitude_dm: i32,
    /// Horizontal accuracy indicator, 0.1 m.
    pub hacc_dm: u16,
    /// Status bits.
    pub status: WinchStatus,
}

/// Rope -> winch glider mass estimate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mass {
    /// Mass, 0.1 kg.
    pub mass_dkg: u16,
    /// Confidence, percent.
    pub confidence_pct: u8,
}

/// Rope -> winch launch summary.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Summary {
    /// Launch duration, 0.1 s.
    pub duration_ds: u16,
    /// Peak rope force (units as [`Telemetry::force`]).
    pub max_force: i32,
    /// Release altitude AMSL, 0.1 m.
    pub release_alt_dm: i32,
    /// Estimated mass, 0.1 kg.
    pub mass_dkg: u16,
}

/// Frame type byte.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MessageType {
    /// [`Telemetry`].
    Telemetry = 1,
    /// [`Mass`].
    Mass = 2,
    /// [`Summary`].
    Summary = 3,
    /// [`LinkReport`].
    LinkReport = 4,
    /// [`WinchPos`].
    WinchPos = 5,
    /// Upload request.
    UploadCmd = 6,
    /// Upload acknowledgement.
    UploadAck = 7,
}

/// Any payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Message {
    /// Live telemetry.
    Telemetry(Telemetry),
    /// Mass estimate.
    Mass(Mass),
    /// Launch summary.
    Summary(Summary),
    /// Downlink quality.
    LinkReport(LinkReport),
    /// Winch position.
    WinchPos(WinchPos),
    /// "Upload your log too" (nonce makes resends idempotent).
    UploadCmd {
        /// Request id.
        nonce: u8,
    },
    /// Acknowledges an `UploadCmd` with the same nonce.
    UploadAck {
        /// Request id being acknowledged.
        nonce: u8,
    },
}

impl Message {
    /// The type byte for this message.
    #[must_use]
    pub const fn message_type(&self) -> MessageType {
        match self {
            Self::Telemetry(_) => MessageType::Telemetry,
            Self::Mass(_) => MessageType::Mass,
            Self::Summary(_) => MessageType::Summary,
            Self::LinkReport(_) => MessageType::LinkReport,
            Self::WinchPos(_) => MessageType::WinchPos,
            Self::UploadCmd { .. } => MessageType::UploadCmd,
            Self::UploadAck { .. } => MessageType::UploadAck,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn phase_roundtrip_and_reject() {
        for v in 0..=7u8 {
            assert_eq!(TowPhase::try_from(v).map(|p| p as u8), Ok(v));
        }
        assert_eq!(TowPhase::try_from(8), Err(8));
    }

    #[test]
    fn flags_contains() {
        let f = TelemetryFlags(TelemetryFlags::GNSS_FIX | TelemetryFlags::CHARGING);
        assert!(f.contains(TelemetryFlags::GNSS_FIX));
        assert!(!f.contains(TelemetryFlags::BATTERY_LOW));
        assert!(!f.contains(TelemetryFlags::GNSS_FIX | TelemetryFlags::BATTERY_LOW));
    }
}
