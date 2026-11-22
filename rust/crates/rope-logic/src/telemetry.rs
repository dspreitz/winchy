//! Builds the TELEMETRY message from the shared state.

use winchy_gnss::time::ClockSource;
use winchy_protocol::{Telemetry, TelemetryFlags, fixed};

use crate::state::RopeState;

/// Telemetry period, ms (2 Hz: SF7/BW250 ~30 ms airtime, ~6 % duty cycle).
pub const PERIOD_MS: u32 = 500;
/// Extra listen time after a frame that requests a link report, ms.
pub const REPORT_WINDOW_MS: u32 = 150;
/// Request a link report every Nth frame (~1 Hz).
pub const REPORT_EVERY: u32 = 2;

/// True if frame number `frame` (1-based) should request a link report.
#[must_use]
pub const fn request_report(frame: u32, every: u32) -> bool {
    every != 0 && frame % every == 0
}

/// Telemetry from the current state (force stays raw counts until the load
/// cell calibration exists, so `FORCE_UNCALIBRATED` is always set).
#[must_use]
pub fn build(s: &RopeState, request_report: bool) -> Telemetry {
    let mut flags = TelemetryFlags::FORCE_UNCALIBRATED;
    for (on, bit) in [
        (s.gnss.fix, TelemetryFlags::GNSS_FIX),
        (s.clock != ClockSource::Unset, TelemetryFlags::TIME_SYNCED),
        (request_report, TelemetryFlags::REQUEST_REPORT),
        (s.power.battery_low, TelemetryFlags::BATTERY_LOW),
        (s.power.charging, TelemetryFlags::CHARGING),
    ] {
        if on {
            flags |= bit;
        }
    }
    Telemetry {
        phase: s.phase,
        flags: TelemetryFlags(flags),
        force: s.force.tared(),
        rope_angle_cdeg: fixed::to_i16(s.attitude.angle_deg, 100.0),
        altitude_dm: s
            .baro
            .altitude_m
            .map_or(0, |a| fixed::to_i32(f64::from(a), 10.0)),
        climb_cms: fixed::to_i16(s.baro.climb_ms, 100.0),
        glider_speed_cms: fixed::to_u16(s.glider_speed_ms, 100.0),
        battery_mv: s.power.battery_mv.unwrap_or(0),
        battery_pct: s.power.battery_pct.unwrap_or(u8::MAX),
        tx_power_dbm: s.link.tx_power_dbm,
    }
}

#[cfg(test)]
mod tests {
    use winchy_protocol::TowPhase;

    use super::*;
    use crate::state::{Baro, Force, Power};

    fn state() -> RopeState {
        let mut s = RopeState::new(14);
        s.force = Force {
            raw: 10_500,
            offset: 500,
            ..Force::default()
        };
        s.attitude.angle_deg = 42.567;
        s.baro = Baro {
            altitude_m: Some(612.34),
            climb_ms: -2.5,
            ..Baro::default()
        };
        s.glider_speed_ms = 28.1;
        s.power = Power {
            battery_mv: Some(3_987),
            battery_pct: Some(87),
            ..Power::default()
        };
        s
    }

    #[test]
    fn converts_units_to_fixed_point() {
        let t = build(&state(), false);
        assert_eq!(t.phase, TowPhase::Idle);
        assert_eq!(t.force, 10_000);
        assert_eq!(t.rope_angle_cdeg, 4_257);
        assert_eq!(t.altitude_dm, 6_123);
        assert_eq!(t.climb_cms, -250);
        assert_eq!(t.glider_speed_cms, 2_810);
        assert_eq!(t.battery_mv, 3_987);
        assert_eq!(t.battery_pct, 87);
        assert_eq!(t.tx_power_dbm, 14);
    }

    #[test]
    fn minimal_flags_when_nothing_is_known() {
        let t = build(&RopeState::new(14), false);
        assert_eq!(t.flags, TelemetryFlags(TelemetryFlags::FORCE_UNCALIBRATED));
        assert_eq!(t.altitude_dm, 0, "no baro sample yet");
        assert_eq!(t.battery_mv, 0, "no cell");
        assert_eq!(t.battery_pct, 255, "unknown");
    }

    #[test]
    fn flags_follow_state() {
        let mut s = state();
        s.gnss.fix = true;
        s.clock = ClockSource::Ntp;
        s.power.battery_low = true;
        s.power.charging = true;
        let t = build(&s, true);
        let f = TelemetryFlags::FORCE_UNCALIBRATED
            | TelemetryFlags::GNSS_FIX
            | TelemetryFlags::TIME_SYNCED
            | TelemetryFlags::REQUEST_REPORT
            | TelemetryFlags::BATTERY_LOW
            | TelemetryFlags::CHARGING;
        assert_eq!(t.flags, TelemetryFlags(f));
    }

    #[test]
    fn every_second_frame_requests_a_report() {
        let pattern: [bool; 4] = core::array::from_fn(|i| request_report(i as u32 + 1, 2));
        assert_eq!(pattern, [false, true, false, true]);
        assert!(!request_report(4, 0), "0 disables requests");
    }
}
