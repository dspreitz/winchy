use super::*;
use crate::state::RxTelemetry;
use winchy_geo::GeoPoint;
use winchy_protocol::Telemetry;

fn telemetry(flags: u8, force: i32) -> Telemetry {
    Telemetry {
        phase: TowPhase::Climb,
        flags: TelemetryFlags(flags),
        force,
        rope_angle_cdeg: 4_449,
        altitude_dm: 12_004,
        climb_cms: 0,
        glider_speed_cms: 2_500, // 90 km/h
        battery_mv: 3_412,
        battery_pct: 20,
        tx_power_dbm: 22,
    }
}

fn with_telemetry(t: Telemetry, at_ms: u64) -> WinchState {
    let mut s = WinchState::new();
    s.telemetry = Some(RxTelemetry {
        telemetry: t,
        rssi_dbm: -97,
        at_ms,
    });
    s.sats = 9;
    s.fix = true;
    s
}

#[test]
fn phase_names_match_the_micropython_winch() {
    let names: Vec<_> = [
        TowPhase::Idle,
        TowPhase::SlackOut,
        TowPhase::GroundRoll,
        TowPhase::Rotation,
        TowPhase::Climb,
        TowPhase::Top,
        TowPhase::Released,
        TowPhase::LinkBreak,
    ]
    .into_iter()
    .map(phase_name)
    .collect();
    assert_eq!(
        names,
        [
            "IDLE", "SLACK", "ROLL", "ROTATE", "CLIMB", "TOP", "RELEASED", "BREAK!"
        ]
    );
}

#[test]
fn telemetry_screen_shows_speed_force_and_status() {
    let s = build(&with_telemetry(telemetry(0, 12_345), 1_000), 1_500, 4);
    assert_eq!(s.header.title.as_str(), "CLIMB");
    assert_eq!((s.header.sats, s.header.fix), (9, true));
    assert_eq!(s.big[0].as_str(), "90km/h");
    assert_eq!(s.big[1].as_str(), "1235N"); // 1234.5 N
    assert_eq!(s.small.as_str(), "");
    assert_eq!(s.bottom.as_str(), "A44 H1200 -97dB");
}

#[test]
fn uncalibrated_force_is_shown_in_thousands_of_counts() {
    let t = telemetry(TelemetryFlags::FORCE_UNCALIBRATED, -107_829);
    let s = build(&with_telemetry(t, 0), 0, 4);
    assert_eq!(s.big[1].as_str(), "-108k"); // floor, as MicroPython's //
}

#[test]
fn low_battery_alternates_with_the_status_line() {
    let state = with_telemetry(telemetry(TelemetryFlags::BATTERY_LOW, 0), 0);
    assert_eq!(build(&state, 0, 0).bottom.as_str(), "!BATT LOW 3.4V");
    assert_eq!(build(&state, 0, 3).bottom.as_str(), "!BATT LOW 3.4V");
    assert_eq!(build(&state, 0, 4).bottom.as_str(), "A44 H1200 -97dB");
}

#[test]
fn stale_telemetry_hands_over_to_the_gps_view() {
    let state = with_telemetry(telemetry(0, 0), 1_000);
    assert!(telemetry_fresh(&state, 1_000 + TELEMETRY_TIMEOUT_MS - 1));
    assert!(!telemetry_fresh(&state, 1_000 + TELEMETRY_TIMEOUT_MS));
    let s = build(&state, 1_000 + TELEMETRY_TIMEOUT_MS, 0);
    assert_eq!(s.header.title.as_str(), "WINCH GPS");
}

#[test]
fn gps_view_without_position_waits_for_fix() {
    let s = build(&WinchState::new(), 0, 0);
    assert_eq!(s.header.title.as_str(), "WINCH GPS");
    assert_eq!((s.header.sats, s.header.fix), (0, false));
    assert_eq!(s.big[0].as_str(), "NO FIX");
    assert_eq!(s.big[1].as_str(), "");
    assert_eq!(s.small.as_str(), "");
    assert_eq!(s.bottom.as_str(), "waiting for fix");
}

#[test]
fn gps_view_shows_survey_progress() {
    let mut state = WinchState::new();
    state.survey.add(GeoPoint::new(48.0, 11.0), 400.0);
    let s = build(&state, 0, 0);
    assert_eq!(s.big[0].as_str(), "-- m"); // one sample: accuracy unknown
    assert_eq!(s.small.as_str(), "n=1");
    assert_eq!(s.bottom.as_str(), "surveying...");
    for i in 0..40 {
        state
            .survey
            .add(GeoPoint::new(48.0 + f64::from(i % 2) * 1e-6, 11.0), 400.0);
    }
    let s = build(&state, 0, 0);
    assert_eq!(s.big[0].as_str(), "0.5 m");
    assert_eq!(s.small.as_str(), "n=41");
    assert_eq!(s.bottom.as_str(), "SURVEYED");
}
