use super::*;
use crate::state::RxTelemetry;
use winchy_geo::GeoPoint;
use winchy_gnss::time::ClockSource;
use winchy_protocol::{Telemetry, TelemetryFlags, TowPhase};

#[test]
fn snapshot_without_rope_or_fix() {
    let j = json(&WinchState::new(), 0, None);
    assert_eq!(
        j.as_str(),
        concat!(
            r#"{"phase":"--","force":0,"uncal":true,"angle":0.0,"alt":0,"speed":0.00,"#,
            r#""rssi":0,"batt":0.0,"battlow":false,"battpct":255,"charging":false,"gps":false,"#,
            r#""rx":0,"lost":0,"wsats":0,"wfix":false,"wlat":null,"wlon":null,"wacc":null,"#,
            r#""wconv":false,"rope":false,"tsync":false,"time":"--:--:--","upstatus":""}"#
        )
    );
}

#[test]
fn snapshot_with_telemetry_survey_and_clock() {
    let mut s = WinchState::new();
    s.telemetry = Some(RxTelemetry {
        telemetry: Telemetry {
            phase: TowPhase::Climb,
            flags: TelemetryFlags(TelemetryFlags::BATTERY_LOW | TelemetryFlags::GNSS_FIX),
            force: 12_345,
            rope_angle_cdeg: 4_449,
            altitude_dm: 12_004,
            climb_cms: 0,
            glider_speed_cms: 2_512,
            battery_mv: 3_412,
            battery_pct: 20,
            tx_power_dbm: 22,
        },
        rssi_dbm: -97,
        at_ms: 1_000,
    });
    s.received = 120;
    s.lost = 3;
    s.sats = 9;
    s.fix = true;
    s.clock = ClockSource::Gnss;
    for _ in 0..40 {
        s.survey.add(GeoPoint::new(48.746_887, 11.449_15), 400.0);
    }
    let t = TimeOfDay {
        hour: 7,
        minute: 5,
        second: 9,
    };
    let j = json(&s, 1_500, Some(t));
    assert_eq!(
        j.as_str(),
        concat!(
            r#"{"phase":"CLIMB","force":1235,"uncal":false,"angle":44.5,"alt":1200,"speed":25.12,"#,
            r#""rssi":-97,"batt":3.4,"battlow":true,"battpct":20,"charging":false,"gps":true,"#,
            r#""rx":120,"lost":3,"wsats":9,"wfix":true,"wlat":48.746887,"wlon":11.449150,"wacc":0.5,"#,
            r#""wconv":true,"rope":true,"tsync":true,"time":"07:05:09","upstatus":""}"#
        )
    );
}

#[test]
fn stale_telemetry_turns_the_rope_light_off_but_keeps_values() {
    let mut s = WinchState::new();
    s.telemetry = Some(RxTelemetry {
        telemetry: Telemetry {
            phase: TowPhase::Idle,
            flags: TelemetryFlags(TelemetryFlags::FORCE_UNCALIBRATED),
            force: -107_829,
            rope_angle_cdeg: 0,
            altitude_dm: 0,
            climb_cms: 0,
            glider_speed_cms: 0,
            battery_mv: 0,
            battery_pct: 255,
            tx_power_dbm: 0,
        },
        rssi_dbm: -50,
        at_ms: 0,
    });
    let j = json(&s, 10_000, None);
    assert!(
        j.contains(r#""phase":"IDLE","force":-107829,"uncal":true"#),
        "{j}"
    );
    assert!(j.contains(r#""rope":false"#));
}

#[test]
fn page_opens_the_websocket() {
    assert!(PAGE.starts_with("<!DOCTYPE html>"));
    assert!(PAGE.contains("new WebSocket('ws://'+location.host+'/ws')"));
}

#[test]
fn page_greets_the_server_on_open() {
    // ESP-IDF 5.5 does not call the URI handler for the WebSocket handshake;
    // the winch learns about a client from its first frame.
    assert!(PAGE.contains("ws.onopen=function(){ws.send('hello');};"));
}

#[test]
fn time_of_day_from_unix_seconds() {
    // 2026-11-28 16:11:08 UTC
    assert_eq!(
        TimeOfDay::from_unix(1_795_882_268),
        TimeOfDay {
            hour: 16,
            minute: 11,
            second: 8
        }
    );
    assert_eq!(
        TimeOfDay::from_unix(-1),
        TimeOfDay {
            hour: 23,
            minute: 59,
            second: 59
        }
    );
}
