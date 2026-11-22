use winchy_protocol::{LinkReport, Telemetry, TelemetryFlags, VERSION, WinchPos, WinchStatus, v6};

use super::*;

fn v6_link_report(rssi: i16, snr_db: i8, loss: u8) -> [u8; 8] {
    let r = rssi.to_le_bytes();
    [6, 5, 0, 0, r[0], r[1], snr_db.to_le_bytes()[0], loss]
}

fn decode_v6_telemetry_power_and_flags(frame: &[u8]) -> (u16, u8) {
    assert_eq!(frame.len(), v6::TELEMETRY_LEN);
    (u16::from_le_bytes([frame[2], frame[3]]), frame[13])
}

fn fixed_power() -> RadioConfig {
    RadioConfig {
        adr: None,
        ..RadioConfig::default()
    }
}

#[test]
fn legacy_cycle_numbers_frames_and_requests_every_second_report() {
    let mut link = RadioLink::new(fixed_power());
    let state = RopeState::new(14);
    let mut out = [0u8; 32];

    let p1 = link.plan_tx(&state, 0, &mut out).unwrap();
    let (seq, flags) = decode_v6_telemetry_power_and_flags(&out[..p1.len]);
    assert_eq!((p1.seq, seq), (0, 0));
    assert!(!p1.report_requested);
    assert_eq!(flags & TelemetryFlags::REQUEST_REPORT, 0);
    assert_eq!(p1.listen_ms, 500);

    let p2 = link.plan_tx(&state, 500, &mut out).unwrap();
    let (seq, flags) = decode_v6_telemetry_power_and_flags(&out[..p2.len]);
    assert_eq!((p2.seq, seq), (1, 1));
    assert!(p2.report_requested);
    assert_ne!(flags & TelemetryFlags::REQUEST_REPORT, 0);
    assert_eq!(p2.listen_ms, 650);
    assert_eq!(p2.tx_power_dbm, 14, "fixed power");
}

#[test]
fn current_wire_carries_the_planned_power() {
    let mut link = RadioLink::new(RadioConfig {
        wire: Wire::Current,
        ..RadioConfig::default()
    });
    let mut out = [0u8; 32];
    let p = link.plan_tx(&RopeState::new(14), 0, &mut out).unwrap();
    let f = Frame::decode(&out[..p.len]).unwrap();
    assert_eq!(out[0], VERSION);
    let Message::Telemetry(t) = f.message else {
        panic!("{f:?}")
    };
    assert_eq!(t.tx_power_dbm, p.tx_power_dbm);
}

#[test]
fn adr_goes_loud_without_feedback_and_trims_on_a_strong_report() {
    let mut link = RadioLink::new(RadioConfig::default());
    let mut state = RopeState::new(14);
    let mut out = [0u8; 32];
    assert_eq!(
        link.plan_tx(&state, 0, &mut out).unwrap().tx_power_dbm,
        22,
        "stale -> max"
    );
    state.link.tx_power_dbm = 22;

    link.on_rx(&v6_link_report(-60, 8, 0), 100, &mut state);
    assert_eq!(
        link.plan_tx(&state, 500, &mut out).unwrap().tx_power_dbm,
        21,
        "strong -> trim 1 dB"
    );
}

#[test]
fn rx_link_report_updates_the_state() {
    let mut link = RadioLink::new(RadioConfig::default());
    let mut state = RopeState::new(14);
    let msg = link.on_rx(&v6_link_report(-97, -6, 12), 1_234, &mut state);
    let report = LinkReport {
        rssi_dbm: -97,
        snr_qdb: -6,
        loss_pct: 12,
    };
    assert_eq!(msg, Some(Message::LinkReport(report)));
    assert_eq!(state.link.last_report, Some(report));
    assert_eq!(state.link.report_ms, Some(1_234));
    assert_eq!(state.link.rx_count, 1);
}

#[test]
fn rx_winch_position_is_stored() {
    let mut link = RadioLink::new(RadioConfig::default());
    let mut state = RopeState::new(14);
    let pos = WinchPos {
        lat_e7: 480_000_000,
        lon_e7: 110_000_000,
        altitude_dm: 5_000,
        hacc_dm: 12,
        status: WinchStatus(WinchStatus::FIX),
    };
    let mut buf = [0u8; 32];
    let n = Frame {
        seq: 3,
        message: Message::WinchPos(pos),
    }
    .encode(&mut buf)
    .unwrap();
    assert!(link.on_rx(&buf[..n], 77, &mut state).is_some());
    assert_eq!(
        state.winch,
        Some(Winch {
            pos,
            received_ms: 77
        })
    );
}

#[test]
fn garbage_is_ignored() {
    let mut link = RadioLink::new(RadioConfig::default());
    let mut state = RopeState::new(14);
    assert_eq!(link.on_rx(&[1, 2, 3, 4, 5], 0, &mut state), None);
    assert_eq!(link.on_rx(&[], 0, &mut state), None);
    assert_eq!(state.link.rx_count, 0);
}

#[test]
fn decode_any_accepts_both_versions() {
    assert!(decode_any(&v6_link_report(-90, 0, 0)).is_some());
    let mut buf = [0u8; 32];
    let n = Frame {
        seq: 1,
        message: Message::UploadAck { nonce: 4 },
    }
    .encode(&mut buf)
    .unwrap();
    assert_eq!(
        decode_any(&buf[..n]).unwrap().message,
        Message::UploadAck { nonce: 4 }
    );
    assert_eq!(decode_any(&[9, 9, 9, 9]), None);
}

#[test]
fn encode_telemetry_switches_format() {
    let t = Telemetry {
        phase: winchy_protocol::TowPhase::Idle,
        flags: TelemetryFlags(0),
        force: 0,
        rope_angle_cdeg: 0,
        altitude_dm: 0,
        climb_cms: 0,
        glider_speed_cms: 0,
        battery_mv: 0,
        battery_pct: 255,
        tx_power_dbm: 0,
    };
    let mut out = [0u8; 32];
    assert_eq!(encode_telemetry(Wire::LegacyV6, 0, &t, &mut out), Ok(17));
    assert_eq!(out[0], 6);
    let n = encode_telemetry(Wire::Current, 0, &t, &mut out).unwrap();
    assert_eq!((out[0], n), (VERSION, 24));
}
