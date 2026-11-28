use super::*;
use winchy_protocol::{LinkReport, Telemetry, TowPhase, WinchStatus};

fn telemetry(seq: u16, flags: u8) -> Frame {
    Frame {
        seq,
        message: Message::Telemetry(Telemetry {
            phase: TowPhase::Idle,
            flags: TelemetryFlags(flags),
            force: 0,
            rope_angle_cdeg: 0,
            altitude_dm: 0,
            climb_cms: 0,
            glider_speed_cms: 0,
            battery_mv: 0,
            battery_pct: 255,
            tx_power_dbm: 22,
        }),
    }
}

const POS: WinchPos = WinchPos {
    lat_e7: 1,
    lon_e7: 2,
    altitude_dm: 3,
    hacc_dm: 4,
    status: WinchStatus(1),
};

#[test]
fn report_request_gets_a_link_report() {
    let mut r = Responder::new();
    let reply = r
        .on_frame(
            &telemetry(7, TelemetryFlags::REQUEST_REPORT),
            -55,
            30,
            0,
            Some(POS),
        )
        .unwrap();
    assert_eq!(
        reply,
        Frame {
            seq: 0,
            message: Message::LinkReport(LinkReport {
                rssi_dbm: -55,
                snr_qdb: 30,
                loss_pct: 0
            })
        }
    );
}

#[test]
fn otherwise_the_due_position_then_nothing() {
    let mut r = Responder::new();
    let reply = r
        .on_frame(&telemetry(1, 0), -55, 30, 100, Some(POS))
        .unwrap();
    assert_eq!(reply.message, Message::WinchPos(POS));
    r.sent(&reply, true, 100);
    assert_eq!(r.on_frame(&telemetry(2, 0), -55, 30, 600, Some(POS)), None);
}

#[test]
fn no_position_without_survey() {
    let mut r = Responder::new();
    assert_eq!(r.on_frame(&telemetry(1, 0), -55, 30, 0, None), None);
}

#[test]
fn failed_position_is_retried_soon() {
    let mut r = Responder::new();
    let reply = r.on_frame(&telemetry(1, 0), -55, 30, 0, Some(POS)).unwrap();
    r.sent(&reply, false, 0);
    assert_eq!(r.on_frame(&telemetry(2, 0), -55, 30, 500, Some(POS)), None);
    assert!(
        r.on_frame(&telemetry(3, 0), -55, 30, 2_000, Some(POS))
            .is_some()
    );
}

#[test]
fn reply_sequence_counts_up() {
    let mut r = Responder::new();
    let a = r
        .on_frame(
            &telemetry(1, TelemetryFlags::REQUEST_REPORT),
            -55,
            30,
            0,
            None,
        )
        .unwrap();
    let b = r
        .on_frame(
            &telemetry(2, TelemetryFlags::REQUEST_REPORT),
            -55,
            30,
            0,
            None,
        )
        .unwrap();
    assert_eq!((a.seq, b.seq), (0, 1));
}

#[test]
fn non_telemetry_frames_are_counted_but_not_answered() {
    let mut r = Responder::new();
    let f = Frame {
        seq: 9,
        message: Message::UploadAck { nonce: 1 },
    };
    assert_eq!(r.on_frame(&f, -55, 30, 0, Some(POS)), None);
    assert_eq!(r.stats().received(), 1);
}
