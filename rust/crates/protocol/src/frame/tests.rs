use super::*;
use crate::message::*;

fn telemetry() -> Telemetry {
    Telemetry {
        phase: TowPhase::Climb,
        flags: TelemetryFlags(TelemetryFlags::GNSS_FIX | TelemetryFlags::TIME_SYNCED),
        force: -123_456,
        rope_angle_cdeg: 4_250,
        altitude_dm: 6_123,
        climb_cms: -250,
        glider_speed_cms: 2_810,
        battery_mv: 3_987,
        battery_pct: 87,
        tx_power_dbm: -9,
    }
}

fn all_messages() -> [Message; 7] {
    [
        Message::Telemetry(telemetry()),
        Message::Mass(Mass {
            mass_dkg: 5_255,
            confidence_pct: 80,
        }),
        Message::Summary(Summary {
            duration_ds: 412,
            max_force: 9_000,
            release_alt_dm: 4_500,
            mass_dkg: 5_255,
        }),
        Message::LinkReport(LinkReport {
            rssi_dbm: -97,
            snr_qdb: -23,
            loss_pct: 12,
        }),
        Message::WinchPos(WinchPos {
            lat_e7: -335_123_456,
            lon_e7: 1_512_345_678,
            altitude_dm: -120,
            hacc_dm: 15,
            status: WinchStatus(WinchStatus::FIX | WinchStatus::SURVEY_DONE),
        }),
        Message::UploadCmd { nonce: 200 },
        Message::UploadAck { nonce: 200 },
    ]
}

fn encode(f: &Frame) -> Vec<u8> {
    let mut out = [0u8; MAX_FRAME_LEN];
    let n = f.encode(&mut out).unwrap();
    out[..n].to_vec()
}

#[test]
fn every_message_roundtrips() {
    for (i, message) in all_messages().into_iter().enumerate() {
        let f = Frame {
            seq: 65_530_u16.wrapping_add(i as u16), // crosses the u16 wrap
            message,
        };
        let bytes = encode(&f);
        assert!(bytes.len() <= MAX_FRAME_LEN);
        assert_eq!(Frame::decode(&bytes), Ok(f), "{message:?}");
    }
}

#[test]
fn header_layout() {
    let bytes = encode(&Frame {
        seq: 0x1234,
        message: Message::UploadCmd { nonce: 9 },
    });
    assert_eq!(
        bytes,
        [VERSION, MessageType::UploadCmd as u8, 0x34, 0x12, 9]
    );
}

#[test]
fn telemetry_golden_bytes() {
    // Pins the wire format: changing it requires bumping VERSION.
    let bytes = encode(&Frame {
        seq: 7,
        message: Message::Telemetry(telemetry()),
    });
    assert_eq!(
        bytes,
        [
            0x20, 0x01, 0x07, 0x00, // version, type, seq
            0x04, // phase Climb
            0x06, // flags
            0xC0, 0x1D, 0xFE, 0xFF, // force -123456
            0x9A, 0x10, // angle 4250
            0xEB, 0x17, 0x00, 0x00, // altitude 6123
            0x06, 0xFF, // climb -250
            0xFA, 0x0A, // speed 2810
            0x93, 0x0F, // battery 3987
            0x57, // 87 %
            0xF7, // -9 dBm
        ]
    );
    assert_eq!(bytes.len(), MAX_FRAME_LEN);
}

#[test]
fn encode_rejects_small_buffer() {
    let f = Frame {
        seq: 0,
        message: Message::Telemetry(telemetry()),
    };
    let mut out = [0u8; MAX_FRAME_LEN - 1];
    assert_eq!(f.encode(&mut out), Err(EncodeError::BufferTooSmall));
}

#[test]
fn rejects_short_and_unknown() {
    assert_eq!(Frame::decode(&[]), Err(DecodeError::TooShort));
    assert_eq!(Frame::decode(&[VERSION, 1, 0]), Err(DecodeError::TooShort));
    assert_eq!(
        Frame::decode(&[6, 1, 0, 0, 0]),
        Err(DecodeError::UnknownVersion(6))
    );
    assert_eq!(
        Frame::decode(&[VERSION, 99, 0, 0]),
        Err(DecodeError::UnknownType(99))
    );
}

#[test]
fn rejects_wrong_length() {
    let mut bytes = encode(&Frame {
        seq: 1,
        message: Message::Telemetry(telemetry()),
    });
    bytes.pop();
    assert_eq!(
        Frame::decode(&bytes),
        Err(DecodeError::BadLength {
            expected: MAX_FRAME_LEN,
            actual: MAX_FRAME_LEN - 1
        })
    );
    let mut long = encode(&Frame {
        seq: 1,
        message: Message::UploadAck { nonce: 1 },
    });
    long.push(0);
    assert_eq!(
        Frame::decode(&long),
        Err(DecodeError::BadLength {
            expected: 5,
            actual: 6
        })
    );
}

#[test]
fn rejects_invalid_phase() {
    let mut bytes = encode(&Frame {
        seq: 1,
        message: Message::Telemetry(telemetry()),
    });
    bytes[4] = 42;
    assert_eq!(Frame::decode(&bytes), Err(DecodeError::BadValue));
}
