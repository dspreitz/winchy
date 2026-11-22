use super::*;
use crate::message::{TelemetryFlags, TowPhase};

fn telemetry() -> Telemetry {
    Telemetry {
        phase: TowPhase::Climb,
        flags: TelemetryFlags(0x0B),
        force: -123_456,
        rope_angle_cdeg: 3_550,
        altitude_dm: 4_123,
        climb_cms: -250,
        glider_speed_cms: 2_730,
        battery_mv: 3_870,
        battery_pct: 87,
        tx_power_dbm: 14,
    }
}

#[test]
fn telemetry_matches_micropython_struct_pack() {
    // struct.pack('<BBHBiBHBBBH', 6, 1, 0x1234, 4, -123456, 71, 412, 38, 0x0B, 87, 273)
    let mut out = [0u8; 32];
    let n = encode_telemetry(0x1234, &telemetry(), &mut out).unwrap();
    assert_eq!(
        &out[..n],
        &[
            6, 1, 52, 18, 4, 192, 29, 254, 255, 71, 156, 1, 38, 11, 87, 17, 1
        ]
    );
    assert_eq!(n, TELEMETRY_LEN);
}

#[test]
fn telemetry_fields_saturate_to_v6_ranges() {
    let t = Telemetry {
        rope_angle_cdeg: -500,
        altitude_dm: -30,
        battery_mv: u16::MAX,
        glider_speed_cms: u16::MAX,
        ..telemetry()
    };
    let mut out = [0u8; 17];
    encode_telemetry(0, &t, &mut out).unwrap();
    assert_eq!(out[9], 0, "negative angle clamps to 0");
    assert_eq!(&out[10..12], &[0, 0], "negative altitude clamps to 0");
    assert_eq!(out[12], 255, "battery clamps to 25.5 V");
    assert_eq!(
        u16::from_le_bytes([out[15], out[16]]),
        6_554,
        "65535 cm/s -> 6554 dm/s"
    );

    let high = Telemetry {
        rope_angle_cdeg: 18_000,
        altitude_dm: 1_000_000,
        ..telemetry()
    };
    encode_telemetry(0, &high, &mut out).unwrap();
    assert_eq!(out[9], 255, "angle clamps to 127.5 deg");
    assert_eq!(&out[10..12], &[255, 255], "altitude clamps to 65535 m");
}

#[test]
fn telemetry_rounds_half_degrees_and_metres() {
    let t = Telemetry {
        rope_angle_cdeg: 125,
        altitude_dm: 15,
        ..telemetry()
    };
    let mut out = [0u8; 17];
    encode_telemetry(0, &t, &mut out).unwrap();
    assert_eq!(out[9], 3, "1.25 deg -> 2.5 half degrees -> 3");
    assert_eq!(&out[10..12], &[2, 0], "1.5 m -> 2 m");
}

#[test]
fn telemetry_needs_room() {
    let mut out = [0u8; 16];
    assert_eq!(
        encode_telemetry(0, &telemetry(), &mut out),
        Err(EncodeError::BufferTooSmall)
    );
}

#[test]
fn decodes_link_report() {
    // struct.pack('<BBHhbB', 6, 5, 7, -97, -6, 12)
    let f = decode(&[6, 5, 7, 0, 159, 255, 250, 12]).unwrap();
    assert_eq!(f.seq, 7);
    assert_eq!(
        f.message,
        Message::LinkReport(LinkReport {
            rssi_dbm: -97,
            snr_qdb: -6,
            loss_pct: 12
        })
    );
}

#[test]
fn link_report_snr_is_already_in_quarter_db() {
    // The MicroPython winch puts the raw SX1262 packet-status SNR byte
    // (`struct.unpack("xBbx", ...)`, 0.25 dB units) into the field it calls
    // "snr_db": 40 means +10 dB, not +40 dB.
    let f = decode(&[6, 5, 0, 0, 0, 0, 40, 0]).unwrap();
    let Message::LinkReport(r) = f.message else {
        panic!("{f:?}")
    };
    assert_eq!(r.snr_qdb, 40);
}

#[test]
fn decodes_winch_pos() {
    // struct.pack('<BBHiiHBB', 6, 6, 9, 481234567, -114567890, 512, 23, 3)
    let f = decode(&[6, 6, 9, 0, 135, 14, 175, 28, 46, 213, 43, 249, 0, 2, 23, 3]).unwrap();
    assert_eq!(f.seq, 9);
    assert_eq!(
        f.message,
        Message::WinchPos(WinchPos {
            lat_e7: 481_234_567,
            lon_e7: -114_567_890,
            altitude_dm: 5_120,
            hacc_dm: 23,
            status: WinchStatus(3),
        })
    );
}

#[test]
fn decodes_upload_cmd_and_ack() {
    assert_eq!(
        decode(&[6, 7, 1, 0, 42]).unwrap().message,
        Message::UploadCmd { nonce: 42 }
    );
    assert_eq!(
        decode(&[6, 8, 2, 0, 43]).unwrap().message,
        Message::UploadAck { nonce: 43 }
    );
}

#[test]
fn rejects_wrong_version_type_and_length() {
    assert_eq!(decode(&[6, 5]), Err(DecodeError::TooShort));
    assert_eq!(
        decode(&[0x20, 5, 0, 0, 0, 0, 0, 0]),
        Err(DecodeError::UnknownVersion(0x20))
    );
    assert_eq!(
        decode(&[6, 1, 0, 0]),
        Err(DecodeError::UnknownType(1)),
        "rope-only type"
    );
    assert_eq!(decode(&[6, 99, 0, 0]), Err(DecodeError::UnknownType(99)));
    assert_eq!(
        decode(&[6, 5, 0, 0, 0]),
        Err(DecodeError::BadLength {
            expected: 8,
            actual: 5
        })
    );
}
