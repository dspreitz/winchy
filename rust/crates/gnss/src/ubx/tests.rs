use super::*;

/// Builds a NAV-PVT payload with known fields (u-blox M10 interface offsets).
#[allow(clippy::too_many_arguments)]
pub(crate) fn make_navpvt(fix_type: u8, gnss_ok: bool, resolved: bool) -> [u8; 92] {
    let mut pl = [0u8; 92];
    pl[4..6].copy_from_slice(&2026u16.to_le_bytes());
    pl[6..11].copy_from_slice(&[6, 27, 16, 45, 10]);
    pl[11] = 0x01 | 0x02 | if resolved { 0x04 } else { 0 };
    pl[12..16].copy_from_slice(&25u32.to_le_bytes());
    pl[16..20].copy_from_slice(&(-123_456i32).to_le_bytes()); // nano
    pl[20] = fix_type;
    pl[21] = u8::from(gnss_ok);
    pl[23] = 12;
    pl[24..28].copy_from_slice(&115_000_000i32.to_le_bytes());
    pl[28..32].copy_from_slice(&485_000_000i32.to_le_bytes());
    pl[36..40].copy_from_slice(&512_345i32.to_le_bytes());
    pl[40..44].copy_from_slice(&1500u32.to_le_bytes());
    pl[44..48].copy_from_slice(&2500u32.to_le_bytes());
    pl[56..60].copy_from_slice(&(-300i32).to_le_bytes());
    pl[60..64].copy_from_slice(&28_400i32.to_le_bytes());
    pl[68..72].copy_from_slice(&500u32.to_le_bytes());
    pl[76..78].copy_from_slice(&123u16.to_le_bytes());
    pl
}

fn frame_vec(class: u8, id: u8, payload: &[u8]) -> Vec<u8> {
    let mut out = vec![0u8; payload.len() + OVERHEAD];
    let n = write_frame(class, id, payload, &mut out).unwrap();
    out.truncate(n);
    out
}

/// Feeds bytes, returns every packet as owned (class, id, payload).
fn feed(f: &mut Framer, bytes: &[u8]) -> Vec<(u8, u8, Vec<u8>)> {
    bytes
        .iter()
        .filter_map(|&b| f.push(b).map(|p| (p.class, p.id, p.payload.to_vec())))
        .collect()
}

#[test]
fn checksum_known_value() {
    // Class 06 id 8A len 0: A = 0x90, B = 06 -> 96 -> 26 -> B6.
    assert_eq!(checksum(&[0x06, 0x8A, 0x00, 0x00]), [0x90, 0xB6]);
}

#[test]
fn write_frame_layout() {
    let f = frame_vec(0x01, 0x07, &[1, 2, 3]);
    assert_eq!(&f[..6], &[0xB5, 0x62, 0x01, 0x07, 3, 0]);
    assert_eq!(&f[6..9], &[1, 2, 3]);
    assert_eq!(f[9..], checksum(&f[2..9]));
}

#[test]
fn write_frame_rejects_small_buffer() {
    let mut out = [0u8; 10];
    assert_eq!(write_frame(1, 7, &[0; 3], &mut out), Err(BufferTooSmall));
}

#[test]
fn framer_extracts_navpvt() {
    let frame = frame_vec(0x01, 0x07, &make_navpvt(3, true, true));
    let got = feed(&mut Framer::new(), &frame);
    assert_eq!(got.len(), 1);
    assert_eq!((got[0].0, got[0].1), msg::NAV_PVT);
    assert_eq!(got[0].2.len(), 92);
}

#[test]
fn framer_resyncs_past_garbage_and_split_reads() {
    let frame = frame_vec(0x01, 0x07, &make_navpvt(3, true, true));
    let mut stream = b"\x00\xb5garbage\xb5".to_vec();
    stream.extend_from_slice(&frame);
    stream.extend_from_slice(&frame);
    let mut f = Framer::new();
    let (a, b) = stream.split_at(37);
    let mut got = feed(&mut f, a);
    got.extend(feed(&mut f, b));
    assert_eq!(got.len(), 2);
}

#[test]
fn framer_drops_bad_checksum_then_returns_good() {
    let good = frame_vec(0x01, 0x07, &make_navpvt(3, true, true));
    let mut bad = good.clone();
    *bad.last_mut().unwrap() ^= 0xFF;
    let mut stream = bad;
    stream.extend_from_slice(&good);
    assert_eq!(feed(&mut Framer::new(), &stream).len(), 1);
}

#[test]
fn framer_skips_bogus_length() {
    let good = frame_vec(0x05, 0x01, &[0x06, 0x8A]);
    let mut stream = vec![0xB5, 0x62, 0x01, 0x07, 0xFF, 0xFF];
    stream.extend_from_slice(&good);
    let got = feed(&mut Framer::new(), &stream);
    assert_eq!(got, vec![(0x05, 0x01, vec![0x06, 0x8A])]);
}

#[test]
fn framer_handles_zero_length_payload() {
    let got = feed(&mut Framer::new(), &frame_vec(0x0A, 0x04, &[]));
    assert_eq!(got, vec![(0x0A, 0x04, vec![])]);
}

#[test]
fn navpvt_3d_fix_fields() {
    let p = NavPvt::parse(&make_navpvt(3, true, true)).unwrap();
    assert_eq!(p.fix, FixType::Fix3D);
    assert_eq!(p.num_sv, 12);
    assert!((p.lat_deg() - 48.5).abs() < 1e-9);
    assert!((p.lon_deg() - 11.5).abs() < 1e-9);
    assert_eq!(p.hmsl_mm, 512_345);
    assert_eq!((p.hacc_mm, p.vacc_mm, p.sacc_mms), (1500, 2500, 500));
    assert_eq!(p.gspeed_mms, 28_400);
    assert!((p.climb_ms() - 0.3).abs() < 1e-6);
    assert_eq!(p.pdop_e2, 123);
    assert_eq!(p.tacc_ns, 25);
    assert_eq!(p.nano_ns, -123_456);
    assert_eq!(
        p.utc,
        Some(UtcDateTime {
            year: 2026,
            month: 6,
            day: 27,
            hour: 16,
            minute: 45,
            second: 10
        })
    );
}

#[test]
fn navpvt_no_fix_when_gnssfixok_clear() {
    assert_eq!(
        NavPvt::parse(&make_navpvt(3, false, true)).unwrap().fix,
        FixType::None
    );
    assert_eq!(
        NavPvt::parse(&make_navpvt(2, true, true)).unwrap().fix,
        FixType::Fix2D
    );
    assert_eq!(
        NavPvt::parse(&make_navpvt(5, true, true)).unwrap().fix,
        FixType::None
    );
}

#[test]
fn navpvt_utc_gated_on_fully_resolved() {
    assert_eq!(
        NavPvt::parse(&make_navpvt(3, true, false)).unwrap().utc,
        None
    );
}

#[test]
fn navpvt_short_payload_rejected() {
    assert_eq!(NavPvt::parse(&[0u8; 91]), None);
}

/// Decodes a VALSET frame into (layers, [(key, value bytes)]).
fn decode_valset(frame: &[u8]) -> (u8, Vec<(u32, Vec<u8>)>) {
    assert_eq!(&frame[..4], &[0xB5, 0x62, 0x06, 0x8A]);
    let len = usize::from(u16::from_le_bytes([frame[4], frame[5]]));
    let p = &frame[6..6 + len];
    assert_eq!(frame[6 + len..], checksum(&frame[2..6 + len]));
    let mut items = vec![];
    let mut i = 4;
    while i < p.len() {
        let key = u32::from_le_bytes(p[i..i + 4].try_into().unwrap());
        i += 4;
        let size = match (key >> 28) & 7 {
            1 | 2 => 1,
            3 => 2,
            4 => 4,
            other => panic!("size nibble {other}"),
        };
        items.push((key, p[i..i + size].to_vec()));
        i += size;
    }
    (p[1], items)
}

fn valset_vec(layers: u8, items: &[(u32, CfgValue)]) -> Vec<u8> {
    let mut out = vec![0u8; 256];
    let n = write_valset(layers, items, &mut out).unwrap();
    out.truncate(n);
    out
}

#[test]
fn valset_encodes_keys_and_layers() {
    let f = valset_vec(
        layer::RAM | layer::BBR,
        &[
            (key::UART1_BAUDRATE, CfgValue::U32(115_200)),
            (key::RATE_MEAS, CfgValue::U16(200)),
        ],
    );
    let (layers, items) = decode_valset(&f);
    assert_eq!(layers, 0x03);
    assert_eq!(
        items,
        vec![
            (key::UART1_BAUDRATE, 115_200u32.to_le_bytes().to_vec()),
            (key::RATE_MEAS, 200u16.to_le_bytes().to_vec()),
        ]
    );
}

#[test]
fn valset_rejects_size_mismatch() {
    let mut out = [0u8; 64];
    assert_eq!(
        write_valset(1, &[(key::RATE_MEAS, CfgValue::U8(1))], &mut out),
        Err(ValsetError::SizeMismatch(key::RATE_MEAS))
    );
}

#[test]
fn valset_rejects_small_buffer() {
    let mut out = [0u8; 12];
    assert_eq!(
        write_valset(1, &[(key::UART1_BAUDRATE, CfgValue::U32(9600))], &mut out),
        Err(ValsetError::BufferTooSmall)
    );
}

#[test]
fn receiver_config_navpvt_on_all_nmea_off() {
    let (_, items) = decode_valset(&valset_vec(3, &receiver_config(5)));
    let get = |k: u32| {
        items
            .iter()
            .find(|(key, _)| *key == k)
            .map(|(_, v)| v.clone())
    };
    assert_eq!(get(key::MSGOUT_NAV_PVT_UART1), Some(vec![1]));
    for k in key::MSGOUT_NMEA_UART1 {
        assert_eq!(get(k), Some(vec![0]), "{k:#x}");
    }
    assert_eq!(get(key::DYNMODEL), Some(vec![DYN_AIRBORNE_2G]));
    for k in [key::SIG_GPS, key::SIG_SBAS, key::SIG_GAL, key::SIG_BDS] {
        assert_eq!(get(k), Some(vec![1]));
    }
    assert_eq!(get(key::RATE_MEAS), Some(200u16.to_le_bytes().to_vec()));
    assert_eq!(get(key::RATE_NAV), Some(1u16.to_le_bytes().to_vec()));
}

#[test]
fn receiver_config_clamps_rate() {
    let meas = |hz| {
        receiver_config(hz)
            .iter()
            .find(|(k, _)| *k == key::RATE_MEAS)
            .map(|(_, v)| *v)
            .unwrap()
    };
    assert_eq!(meas(50), CfgValue::U16(50));
    assert_eq!(meas(0), CfgValue::U16(1000));
    assert_eq!(meas(1), CfgValue::U16(1000));
}

#[test]
fn mga_ini_time_utc_well_formed() {
    let t = UtcDateTime {
        year: 2026,
        month: 6,
        day: 27,
        hour: 16,
        minute: 45,
        second: 10,
    };
    let mut out = [0u8; 40];
    let n = write_mga_ini_time_utc(t, 2, &mut out).unwrap();
    assert_eq!(n, MGA_INI_TIME_UTC_LEN + OVERHEAD);
    assert_eq!(&out[2..6], &[0x13, 0x40, 24, 0]);
    let p = &out[6..30];
    assert_eq!(p[0], 0x10); // type TIME_UTC
    assert_eq!(p[3], 0x80); // leap seconds unknown
    assert_eq!(u16::from_le_bytes([p[4], p[5]]), 2026);
    assert_eq!(&p[6..11], &[6, 27, 16, 45, 10]);
    assert_eq!(u16::from_le_bytes([p[16], p[17]]), 2);
    assert_eq!(out[30..32], checksum(&out[2..30]));
}
