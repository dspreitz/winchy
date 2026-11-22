use super::*;
use crate::record::event;

const IMU: Record = Record::Imu {
    accel: [1, 2, 3],
    gyro: [4, 5, 6],
};
const FORCE: Record = Record::Force { raw: -5 };

fn sealed(w: &mut BlockWriter, seq: u32) -> [u8; BLOCK_SIZE] {
    *w.seal(seq).unwrap()
}

#[test]
fn crc32_matches_zlib() {
    assert_eq!(crc32(&[b"123456789"]), 0xCBF4_3926);
    assert_eq!(
        crc32(&[b"1234", b"56789"]),
        0xCBF4_3926,
        "parts are concatenated"
    );
    assert_eq!(crc32(&[]), 0);
}

#[test]
fn records_round_trip_with_exact_times() {
    let mut w = BlockWriter::new();
    assert!(w.is_empty());
    let off = Some(1_795_368_000_000_000);
    assert!(w.push(5_000_000, off, &IMU));
    assert!(w.push(5_020_123, off, &FORCE));
    assert!(!w.is_empty());
    let block = sealed(&mut w, 42);
    assert!(w.is_empty());

    let (h, it) = entries(&block).unwrap();
    assert_eq!(h.seq, 42);
    assert_eq!(h.base_mono_us, 5_000_000);
    assert_eq!(h.utc_offset_us, off);
    assert_eq!(usize::from(h.used), 2 * RECORD_OVERHEAD + 12 + 4);
    let got: Vec<Entry> = it.collect();
    assert_eq!(
        got,
        [
            Entry {
                mono_us: 5_000_000,
                utc_us: Some(1_795_368_005_000_000),
                record: IMU
            },
            Entry {
                mono_us: 5_020_123,
                utc_us: Some(1_795_368_005_020_123),
                record: FORCE
            },
        ]
    );
}

#[test]
fn unknown_clock_gives_no_utc() {
    let mut w = BlockWriter::new();
    w.push(1, None, &FORCE);
    let block = sealed(&mut w, 0);
    let e = entries(&block).unwrap().1.next().unwrap();
    assert_eq!(e.utc_us, None);
}

#[test]
fn slightly_earlier_records_from_other_tasks_are_fine() {
    let mut w = BlockWriter::new();
    assert!(w.push(10_000, None, &IMU));
    assert!(w.push(9_000, None, &FORCE));
    let block = sealed(&mut w, 0);
    let times: Vec<u64> = entries(&block).unwrap().1.map(|e| e.mono_us).collect();
    assert_eq!(times, [10_000, 9_000]);
}

#[test]
fn full_block_refuses_until_sealed() {
    let mut w = BlockWriter::new();
    let mut n = 0u64;
    while w.push(n * 20_000, None, &IMU) {
        n += 1;
    }
    assert_eq!(
        n as usize,
        (BLOCK_SIZE - HEADER_SIZE) / (RECORD_OVERHEAD + 12)
    );
    let block = sealed(&mut w, 1);
    assert_eq!(entries(&block).unwrap().1.count() as u64, n);
    assert!(w.push(n * 20_000, None, &IMU), "fresh block after seal");
}

#[test]
fn clock_step_starts_a_new_block() {
    let mut w = BlockWriter::new();
    assert!(w.push(0, Some(1_000_000), &FORCE));
    assert!(
        w.push(1, Some(1_000_000 + OFFSET_TOLERANCE_US), &FORCE),
        "within tolerance"
    );
    assert!(!w.push(2, Some(1_000_000 + OFFSET_TOLERANCE_US + 1), &FORCE));
    assert!(!w.push(2, None, &FORCE), "clock lost");
    let mut unset = BlockWriter::new();
    unset.push(0, None, &FORCE);
    assert!(!unset.push(1, Some(0), &FORCE), "clock set");
}

#[test]
fn time_delta_out_of_range_needs_a_new_block() {
    let mut w = BlockWriter::new();
    assert!(w.push(0, None, &FORCE));
    assert!(w.push(i32::MAX as u64, None, &FORCE));
    assert!(!w.push(i32::MAX as u64 + 1, None, &FORCE));
}

#[test]
fn seal_of_an_empty_writer_is_none() {
    assert!(BlockWriter::new().seal(0).is_none());
}

#[test]
fn unused_tail_stays_erased() {
    let mut w = BlockWriter::new();
    w.push(0, None, &FORCE);
    let block = sealed(&mut w, 0);
    assert!(
        block[HEADER_SIZE + RECORD_OVERHEAD + 4..]
            .iter()
            .all(|&b| b == 0xFF)
    );
}

#[test]
fn erased_foreign_and_torn_blocks_are_rejected() {
    assert!(entries(&[0xFF; BLOCK_SIZE]).is_none());
    assert_eq!(BlockHeader::peek_seq(&[0xFF; HEADER_SIZE]), None);
    let mut w = BlockWriter::new();
    w.push(
        0,
        None,
        &Record::Event {
            code: event::BOOT,
            value: 1,
        },
    );
    let good = sealed(&mut w, 7);
    assert_eq!(BlockHeader::peek_seq(&good), Some(7));
    let mut torn = good;
    torn[HEADER_SIZE + 2] ^= 0x01;
    assert!(entries(&torn).is_none(), "payload bit flip");
    let mut bad_header = good;
    bad_header[8] ^= 0x01;
    assert!(entries(&bad_header).is_none(), "header bit flip");
    assert!(entries(&good[..HEADER_SIZE]).is_none(), "too short");
}
