//! Data-log partition dump -> CSV, one file per record type.

use winchy_datalog::block::{BLOCK_SIZE, Entry, entries};
use winchy_datalog::record::{Record, event};

/// Valid blocks of a partition dump, oldest first (by sequence number).
#[must_use]
pub fn ordered_blocks(dump: &[u8]) -> Vec<&[u8]> {
    let mut blocks: Vec<(u32, &[u8])> = dump
        .chunks_exact(BLOCK_SIZE)
        .filter_map(|b| entries(b).map(|(h, _)| (h.seq, b)))
        .collect();
    blocks.sort_by_key(|&(seq, _)| seq);
    blocks.into_iter().map(|(_, b)| b).collect()
}

/// Gives entries measured before the clock was set (same boot) their UTC
/// time: the monotonic clock runs on, so the first known UTC offset of that
/// boot applies backwards. A boot event or the monotonic time jumping back by
/// more than a second starts a new boot.
pub fn backfill_utc(entries: &mut [Entry]) {
    let mut start = 0;
    let mut newest = 0u64;
    for i in 0..=entries.len() {
        let new_boot = entries.get(i).is_none_or(|e| {
            let boot = matches!(e.record, Record::Event { code, .. } if code == event::BOOT);
            i > start && (boot || e.mono_us + 1_000_000 < newest)
        });
        if new_boot {
            fill_segment(&mut entries[start..i]);
            start = i;
            newest = 0;
        }
        if let Some(e) = entries.get(i) {
            newest = newest.max(e.mono_us);
        }
    }
}

/// Applies the best offset of one boot to its entries without UTC: the one
/// from the PPS-lock event if there is one, else the first known.
fn fill_segment(seg: &mut [Entry]) {
    let offset_of = |e: &Entry| e.utc_us.map(|u| u - i64::try_from(e.mono_us).unwrap_or(0));
    let pps_locked = |e: &&Entry| {
        matches!(e.record, Record::Event { code, value }
            if code == event::CLOCK && value == event::CLOCK_PPS)
    };
    let Some(offset) = seg
        .iter()
        .find(pps_locked)
        .and_then(offset_of)
        .or_else(|| seg.iter().find_map(offset_of))
    else {
        return;
    };
    for e in seg.iter_mut().take_while(|e| e.utc_us.is_none()) {
        e.utc_us = Some(i64::try_from(e.mono_us).unwrap_or(0) + offset);
    }
}

/// CSV file stem for a record (`imu`, `force`, ...).
#[must_use]
pub fn file_stem(record: &Record) -> &'static str {
    match record {
        Record::Imu { .. } => "imu",
        Record::Force { .. } => "force",
        Record::Baro { .. } => "baro",
        Record::Gnss { .. } => "gnss",
        Record::Power { .. } => "power",
        Record::Link { .. } => "link",
        Record::Event { .. } => "events",
    }
}

/// CSV header line for the file of `record`'s type.
#[must_use]
pub fn csv_header(record: &Record) -> &'static str {
    match record {
        Record::Imu { .. } => "utc,mono_us,ax,ay,az,gx,gy,gz",
        Record::Force { .. } => "utc,mono_us,raw",
        Record::Baro { .. } => "utc,mono_us,pressure_hpa,temperature_c",
        Record::Gnss { .. } => {
            "utc,mono_us,fix,sats,lat,lon,hmsl_m,hacc_m,gspeed_ms,climb_ms,gnss_utc"
        }
        Record::Power { .. } => "utc,mono_us,battery_mv,battery_pct,system_mv,charging",
        Record::Link { .. } => "utc,mono_us,rssi_dbm,snr_db,loss_pct,tx_power_dbm",
        Record::Event { .. } => "utc,mono_us,code,value",
    }
}

/// One CSV row: `utc` (ISO 8601 with µs, empty if unknown), `mono_us`, fields.
#[must_use]
pub fn csv_row(e: &Entry) -> String {
    let utc = e.utc_us.map(iso8601_us).unwrap_or_default();
    let fields = match e.record {
        Record::Imu { accel: a, gyro: g } => {
            format!("{},{},{},{},{},{}", a[0], a[1], a[2], g[0], g[1], g[2])
        }
        Record::Force { raw } => raw.to_string(),
        Record::Baro {
            pressure_cpa,
            temperature_cdeg,
        } => format!(
            "{:.5},{:.2}",
            f64::from(pressure_cpa) / 10_000.0,
            f64::from(temperature_cdeg) / 100.0
        ),
        Record::Gnss {
            fix,
            sats,
            lat_e7,
            lon_e7,
            hmsl_mm,
            hacc_cm,
            gspeed_cms,
            climb_cms,
            utc_ms,
        } => format!(
            "{fix},{sats},{:.7},{:.7},{:.3},{:.2},{:.2},{:.2},{}",
            f64::from(lat_e7) * 1e-7,
            f64::from(lon_e7) * 1e-7,
            f64::from(hmsl_mm) / 1_000.0,
            f64::from(hacc_cm) / 100.0,
            f64::from(gspeed_cms) / 100.0,
            f64::from(climb_cms) / 100.0,
            if utc_ms == 0 {
                String::new()
            } else {
                iso8601_us(utc_ms * 1_000)
            }
        ),
        Record::Power {
            battery_mv,
            battery_pct,
            system_mv,
            charging,
        } => format!(
            "{battery_mv},{battery_pct},{system_mv},{}",
            u8::from(charging)
        ),
        Record::Link {
            rssi_dbm,
            snr_qdb,
            loss_pct,
            tx_power_dbm,
        } => format!(
            "{rssi_dbm},{:.2},{loss_pct},{tx_power_dbm}",
            f64::from(snr_qdb) / 4.0
        ),
        Record::Event { code, value } => format!("{code},{value}"),
    };
    format!("{utc},{},{fields}", e.mono_us)
}

/// ISO 8601 UTC with microseconds, e.g. `2026-11-22T17:28:49.123456Z`.
#[must_use]
pub fn iso8601_us(unix_us: i64) -> String {
    let secs = unix_us.div_euclid(1_000_000);
    let micros = unix_us.rem_euclid(1_000_000);
    let (days, sod) = (secs.div_euclid(86_400), secs.rem_euclid(86_400));
    // Civil from days (H. Hinnant).
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1_460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    format!(
        "{year:04}-{month:02}-{day:02}T{:02}:{:02}:{:02}.{micros:06}Z",
        sod / 3_600,
        sod / 60 % 60,
        sod % 60
    )
}

#[cfg(test)]
mod tests {
    use winchy_datalog::block::BlockWriter;

    use super::*;

    fn block(seq: u32, mono: u64) -> Vec<u8> {
        let mut w = BlockWriter::new();
        w.push(mono, Some(0), &Record::Force { raw: 1 });
        w.seal(seq).unwrap().to_vec()
    }

    #[test]
    fn blocks_are_ordered_by_sequence_and_junk_skipped() {
        let mut dump = block(8, 800);
        dump.extend(vec![0xFF; BLOCK_SIZE]);
        dump.extend(block(7, 700));
        dump.extend(vec![0x00; 100]); // truncated tail
        let blocks = ordered_blocks(&dump);
        let firsts: Vec<u64> = blocks
            .iter()
            .map(|b| entries(b).unwrap().1.next().unwrap().mono_us)
            .collect();
        assert_eq!(firsts, [700, 800]);
    }

    #[test]
    fn iso_timestamps_keep_microseconds() {
        assert_eq!(iso8601_us(0), "1970-01-01T00:00:00.000000Z");
        assert_eq!(
            iso8601_us(1_795_368_529_123_456),
            "2026-11-22T17:28:49.123456Z"
        );
        assert_eq!(
            iso8601_us(951_782_400_000_001),
            "2000-02-29T00:00:00.000001Z"
        );
    }

    #[test]
    fn rows_match_headers() {
        let records = [
            Record::Imu {
                accel: [1, -2, 3],
                gyro: [4, 5, -6],
            },
            Record::Force { raw: -139_983 },
            Record::Baro {
                pressure_cpa: 9_672_345,
                temperature_cdeg: -1_234,
            },
            Record::Gnss {
                fix: 3,
                sats: 17,
                lat_e7: 487_467_300,
                lon_e7: 114_492_470,
                hmsl_mm: 362_123,
                hacc_cm: 65,
                gspeed_cms: 12,
                climb_cms: -3,
                utc_ms: 1_795_368_529_100,
            },
            Record::Power {
                battery_mv: 3_987,
                battery_pct: 87,
                system_mv: 5_012,
                charging: true,
            },
            Record::Link {
                rssi_dbm: -97,
                snr_qdb: -6,
                loss_pct: 12,
                tx_power_dbm: -9,
            },
            Record::Event {
                code: event::TARE,
                value: -5,
            },
        ];
        let expected = [
            ("imu", "utc,mono_us,ax,ay,az,gx,gy,gz", "1,-2,3,4,5,-6"),
            ("force", "utc,mono_us,raw", "-139983"),
            (
                "baro",
                "utc,mono_us,pressure_hpa,temperature_c",
                "967.23450,-12.34",
            ),
            (
                "gnss",
                "utc,mono_us,fix,sats,lat,lon,hmsl_m,hacc_m,gspeed_ms,climb_ms,gnss_utc",
                "3,17,48.7467300,11.4492470,362.123,0.65,0.12,-0.03,2026-11-22T17:28:49.100000Z",
            ),
            (
                "power",
                "utc,mono_us,battery_mv,battery_pct,system_mv,charging",
                "3987,87,5012,1",
            ),
            (
                "link",
                "utc,mono_us,rssi_dbm,snr_db,loss_pct,tx_power_dbm",
                "-97,-1.50,12,-9",
            ),
            ("events", "utc,mono_us,code,value", "2,-5"),
        ];
        for (r, (stem, header, fields)) in records.iter().zip(expected) {
            assert_eq!(file_stem(r), stem);
            assert_eq!(csv_header(r), header);
            let e = Entry {
                mono_us: 5_000_000,
                utc_us: Some(1_795_368_529_000_001),
                record: *r,
            };
            assert_eq!(
                csv_row(&e),
                format!("2026-11-22T17:28:49.000001Z,5000000,{fields}")
            );
            assert_eq!(
                csv_header(r).split(',').count(),
                csv_row(&e).split(',').count()
            );
        }
    }

    fn entry(mono_us: u64, utc_us: Option<i64>, record: Record) -> Entry {
        Entry {
            mono_us,
            utc_us,
            record,
        }
    }

    #[test]
    fn early_entries_get_the_first_offset_of_their_boot() {
        let force = Record::Force { raw: 0 };
        let boot = Record::Event {
            code: event::BOOT,
            value: 1,
        };
        let mut e = vec![
            entry(100, None, boot),
            entry(1_000, None, force),
            entry(2_000, Some(1_000_002_000), force),
            entry(3_000, Some(1_000_003_001), force),
            // reboot: new boot, no clock yet, then a different offset
            entry(50, None, boot),
            entry(500, None, force),
            entry(900, Some(5_000_000_900), force),
            // reset without a boot event: monotonic time jumps back
            entry(10, None, force),
        ];
        backfill_utc(&mut e);
        let utc: Vec<Option<i64>> = e.iter().map(|x| x.utc_us).collect();
        assert_eq!(
            utc,
            [
                Some(1_000_000_100),
                Some(1_000_001_000),
                Some(1_000_002_000),
                Some(1_000_003_001),
                Some(5_000_000_050),
                Some(5_000_000_500),
                Some(5_000_000_900),
                None,
            ]
        );
    }

    #[test]
    fn the_pps_locked_offset_is_preferred_over_the_coarse_one() {
        let force = Record::Force { raw: 0 };
        let pps = Record::Event {
            code: event::CLOCK,
            value: event::CLOCK_PPS,
        };
        let mut e = vec![
            entry(1_000, None, force),
            entry(2_000, Some(1_000_002_068), force), // coarse GNSS set, 68 us late
            entry(3_000, Some(1_000_003_000), pps),
            entry(4_000, Some(1_000_004_000), force),
        ];
        backfill_utc(&mut e);
        assert_eq!(e[0].utc_us, Some(1_000_001_000));
        assert_eq!(e[1].utc_us, Some(1_000_002_068), "measured times are kept");
    }

    #[test]
    fn small_backwards_steps_between_tasks_stay_in_the_same_boot() {
        let force = Record::Force { raw: 0 };
        let mut e = vec![
            entry(2_000_000, None, force),
            entry(1_999_000, None, force),
            entry(2_100_000, Some(10_002_100_000), force),
        ];
        backfill_utc(&mut e);
        assert_eq!(e[1].utc_us, Some(10_001_999_000));
    }

    #[test]
    fn unknown_times_are_empty() {
        let e = Entry {
            mono_us: 7,
            utc_us: None,
            record: Record::Force { raw: 1 },
        };
        assert_eq!(csv_row(&e), ",7,1");
        let g = Entry {
            record: Record::Gnss {
                fix: 0,
                sats: 0,
                lat_e7: 0,
                lon_e7: 0,
                hmsl_mm: 0,
                hacc_cm: 0,
                gspeed_cms: 0,
                climb_cms: 0,
                utc_ms: 0,
            },
            ..e
        };
        assert!(csv_row(&g).ends_with(','), "unresolved receiver time");
    }
}
