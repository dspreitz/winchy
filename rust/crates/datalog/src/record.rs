//! Measurement records and their compact little-endian encoding.

/// Event codes of [`Record::Event`].
pub mod event {
    /// Boot; value = ESP reset reason.
    pub const BOOT: u8 = 1;
    /// Load cell tared; value = offset in counts.
    pub const TARE: u8 = 2;
    /// Clock changed; value = [`CLOCK_NTP`], [`CLOCK_GNSS`] or [`CLOCK_PPS`].
    pub const CLOCK: u8 = 3;
    /// [`CLOCK`] value: set from SNTP.
    pub const CLOCK_NTP: i32 = 1;
    /// [`CLOCK`] value: set coarsely from a GNSS solution (off by the UART latency).
    pub const CLOCK_GNSS: i32 = 2;
    /// [`CLOCK`] value: locked to the 1PPS edge (sub-ms); later offsets are exact.
    pub const CLOCK_PPS: i32 = 3;
    /// Records dropped because the logger fell behind; value = count.
    pub const DROPPED: u8 = 4;
}

/// One measurement (raw sensor units, so the data can be replayed).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Record {
    /// IMU raw counts.
    Imu {
        /// Accelerometer counts.
        accel: [i16; 3],
        /// Gyroscope counts (not bias-corrected).
        gyro: [i16; 3],
    },
    /// Load cell raw conversion (untared counts).
    Force {
        /// Counts.
        raw: i32,
    },
    /// Barometer.
    Baro {
        /// Pressure, 0.01 Pa.
        pressure_cpa: u32,
        /// Temperature, 0.01 degC.
        temperature_cdeg: i16,
    },
    /// GNSS solution.
    Gnss {
        /// 0 none, 2 = 2-D, 3 = 3-D.
        fix: u8,
        /// Satellites used.
        sats: u8,
        /// Latitude, 1e-7 deg.
        lat_e7: i32,
        /// Longitude, 1e-7 deg.
        lon_e7: i32,
        /// Height above MSL, mm.
        hmsl_mm: i32,
        /// Horizontal accuracy, cm (saturated).
        hacc_cm: u16,
        /// Ground speed, cm/s.
        gspeed_cms: i16,
        /// Climb rate (up positive), cm/s.
        climb_cms: i16,
        /// UTC of the solution epoch (receiver time), ms; 0 if unresolved.
        utc_ms: i64,
    },
    /// Power.
    Power {
        /// Cell voltage, mV (0 = no cell).
        battery_mv: u16,
        /// Fuel gauge, percent (255 = unknown).
        battery_pct: u8,
        /// System rail, mV.
        system_mv: u16,
        /// Charging.
        charging: bool,
    },
    /// Link report received from the winch.
    Link {
        /// Downlink RSSI at the winch, dBm.
        rssi_dbm: i16,
        /// SNR, 0.25 dB.
        snr_qdb: i8,
        /// Loss, percent.
        loss_pct: u8,
        /// Our TX power at the time, dBm.
        tx_power_dbm: i8,
    },
    /// Discrete event (see [`event`]).
    Event {
        /// Event code.
        code: u8,
        /// Event value.
        value: i32,
    },
}

/// Longest encoded payload.
pub const MAX_PAYLOAD: usize = 28;

impl Record {
    /// Type byte on flash.
    #[must_use]
    pub const fn kind(&self) -> u8 {
        match self {
            Self::Imu { .. } => 1,
            Self::Force { .. } => 2,
            Self::Baro { .. } => 3,
            Self::Gnss { .. } => 4,
            Self::Power { .. } => 5,
            Self::Link { .. } => 6,
            Self::Event { .. } => 7,
        }
    }

    /// Encodes the payload (without type byte) into `out`; returns its length.
    /// `out` must hold [`MAX_PAYLOAD`] bytes.
    pub fn encode_payload(&self, out: &mut [u8; MAX_PAYLOAD]) -> usize {
        let mut w = Out { buf: out, len: 0 };
        match *self {
            Self::Imu { accel, gyro } => {
                for v in accel.into_iter().chain(gyro) {
                    w.put(&v.to_le_bytes());
                }
            }
            Self::Force { raw } => w.put(&raw.to_le_bytes()),
            Self::Baro {
                pressure_cpa,
                temperature_cdeg,
            } => {
                w.put(&pressure_cpa.to_le_bytes());
                w.put(&temperature_cdeg.to_le_bytes());
            }
            Self::Gnss {
                fix,
                sats,
                lat_e7,
                lon_e7,
                hmsl_mm,
                hacc_cm,
                gspeed_cms,
                climb_cms,
                utc_ms,
            } => {
                w.put(&[fix, sats]);
                w.put(&lat_e7.to_le_bytes());
                w.put(&lon_e7.to_le_bytes());
                w.put(&hmsl_mm.to_le_bytes());
                w.put(&hacc_cm.to_le_bytes());
                w.put(&gspeed_cms.to_le_bytes());
                w.put(&climb_cms.to_le_bytes());
                w.put(&utc_ms.to_le_bytes());
            }
            Self::Power {
                battery_mv,
                battery_pct,
                system_mv,
                charging,
            } => {
                w.put(&battery_mv.to_le_bytes());
                w.put(&[battery_pct]);
                w.put(&system_mv.to_le_bytes());
                w.put(&[u8::from(charging)]);
            }
            Self::Link {
                rssi_dbm,
                snr_qdb,
                loss_pct,
                tx_power_dbm,
            } => {
                w.put(&rssi_dbm.to_le_bytes());
                w.put(&snr_qdb.to_le_bytes());
                w.put(&[loss_pct]);
                w.put(&tx_power_dbm.to_le_bytes());
            }
            Self::Event { code, value } => {
                w.put(&[code]);
                w.put(&value.to_le_bytes());
            }
        }
        w.len
    }

    /// Payload length of a record type, None for unknown types.
    #[must_use]
    pub const fn payload_len(kind: u8) -> Option<usize> {
        Some(match kind {
            1 => 12,
            2 => 4,
            3 | 5 => 6,
            4 => 28,
            6 | 7 => 5,
            _ => return None,
        })
    }

    /// Decodes a payload of type `kind`.
    #[must_use]
    pub fn decode(kind: u8, payload: &[u8]) -> Option<Self> {
        if Self::payload_len(kind)? != payload.len() {
            return None;
        }
        let mut r = In(payload);
        Some(match kind {
            1 => {
                let mut v = [0i16; 6];
                for x in &mut v {
                    *x = i16::from_le_bytes(r.take());
                }
                Self::Imu {
                    accel: [v[0], v[1], v[2]],
                    gyro: [v[3], v[4], v[5]],
                }
            }
            2 => Self::Force {
                raw: i32::from_le_bytes(r.take()),
            },
            3 => Self::Baro {
                pressure_cpa: u32::from_le_bytes(r.take()),
                temperature_cdeg: i16::from_le_bytes(r.take()),
            },
            4 => Self::Gnss {
                fix: r.byte(),
                sats: r.byte(),
                lat_e7: i32::from_le_bytes(r.take()),
                lon_e7: i32::from_le_bytes(r.take()),
                hmsl_mm: i32::from_le_bytes(r.take()),
                hacc_cm: u16::from_le_bytes(r.take()),
                gspeed_cms: i16::from_le_bytes(r.take()),
                climb_cms: i16::from_le_bytes(r.take()),
                utc_ms: i64::from_le_bytes(r.take()),
            },
            5 => Self::Power {
                battery_mv: u16::from_le_bytes(r.take()),
                battery_pct: r.byte(),
                system_mv: u16::from_le_bytes(r.take()),
                charging: r.byte() != 0,
            },
            6 => Self::Link {
                rssi_dbm: i16::from_le_bytes(r.take()),
                snr_qdb: i8::from_le_bytes(r.take()),
                loss_pct: r.byte(),
                tx_power_dbm: i8::from_le_bytes(r.take()),
            },
            _ => Self::Event {
                code: r.byte(),
                value: i32::from_le_bytes(r.take()),
            },
        })
    }
}

struct Out<'a> {
    buf: &'a mut [u8; MAX_PAYLOAD],
    len: usize,
}

impl Out<'_> {
    fn put(&mut self, b: &[u8]) {
        self.buf[self.len..self.len + b.len()].copy_from_slice(b);
        self.len += b.len();
    }
}

/// Reader over a payload whose length was checked against `payload_len`.
struct In<'a>(&'a [u8]);

impl In<'_> {
    fn take<const N: usize>(&mut self) -> [u8; N] {
        let mut out = [0u8; N];
        if let Some((head, tail)) = self.0.split_at_checked(N) {
            out.copy_from_slice(head);
            self.0 = tail;
        }
        out
    }

    fn byte(&mut self) -> u8 {
        self.take::<1>()[0]
    }
}

#[cfg(test)]
mod tests {

    use super::*;

    pub(crate) fn samples() -> [Record; 7] {
        [
            Record::Imu {
                accel: [100, -200, 16_384],
                gyro: [-1, 2, i16::MIN],
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
                utc_ms: 1_795_368_488_100,
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
                value: -139_983,
            },
        ]
    }

    #[test]
    fn every_record_round_trips() {
        for r in samples() {
            let mut buf = [0u8; MAX_PAYLOAD];
            let n = r.encode_payload(&mut buf);
            assert_eq!(Some(n), Record::payload_len(r.kind()), "{r:?}");
            assert_eq!(Record::decode(r.kind(), &buf[..n]), Some(r));
        }
    }

    #[test]
    fn kinds_are_distinct_and_stable() {
        let kinds = samples().map(|r| r.kind());
        assert_eq!(kinds, [1, 2, 3, 4, 5, 6, 7]);
    }

    #[test]
    fn compact_sizes() {
        let lens = samples().map(|r| Record::payload_len(r.kind()).unwrap());
        assert_eq!(lens, [12, 4, 6, 28, 6, 5, 5]);
        assert!(lens.iter().all(|&l| l <= MAX_PAYLOAD));
    }

    #[test]
    fn imu_payload_is_little_endian_counts() {
        let mut buf = [0u8; MAX_PAYLOAD];
        let n = samples()[0].encode_payload(&mut buf);
        assert_eq!(&buf[..4], &[100, 0, 0x38, 0xFF]);
        assert_eq!(&buf[10..n], &[0x00, 0x80]);
    }

    #[test]
    fn unknown_type_or_wrong_length_is_rejected() {
        assert_eq!(Record::payload_len(0), None);
        assert_eq!(Record::payload_len(0xFF), None);
        assert_eq!(Record::decode(9, &[0; 4]), None);
        assert_eq!(Record::decode(2, &[0; 3]), None);
    }
}
