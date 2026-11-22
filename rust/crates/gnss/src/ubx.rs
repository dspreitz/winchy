//! u-blox UBX binary protocol (M10 config database interface).

/// UBX sync characters.
pub const SYNC: [u8; 2] = [0xB5, 0x62];
/// Largest payload the framer accepts; longer lengths are treated as noise.
pub const MAX_PAYLOAD: usize = 512;
/// Sync + class + id + length + checksum.
pub const OVERHEAD: usize = 8;

/// Message class/id pairs used by Winchy.
pub mod msg {
    /// UBX-NAV-PVT.
    pub const NAV_PVT: (u8, u8) = (0x01, 0x07);
    /// UBX-CFG-VALSET.
    pub const CFG_VALSET: (u8, u8) = (0x06, 0x8A);
    /// UBX-MGA-INI.
    pub const MGA_INI: (u8, u8) = (0x13, 0x40);
    /// UBX-SEC-UNIQID (receiver unique id, for AssistNow ZTP).
    pub const SEC_UNIQID: (u8, u8) = (0x27, 0x03);
    /// UBX-MON-VER (firmware/hardware version, for AssistNow ZTP).
    pub const MON_VER: (u8, u8) = (0x0A, 0x04);
}

/// Output buffer too small for the message.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BufferTooSmall;

/// 8-bit Fletcher checksum over class..payload.
#[must_use]
pub fn checksum(body: &[u8]) -> [u8; 2] {
    let (mut a, mut b) = (0u8, 0u8);
    for &x in body {
        a = a.wrapping_add(x);
        b = b.wrapping_add(a);
    }
    [a, b]
}

/// Writes a complete UBX frame into `out`, returns its length.
///
/// # Errors
/// [`BufferTooSmall`] if `out` cannot hold `payload.len() + 8` bytes or the
/// payload exceeds [`MAX_PAYLOAD`].
pub fn write_frame(
    class: u8,
    id: u8,
    payload: &[u8],
    out: &mut [u8],
) -> Result<usize, BufferTooSmall> {
    let n = payload.len();
    if n > MAX_PAYLOAD || out.len() < n + OVERHEAD {
        return Err(BufferTooSmall);
    }
    let len = (n as u16).to_le_bytes();
    out[..6].copy_from_slice(&[SYNC[0], SYNC[1], class, id, len[0], len[1]]);
    out[6..6 + n].copy_from_slice(payload);
    let ck = checksum(&out[2..6 + n]);
    out[6 + n..n + OVERHEAD].copy_from_slice(&ck);
    Ok(n + OVERHEAD)
}

/// A complete, checksum-valid UBX message borrowed from a [`Framer`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Packet<'a> {
    /// Message class.
    pub class: u8,
    /// Message id.
    pub id: u8,
    /// Payload bytes.
    pub payload: &'a [u8],
}

/// Streaming UBX framer: feed received bytes one at a time.
///
/// Resyncs past noise, drops frames with a bad checksum or an implausible
/// length. Holds one frame in a fixed buffer, no allocation.
#[derive(Debug, Clone)]
pub struct Framer {
    buf: [u8; MAX_PAYLOAD + 6],
    len: usize,
    need: usize,
    state: State,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Sync1,
    Sync2,
    Header,
    Body,
}

impl Framer {
    /// Creates an empty framer.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            buf: [0; MAX_PAYLOAD + 6],
            len: 0,
            need: 0,
            state: State::Sync1,
        }
    }

    /// Feeds one byte; returns a packet when it completes a valid frame.
    pub fn push(&mut self, byte: u8) -> Option<Packet<'_>> {
        match self.state {
            State::Sync1 => {
                if byte == SYNC[0] {
                    self.state = State::Sync2;
                }
            }
            State::Sync2 => {
                self.state = match byte {
                    b if b == SYNC[1] => {
                        self.len = 0;
                        State::Header
                    }
                    b if b == SYNC[0] => State::Sync2,
                    _ => State::Sync1,
                };
            }
            State::Header => {
                self.buf[self.len] = byte;
                self.len += 1;
                if self.len == 4 {
                    let payload = usize::from(u16::from_le_bytes([self.buf[2], self.buf[3]]));
                    if payload > MAX_PAYLOAD {
                        self.state = State::Sync1; // implausible length: noise
                    } else {
                        self.need = 4 + payload + 2;
                        self.state = State::Body;
                    }
                }
            }
            State::Body => {
                // class, id, len(2), payload, ck_a, ck_b - all fit in `buf`.
                self.buf[self.len] = byte;
                self.len += 1;
                if self.len == self.need {
                    self.state = State::Sync1;
                    let end = self.need - 2;
                    if checksum(&self.buf[..end]) == [self.buf[end], self.buf[end + 1]] {
                        return Some(Packet {
                            class: self.buf[0],
                            id: self.buf[1],
                            payload: &self.buf[4..end],
                        });
                    }
                }
            }
        }
        None
    }
}

impl Default for Framer {
    fn default() -> Self {
        Self::new()
    }
}

pub use crate::time::UtcDateTime;

/// Fix quality from NAV-PVT, gated on the `gnssFixOK` flag.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FixType {
    /// No (trusted) fix.
    None,
    /// 2-D fix.
    Fix2D,
    /// 3-D fix.
    Fix3D,
}

/// Decoded UBX-NAV-PVT (position, velocity, time). Integer units as on the
/// wire so no precision is lost; helpers convert.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NavPvt {
    /// Fix quality (None unless `gnssFixOK`).
    pub fix: FixType,
    /// Satellites used.
    pub num_sv: u8,
    /// Latitude, 1e-7 degrees.
    pub lat_e7: i32,
    /// Longitude, 1e-7 degrees.
    pub lon_e7: i32,
    /// Height above mean sea level, mm.
    pub hmsl_mm: i32,
    /// Horizontal accuracy estimate, mm.
    pub hacc_mm: u32,
    /// Vertical accuracy estimate, mm.
    pub vacc_mm: u32,
    /// Down velocity, mm/s (positive down).
    pub vel_d_mms: i32,
    /// 2-D ground speed, mm/s.
    pub gspeed_mms: i32,
    /// Speed accuracy estimate, mm/s.
    pub sacc_mms: u32,
    /// Position DOP, 0.01.
    pub pdop_e2: u16,
    /// UTC time, only when the receiver reports it fully resolved.
    pub utc: Option<UtcDateTime>,
    /// Time accuracy estimate, ns.
    pub tacc_ns: u32,
    /// Fraction of the UTC second, ns (-1e9..=1e9; may be negative).
    pub nano_ns: i32,
}

impl NavPvt {
    /// Minimum NAV-PVT payload length.
    pub const LEN: usize = 92;

    /// Parses a NAV-PVT payload; `None` if too short.
    #[must_use]
    pub fn parse(payload: &[u8]) -> Option<Self> {
        if payload.len() < Self::LEN {
            return None;
        }
        let u16_at = |o: usize| u16::from_le_bytes([payload[o], payload[o + 1]]);
        let u32_at = |o: usize| {
            u32::from_le_bytes([payload[o], payload[o + 1], payload[o + 2], payload[o + 3]])
        };
        let i32_at = |o: usize| {
            i32::from_le_bytes([payload[o], payload[o + 1], payload[o + 2], payload[o + 3]])
        };
        let gnss_fix_ok = payload[21] & 0x01 != 0;
        let fix = match (gnss_fix_ok, payload[20]) {
            (true, 2) => FixType::Fix2D,
            (true, 3) => FixType::Fix3D,
            _ => FixType::None,
        };
        let fully_resolved = payload[11] & 0x04 != 0;
        let utc = fully_resolved.then(|| UtcDateTime {
            year: u16_at(4),
            month: payload[6],
            day: payload[7],
            hour: payload[8],
            minute: payload[9],
            second: payload[10],
        });
        Some(Self {
            fix,
            num_sv: payload[23],
            lon_e7: i32_at(24),
            lat_e7: i32_at(28),
            hmsl_mm: i32_at(36),
            hacc_mm: u32_at(40),
            vacc_mm: u32_at(44),
            vel_d_mms: i32_at(56),
            gspeed_mms: i32_at(60),
            sacc_mms: u32_at(68),
            pdop_e2: u16_at(76),
            utc,
            tacc_ns: u32_at(12),
            nano_ns: i32_at(16),
        })
    }

    /// Latitude in degrees.
    #[must_use]
    pub fn lat_deg(&self) -> f64 {
        f64::from(self.lat_e7) * 1e-7
    }

    /// Longitude in degrees.
    #[must_use]
    pub fn lon_deg(&self) -> f64 {
        f64::from(self.lon_e7) * 1e-7
    }

    /// Climb rate in m/s (positive up).
    #[must_use]
    pub fn climb_ms(&self) -> f32 {
        -(self.vel_d_mms as f32) / 1000.0
    }
}

/// Configuration layers for CFG-VALSET.
pub mod layer {
    /// Apply now.
    pub const RAM: u8 = 0x01;
    /// Battery-backed RAM: survives power cycles while the backup rail holds.
    pub const BBR: u8 = 0x02;
}

/// A value for one config-database key. Its size must match the key's size
/// nibble (bits 28..30): 1 = bit, 2 = byte, 3 = u16, 4 = u32.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CfgValue {
    /// L / U1 / E1 (one byte on the wire).
    U8(u8),
    /// U2.
    U16(u16),
    /// U4.
    U32(u32),
}

/// CFG-VALSET encoding error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ValsetError {
    /// Output buffer too small.
    BufferTooSmall,
    /// Value size does not match the key's size nibble.
    SizeMismatch(u32),
}

/// Writes a UBX-CFG-VALSET frame for `items` into `out`.
///
/// # Errors
/// [`ValsetError`] on a too-small buffer or a key/value size mismatch.
pub fn write_valset(
    layers: u8,
    items: &[(u32, CfgValue)],
    out: &mut [u8],
) -> Result<usize, ValsetError> {
    // Payload: version, layers, 2 reserved, then key/value pairs.
    let mut payload = [0u8; MAX_PAYLOAD];
    payload[1] = layers;
    let mut n = 4;
    for &(k, v) in items {
        let size_nibble = (k >> 28) & 0x7;
        let mut bytes = [0u8; 4];
        let size = match (size_nibble, v) {
            (1 | 2, CfgValue::U8(x)) => {
                bytes[0] = x;
                1
            }
            (3, CfgValue::U16(x)) => {
                bytes[..2].copy_from_slice(&x.to_le_bytes());
                2
            }
            (4, CfgValue::U32(x)) => {
                bytes.copy_from_slice(&x.to_le_bytes());
                4
            }
            _ => return Err(ValsetError::SizeMismatch(k)),
        };
        if n + 4 + size > MAX_PAYLOAD {
            return Err(ValsetError::BufferTooSmall);
        }
        payload[n..n + 4].copy_from_slice(&k.to_le_bytes());
        payload[n + 4..n + 4 + size].copy_from_slice(&bytes[..size]);
        n += 4 + size;
    }
    let (class, id) = msg::CFG_VALSET;
    write_frame(class, id, &payload[..n], out).map_err(|BufferTooSmall| ValsetError::BufferTooSmall)
}

/// Config keys Winchy sets.
pub mod key {
    /// CFG-UART1-BAUDRATE (U4).
    pub const UART1_BAUDRATE: u32 = 0x4052_0001;
    /// CFG-RATE-MEAS, ms between measurements (U2).
    pub const RATE_MEAS: u32 = 0x3021_0001;
    /// CFG-RATE-NAV, measurements per solution (U2).
    pub const RATE_NAV: u32 = 0x3021_0002;
    /// CFG-NAVSPG-DYNMODEL (E1).
    pub const DYNMODEL: u32 = 0x2011_0021;
    /// CFG-SIGNAL-GPS_ENA (L).
    pub const SIG_GPS: u32 = 0x1031_001F;
    /// CFG-SIGNAL-SBAS_ENA (L).
    pub const SIG_SBAS: u32 = 0x1031_0020;
    /// CFG-SIGNAL-GAL_ENA (L).
    pub const SIG_GAL: u32 = 0x1031_0021;
    /// CFG-SIGNAL-BDS_ENA (L).
    pub const SIG_BDS: u32 = 0x1031_0022;
    /// CFG-MSGOUT-UBX_NAV_PVT_UART1 (U1).
    pub const MSGOUT_NAV_PVT_UART1: u32 = 0x2091_0007;
    /// CFG-MSGOUT-NMEA_ID_* _UART1 (U1): GGA, RMC, GLL, GSA, GSV, VTG.
    pub const MSGOUT_NMEA_UART1: [u32; 6] = [
        0x2091_00BB,
        0x2091_00AC,
        0x2091_00CA,
        0x2091_00C0,
        0x2091_00C5,
        0x2091_00B1,
    ];
}

/// Navigation dynamic model "airborne < 2 g" (tolerates launch dynamics).
pub const DYN_AIRBORNE_2G: u8 = 7;

/// Number of items in [`receiver_config`].
pub const RECEIVER_CONFIG_ITEMS: usize = 14;

/// Winchy's receiver setup: NAV-PVT only (all NMEA off), `rate_hz` solutions
/// (measurement period clamped to 50..=1000 ms), airborne dynamics,
/// GPS+SBAS+Galileo+BeiDou.
#[must_use]
pub fn receiver_config(rate_hz: u16) -> [(u32, CfgValue); RECEIVER_CONFIG_ITEMS] {
    let meas_ms = 1000u16.checked_div(rate_hz).unwrap_or(1000).clamp(50, 1000);
    let [gga, rmc, gll, gsa, gsv, vtg] = key::MSGOUT_NMEA_UART1;
    [
        (key::RATE_MEAS, CfgValue::U16(meas_ms)),
        (key::RATE_NAV, CfgValue::U16(1)),
        (gga, CfgValue::U8(0)),
        (rmc, CfgValue::U8(0)),
        (gll, CfgValue::U8(0)),
        (gsa, CfgValue::U8(0)),
        (gsv, CfgValue::U8(0)),
        (vtg, CfgValue::U8(0)),
        (key::DYNMODEL, CfgValue::U8(DYN_AIRBORNE_2G)),
        (key::SIG_GPS, CfgValue::U8(1)),
        (key::SIG_SBAS, CfgValue::U8(1)),
        (key::SIG_GAL, CfgValue::U8(1)),
        (key::SIG_BDS, CfgValue::U8(1)),
        (key::MSGOUT_NAV_PVT_UART1, CfgValue::U8(1)),
    ]
}

/// Payload length of MGA-INI-TIME_UTC.
pub const MGA_INI_TIME_UTC_LEN: usize = 24;

/// Writes a UBX-MGA-INI-TIME_UTC (coarse time aiding) frame into `out`.
///
/// # Errors
/// [`BufferTooSmall`] if `out` is shorter than 32 bytes.
pub fn write_mga_ini_time_utc(
    t: UtcDateTime,
    acc_s: u16,
    out: &mut [u8],
) -> Result<usize, BufferTooSmall> {
    let mut p = [0u8; MGA_INI_TIME_UTC_LEN];
    p[0] = 0x10; // type: TIME_UTC
    p[3] = 0x80; // leapSecs unknown
    p[4..6].copy_from_slice(&t.year.to_le_bytes());
    p[6..11].copy_from_slice(&[t.month, t.day, t.hour, t.minute, t.second]);
    p[16..18].copy_from_slice(&acc_s.to_le_bytes()); // tAccS
    let (class, id) = msg::MGA_INI;
    write_frame(class, id, &p, out)
}

#[cfg(test)]
mod tests;
