//! Minimal NMEA 0183 parser: GGA (fix, position) and RMC (UTC, ground speed).
//! The talker prefix is ignored (`$GPGGA`, `$GNGGA` both parse). A valid
//! `*hh` checksum is REQUIRED: corrupted UART data must not yield a position.

use crate::time::UtcDateTime;

/// Knots to metres per second.
pub const KNOTS_TO_MS: f32 = 0.514_444;

/// GGA: fix quality and position.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gga {
    /// Fix quality (0 = invalid, 1 = GPS, 2 = DGPS, ...).
    pub quality: u8,
    /// Satellites in use.
    pub sats: u8,
    /// Latitude, degrees (None if blank).
    pub lat_deg: Option<f64>,
    /// Longitude, degrees (None if blank).
    pub lon_deg: Option<f64>,
    /// Altitude above MSL, m (None if blank).
    pub alt_m: Option<f32>,
    /// Horizontal DOP (None if blank).
    pub hdop: Option<f32>,
}

/// RMC: validity, UTC and ground speed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rmc {
    /// Status `A` (valid).
    pub valid: bool,
    /// UTC, only when valid and both time and date fields are present.
    pub utc: Option<UtcDateTime>,
    /// Ground speed, m/s (None if blank).
    pub speed_ms: Option<f32>,
}

/// A parsed sentence.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Sentence {
    /// GGA sentence.
    Gga(Gga),
    /// RMC sentence.
    Rmc(Rmc),
}

/// True if `line` is `$...*hh` with a matching XOR checksum (trailing CR/LF ok).
#[must_use]
pub fn checksum_ok(line: &[u8]) -> bool {
    let line = trim_eol(line);
    let Some((&b'$', rest)) = line.split_first() else {
        return false;
    };
    let Some(star) = rest.iter().rposition(|&c| c == b'*') else {
        return false;
    };
    let (body, tail) = (&rest[..star], &rest[star + 1..]);
    if body.is_empty() || tail.len() != 2 {
        return false;
    }
    let (Some(hi), Some(lo)) = (hex(tail[0]), hex(tail[1])) else {
        return false;
    };
    body.iter().fold(0u8, |a, &c| a ^ c) == (hi << 4 | lo)
}

fn trim_eol(mut line: &[u8]) -> &[u8] {
    while let [head @ .., b'\r' | b'\n'] = line {
        line = head;
    }
    line
}

fn hex(c: u8) -> Option<u8> {
    match c {
        b'0'..=b'9' => Some(c - b'0'),
        b'A'..=b'F' => Some(c - b'A' + 10),
        b'a'..=b'f' => Some(c - b'a' + 10),
        _ => None,
    }
}

/// Splits the sentence body (between `$` and `*`) into at most `N` fields.
fn fields<const N: usize>(body: &str) -> ([&str; N], usize) {
    let mut out = [""; N];
    let mut n = 0;
    for (slot, f) in out.iter_mut().zip(body.split(',')) {
        *slot = f;
        n += 1;
    }
    (out, n)
}

fn opt<T: core::str::FromStr>(s: &str) -> Result<Option<T>, ()> {
    if s.is_empty() {
        Ok(None)
    } else {
        s.parse().map(Some).map_err(|_| ())
    }
}

/// `ddmm.mmmm` / `dddmm.mmmm` + hemisphere -> signed degrees.
fn coord(value: &str, hemisphere: &str, degree_digits: usize) -> Result<Option<f64>, ()> {
    if value.is_empty() {
        return Ok(None);
    }
    let deg: f64 = value
        .get(..degree_digits)
        .ok_or(())?
        .parse()
        .map_err(|_| ())?;
    let min: f64 = value
        .get(degree_digits..)
        .ok_or(())?
        .parse()
        .map_err(|_| ())?;
    let v = deg + min / 60.0;
    Ok(Some(if matches!(hemisphere, "S" | "W") {
        -v
    } else {
        v
    }))
}

fn two(s: &str, at: usize) -> Result<u8, ()> {
    s.get(at..at + 2).ok_or(())?.parse().map_err(|_| ())
}

/// Parses one sentence; `None` for other types, bad checksum or malformed data.
#[must_use]
pub fn parse(line: &[u8]) -> Option<Sentence> {
    if !checksum_ok(line) {
        return None;
    }
    let line = trim_eol(line);
    let star = line.iter().rposition(|&c| c == b'*')?;
    let body = core::str::from_utf8(&line[1..star]).ok()?;
    let (f, n) = fields::<20>(body);
    let kind = f[0].get(f[0].len().checked_sub(3)?..)?;
    let parsed: Result<Option<Sentence>, ()> = match kind {
        "GGA" if n >= 10 => (|| {
            Ok(Some(Sentence::Gga(Gga {
                quality: opt(f[6])?.unwrap_or(0),
                sats: opt(f[7])?.unwrap_or(0),
                lat_deg: coord(f[2], f[3], 2)?,
                lon_deg: coord(f[4], f[5], 3)?,
                alt_m: opt(f[9])?,
                hdop: opt(f[8])?,
            })))
        })(),
        "RMC" if n >= 10 => (|| {
            let valid = f[2] == "A";
            let (t, d) = (f[1], f[9]);
            let utc = if valid && t.len() >= 6 && d.len() >= 6 {
                let sec: f32 = t.get(4..).ok_or(())?.parse().map_err(|_| ())?;
                Some(UtcDateTime {
                    year: 2000 + u16::from(two(d, 4)?),
                    month: two(d, 2)?,
                    day: two(d, 0)?,
                    hour: two(t, 0)?,
                    minute: two(t, 2)?,
                    second: sec as u8,
                })
            } else {
                None
            };
            let speed_ms = opt::<f32>(f[7])?.map(|kn| kn * KNOTS_TO_MS);
            Ok(Some(Sentence::Rmc(Rmc {
                valid,
                utc,
                speed_ms,
            })))
        })(),
        _ => Ok(None),
    };
    parsed.ok().flatten()
}

#[cfg(test)]
mod tests {
    use super::*;

    const GGA: &[u8] = b"$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*59\r\n";
    const RMC: &[u8] = b"$GNRMC,081836,A,4807.038,N,01131.000,E,10.0,084.4,260606,,,A*55";

    #[test]
    fn checksum_validation() {
        assert!(checksum_ok(GGA));
        assert!(checksum_ok(RMC));
        assert!(!checksum_ok(b"$GNGGA,123519*00"));
        assert!(!checksum_ok(b"$GPGGA,no,checksum"));
        assert!(!checksum_ok(b"$G\x99 random *ZZ"));
        assert!(!checksum_ok(b"GNVTG,,T,,M,0.0,N,0.0,K,A*3D"));
        assert!(!checksum_ok(b""));
    }

    #[test]
    fn gga() {
        let Some(Sentence::Gga(g)) = parse(GGA) else {
            panic!("not GGA")
        };
        assert_eq!((g.quality, g.sats), (1, 8));
        assert!((g.lat_deg.unwrap() - (48.0 + 7.038 / 60.0)).abs() < 1e-9);
        assert!((g.lon_deg.unwrap() - (11.0 + 31.0 / 60.0)).abs() < 1e-9);
        assert!((g.alt_m.unwrap() - 545.4).abs() < 1e-4);
        assert!((g.hdop.unwrap() - 0.9).abs() < 1e-6);
    }

    #[test]
    fn gga_southern_western_negative_altitude() {
        let line = b"$GNGGA,123519,3352.000,S,15112.000,W,1,08,0.9,-12.5,M,46.9,M,,*4E";
        let Some(Sentence::Gga(g)) = parse(line) else {
            panic!("not GGA")
        };
        assert!((g.lat_deg.unwrap() + (33.0 + 52.0 / 60.0)).abs() < 1e-9);
        assert!((g.lon_deg.unwrap() + (151.0 + 12.0 / 60.0)).abs() < 1e-9);
        assert!((g.alt_m.unwrap() + 12.5).abs() < 1e-4);
    }

    #[test]
    fn rmc_datetime_and_speed() {
        let Some(Sentence::Rmc(r)) = parse(RMC) else {
            panic!("not RMC")
        };
        assert!(r.valid);
        assert_eq!(
            r.utc,
            Some(UtcDateTime {
                year: 2006,
                month: 6,
                day: 26,
                hour: 8,
                minute: 18,
                second: 36
            })
        );
        assert!((r.speed_ms.unwrap() - 10.0 * KNOTS_TO_MS).abs() < 1e-5);
    }

    #[test]
    fn rmc_invalid_has_no_time() {
        let Some(Sentence::Rmc(r)) = parse(b"$GNRMC,081836,V,,,,,,,260606,,,N*4D") else {
            panic!("not RMC")
        };
        assert!(!r.valid);
        assert_eq!(r.utc, None);
        assert_eq!(r.speed_ms, None);
    }

    #[test]
    fn ignores_other_and_bad_sentences() {
        assert_eq!(parse(b"$GNVTG,,T,,M,0.0,N,0.0,K,A*3D"), None);
        assert_eq!(parse(b"not a sentence"), None);
        let mut corrupt = GGA.to_vec();
        corrupt[10] = b'9';
        assert_eq!(parse(&corrupt), None);
    }
}
