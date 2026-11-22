//! Baud-rate detection helper: does a buffer contain real GNSS framing?
//!
//! At the wrong baud a bare `b5 62` or `$G` turns up in ~10% of garbage
//! reads. Requiring a checksum-valid UBX frame (~1/65536 false positive) or
//! NMEA sentence (~1/256) makes the probe trustworthy.

use crate::nmea;
use crate::ubx::{MAX_PAYLOAD, SYNC, checksum};

/// True if `buf` contains at least one checksum-valid UBX frame or NMEA sentence.
#[must_use]
pub fn contains_gnss_frame(buf: &[u8]) -> bool {
    contains_ubx(buf) || contains_nmea(buf)
}

fn contains_ubx(buf: &[u8]) -> bool {
    buf.windows(2).enumerate().any(|(i, w)| {
        if w != SYNC {
            return false;
        }
        let Some(header) = buf.get(i + 2..i + 6) else {
            return false;
        };
        let len = usize::from(u16::from_le_bytes([header[2], header[3]]));
        if len > MAX_PAYLOAD {
            return false;
        }
        let end = i + 6 + len;
        buf.get(end..end + 2)
            .is_some_and(|ck| checksum(&buf[i + 2..end]) == [ck[0], ck[1]])
    })
}

fn contains_nmea(buf: &[u8]) -> bool {
    buf.split(|&c| c == b'\r' || c == b'\n').any(|seg| {
        seg.iter()
            .position(|&c| c == b'$')
            .is_some_and(|d| nmea::checksum_ok(&seg[d..]))
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ubx::{OVERHEAD, write_frame};

    fn ubx(payload: &[u8]) -> Vec<u8> {
        let mut out = vec![0u8; payload.len() + OVERHEAD];
        write_frame(0x01, 0x07, payload, &mut out).unwrap();
        out
    }

    const RMC: &[u8] = b"$GNRMC,081836,A,4807.038,N,01131.000,E,10.0,084.4,260606,,,A*55";

    #[test]
    fn accepts_valid_ubx_in_noise() {
        let mut buf = b"noise\x00\x11".to_vec();
        buf.extend(ubx(&[0u8; 92]));
        buf.extend(b"tail");
        assert!(contains_gnss_frame(&buf));
    }

    #[test]
    fn accepts_valid_nmea_in_noise() {
        let mut buf = b"\x00\x01".to_vec();
        buf.extend(RMC);
        buf.extend(b"\r\n");
        assert!(contains_gnss_frame(&buf));
    }

    #[test]
    fn rejects_bare_ubx_sync() {
        assert!(!contains_gnss_frame(b"junk\xb5\x62\x10\x20\x05\x00abc"));
    }

    #[test]
    fn rejects_bad_checksums() {
        let mut bad = ubx(&[0u8; 92]);
        *bad.last_mut().unwrap() ^= 0xFF;
        assert!(!contains_gnss_frame(&bad));
        assert!(!contains_gnss_frame(b"$GPGGA,no,checksum,field,here"));
        assert!(!contains_gnss_frame(b"$G\x99\x88 random *ZZ"));
    }

    #[test]
    fn rejects_incomplete_and_empty() {
        assert!(!contains_gnss_frame(&ubx(&[0u8; 92])[..40]));
        assert!(!contains_gnss_frame(b""));
    }

    #[test]
    fn finds_valid_frame_after_a_false_sync() {
        let mut buf = vec![0xB5, 0x62, 0x01];
        buf.extend(ubx(&[1, 2, 3]));
        assert!(contains_gnss_frame(&buf));
    }
}
