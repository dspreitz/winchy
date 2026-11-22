//! Legacy MicroPython protocol v6, kept only while the winch still runs the
//! MicroPython firmware. The rope sends v6 TELEMETRY and understands the
//! winch's v6 replies; everything maps onto the current [`Message`] types.
//! Delete this module once the winch speaks [`crate::VERSION`].

use crate::frame::{DecodeError, EncodeError, Frame};
use crate::message::{LinkReport, Message, Telemetry, WinchPos, WinchStatus};

/// v6 version byte.
pub const VERSION: u8 = 6;
/// Encoded v6 TELEMETRY length.
pub const TELEMETRY_LEN: usize = 17;

/// Encodes `t` as a v6 TELEMETRY frame (lossy: half degrees, whole metres,
/// 100 mV, dm/s, no climb rate or TX power).
///
/// # Errors
/// [`EncodeError::BufferTooSmall`].
pub fn encode_telemetry(seq: u16, t: &Telemetry, out: &mut [u8]) -> Result<usize, EncodeError> {
    let out = out
        .get_mut(..TELEMETRY_LEN)
        .ok_or(EncodeError::BufferTooSmall)?;
    let clamp_u8 = |v: i64| u8::try_from(v.clamp(0, 255)).unwrap_or(u8::MAX);
    let clamp_u16 = |v: i64| u16::try_from(v.clamp(0, 65_535)).unwrap_or(u16::MAX);
    out[0] = VERSION;
    out[1] = TELEMETRY;
    out[2..4].copy_from_slice(&seq.to_le_bytes());
    out[4] = t.phase as u8;
    out[5..9].copy_from_slice(&t.force.to_le_bytes());
    out[9] = clamp_u8(div_round(i64::from(t.rope_angle_cdeg), 50));
    out[10..12].copy_from_slice(&clamp_u16(div_round(i64::from(t.altitude_dm), 10)).to_le_bytes());
    out[12] = clamp_u8(i64::from(t.battery_mv / 100));
    out[13] = t.flags.0;
    out[14] = t.battery_pct;
    out[15..17]
        .copy_from_slice(&clamp_u16(div_round(i64::from(t.glider_speed_cms), 10)).to_le_bytes());
    Ok(TELEMETRY_LEN)
}

/// Decodes the v6 frames the winch sends (link report, winch position,
/// upload command/ack).
///
/// # Errors
/// [`DecodeError`].
pub fn decode(buf: &[u8]) -> Result<Frame, DecodeError> {
    if buf.len() < 4 {
        return Err(DecodeError::TooShort);
    }
    if buf[0] != VERSION {
        return Err(DecodeError::UnknownVersion(buf[0]));
    }
    let expected = match buf[1] {
        LINK_REPORT => 8,
        WINCH_POS => 16,
        UPLOAD_CMD | UPLOAD_ACK => 5,
        other => return Err(DecodeError::UnknownType(other)),
    };
    if buf.len() != expected {
        return Err(DecodeError::BadLength {
            expected,
            actual: buf.len(),
        });
    }
    let seq = u16::from_le_bytes([buf[2], buf[3]]);
    let i32_at = |o: usize| i32::from_le_bytes([buf[o], buf[o + 1], buf[o + 2], buf[o + 3]]);
    let message = match buf[1] {
        LINK_REPORT => Message::LinkReport(LinkReport {
            rssi_dbm: i16::from_le_bytes([buf[4], buf[5]]),
            // The winch sends the raw SX1262 SNR byte, already 0.25 dB.
            snr_qdb: i8::from_le_bytes([buf[6]]),
            loss_pct: buf[7],
        }),
        WINCH_POS => Message::WinchPos(WinchPos {
            lat_e7: i32_at(4),
            lon_e7: i32_at(8),
            altitude_dm: i32::from(u16::from_le_bytes([buf[12], buf[13]])) * 10,
            hacc_dm: u16::from(buf[14]),
            status: WinchStatus(buf[15]),
        }),
        UPLOAD_CMD => Message::UploadCmd { nonce: buf[4] },
        _ => Message::UploadAck { nonce: buf[4] },
    };
    Ok(Frame { seq, message })
}

const TELEMETRY: u8 = 1;
const LINK_REPORT: u8 = 5;
const WINCH_POS: u8 = 6;
const UPLOAD_CMD: u8 = 7;
const UPLOAD_ACK: u8 = 8;

/// `x / d` rounded half away from zero (`d > 0`).
fn div_round(x: i64, d: i64) -> i64 {
    if x >= 0 {
        (x + d / 2) / d
    } else {
        (x - d / 2) / d
    }
}

#[cfg(test)]
mod tests;
