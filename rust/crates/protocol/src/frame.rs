//! Frame header + payload encoding/decoding.

use crate::message::{
    LinkReport, Mass, Message, MessageType, Summary, Telemetry, TelemetryFlags, TowPhase, WinchPos,
    WinchStatus,
};

/// Protocol version byte. 0x20 = Rust firmware generation (the `MicroPython`
/// firmware used 1..=6, so mixed units ignore each other cleanly).
pub const VERSION: u8 = 0x20;

/// Header length (version, type, seq).
pub const HEADER_LEN: usize = 4;

/// Longest encoded frame, bytes.
pub const MAX_FRAME_LEN: usize = 24;

/// A complete frame: sequence number + message.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame {
    /// Transmitter-global sequence number (wraps).
    pub seq: u16,
    /// Payload.
    pub message: Message,
}

/// Encoding error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EncodeError {
    /// Output buffer too small.
    BufferTooSmall,
}

/// Decoding error (receivers drop the frame, they never panic).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeError {
    /// Shorter than a header.
    TooShort,
    /// Unknown protocol version.
    UnknownVersion(u8),
    /// Unknown message type.
    UnknownType(u8),
    /// Payload length does not match the type.
    BadLength {
        /// Expected total frame length.
        expected: usize,
        /// Actual frame length.
        actual: usize,
    },
    /// A field holds an invalid value (e.g. unknown tow phase).
    BadValue,
}

impl Frame {
    /// Encodes into `out`, returns the frame length.
    ///
    /// # Errors
    /// [`EncodeError::BufferTooSmall`] if `out` is too short.
    pub fn encode(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        let mut w = Writer { buf: out, pos: 0 };
        w.u8(VERSION)?;
        w.u8(self.message.message_type() as u8)?;
        w.bytes(&self.seq.to_le_bytes())?;
        match self.message {
            Message::Telemetry(t) => {
                w.u8(t.phase as u8)?;
                w.u8(t.flags.0)?;
                w.bytes(&t.force.to_le_bytes())?;
                w.bytes(&t.rope_angle_cdeg.to_le_bytes())?;
                w.bytes(&t.altitude_dm.to_le_bytes())?;
                w.bytes(&t.climb_cms.to_le_bytes())?;
                w.bytes(&t.glider_speed_cms.to_le_bytes())?;
                w.bytes(&t.battery_mv.to_le_bytes())?;
                w.u8(t.battery_pct)?;
                w.bytes(&t.tx_power_dbm.to_le_bytes())?;
            }
            Message::Mass(m) => {
                w.bytes(&m.mass_dkg.to_le_bytes())?;
                w.u8(m.confidence_pct)?;
            }
            Message::Summary(s) => {
                w.bytes(&s.duration_ds.to_le_bytes())?;
                w.bytes(&s.max_force.to_le_bytes())?;
                w.bytes(&s.release_alt_dm.to_le_bytes())?;
                w.bytes(&s.mass_dkg.to_le_bytes())?;
            }
            Message::LinkReport(r) => {
                w.bytes(&r.rssi_dbm.to_le_bytes())?;
                w.bytes(&r.snr_qdb.to_le_bytes())?;
                w.u8(r.loss_pct)?;
            }
            Message::WinchPos(p) => {
                w.bytes(&p.lat_e7.to_le_bytes())?;
                w.bytes(&p.lon_e7.to_le_bytes())?;
                w.bytes(&p.altitude_dm.to_le_bytes())?;
                w.bytes(&p.hacc_dm.to_le_bytes())?;
                w.u8(p.status.0)?;
            }
            Message::UploadCmd { nonce } | Message::UploadAck { nonce } => w.u8(nonce)?,
        }
        Ok(w.pos)
    }

    /// Decodes one frame.
    ///
    /// # Errors
    /// [`DecodeError`] describing why the frame was rejected.
    pub fn decode(buf: &[u8]) -> Result<Self, DecodeError> {
        if buf.len() < HEADER_LEN {
            return Err(DecodeError::TooShort);
        }
        if buf[0] != VERSION {
            return Err(DecodeError::UnknownVersion(buf[0]));
        }
        let ty = message_type(buf[1]).ok_or(DecodeError::UnknownType(buf[1]))?;
        let expected = HEADER_LEN + payload_len(ty);
        if buf.len() != expected {
            return Err(DecodeError::BadLength {
                expected,
                actual: buf.len(),
            });
        }
        let seq = u16::from_le_bytes([buf[2], buf[3]]);
        let mut r = Reader {
            buf: &buf[HEADER_LEN..],
        };
        let message = match ty {
            MessageType::Telemetry => Message::Telemetry(Telemetry {
                phase: TowPhase::try_from(r.u8()).map_err(|_| DecodeError::BadValue)?,
                flags: TelemetryFlags(r.u8()),
                force: i32::from_le_bytes(r.array()),
                rope_angle_cdeg: i16::from_le_bytes(r.array()),
                altitude_dm: i32::from_le_bytes(r.array()),
                climb_cms: i16::from_le_bytes(r.array()),
                glider_speed_cms: u16::from_le_bytes(r.array()),
                battery_mv: u16::from_le_bytes(r.array()),
                battery_pct: r.u8(),
                tx_power_dbm: i8::from_le_bytes(r.array()),
            }),
            MessageType::Mass => Message::Mass(Mass {
                mass_dkg: u16::from_le_bytes(r.array()),
                confidence_pct: r.u8(),
            }),
            MessageType::Summary => Message::Summary(Summary {
                duration_ds: u16::from_le_bytes(r.array()),
                max_force: i32::from_le_bytes(r.array()),
                release_alt_dm: i32::from_le_bytes(r.array()),
                mass_dkg: u16::from_le_bytes(r.array()),
            }),
            MessageType::LinkReport => Message::LinkReport(LinkReport {
                rssi_dbm: i16::from_le_bytes(r.array()),
                snr_qdb: i8::from_le_bytes(r.array()),
                loss_pct: r.u8(),
            }),
            MessageType::WinchPos => Message::WinchPos(WinchPos {
                lat_e7: i32::from_le_bytes(r.array()),
                lon_e7: i32::from_le_bytes(r.array()),
                altitude_dm: i32::from_le_bytes(r.array()),
                hacc_dm: u16::from_le_bytes(r.array()),
                status: WinchStatus(r.u8()),
            }),
            MessageType::UploadCmd => Message::UploadCmd { nonce: r.u8() },
            MessageType::UploadAck => Message::UploadAck { nonce: r.u8() },
        };
        Ok(Self { seq, message })
    }
}

fn message_type(b: u8) -> Option<MessageType> {
    Some(match b {
        1 => MessageType::Telemetry,
        2 => MessageType::Mass,
        3 => MessageType::Summary,
        4 => MessageType::LinkReport,
        5 => MessageType::WinchPos,
        6 => MessageType::UploadCmd,
        7 => MessageType::UploadAck,
        _ => return None,
    })
}

/// Payload length per type (the single source of truth for frame sizes).
const fn payload_len(ty: MessageType) -> usize {
    match ty {
        MessageType::Telemetry => 20,
        MessageType::Mass => 3,
        MessageType::Summary => 12,
        MessageType::LinkReport => 4,
        MessageType::WinchPos => 15,
        MessageType::UploadCmd | MessageType::UploadAck => 1,
    }
}

struct Writer<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl Writer<'_> {
    fn bytes(&mut self, b: &[u8]) -> Result<(), EncodeError> {
        let dst = self
            .buf
            .get_mut(self.pos..self.pos + b.len())
            .ok_or(EncodeError::BufferTooSmall)?;
        dst.copy_from_slice(b);
        self.pos += b.len();
        Ok(())
    }

    fn u8(&mut self, v: u8) -> Result<(), EncodeError> {
        self.bytes(&[v])
    }
}

/// Reads from a payload whose length was already validated.
struct Reader<'a> {
    buf: &'a [u8],
}

impl Reader<'_> {
    fn array<const N: usize>(&mut self) -> [u8; N] {
        let mut out = [0u8; N];
        if let Some((head, tail)) = self.buf.split_at_checked(N) {
            out.copy_from_slice(head);
            self.buf = tail;
        }
        out
    }

    fn u8(&mut self) -> u8 {
        self.array::<1>()[0]
    }
}

#[cfg(test)]
mod tests;
