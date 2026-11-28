//! Winchy radio protocol: versioned, little-endian binary frames shared by the
//! rope and winch units.
//!
//! Every frame starts with `version: u8, type: u8, seq: u16`. The sequence
//! number is global per transmitter so the receiver can measure loss across
//! frame types. All payload fields are fixed-point integers in the units named
//! on each field; [`fixed`] converts from engineering floats with clamping.
//!
//! The link is asymmetric: the rope sends [`Telemetry`], [`Mass`] and
//! [`Summary`]; the winch sends [`LinkReport`] and [`WinchPos`]. Either side
//! sends [`Message::UploadCmd`]/[`Message::UploadAck`]. The back channel is
//! advisory: losing it must never stall rope->winch telemetry.
#![cfg_attr(not(test), no_std)]

pub mod fixed;
mod frame;
mod message;
pub mod v6;

pub use frame::{DecodeError, EncodeError, Frame, MAX_FRAME_LEN, VERSION};
pub use message::{
    LinkReport, Mass, Message, MessageType, Summary, Telemetry, TelemetryFlags, TowPhase, WinchPos,
    WinchStatus,
};

/// Decodes a received frame in either format (current or MicroPython v6).
#[must_use]
pub fn decode_any(buf: &[u8]) -> Option<Frame> {
    Frame::decode(buf).or_else(|_| v6::decode(buf)).ok()
}

#[cfg(test)]
mod tests;
