//! One radio cycle: ADR decision, telemetry frame, and handling of what the
//! winch sends back.

use winchy_link::adr::{AdrConfig, next_tx_power};
use winchy_protocol::{EncodeError, Frame, Message, v6};

use crate::geometry::adr_distance_m;
use crate::link::{LinkTracker, REPORT_TIMEOUT_MS};
use crate::state::{RopeState, Winch};
use crate::telemetry::{self, build, request_report};

/// On-air format of outgoing telemetry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Wire {
    /// MicroPython protocol v6 (while the winch still runs MicroPython).
    LegacyV6,
    /// Current protocol ([`winchy_protocol::VERSION`]).
    Current,
}

/// Encodes a telemetry frame in the chosen format.
///
/// # Errors
/// [`EncodeError::BufferTooSmall`].
pub fn encode_telemetry(
    wire: Wire,
    seq: u16,
    t: &winchy_protocol::Telemetry,
    out: &mut [u8],
) -> Result<usize, EncodeError> {
    match wire {
        Wire::LegacyV6 => v6::encode_telemetry(seq, t, out),
        Wire::Current => Frame {
            seq,
            message: Message::Telemetry(*t),
        }
        .encode(out),
    }
}

/// Decodes a received frame in either format.
#[must_use]
pub fn decode_any(buf: &[u8]) -> Option<Frame> {
    Frame::decode(buf).or_else(|_| v6::decode(buf)).ok()
}

/// Radio cycle configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RadioConfig {
    /// Outgoing format.
    pub wire: Wire,
    /// Telemetry period, ms.
    pub period_ms: u32,
    /// Extra listen time after a report request, ms.
    pub report_window_ms: u32,
    /// Request a link report every Nth frame (0 = never).
    pub report_every: u32,
    /// Adaptive TX power (None = fixed power).
    pub adr: Option<AdrConfig>,
}

impl Default for RadioConfig {
    fn default() -> Self {
        Self {
            wire: Wire::LegacyV6,
            period_ms: telemetry::PERIOD_MS,
            report_window_ms: telemetry::REPORT_WINDOW_MS,
            report_every: telemetry::REPORT_EVERY,
            adr: Some(AdrConfig::default()),
        }
    }
}

/// What to transmit this cycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TxPlan {
    /// Encoded frame length in the caller's buffer.
    pub len: usize,
    /// Sequence number used.
    pub seq: u16,
    /// TX power for this frame, dBm (apply before transmitting if changed).
    pub tx_power_dbm: i8,
    /// The frame asks the winch for a link report.
    pub report_requested: bool,
    /// Listen this long after TX before the next cycle, ms.
    pub listen_ms: u32,
}

/// Radio cycle state (sequence, frame count, downlink feedback).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RadioLink {
    cfg: RadioConfig,
    tracker: LinkTracker,
    seq: u16,
    frames: u32,
}

impl RadioLink {
    /// Fresh link state.
    #[must_use]
    pub const fn new(cfg: RadioConfig) -> Self {
        Self {
            cfg,
            tracker: LinkTracker::new(REPORT_TIMEOUT_MS),
            seq: 0,
            frames: 0,
        }
    }

    /// Plans the next transmission from `state` into `out`.
    ///
    /// # Errors
    /// [`EncodeError::BufferTooSmall`].
    pub fn plan_tx(
        &mut self,
        state: &RopeState,
        now_ms: u64,
        out: &mut [u8],
    ) -> Result<TxPlan, EncodeError> {
        self.frames = self.frames.wrapping_add(1);
        let report_requested = request_report(self.frames, self.cfg.report_every);
        let current = state.link.tx_power_dbm;
        let tx_power_dbm = self.cfg.adr.map_or(current, |adr| {
            next_tx_power(
                &adr,
                current,
                self.tracker.feedback(now_ms),
                adr_distance_m(state),
            )
        });
        let mut t = build(state, report_requested);
        t.tx_power_dbm = tx_power_dbm;
        let seq = self.seq;
        let len = encode_telemetry(self.cfg.wire, seq, &t, out)?;
        self.seq = seq.wrapping_add(1);
        let listen_ms = self.cfg.period_ms
            + if report_requested {
                self.cfg.report_window_ms
            } else {
                0
            };
        Ok(TxPlan {
            len,
            seq,
            tx_power_dbm,
            report_requested,
            listen_ms,
        })
    }

    /// Handles a received frame; updates `state` and the ADR feedback.
    /// Returns the decoded message, or None for foreign/corrupt frames.
    pub fn on_rx(&mut self, buf: &[u8], now_ms: u64, state: &mut RopeState) -> Option<Message> {
        let msg = decode_any(buf)?.message;
        state.link.rx_count = state.link.rx_count.wrapping_add(1);
        match msg {
            Message::LinkReport(r) => {
                self.tracker.on_report(r, now_ms);
                state.link.last_report = Some(r);
                state.link.report_ms = Some(now_ms);
            }
            Message::WinchPos(pos) => {
                state.winch = Some(Winch {
                    pos,
                    received_ms: now_ms,
                });
            }
            _ => {}
        }
        Some(msg)
    }
}

#[cfg(test)]
mod tests;
