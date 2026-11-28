//! What the winch sends back. The link is half duplex and the rope listens
//! for one telemetry period after each frame, so the winch answers each
//! received telemetry frame with at most one frame: the link report when the
//! rope asks for one, otherwise the winch position when it is due.

use winchy_protocol::{Frame, Message, TelemetryFlags, WinchPos};

/// Wait this long after a received frame before replying, ms: the rope needs
/// a few ms after its `TX_DONE` to be back in receive, and the LoRa preamble
/// (~6 ms at SF7/BW250) must not start before that. The rope listens for a
/// whole telemetry period, so the delay costs nothing.
pub const REPLY_GUARD_MS: u32 = 20;

use crate::link_stats::LinkStats;
use crate::winch_pos::Schedule;

/// Reply planner plus the link statistics it reports.
#[derive(Debug, Clone, Default)]
pub struct Responder {
    stats: LinkStats,
    schedule: Schedule,
    tx_seq: u16,
}

impl Responder {
    /// Fresh responder.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Handles a received frame (signal quality `rssi_dbm`/`snr_qdb`) and
    /// returns the reply to send now, if any. `pos` is the current winch
    /// position, if the survey has one.
    pub fn on_frame(
        &mut self,
        frame: &Frame,
        rssi_dbm: i16,
        snr_qdb: i8,
        now_ms: u64,
        pos: Option<WinchPos>,
    ) -> Option<Frame> {
        self.stats.on_frame(frame.seq, rssi_dbm, snr_qdb);
        let Message::Telemetry(t) = frame.message else {
            return None;
        };
        let message = if t.flags.contains(TelemetryFlags::REQUEST_REPORT) {
            Message::LinkReport(self.stats.report())
        } else {
            match pos {
                Some(p) if self.schedule.due(now_ms) => Message::WinchPos(p),
                _ => return None,
            }
        };
        let seq = self.tx_seq;
        self.tx_seq = seq.wrapping_add(1);
        Some(Frame { seq, message })
    }

    /// Reports whether a reply returned by [`on_frame`](Self::on_frame) went out.
    pub fn sent(&mut self, reply: &Frame, ok: bool, now_ms: u64) {
        if let Message::WinchPos(_) = reply.message {
            if ok {
                self.schedule.sent(now_ms);
            } else {
                self.schedule.failed(now_ms);
            }
        }
    }

    /// Link statistics.
    #[must_use]
    pub fn stats(&self) -> &LinkStats {
        &self.stats
    }
}

#[cfg(test)]
mod tests;
