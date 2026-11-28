//! Downlink quality as seen at the winch: frame loss from sequence gaps
//! (the rope numbers every frame it sends), smoothed for the link report.

use winchy_protocol::LinkReport;

/// EMA weight of a report window's loss: one gap in the tiny window between
/// two reports must not swing the reported figure to 50 or 100 %.
pub const LOSS_EMA_ALPHA: f32 = 0.3;

/// Received/lost counters and the smoothed loss.
#[derive(Debug, Clone, Default)]
pub struct LinkStats {
    last_seq: Option<u16>,
    received: u32,
    lost: u32,
    window_received: u32,
    window_lost: u32,
    loss_ema: f32,
    rssi_dbm: i16,
    snr_qdb: i8,
}

impl LinkStats {
    /// No frames yet.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Counts a received frame with sequence `seq` and its signal quality. A
    /// forward gap counts as lost frames; a backward jump is a rope restart.
    pub fn on_frame(&mut self, seq: u16, rssi_dbm: i16, snr_qdb: i8) {
        if let Some(last) = self.last_seq {
            let gap = seq.wrapping_sub(last);
            if (2..0x8000).contains(&gap) {
                let missing = u32::from(gap - 1);
                self.lost += missing;
                self.window_lost += missing;
            }
        }
        self.last_seq = Some(seq);
        self.received += 1;
        self.window_received += 1;
        self.rssi_dbm = rssi_dbm;
        self.snr_qdb = snr_qdb;
    }

    /// Builds a link report from the window since the last one and starts a
    /// new window.
    pub fn report(&mut self) -> LinkReport {
        let total = self.window_received + self.window_lost;
        let window_pct = if total == 0 {
            0.0
        } else {
            100.0 * self.window_lost as f32 / total as f32
        };
        self.loss_ema = LOSS_EMA_ALPHA * window_pct + (1.0 - LOSS_EMA_ALPHA) * self.loss_ema;
        self.window_received = 0;
        self.window_lost = 0;
        LinkReport {
            rssi_dbm: self.rssi_dbm,
            snr_qdb: self.snr_qdb,
            loss_pct: winchy_protocol::fixed::to_u8(self.loss_ema, 1.0),
        }
    }

    /// Frames received since boot.
    #[must_use]
    pub fn received(&self) -> u32 {
        self.received
    }

    /// Frames lost since boot.
    #[must_use]
    pub fn lost(&self) -> u32 {
        self.lost
    }
}

#[cfg(test)]
mod tests;
