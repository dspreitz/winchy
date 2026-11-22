//! Downlink feedback for ADR: is the winch's last link report still fresh?

use winchy_link::adr::Feedback;
use winchy_protocol::LinkReport;

/// A report older than this no longer counts as feedback, ms.
pub const REPORT_TIMEOUT_MS: u64 = 4_000;

/// Remembers the last report and counts decision cycles without one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LinkTracker {
    timeout_ms: u64,
    last: Option<(LinkReport, u64)>,
    stale_cycles: u32,
}

impl LinkTracker {
    /// Tracker with the given freshness timeout.
    #[must_use]
    pub const fn new(timeout_ms: u64) -> Self {
        Self {
            timeout_ms,
            last: None,
            stale_cycles: 0,
        }
    }

    /// Records a report received at `now_ms`.
    pub fn on_report(&mut self, report: LinkReport, now_ms: u64) {
        self.last = Some((report, now_ms));
    }

    /// Feedback for one ADR decision at `now_ms` (counts stale cycles).
    pub fn feedback(&mut self, now_ms: u64) -> Feedback {
        match self.last {
            Some((r, at)) if now_ms.saturating_sub(at) <= self.timeout_ms => {
                self.stale_cycles = 0;
                Feedback::Fresh {
                    loss_pct: r.loss_pct,
                    rssi_dbm: r.rssi_dbm,
                }
            }
            _ => {
                self.stale_cycles = self.stale_cycles.saturating_add(1);
                Feedback::Stale {
                    cycles: self.stale_cycles,
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const REPORT: LinkReport = LinkReport {
        rssi_dbm: -97,
        snr_qdb: 20,
        loss_pct: 5,
    };

    #[test]
    fn counts_stale_cycles_without_reports() {
        let mut t = LinkTracker::new(REPORT_TIMEOUT_MS);
        assert_eq!(t.feedback(0), Feedback::Stale { cycles: 1 });
        assert_eq!(t.feedback(500), Feedback::Stale { cycles: 2 });
    }

    #[test]
    fn fresh_report_resets_the_stale_count() {
        let mut t = LinkTracker::new(REPORT_TIMEOUT_MS);
        let _ = t.feedback(0);
        t.on_report(REPORT, 1_000);
        let fresh = Feedback::Fresh {
            loss_pct: 5,
            rssi_dbm: -97,
        };
        assert_eq!(t.feedback(1_000), fresh);
        assert_eq!(t.feedback(5_000), fresh, "still fresh at the timeout");
        assert_eq!(t.feedback(5_001), Feedback::Stale { cycles: 1 });
    }

    #[test]
    fn clock_before_report_counts_as_fresh() {
        let mut t = LinkTracker::new(REPORT_TIMEOUT_MS);
        t.on_report(REPORT, 2_000);
        assert!(matches!(t.feedback(1_000), Feedback::Fresh { .. }));
    }
}
