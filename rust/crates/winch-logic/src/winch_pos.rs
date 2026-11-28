//! The surveyed winch position sent to the rope (for the rope<->winch
//! geometry): built from the live survey-in, never from a cached position.

use winchy_gnss::survey::SurveyIn;
use winchy_protocol::fixed::{to_i32, to_u16};
use winchy_protocol::{WinchPos, WinchStatus};

/// Resend the position this often, ms.
pub const PERIOD_MS: u64 = 15_000;
/// After a failed send, retry this soon, ms.
pub const RETRY_MS: u64 = 2_000;

/// Position message from the survey; `None` until the survey has a position.
/// The accuracy saturates at the field's maximum (an unknown accuracy, fewer
/// than two samples, is sent as the maximum).
#[must_use]
pub fn build(survey: &SurveyIn, fix: bool) -> Option<WinchPos> {
    let (p, alt_m) = survey.position()?;
    let mut status = 0;
    if fix {
        status |= WinchStatus::FIX;
    }
    if survey.is_converged() {
        status |= WinchStatus::SURVEY_DONE;
    }
    Some(WinchPos {
        lat_e7: to_i32(p.lat_deg, 1e7),
        lon_e7: to_i32(p.lon_deg, 1e7),
        altitude_dm: to_i32(alt_m, 10.0),
        hacc_dm: to_u16(survey.accuracy_m() as f32, 10.0),
        status: WinchStatus(status),
    })
}

/// When the next position is due.
#[derive(Debug, Clone, Default)]
pub struct Schedule {
    next_ms: u64,
}

impl Schedule {
    /// Due immediately (the rope wants the geometry as soon as possible).
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// True if a position should be sent now.
    #[must_use]
    pub fn due(&self, now_ms: u64) -> bool {
        now_ms >= self.next_ms
    }

    /// A position went out at `now_ms`.
    pub fn sent(&mut self, now_ms: u64) {
        self.next_ms = now_ms + PERIOD_MS;
    }

    /// Sending failed at `now_ms`.
    pub fn failed(&mut self, now_ms: u64) {
        self.next_ms = now_ms + RETRY_MS;
    }
}

#[cfg(test)]
mod tests;
