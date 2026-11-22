//! UTC timestamps and the GNSS time-trust decision.
//!
//! A glitched "fully resolved" GNSS time (seen ~37 min off in the field) must
//! never latch over a good clock. [`TimeFix`] requires agreement with the
//! current reference, or two consecutive consistent frames when there is none.

/// UTC calendar date and time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UtcDateTime {
    /// Year (e.g. 2026).
    pub year: u16,
    /// Month 1..=12.
    pub month: u8,
    /// Day 1..=31.
    pub day: u8,
    /// Hour 0..=23.
    pub hour: u8,
    /// Minute 0..=59.
    pub minute: u8,
    /// Second 0..=60 (60 = leap second).
    pub second: u8,
}

impl UtcDateTime {
    /// Seconds since 1970-01-01 UTC, or `None` for an invalid calendar value.
    #[must_use]
    pub fn to_unix(&self) -> Option<i64> {
        let (y, m, d) = (i64::from(self.year), self.month, self.day);
        let leap = (y % 4 == 0 && y % 100 != 0) || y % 400 == 0;
        let days_in_month = match m {
            1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
            4 | 6 | 9 | 11 => 30,
            2 if leap => 29,
            2 => 28,
            _ => return None,
        };
        if y < 1970
            || d == 0
            || d > days_in_month
            || self.hour > 23
            || self.minute > 59
            || self.second > 60
        {
            return None;
        }
        // Days from civil (H. Hinnant), March-based year.
        let (yy, mm) = if m <= 2 {
            (y - 1, i64::from(m) + 9)
        } else {
            (y, i64::from(m) - 3)
        };
        let era = yy.div_euclid(400);
        let yoe = yy - era * 400;
        let doy = (153 * mm + 2) / 5 + i64::from(d) - 1;
        let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
        let days = era * 146_097 + doe - 719_468;
        Some(
            days * 86_400
                + i64::from(self.hour) * 3600
                + i64::from(self.minute) * 60
                + i64::from(self.second),
        )
    }
}

/// Where the current clock came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockSource {
    /// No clock yet.
    Unset,
    /// SNTP over WiFi.
    Ntp,
    /// GNSS (PPS-disciplined).
    Gnss,
}

/// What to do with one GNSS time sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimeAction {
    /// Adopt the GNSS time as the clock.
    Set,
    /// Already on GNSS and it agrees: just keep PPS disciplining armed.
    Arm,
    /// Distrust this sample; keep the current clock.
    Reject,
    /// Hold: one more consistent frame is needed.
    Wait,
}

/// Thresholds for [`TimeFix`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimeFixPolicy {
    /// Reject samples whose reported accuracy is worse (0 = unknown, skip gate).
    pub max_tacc_ns: u32,
    /// Max disagreement with an NTP clock to adopt GNSS, s.
    pub ntp_skew_s: i64,
    /// Max drift of a GNSS clock before re-setting, s.
    pub drift_s: i64,
    /// Max difference between two frames to count as consistent, s.
    pub consist_s: i64,
}

impl Default for TimeFixPolicy {
    fn default() -> Self {
        Self {
            max_tacc_ns: 1_000_000_000,
            ntp_skew_s: 5,
            drift_s: 3,
            consist_s: 2,
        }
    }
}

/// Stateful GNSS time-trust decision (remembers one consistency candidate).
#[derive(Debug, Clone)]
pub struct TimeFix {
    policy: TimeFixPolicy,
    candidate: Option<i64>,
}

impl TimeFix {
    /// Creates a decision state with no pending candidate.
    #[must_use]
    pub fn new(policy: TimeFixPolicy) -> Self {
        Self {
            policy,
            candidate: None,
        }
    }

    /// Decides on one GNSS sample (`gnss_s`, Unix seconds) given the current
    /// clock `source`/`clock_s` and the reported accuracy `tacc_ns`.
    pub fn decide(
        &mut self,
        source: ClockSource,
        gnss_s: i64,
        clock_s: i64,
        tacc_ns: u32,
    ) -> TimeAction {
        let p = self.policy;
        if tacc_ns != 0 && tacc_ns > p.max_tacc_ns {
            return TimeAction::Reject; // low confidence; keep any candidate
        }
        let consistent = self
            .candidate
            .is_some_and(|c| (gnss_s - c).abs() <= p.consist_s);
        let action = match source {
            ClockSource::Gnss if (gnss_s - clock_s).abs() <= p.drift_s => TimeAction::Arm,
            ClockSource::Ntp if (gnss_s - clock_s).abs() <= p.ntp_skew_s => TimeAction::Set,
            ClockSource::Ntp => return TimeAction::Reject,
            ClockSource::Gnss | ClockSource::Unset if consistent => TimeAction::Set,
            ClockSource::Gnss | ClockSource::Unset => {
                self.candidate = Some(gnss_s);
                return TimeAction::Wait;
            }
        };
        self.candidate = None;
        action
    }
}

impl Default for TimeFix {
    fn default() -> Self {
        Self::new(TimeFixPolicy::default())
    }
}

/// A fix may arm the next PPS edge only from the first half of its second:
/// a late solution could arrive after the edge it would label.
pub const ARM_WINDOW_NS: i64 = 500_000_000;
/// An armed value older than this is stale (fixes stopped) and never applied.
pub const ARM_MAX_AGE_MS: u64 = 1_000;

/// Labels PPS edges with the UTC second they mark.
///
/// Each early-in-the-second fix arms "next whole second"; the PPS edge then
/// takes it (once). Monotonic milliseconds guard against stale arming.
#[derive(Debug, Clone, Default)]
pub struct PpsDiscipline {
    armed: Option<(i64, u64)>,
}

impl PpsDiscipline {
    /// Creates a disarmed discipline.
    #[must_use]
    pub const fn new() -> Self {
        Self { armed: None }
    }

    /// Arms from a fix: `utc_s` + `nano_ns` (NAV-PVT, nano may be negative),
    /// received at monotonic `now_ms`. Returns the armed second, or `None` if
    /// the solution lies too late in its second (previous arming is kept).
    pub fn arm(&mut self, utc_s: i64, nano_ns: i32, now_ms: u64) -> Option<i64> {
        let nano = i64::from(nano_ns);
        let second = utc_s + nano.div_euclid(1_000_000_000);
        if nano.rem_euclid(1_000_000_000) >= ARM_WINDOW_NS {
            return None;
        }
        let next = second + 1;
        self.armed = Some((next, now_ms));
        Some(next)
    }

    /// A PPS edge at monotonic `now_ms`: the UTC second it marks, if armed
    /// recently enough. Disarms either way.
    pub fn on_edge(&mut self, now_ms: u64) -> Option<i64> {
        let (second, at) = self.armed.take()?;
        (now_ms >= at && now_ms - at <= ARM_MAX_AGE_MS).then_some(second)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const fn dt(year: u16, month: u8, day: u8, hour: u8, minute: u8, second: u8) -> UtcDateTime {
        UtcDateTime {
            year,
            month,
            day,
            hour,
            minute,
            second,
        }
    }

    #[test]
    fn pps_arms_next_second_early_in_the_second() {
        let mut p = PpsDiscipline::new();
        assert_eq!(p.arm(1000, 0, 0), Some(1001));
        assert_eq!(p.arm(1000, 100_000_000, 0), Some(1001));
        assert_eq!(p.arm(1000, 499_999_999, 0), Some(1001));
    }

    #[test]
    fn pps_ignores_late_solutions_and_keeps_previous_arming() {
        let mut p = PpsDiscipline::new();
        assert_eq!(p.arm(1000, 100_000_000, 10), Some(1001));
        assert_eq!(p.arm(1000, 900_000_000, 810), None);
        assert_eq!(p.on_edge(910), Some(1001));
    }

    #[test]
    fn pps_handles_negative_nano() {
        let mut p = PpsDiscipline::new();
        // 1000 s - 1 us = 999.999999 s: late in second 999.
        assert_eq!(p.arm(1000, -1_000, 0), None);
        // 1000 s - 0.6 s = 999.4 s: early in second 999 -> next edge is 1000.
        assert_eq!(p.arm(1000, -600_000_000, 0), Some(1000));
    }

    #[test]
    fn pps_edge_applies_once() {
        let mut p = PpsDiscipline::new();
        assert_eq!(p.on_edge(0), None);
        p.arm(1000, 0, 0);
        assert_eq!(p.on_edge(900), Some(1001));
        assert_eq!(p.on_edge(1900), None);
    }

    #[test]
    fn pps_never_applies_stale_arming() {
        let mut p = PpsDiscipline::new();
        p.arm(1000, 0, 0);
        assert_eq!(p.on_edge(1_200), None);
        p.arm(1000, 0, 5_000);
        assert_eq!(p.on_edge(4_000), None, "edge before arming is not ours");
    }

    #[test]
    fn unix_epoch_and_known_dates() {
        assert_eq!(dt(1970, 1, 1, 0, 0, 0).to_unix(), Some(0));
        assert_eq!(dt(2026, 6, 27, 16, 45, 10).to_unix(), Some(1_782_578_710));
        assert_eq!(dt(2000, 2, 29, 0, 0, 0).to_unix(), Some(951_782_400));
        assert_eq!(dt(2050, 12, 31, 23, 59, 59).to_unix(), Some(2_556_143_999));
    }

    #[test]
    fn invalid_calendar_values_rejected() {
        assert_eq!(dt(2026, 0, 1, 0, 0, 0).to_unix(), None);
        assert_eq!(dt(2026, 13, 1, 0, 0, 0).to_unix(), None);
        assert_eq!(dt(2026, 2, 29, 0, 0, 0).to_unix(), None); // not a leap year
        assert_eq!(dt(2026, 4, 31, 0, 0, 0).to_unix(), None);
        assert_eq!(dt(2026, 1, 0, 0, 0, 0).to_unix(), None);
        assert_eq!(dt(2026, 1, 1, 24, 0, 0).to_unix(), None);
        assert_eq!(dt(2026, 1, 1, 0, 60, 0).to_unix(), None);
        assert_eq!(dt(2026, 1, 1, 0, 0, 61).to_unix(), None);
        assert_eq!(dt(1969, 12, 31, 0, 0, 0).to_unix(), None);
    }

    use ClockSource::{Gnss, Ntp, Unset};
    use TimeAction::{Arm, Reject, Set, Wait};

    #[test]
    fn no_clock_needs_two_consistent_frames() {
        let mut t = TimeFix::default();
        assert_eq!(t.decide(Unset, 1000, 0, 25), Wait);
        assert_eq!(t.decide(Unset, 1001, 0, 25), Set);
        // Candidate cleared after Set: a fresh start waits again.
        assert_eq!(t.decide(Unset, 5000, 0, 25), Wait);
    }

    #[test]
    fn single_glitch_never_sets_without_reference() {
        let mut t = TimeFix::default();
        assert_eq!(t.decide(Unset, 1000, 0, 25), Wait);
        assert_eq!(t.decide(Unset, 1000 + 2220, 0, 25), Wait);
        assert_eq!(t.decide(Unset, 1000 + 2221, 0, 25), Set);
    }

    #[test]
    fn rejects_gnss_disagreeing_with_ntp() {
        assert_eq!(
            TimeFix::default().decide(Ntp, 1000 - 2220, 1000, 25),
            Reject
        );
    }

    #[test]
    fn adopts_gnss_agreeing_with_ntp() {
        assert_eq!(TimeFix::default().decide(Ntp, 1003, 1000, 25), Set);
    }

    #[test]
    fn rejects_low_confidence_tacc_and_keeps_candidate() {
        let mut t = TimeFix::default();
        assert_eq!(t.decide(Unset, 1000, 0, 25), Wait);
        assert_eq!(t.decide(Unset, 1000, 0, 2_000_000_000), Reject);
        assert_eq!(t.decide(Unset, 1001, 0, 25), Set);
    }

    #[test]
    fn arms_pps_when_gnss_agrees() {
        assert_eq!(TimeFix::default().decide(Gnss, 1001, 1000, 25), Arm);
    }

    #[test]
    fn self_heals_bad_latched_time() {
        let mut t = TimeFix::default();
        assert_eq!(t.decide(Gnss, 1000 + 2220, 1000, 25), Wait);
        assert_eq!(t.decide(Gnss, 1000 + 2221, 1000, 25), Set);
    }

    #[test]
    fn tacc_zero_skips_the_gate() {
        let mut t = TimeFix::default();
        assert_eq!(t.decide(Ntp, 1003, 1000, 0), Set);
        assert_eq!(t.decide(Ntp, 1000 - 2220, 1000, 0), Reject);
    }
}
