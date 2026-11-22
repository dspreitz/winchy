//! Load-cell tare and the periodic ADC offset recalibration policy.

/// Samples averaged for the boot tare.
pub const TARE_SAMPLES: usize = 10;

/// Tare offset: mean of the samples (None if there are none).
#[must_use]
pub fn tare(samples: &[i32]) -> Option<i32> {
    let n = i64::try_from(samples.len()).ok().filter(|&n| n > 0)?;
    let sum: i64 = samples.iter().map(|&s| i64::from(s)).sum();
    i32::try_from(sum / n).ok()
}

/// When to rerun the ADS1232 on-chip offset calibration (sheds thermal drift
/// without a re-tare). Never during a tow or under load.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RecalPolicy {
    /// Minimum interval, ms.
    pub period_ms: u64,
    /// Only while |tared force| is below this, counts (~6 % full scale).
    pub quiet_counts: i32,
}

impl Default for RecalPolicy {
    fn default() -> Self {
        Self {
            period_ms: 600_000,
            quiet_counts: 500_000,
        }
    }
}

impl RecalPolicy {
    /// True if a recalibration is due.
    #[must_use]
    pub fn due(&self, now_ms: u64, last_ms: u64, tared: i32, idle: bool) -> bool {
        idle && now_ms.saturating_sub(last_ms) > self.period_ms
            && tared.unsigned_abs() < self.quiet_counts.unsigned_abs()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tare_is_the_mean_without_overflow() {
        assert_eq!(tare(&[]), None);
        assert_eq!(tare(&[10, 20, 30]), Some(20));
        assert_eq!(tare(&[-5, -6]), Some(-5), "truncates towards zero");
        assert_eq!(tare(&[i32::MAX, i32::MAX]), Some(i32::MAX));
    }

    #[test]
    fn recal_after_the_period_when_idle_and_quiet() {
        let p = RecalPolicy::default();
        assert!(
            !p.due(600_000, 0, 0, true),
            "needs strictly more than the period"
        );
        assert!(p.due(600_001, 0, 0, true));
        assert!(!p.due(600_001, 0, 0, false), "towing");
        assert!(!p.due(600_001, 0, 500_000, true), "under load");
        assert!(!p.due(600_001, 0, -500_000, true), "under load (negative)");
        assert!(!p.due(10, 20, 0, true), "clock before last");
    }
}
