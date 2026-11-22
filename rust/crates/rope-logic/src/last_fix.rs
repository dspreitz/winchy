//! Persisting the field elevation so the next boot can seed QNH.

/// Lowest plausible altitude (Dead Sea shore), m.
pub const ALT_MIN_M: f32 = -430.0;
/// Highest plausible altitude for a winch launch site, m.
pub const ALT_MAX_M: f32 = 9_000.0;
/// Minimum interval between saves (flash wear), ms.
pub const SAVE_INTERVAL_MS: u64 = 120_000;
/// Only persist a confident fix.
pub const MIN_SATS: u8 = 5;

/// True for an altitude a launch site can have (a poisoned 58 km value once
/// caused a boot loop in the MicroPython firmware).
#[must_use]
pub fn plausible_altitude(alt_m: f32) -> bool {
    (ALT_MIN_M..=ALT_MAX_M).contains(&alt_m)
}

/// Throttles saving the last fix.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct LastFixSaver {
    last_ms: Option<u64>,
}

impl LastFixSaver {
    /// True if this fix should be saved now (and records the save).
    pub fn should_save(
        &mut self,
        now_ms: u64,
        idle: bool,
        fix: bool,
        sats: u8,
        alt_m: f32,
    ) -> bool {
        let due = self
            .last_ms
            .is_none_or(|t| now_ms.saturating_sub(t) >= SAVE_INTERVAL_MS);
        if idle && fix && sats >= MIN_SATS && plausible_altitude(alt_m) && due {
            self.last_ms = Some(now_ms);
            return true;
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plausible_range() {
        assert!(plausible_altitude(-430.0));
        assert!(plausible_altitude(612.0));
        assert!(plausible_altitude(9_000.0));
        assert!(!plausible_altitude(-431.0));
        assert!(!plausible_altitude(58_316.0));
        assert!(!plausible_altitude(f32::NAN));
    }

    #[test]
    fn saves_a_confident_idle_fix_then_throttles() {
        let mut s = LastFixSaver::default();
        assert!(s.should_save(1_000, true, true, 7, 612.0));
        assert!(!s.should_save(1_000 + SAVE_INTERVAL_MS - 1, true, true, 7, 612.0));
        assert!(s.should_save(1_000 + SAVE_INTERVAL_MS, true, true, 7, 612.0));
    }

    #[test]
    fn refuses_weak_moving_or_implausible_fixes() {
        let mut s = LastFixSaver::default();
        assert!(!s.should_save(0, false, true, 7, 612.0), "towing");
        assert!(!s.should_save(0, true, false, 7, 612.0), "no fix");
        assert!(!s.should_save(0, true, true, 4, 612.0), "few sats");
        assert!(!s.should_save(0, true, true, 7, 58_316.0), "implausible");
        assert!(
            s.should_save(0, true, true, 7, 612.0),
            "refusals don't throttle"
        );
    }
}
