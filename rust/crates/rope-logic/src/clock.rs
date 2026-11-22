//! System clock discipline from the GNSS 1PPS edge.
//!
//! The ISR only timestamps the edge (monotonic µs); the GNSS task later knows
//! which UTC second that edge marked and steps the clock if it is off.

/// Steps smaller than this are left alone (avoids churn), µs.
pub const STEP_TOLERANCE_US: i64 = 2_000;

/// UTC now, µs since the epoch, given that the edge at monotonic `edge_us`
/// marked UTC second `edge_utc_s` and the monotonic clock reads `now_us`.
#[must_use]
pub fn utc_now_us(edge_utc_s: i64, edge_us: i64, now_us: i64) -> i64 {
    edge_utc_s * 1_000_000 + (now_us - edge_us)
}

/// Correction to apply to the system clock (`truth - clock`), or None if it
/// is within `tolerance_us`.
#[must_use]
pub fn step_us(clock_us: i64, truth_us: i64, tolerance_us: i64) -> Option<i64> {
    let d = truth_us - clock_us;
    (d.abs() > tolerance_us).then_some(d)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn utc_is_the_edge_second_plus_elapsed_time() {
        assert_eq!(
            utc_now_us(1_700_000_000, 5_000_000, 5_012_345),
            1_700_000_000_012_345
        );
    }

    #[test]
    fn small_errors_are_ignored_large_ones_corrected() {
        assert_eq!(step_us(1_000_000, 1_001_999, STEP_TOLERANCE_US), None);
        assert_eq!(step_us(1_000_000, 998_000, STEP_TOLERANCE_US), None);
        assert_eq!(
            step_us(1_000_000, 1_002_001, STEP_TOLERANCE_US),
            Some(2_001)
        );
        assert_eq!(
            step_us(5_000_000, 1_000_000, STEP_TOLERANCE_US),
            Some(-4_000_000)
        );
    }
}
