//! Adaptive TX power ("ADR").
//!
//! The loop is survivorship-biased: only frames that got through report back.
//! So stale feedback or heavy loss must fail LOUD (full power) at range, but
//! fail QUIET at bench distance, where loss means receiver saturation.

/// Default LoRa centre frequency, MHz.
pub const DEFAULT_FREQ_MHZ: f32 = 869.525;

/// Tuning of the TX power controller.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AdrConfig {
    /// PA floor, dBm.
    pub power_min_dbm: i8,
    /// PA maximum, dBm.
    pub power_max_dbm: i8,
    /// Below this downlink RSSI, raise power.
    pub rssi_low_dbm: i16,
    /// Above this downlink RSSI, trim power.
    pub rssi_high_dbm: i16,
    /// Raise step, dB.
    pub step_up_db: i8,
    /// Trim step, dB.
    pub step_down_db: i8,
    /// Loss at or above which feedback counts as bad, percent.
    pub loss_high_pct: u8,
    /// Below this distance, bad feedback means saturation, not range, m.
    pub near_m: f32,
    /// RSSI the distance cap aims for, dBm.
    pub target_rssi_dbm: f32,
    /// Fading/obstruction margin on top of free-space loss, dB.
    pub margin_db: f32,
    /// Carrier frequency, MHz.
    pub freq_mhz: f32,
    /// Stale cycles (distance unknown) before the probe ladder starts.
    pub probe_start: u32,
    /// Cycles spent on each probe ladder step.
    pub probe_hold: u32,
}

impl Default for AdrConfig {
    fn default() -> Self {
        Self {
            power_min_dbm: -9,
            power_max_dbm: 22,
            rssi_low_dbm: -90,
            rssi_high_dbm: -70,
            step_up_db: 4,
            step_down_db: 1,
            loss_high_pct: 20,
            near_m: 50.0,
            target_rssi_dbm: -80.0,
            margin_db: 15.0,
            freq_mhz: DEFAULT_FREQ_MHZ,
            probe_start: 24,
            probe_hold: 16,
        }
    }
}

/// What the controller knows about the downlink this cycle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Feedback {
    /// A fresh `LinkReport` frame arrived.
    Fresh {
        /// Downlink packet loss, percent.
        loss_pct: u8,
        /// Downlink RSSI at the winch, dBm.
        rssi_dbm: i16,
    },
    /// No fresh report for `cycles` consecutive decision cycles.
    Stale {
        /// Consecutive cycles without feedback.
        cycles: u32,
    },
}

/// Free-space path loss in dB (distance floored at 1 m).
#[must_use]
pub fn fspl_db(distance_m: f32, freq_mhz: f32) -> f32 {
    let d = distance_m.max(1.0);
    32.45 + 20.0 * libm::log10f(freq_mhz) + 20.0 * libm::log10f(d / 1000.0)
}

/// Distance-derived TX power cap: enough for `target_rssi_dbm` plus margin.
#[must_use]
pub fn power_cap_dbm(cfg: &AdrConfig, distance_m: f32) -> i8 {
    let need = cfg.target_rssi_dbm + fspl_db(distance_m, cfg.freq_mhz) + cfg.margin_db;
    let clamped =
        libm::roundf(need).clamp(f32::from(cfg.power_min_dbm), f32::from(cfg.power_max_dbm));
    clamped as i8
}

/// Next TX power (dBm), clamped to the PA range and the distance cap.
#[must_use]
pub fn next_tx_power(
    cfg: &AdrConfig,
    power_dbm: i8,
    feedback: Feedback,
    distance_m: Option<f32>,
) -> i8 {
    let (min, max) = (cfg.power_min_dbm, cfg.power_max_dbm);
    let cap = distance_m.map_or(max, |d| power_cap_dbm(cfg, d));

    let stale_cycles = match feedback {
        Feedback::Fresh { loss_pct, .. } if loss_pct >= cfg.loss_high_pct => Some(0),
        Feedback::Fresh { .. } => None,
        Feedback::Stale { cycles } => Some(cycles),
    };

    if let Some(cycles) = stale_cycles {
        if distance_m.is_some_and(|d| d < cfg.near_m) {
            // Near range: loss means RX saturation -> fail quiet to the cap.
            return cap;
        }
        if cycles >= cfg.probe_start && cfg.probe_hold > 0 {
            // Don't latch at max forever: walk max -> mid -> min, cycling.
            let mid = (i16::from(max) + i16::from(min)).div_euclid(2) as i8;
            let step = ((cycles - cfg.probe_start) / cfg.probe_hold) % 3;
            let probe = [max, mid, min][step as usize];
            return probe.min(cap);
        }
        return max.min(cap);
    }

    let Feedback::Fresh { rssi_dbm, .. } = feedback else {
        return max.min(cap);
    };
    if rssi_dbm < cfg.rssi_low_dbm {
        power_dbm.saturating_add(cfg.step_up_db).min(max).min(cap)
    } else if rssi_dbm > cfg.rssi_high_dbm {
        power_dbm.saturating_sub(cfg.step_down_db).min(cap).max(min)
    } else {
        power_dbm.min(cap)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> AdrConfig {
        AdrConfig::default()
    }

    fn fresh(loss_pct: u8, rssi_dbm: i16) -> Feedback {
        Feedback::Fresh { loss_pct, rssi_dbm }
    }

    fn stale(cycles: u32) -> Feedback {
        Feedback::Stale { cycles }
    }

    fn next(p: i8, f: Feedback) -> i8 {
        next_tx_power(&cfg(), p, f, None)
    }

    fn next_at(p: i8, f: Feedback, d: f32) -> i8 {
        next_tx_power(&cfg(), p, f, Some(d))
    }

    #[test]
    fn stale_report_jumps_to_max() {
        assert_eq!(next(0, stale(1)), 22);
    }

    #[test]
    fn high_loss_jumps_to_max() {
        assert_eq!(next(0, fresh(20, -75)), 22);
        assert_eq!(next(-9, fresh(80, -60)), 22);
    }

    #[test]
    fn low_rssi_raises_by_step_up() {
        assert_eq!(next(10, fresh(0, -95)), 14);
    }

    #[test]
    fn raise_clamps_at_max() {
        assert_eq!(next(21, fresh(0, -95)), 22);
    }

    #[test]
    fn high_rssi_trims_by_one_db() {
        assert_eq!(next(10, fresh(0, -60)), 9);
    }

    #[test]
    fn trim_clamps_at_min() {
        assert_eq!(next(-9, fresh(0, -40)), -9);
    }

    #[test]
    fn rssi_in_window_holds() {
        assert_eq!(next(10, fresh(0, -80)), 10);
        assert_eq!(next(10, fresh(0, -90)), 10);
        assert_eq!(next(10, fresh(0, -70)), 10);
    }

    #[test]
    fn loss_beats_good_rssi() {
        assert_eq!(next(5, fresh(50, -60)), 22);
    }

    #[test]
    fn fspl_sane() {
        assert!((31.0..32.0).contains(&fspl_db(1.0, DEFAULT_FREQ_MHZ)));
        assert!((51.0..52.0).contains(&fspl_db(10.0, DEFAULT_FREQ_MHZ)));
        assert!((91.0..92.0).contains(&fspl_db(1000.0, DEFAULT_FREQ_MHZ)));
        assert!((fspl_db(0.1, DEFAULT_FREQ_MHZ) - fspl_db(1.0, DEFAULT_FREQ_MHZ)).abs() < 1e-4);
    }

    #[test]
    fn power_cap_curve() {
        let c = cfg();
        assert_eq!(power_cap_dbm(&c, 1.0), -9);
        assert!((0..=1).contains(&power_cap_dbm(&c, 50.0)));
        assert!((15..=17).contains(&power_cap_dbm(&c, 300.0)));
        assert_eq!(power_cap_dbm(&c, 600.0), 22);
        assert_eq!(power_cap_dbm(&c, 2000.0), 22);
    }

    #[test]
    fn near_bad_feedback_fails_quiet_to_cap() {
        assert_eq!(next_at(22, stale(1), 2.0), -9);
        assert_eq!(next_at(22, fresh(80, 0), 2.0), -9);
    }

    #[test]
    fn far_stale_fails_loud() {
        assert_eq!(next_at(5, stale(1), 800.0), 22);
    }

    #[test]
    fn cap_bounds_good_link_too() {
        assert_eq!(next_at(10, fresh(0, -95), 2.0), -9);
        assert_eq!(next_at(10, fresh(0, -80), 2.0), -9);
    }

    #[test]
    fn probe_ladder_when_stale_without_distance() {
        assert_eq!(next(22, stale(0)), 22);
        assert_eq!(next(22, stale(23)), 22);
        assert_eq!(next(22, stale(24)), 22); // step 0: max
        assert_eq!(next(22, stale(40)), 6); // step 1: mid
        assert_eq!(next(22, stale(56)), -9); // step 2: min
        assert_eq!(next(22, stale(72)), 22); // cycles
    }

    #[test]
    fn probe_ladder_respects_far_distance_cap() {
        // Far (cap = max) and stale long enough: ladder still applies.
        assert_eq!(next_at(22, stale(40), 800.0), 6);
    }
}
