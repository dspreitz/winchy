//! Gyro bias: a boot-time window judge and a self-healing live tracker.

use crate::Vec3;

/// Result of judging one boot-time gyro window.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WindowBias {
    /// Per-axis mean, dps (the bias candidate).
    pub mean: Vec3,
    /// True if the window is trustworthy (low spread AND plausible magnitude).
    pub still: bool,
}

/// Thresholds for [`window_bias`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WindowLimits {
    /// Maximum per-axis spread (max - min), dps.
    pub max_spread_dps: f32,
    /// Maximum plausible per-axis bias magnitude, dps.
    pub max_mean_dps: f32,
}

impl Default for WindowLimits {
    fn default() -> Self {
        Self {
            max_spread_dps: 4.0,
            max_mean_dps: 10.0,
        }
    }
}

/// Judges a boot-time window of gyro samples (dps).
#[must_use]
pub fn window_bias(samples: &[Vec3], limits: WindowLimits) -> WindowBias {
    let Some(first) = samples.first() else {
        return WindowBias {
            mean: [0.0; 3],
            still: false,
        };
    };
    let (mut lo, mut hi, mut sum) = (*first, *first, [0.0_f32; 3]);
    for s in samples {
        for i in 0..3 {
            lo[i] = lo[i].min(s[i]);
            hi[i] = hi[i].max(s[i]);
            sum[i] += s[i];
        }
    }
    let n = samples.len() as f32;
    let mean = sum.map(|v| v / n);
    let still = (0..3)
        .all(|i| hi[i] - lo[i] <= limits.max_spread_dps && mean[i].abs() <= limits.max_mean_dps);
    WindowBias { mean, still }
}

/// Tuning of [`BiasTracker`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrackerConfig {
    /// EMA weight for the raw mean/variance statistics.
    pub alpha_stat: f32,
    /// EMA weight for easing the bias toward the raw mean while still.
    pub alpha_bias: f32,
    /// Per-axis variance below which the unit counts as at rest, dps^2.
    pub still_var_dps2: f32,
    /// Bias magnitude cap, dps (a steady turn must never be learned away).
    pub max_bias_dps: f32,
}

impl Default for TrackerConfig {
    fn default() -> Self {
        Self {
            alpha_stat: 0.02,
            alpha_bias: 0.01,
            still_var_dps2: 4.0,
            max_bias_dps: 10.0,
        }
    }
}

/// Self-healing live gyro-bias learner; call [`update`](Self::update) per sample.
///
/// Stillness is judged from the variance of the RAW gyro, independent of the
/// current bias, so a poisoned boot bias heals within seconds of real rest.
#[derive(Debug, Clone)]
pub struct BiasTracker {
    cfg: TrackerConfig,
    bias: Vec3,
    mean: Vec3,
    var: Vec3,
}

impl BiasTracker {
    /// Starts from an initial bias estimate (e.g. the boot window mean).
    #[must_use]
    pub fn new(initial_bias: Vec3, cfg: TrackerConfig) -> Self {
        Self {
            cfg,
            bias: initial_bias,
            mean: initial_bias,
            var: [cfg.still_var_dps2; 3], // start "not proven still"
        }
    }

    /// Feeds one raw gyro sample (dps).
    pub fn update(&mut self, gyro_dps: Vec3) {
        let a = self.cfg.alpha_stat;
        for ((raw, mean), var) in gyro_dps.iter().zip(&mut self.mean).zip(&mut self.var) {
            let d = raw - *mean;
            *mean += a * d;
            *var += a * (d * d - *var);
        }
        if self.is_still() {
            let cap = self.cfg.max_bias_dps;
            for (bias, mean) in self.bias.iter_mut().zip(self.mean) {
                *bias += self.cfg.alpha_bias * (mean.clamp(-cap, cap) - *bias);
            }
        }
    }

    /// Current bias estimate, dps.
    #[must_use]
    pub fn bias(&self) -> Vec3 {
        self.bias
    }

    /// True while every axis' raw variance is below the stillness threshold.
    #[must_use]
    pub fn is_still(&self) -> bool {
        self.var.iter().all(|&v| v < self.cfg.still_var_dps2)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use winchy_testkit::Rng;

    const TRUE_BIAS: Vec3 = [2.3, 1.8, 0.1];

    fn uni(r: &mut Rng, lo: f32, hi: f32) -> f32 {
        lo + (hi - lo) * r.uniform() as f32
    }

    fn rest(n: usize, r: &mut Rng) -> Vec<Vec3> {
        (0..n)
            .map(|_| core::array::from_fn(|i| TRUE_BIAS[i] + uni(r, -0.3, 0.3)))
            .collect()
    }

    fn handling(n: usize, r: &mut Rng) -> Vec<Vec3> {
        (0..n)
            .map(|_| core::array::from_fn(|_| uni(r, -40.0, 40.0)))
            .collect()
    }

    fn steady(n: usize, r: &mut Rng, rate: f32, jitter: f32) -> Vec<Vec3> {
        (0..n)
            .map(|_| core::array::from_fn(|_| rate + uni(r, -jitter, jitter)))
            .collect()
    }

    fn tracker(b: Vec3) -> BiasTracker {
        BiasTracker::new(b, TrackerConfig::default())
    }

    fn near(a: Vec3, b: Vec3, tol: f32) -> bool {
        a.iter().zip(b).all(|(x, y)| (x - y).abs() < tol)
    }

    #[test]
    fn still_window_accepted_with_true_bias() {
        let w = window_bias(&rest(50, &mut Rng::new(1)), WindowLimits::default());
        assert!(w.still);
        assert!(near(w.mean, TRUE_BIAS, 0.2), "{w:?}");
    }

    #[test]
    fn handling_window_rejected() {
        let w = window_bias(&handling(50, &mut Rng::new(2)), WindowLimits::default());
        assert!(!w.still);
    }

    #[test]
    fn single_jolt_spoils_window() {
        let mut r = Rng::new(3);
        let mut win = rest(40, &mut r);
        win.push([25.0, -18.0, 30.0]);
        win.extend(rest(9, &mut r));
        assert!(!window_bias(&win, WindowLimits::default()).still);
    }

    #[test]
    fn steady_rotation_rejected_despite_low_spread() {
        let win = steady(50, &mut Rng::new(4), 20.0, 0.2);
        assert!(!window_bias(&win, WindowLimits::default()).still);
    }

    #[test]
    fn empty_window_is_not_still() {
        let w = window_bias(&[], WindowLimits::default());
        assert_eq!(
            w,
            WindowBias {
                mean: [0.0; 3],
                still: false
            }
        );
    }

    #[test]
    #[expect(clippy::float_cmp, reason = "motion must leave the bias bit-identical")]
    fn tracker_holds_bias_during_motion() {
        let mut t = tracker(TRUE_BIAS);
        for g in handling(500, &mut Rng::new(5)) {
            t.update(g);
        }
        assert_eq!(t.bias(), TRUE_BIAS);
    }

    #[test]
    fn tracker_heals_poisoned_bias_at_rest() {
        let mut t = tracker([41.0, -38.0, 25.0]);
        for g in rest(3000, &mut Rng::new(6)) {
            t.update(g);
        }
        assert!(near(t.bias(), TRUE_BIAS, 1.0), "{:?}", t.bias());
    }

    #[test]
    fn tracker_converges_from_zero() {
        let mut t = tracker([0.0; 3]);
        for g in rest(3000, &mut Rng::new(7)) {
            t.update(g);
        }
        assert!(near(t.bias(), TRUE_BIAS, 1.0), "{:?}", t.bias());
    }

    #[test]
    fn tracker_never_learns_a_rate_as_bias() {
        let mut t = tracker(TRUE_BIAS);
        for g in steady(3000, &mut Rng::new(8), 35.0, 0.1) {
            t.update(g);
        }
        assert!(t.bias().iter().all(|b| b.abs() <= 10.0), "{:?}", t.bias());
    }

    #[test]
    fn tracker_recovers_stillness_after_motion() {
        let mut r = Rng::new(9);
        let mut t = tracker([10.0; 3]);
        for g in handling(500, &mut r) {
            t.update(g);
        }
        for g in rest(4000, &mut r) {
            t.update(g);
        }
        assert!(t.is_still());
        assert!(near(t.bias(), TRUE_BIAS, 1.0), "{:?}", t.bias());
    }
}
