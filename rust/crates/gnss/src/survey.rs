//! Survey-in: running-mean position of a stationary receiver (the winch),
//! with an outlier/reposition guard.

use winchy_geo::{GeoPoint, horizontal_distance_m};

/// Tuning of [`SurveyIn`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurveyConfig {
    /// A fix farther than this from the mean may be a move, m.
    pub reset_dist_m: f64,
    /// Consecutive far fixes that re-seed the survey (before convergence).
    pub reset_hits: u32,
    /// Samples needed before convergence can be claimed.
    pub min_samples: u32,
    /// Accuracy indicator needed for convergence, m.
    pub target_accuracy_m: f64,
}

impl Default for SurveyConfig {
    fn default() -> Self {
        Self {
            reset_dist_m: 15.0,
            reset_hits: 5,
            min_samples: 30,
            target_accuracy_m: 2.0,
        }
    }
}

/// Floor of the accuracy indicator, m.
pub const ACCURACY_FLOOR_M: f64 = 0.5;

/// Survey-in averager.
///
/// `accuracy_m` is an indicator (standard error of the mean, floored at
/// [`ACCURACY_FLOOR_M`]), not a rigorous CEP.
#[derive(Debug, Clone)]
pub struct SurveyIn {
    cfg: SurveyConfig,
    n: u32,
    mean: Option<(GeoPoint, f64)>,
    far: u32,
    var_m2: f64,
}

impl SurveyIn {
    /// Creates an empty survey.
    #[must_use]
    pub fn new(cfg: SurveyConfig) -> Self {
        Self {
            cfg,
            n: 0,
            mean: None,
            far: 0,
            var_m2: 0.0,
        }
    }

    /// Feeds one fix. Returns true if it was averaged in, false if it was
    /// rejected as an outlier or triggered a re-seed.
    pub fn add(&mut self, p: GeoPoint, alt_m: f64) -> bool {
        let Some((mean, mean_alt)) = self.mean else {
            self.seed(p, alt_m);
            return true;
        };
        let d = horizontal_distance_m(mean, p);
        if d > self.cfg.reset_dist_m {
            self.far += 1;
            // Re-seed only BEFORE convergence (a bad initial seed). Once
            // converged, a parked winch does not move within a session, so a
            // burst of far fixes is receiver noise: keep the good position.
            if self.far >= self.cfg.reset_hits && !self.is_converged() {
                self.seed(p, alt_m);
            }
            return false;
        }
        self.far = 0;
        // Incremental mean and 1/n-weighted spread estimate.
        self.n += 1;
        let k = 1.0 / f64::from(self.n);
        self.mean = Some((
            GeoPoint::new(
                mean.lat_deg + (p.lat_deg - mean.lat_deg) * k,
                mean.lon_deg + (p.lon_deg - mean.lon_deg) * k,
            ),
            mean_alt + (alt_m - mean_alt) * k,
        ));
        self.var_m2 += (d * d - self.var_m2) * k;
        true
    }

    fn seed(&mut self, p: GeoPoint, alt_m: f64) {
        self.mean = Some((p, alt_m));
        self.n = 1;
        self.var_m2 = 0.0;
        self.far = 0;
    }

    /// Mean position and altitude, `None` before the first fix.
    #[must_use]
    pub fn position(&self) -> Option<(GeoPoint, f64)> {
        self.mean
    }

    /// Number of averaged fixes.
    #[must_use]
    pub fn samples(&self) -> u32 {
        self.n
    }

    /// Accuracy indicator, m (`INFINITY` with fewer than two fixes).
    #[must_use]
    pub fn accuracy_m(&self) -> f64 {
        if self.n < 2 {
            return f64::INFINITY;
        }
        let sem = libm_sqrt(self.var_m2 / f64::from(self.n));
        sem.max(ACCURACY_FLOOR_M)
    }

    /// True once enough fixes are averaged and the accuracy target is met.
    #[must_use]
    pub fn is_converged(&self) -> bool {
        self.n >= self.cfg.min_samples && self.accuracy_m() <= self.cfg.target_accuracy_m
    }
}

fn libm_sqrt(x: f64) -> f64 {
    libm::sqrt(x)
}

impl Default for SurveyIn {
    fn default() -> Self {
        Self::new(SurveyConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use winchy_testkit::Rng;

    const M_PER_DEG: f64 = winchy_geo::EARTH_RADIUS_M * core::f64::consts::PI / 180.0;

    fn jitter(p: GeoPoint, sigma_m: f64, r: &mut Rng) -> GeoPoint {
        let dn = r.gauss(0.0, sigma_m);
        let de = r.gauss(0.0, sigma_m);
        GeoPoint::new(
            p.lat_deg + dn / M_PER_DEG,
            p.lon_deg + de / (M_PER_DEG * p.lat_deg.to_radians().cos()),
        )
    }

    const HOME: GeoPoint = GeoPoint::new(48.0, 11.0);
    const FAR: GeoPoint = GeoPoint::new(48.01, 11.0); // ~1.1 km north

    #[test]
    fn empty_is_not_converged() {
        let s = SurveyIn::default();
        assert_eq!(s.position(), None);
        assert_eq!(s.samples(), 0);
        assert!(s.accuracy_m().is_infinite());
        assert!(!s.is_converged());
    }

    #[test]
    fn converges_near_truth() {
        let mut r = Rng::new(1);
        let mut s = SurveyIn::default();
        let truth = GeoPoint::new(48.5, 11.3);
        for _ in 0..200 {
            s.add(jitter(truth, 2.5, &mut r), 500.0 + r.gauss(0.0, 3.0));
        }
        let (p, alt) = s.position().unwrap();
        assert!(horizontal_distance_m(truth, p) < 1.0);
        assert!(s.is_converged());
        assert!(s.accuracy_m() <= 2.0);
        assert!((alt - 500.0).abs() < 5.0);
    }

    #[test]
    fn accuracy_shrinks_with_samples() {
        let mut r = Rng::new(2);
        let mut s = SurveyIn::default();
        for _ in 0..5 {
            s.add(jitter(HOME, 3.0, &mut r), 0.0);
        }
        let early = s.accuracy_m();
        for _ in 0..300 {
            s.add(jitter(HOME, 3.0, &mut r), 0.0);
        }
        assert!(s.accuracy_m() < early);
    }

    #[test]
    fn single_outlier_ignored() {
        let mut s = SurveyIn::default();
        for _ in 0..40 {
            s.add(HOME, 0.0);
        }
        let before = (s.position(), s.samples());
        assert!(!s.add(FAR, 0.0));
        assert_eq!((s.position(), s.samples()), before);
    }

    #[test]
    fn far_fixes_rejected_after_convergence() {
        let mut s = SurveyIn::default();
        for _ in 0..40 {
            s.add(HOME, 0.0);
        }
        assert!(s.is_converged());
        let before = (s.position(), s.samples());
        for _ in 0..10 {
            assert!(!s.add(FAR, 0.0));
        }
        assert_eq!((s.position(), s.samples()), before);
        assert!(s.is_converged());
    }

    #[test]
    fn reposition_reseeds_before_convergence() {
        let mut s = SurveyIn::default();
        for _ in 0..3 {
            s.add(HOME, 0.0);
        }
        assert!(!s.is_converged());
        for _ in 0..5 {
            s.add(FAR, 0.0);
        }
        assert_eq!(s.samples(), 1);
        assert_eq!(s.position().map(|(p, _)| p), Some(FAR));
        assert!(!s.is_converged());
    }

    #[test]
    fn near_fix_resets_far_counter() {
        let mut s = SurveyIn::default();
        for _ in 0..3 {
            s.add(HOME, 0.0);
        }
        for _ in 0..4 {
            s.add(FAR, 0.0);
        }
        s.add(HOME, 0.0); // breaks the streak
        for _ in 0..4 {
            s.add(FAR, 0.0);
        }
        assert_eq!(s.position().map(|(p, _)| p), Some(HOME));
    }
}
