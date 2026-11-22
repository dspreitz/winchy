//! Altitude and climb rate from a noisy altitude source (barometer).
//! Constant-velocity model driven by white acceleration noise, 2-state Kalman.

/// Tuning of [`VerticalKalman`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VerticalConfig {
    /// White acceleration noise density, (m/s^2)^2.
    pub accel_var: f32,
    /// Altitude measurement variance, m^2.
    pub meas_var: f32,
}

impl Default for VerticalConfig {
    fn default() -> Self {
        Self {
            accel_var: 0.4,
            meas_var: 1.0,
        }
    }
}

/// 2-state (altitude, vertical rate) Kalman filter.
#[derive(Debug, Clone)]
pub struct VerticalKalman {
    cfg: VerticalConfig,
    alt: Option<f32>,
    vrate: f32,
    p: [[f32; 2]; 2],
}

impl VerticalKalman {
    /// Creates an unseeded filter (first [`update`](Self::update) seeds altitude).
    #[must_use]
    pub fn new(cfg: VerticalConfig) -> Self {
        Self {
            cfg,
            alt: None,
            vrate: 0.0,
            p: [[100.0, 0.0], [0.0, 10.0]],
        }
    }

    /// Propagates by `dt_s` seconds.
    pub fn predict(&mut self, dt_s: f32) {
        let Some(alt) = self.alt.as_mut() else { return };
        *alt += self.vrate * dt_s;
        // P = F P F' + Q with F = [[1, dt], [0, 1]], Q for white acceleration.
        let [[p00, p01], [_, p11]] = self.p;
        let (dt, q) = (dt_s, self.cfg.accel_var);
        let n00 = p00 + dt * (2.0 * p01 + dt * p11) + q * dt * dt * dt * dt / 4.0;
        let n01 = p01 + dt * p11 + q * dt * dt * dt / 2.0;
        let n11 = p11 + q * dt * dt;
        self.p = [[n00, n01], [n01, n11]];
    }

    /// Fuses one altitude measurement (m).
    pub fn update(&mut self, alt_m: f32) {
        let Some(alt) = self.alt.as_mut() else {
            self.alt = Some(alt_m);
            return;
        };
        let [[p00, p01], [_, p11]] = self.p;
        let s = p00 + self.cfg.meas_var;
        let (k0, k1) = (p00 / s, p01 / s);
        let innovation = alt_m - *alt;
        *alt += k0 * innovation;
        self.vrate += k1 * innovation;
        let n01 = (1.0 - k0) * p01;
        self.p = [[(1.0 - k0) * p00, n01], [n01, p11 - k1 * p01]];
    }

    /// Altitude estimate (m), `None` until seeded.
    #[must_use]
    pub fn altitude_m(&self) -> Option<f32> {
        self.alt
    }

    /// Vertical rate estimate (m/s, positive up).
    #[must_use]
    pub fn climb_rate_ms(&self) -> f32 {
        self.vrate
    }
}

impl Default for VerticalKalman {
    fn default() -> Self {
        Self::new(VerticalConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use winchy_testkit::Rng;

    #[test]
    fn unseeded_until_first_measurement() {
        let mut v = VerticalKalman::default();
        v.predict(1.0);
        assert_eq!(v.altitude_m(), None);
        v.update(360.0);
        assert_eq!(v.altitude_m(), Some(360.0));
        assert!(v.climb_rate_ms().abs() < f32::EPSILON);
    }

    #[test]
    fn estimates_climb_rate() {
        let mut r = Rng::new(5);
        let mut v = VerticalKalman::default();
        let mut alt = 100.0_f32;
        for _ in 0..30 {
            alt += 4.0;
            v.predict(1.0);
            v.update(alt + r.gauss(0.0, 0.7) as f32);
        }
        assert!(
            (v.climb_rate_ms() - 4.0).abs() < 0.6,
            "{}",
            v.climb_rate_ms()
        );
        assert!((v.altitude_m().unwrap() - alt).abs() < 1.5);
    }

    #[test]
    fn stationary() {
        let mut r = Rng::new(6);
        let mut v = VerticalKalman::default();
        for _ in 0..30 {
            v.predict(1.0);
            v.update(360.0 + r.gauss(0.0, 0.7) as f32);
        }
        assert!(v.climb_rate_ms().abs() < 0.4, "{}", v.climb_rate_ms());
        assert!((v.altitude_m().unwrap() - 360.0).abs() < 1.0);
    }
}
