//! Rope-angle rate and the glider speed estimate.

/// EMA weight of the rope-angle rate.
pub const ANGLE_RATE_ALPHA: f32 = 0.3;
/// Finite-difference spikes are clamped to this, dps.
pub const ANGLE_RATE_MAX_DPS: f32 = 200.0;

/// Smoothed rope-angle rate from successive angles.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AngleRate {
    alpha: f32,
    max_dps: f32,
    prev: Option<f32>,
    rate: f32,
}

impl AngleRate {
    /// Estimator with EMA weight `alpha` and spike clamp `max_dps`.
    #[must_use]
    pub const fn new(alpha: f32, max_dps: f32) -> Self {
        Self {
            alpha,
            max_dps,
            prev: None,
            rate: 0.0,
        }
    }

    /// Feeds an angle `dt_s` after the previous one; returns the rate, dps.
    pub fn update(&mut self, angle_deg: f32, dt_s: f32) -> f32 {
        if dt_s > 0.0
            && let Some(prev) = self.prev
        {
            let raw = ((angle_deg - prev) / dt_s).clamp(-self.max_dps, self.max_dps);
            self.rate += self.alpha * (raw - self.rate);
        }
        self.prev = Some(angle_deg);
        self.rate
    }
}

/// The rope segment's own 3-D speed, m/s. Used as the glider speed: the 5 m
/// lever correction (`winchy_fusion::speed`) turns any spin of the segment
/// into fake airspeed and stays off until validated on a real launch.
#[must_use]
pub fn segment_speed_ms(ground_speed_ms: f32, climb_ms: f32) -> f32 {
    libm::hypotf(ground_speed_ms, climb_ms)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_sample_has_no_rate() {
        let mut r = AngleRate::new(ANGLE_RATE_ALPHA, ANGLE_RATE_MAX_DPS);
        assert!(r.update(30.0, 0.02).abs() < f32::EPSILON);
    }

    #[test]
    fn converges_to_a_constant_rate() {
        let mut r = AngleRate::new(ANGLE_RATE_ALPHA, ANGLE_RATE_MAX_DPS);
        let mut rate = 0.0;
        for i in 0..100 {
            rate = r.update(i as f32 * 0.2, 0.02); // 10 dps
        }
        assert!((rate - 10.0).abs() < 1e-3, "{rate}");
    }

    #[test]
    fn spikes_are_clamped_before_smoothing() {
        let mut r = AngleRate::new(ANGLE_RATE_ALPHA, ANGLE_RATE_MAX_DPS);
        r.update(0.0, 0.02);
        let rate = r.update(100.0, 0.02); // 5000 dps raw
        assert!((rate - 60.0).abs() < 1e-3, "0.3 * 200 = {rate}");
        let mut down = AngleRate::new(ANGLE_RATE_ALPHA, ANGLE_RATE_MAX_DPS);
        down.update(100.0, 0.02);
        assert!((down.update(0.0, 0.02) + 60.0).abs() < 1e-3);
    }

    #[test]
    fn non_positive_dt_keeps_the_rate() {
        let mut r = AngleRate::new(ANGLE_RATE_ALPHA, ANGLE_RATE_MAX_DPS);
        r.update(0.0, 0.02);
        let rate = r.update(1.0, 0.02);
        assert!((r.update(50.0, 0.0) - rate).abs() < f32::EPSILON);
    }

    #[test]
    fn segment_speed_is_the_3d_magnitude() {
        assert!((segment_speed_ms(3.0, 4.0) - 5.0).abs() < 1e-6);
        assert!((segment_speed_ms(0.0, -2.0) - 2.0).abs() < 1e-6);
    }
}
