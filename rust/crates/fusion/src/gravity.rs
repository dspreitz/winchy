//! Gravity-vector Kalman filter: rope angle that survives launch acceleration
//! and rope spin.
//!
//! Prediction rotates the estimate with the gyro (exact Rodrigues rotation);
//! the measurement is the normalised accelerometer. With isotropic noise and
//! rotation as the only dynamics the covariance stays a multiple of the
//! identity, so the scalar form here is exact. The measurement variance grows
//! with `(|a| - 1 g)^2`: under launch acceleration the accelerometer is
//! de-weighted and the gyro carries the estimate.

use crate::Vec3;

const DOWN_ROPE: Vec3 = [0.0, 1.0, 0.0];

fn norm(v: Vec3) -> f32 {
    libm::sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
}

fn normalized(v: Vec3) -> Option<Vec3> {
    let n = norm(v);
    (n > 1e-9).then(|| [v[0] / n, v[1] / n, v[2] / n])
}

/// Tuning of [`GravityKalman`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GravityConfig {
    /// Process noise per second.
    pub q: f32,
    /// Measurement variance at exactly 1 g.
    pub r_static: f32,
    /// Extra measurement variance per `(|a| - 1 g)^2`.
    pub r_dynamic: f32,
}

impl Default for GravityConfig {
    fn default() -> Self {
        Self {
            q: 0.01,
            r_static: 0.05,
            r_dynamic: 50.0,
        }
    }
}

/// Tracks the gravity direction as a unit vector in the body frame.
#[derive(Debug, Clone)]
pub struct GravityKalman {
    cfg: GravityConfig,
    g: Option<Vec3>,
    p: f32,
}

impl GravityKalman {
    /// Creates an uninitialised filter (first [`update`](Self::update) seeds it).
    #[must_use]
    pub fn new(cfg: GravityConfig) -> Self {
        Self {
            cfg,
            g: None,
            p: 1.0,
        }
    }

    /// Propagates the estimate by the body rate `gyro_dps` over `dt_s`.
    pub fn predict(&mut self, gyro_dps: Vec3, dt_s: f32) {
        let Some(grav) = self.g else { return };
        let omega = gyro_dps.map(f32::to_radians);
        let rate = norm(omega);
        let angle = rate * dt_s;
        if angle > 1e-9 {
            // Rodrigues rotation of g by -angle about the gyro axis (the body
            // turns by +angle, so gravity turns by -angle in the body frame).
            // Exact even at rope-spin rates of several degrees per step.
            let axis = omega.map(|v| v / rate);
            let (sin, cos) = (libm::sinf(angle), libm::cosf(angle));
            let dot = (axis[0] * grav[0] + axis[1] * grav[1] + axis[2] * grav[2]) * (1.0 - cos);
            let cross = [
                axis[1] * grav[2] - axis[2] * grav[1],
                axis[2] * grav[0] - axis[0] * grav[2],
                axis[0] * grav[1] - axis[1] * grav[0],
            ];
            let rotated: Vec3 =
                core::array::from_fn(|i| grav[i] * cos - cross[i] * sin + axis[i] * dot);
            if let Some(n) = normalized(rotated) {
                self.g = Some(n);
            }
        }
        self.p += self.cfg.q * dt_s;
    }

    /// Fuses one accelerometer sample (specific force, g).
    pub fn update(&mut self, accel_g: Vec3) {
        let magnitude = norm(accel_g);
        let Some(z) = normalized(accel_g) else { return };
        let Some(g) = self.g else {
            self.g = Some(z);
            return;
        };
        // Distrust the accelerometer in proportion to its deviation from 1 g.
        let dyn_g = magnitude - 1.0;
        let r = self.cfg.r_static + self.cfg.r_dynamic * dyn_g * dyn_g;
        let k = self.p / (self.p + r);
        let blended: Vec3 = core::array::from_fn(|i| g[i] + k * (z[i] - g[i]));
        if let Some(n) = normalized(blended) {
            self.g = Some(n);
        }
        self.p *= 1.0 - k;
    }

    /// Current unit gravity estimate, or straight along the rope if unseeded.
    #[must_use]
    pub fn gravity(&self) -> Vec3 {
        self.g.unwrap_or(DOWN_ROPE)
    }
}

impl Default for GravityKalman {
    fn default() -> Self {
        Self::new(GravityConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::attitude::rope_inclination_deg;
    use winchy_testkit::Rng;

    const DT: f32 = 0.02; // 50 Hz

    fn noise(r: &mut Rng, s: f64) -> f32 {
        r.gauss(0.0, s) as f32
    }

    fn angle(f: &GravityKalman) -> f32 {
        rope_inclination_deg(f.gravity())
    }

    #[test]
    #[allow(clippy::float_cmp, reason = "exact passthrough of a constant")]
    fn unseeded_points_along_rope() {
        assert_eq!(GravityKalman::default().gravity(), [0.0, 1.0, 0.0]);
    }

    #[test]
    #[allow(clippy::float_cmp, reason = "axis-aligned input normalises exactly")]
    fn first_update_seeds_normalised() {
        let mut f = GravityKalman::default();
        f.update([0.0, 0.0, 2.0]);
        assert_eq!(f.gravity(), [0.0, 0.0, 1.0]);
    }

    #[test]
    #[allow(clippy::float_cmp, reason = "state must stay bit-identical")]
    fn zero_accel_is_ignored() {
        let mut f = GravityKalman::default();
        f.update([0.0, 0.0, 0.0]);
        assert_eq!(f.gravity(), [0.0, 1.0, 0.0]);
    }

    #[test]
    fn static_convergence() {
        let mut r = Rng::new(1);
        let mut f = GravityKalman::default();
        for _ in 0..200 {
            let a = [
                noise(&mut r, 0.02),
                1.0 + noise(&mut r, 0.02),
                noise(&mut r, 0.02),
            ];
            f.predict([0.0; 3], DT);
            f.update(a);
        }
        assert!(angle(&f).abs() < 0.8, "{}", angle(&f));
    }

    #[test]
    fn predict_alone_rotates_exactly() {
        // 90 deg about X in one step: g (0,1,0) -> (0,0,-1) in the body frame.
        let mut f = GravityKalman::default();
        f.update([0.0, 1.0, 0.0]);
        f.predict([90.0, 0.0, 0.0], 1.0);
        let g = f.gravity();
        assert!(
            g[0].abs() < 1e-5 && g[1].abs() < 1e-5 && (g[2] + 1.0).abs() < 1e-5,
            "{g:?}"
        );
    }

    #[test]
    fn tracks_clean_rotation() {
        let mut f = GravityKalman::default();
        let rate = 15.0_f32;
        let mut theta = 0.0_f32;
        for _ in 0..(3.0 / DT) as usize {
            theta += rate * DT;
            let t = theta.to_radians();
            f.predict([rate, 0.0, 0.0], DT);
            f.update([0.0, t.cos(), -t.sin()]);
        }
        assert!((angle(&f) - 45.0).abs() < 1.5, "{}", angle(&f));
    }

    #[test]
    fn gyro_carries_through_launch_acceleration() {
        let mut f = GravityKalman::default();
        for _ in 0..100 {
            f.predict([0.0; 3], DT);
            f.update([0.0, 1.0, 0.0]);
        }
        let rate = 10.0_f32;
        let mut theta = 0.0_f32;
        let (mut max_filter, mut max_raw) = (0.0_f32, 0.0_f32);
        for i in 0..(4.0 / DT) as usize {
            let now = i as f32 * DT;
            theta += rate * DT;
            let t = theta.to_radians();
            let kin = if now > 1.0 && now < 3.0 { 0.8 } else { 0.0 };
            let a = [0.0, t.cos() + kin, -t.sin()];
            f.predict([rate, 0.0, 0.0], DT);
            f.update(a);
            if now > 1.0 && now < 3.0 {
                max_filter = max_filter.max((angle(&f) - theta).abs());
                max_raw = max_raw.max((rope_inclination_deg(a) - theta).abs());
            }
        }
        assert!(max_raw > 10.0, "problem is real: {max_raw}");
        assert!(max_filter < 3.0, "filter solves it: {max_filter}");
    }

    #[test]
    fn spin_immunity() {
        let mut r = Rng::new(4);
        let mut f = GravityKalman::default();
        let t = 20.0_f32.to_radians();
        for _ in 0..100 {
            f.predict([0.0; 3], DT);
            f.update([0.0, t.cos(), -t.sin()]);
        }
        let spin = 360.0_f32;
        let mut phi = 0.0_f32;
        for _ in 0..(5.0 / DT) as usize {
            phi += spin.to_radians() * DT;
            let a = [
                t.sin() * phi.sin() + noise(&mut r, 0.02),
                t.cos() + noise(&mut r, 0.02),
                -t.sin() * phi.cos() + noise(&mut r, 0.02),
            ];
            f.predict([0.0, spin, 0.0], DT);
            f.update(a);
            assert!((angle(&f) - 20.0).abs() < 2.0, "{}", angle(&f));
        }
    }
}
