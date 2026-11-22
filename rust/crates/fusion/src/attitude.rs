//! Rope angle from a specific-force (gravity) vector.

use crate::Vec3;

/// Angle between the rope (body Y axis) and gravity, degrees.
/// 0 = rope vertical, 90 = rope horizontal. Immune to rope spin about Y.
#[must_use]
pub fn rope_inclination_deg(g: Vec3) -> f32 {
    let norm = libm::sqrtf(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
    if norm <= f32::MIN_POSITIVE {
        return 0.0;
    }
    libm::acosf((g[1].abs() / norm).min(1.0)).to_degrees()
}

/// Rope elevation above the ground ("Seilwinkel"), degrees.
#[must_use]
pub fn rope_angle_above_ground_deg(g: Vec3) -> f32 {
    90.0 - rope_inclination_deg(g)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vertical_rope() {
        assert!(rope_inclination_deg([0.0, 1.0, 0.0]).abs() < 1e-3);
        assert!(rope_inclination_deg([0.0, -1.0, 0.0]).abs() < 1e-3);
        assert!((rope_angle_above_ground_deg([0.0, 1.0, 0.0]) - 90.0).abs() < 1e-3);
    }

    #[test]
    fn horizontal_rope() {
        assert!((rope_inclination_deg([0.0, 0.0, 1.0]) - 90.0).abs() < 1e-3);
        assert!(rope_angle_above_ground_deg([1.0, 0.0, 0.0]).abs() < 1e-3);
    }

    #[test]
    fn independent_of_magnitude_and_spin() {
        let t = 30.0_f32.to_radians();
        for phi in [0.0_f32, 1.0, 2.5, 4.0] {
            let g = [
                2.0 * t.sin() * phi.sin(),
                2.0 * t.cos(),
                -2.0 * t.sin() * phi.cos(),
            ];
            assert!((rope_inclination_deg(g) - 30.0).abs() < 1e-3);
        }
    }

    #[test]
    fn zero_vector_does_not_produce_nan() {
        assert!(rope_inclination_deg([0.0, 0.0, 0.0]).is_finite());
    }

    #[test]
    fn rounding_past_unity_does_not_produce_nan() {
        // |ay|/norm can exceed 1 by an ulp; acos must not see it.
        assert!(rope_inclination_deg([1e-30, 1.0, 0.0]).is_finite());
    }
}
