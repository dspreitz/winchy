//! Glider (CG-hook) speed from the rope segment's motion.

/// Speed of the ring in the glider's CG hook (~ glider airspeed, wind ignored).
///
/// The hook sits `hook_distance_m` up the taut rope, a rigid offset whose
/// orientation is the rope angle theta, so `v_H = v_P + L * (thetadot x r)`:
///
/// ```text
/// v_Hx = ground_speed - L * wr * sin(theta)
/// v_Hz = climb_rate   + L * wr * cos(theta)
/// ```
#[must_use]
pub fn glider_speed_ms(
    ground_speed_ms: f32,
    climb_rate_ms: f32,
    rope_angle_deg: f32,
    rope_rate_dps: f32,
    hook_distance_m: f32,
) -> f32 {
    let th = rope_angle_deg.to_radians();
    let wr = rope_rate_dps.to_radians();
    let vx = ground_speed_ms - hook_distance_m * wr * libm::sinf(th);
    let vz = climb_rate_ms + hook_distance_m * wr * libm::cosf(th);
    libm::hypotf(vx, vz)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steady_climb_is_segment_speed() {
        let v = glider_speed_ms(20.0, 15.0, 40.0, 0.0, 5.0);
        assert!((v - 25.0).abs() < 1e-4, "{v}");
    }

    #[test]
    fn ground_roll_without_rotation() {
        assert!((glider_speed_ms(12.0, 0.0, 0.0, 0.0, 5.0) - 12.0).abs() < 1e-5);
    }

    #[test]
    fn rotation_at_horizontal_rope_adds_vertical_component() {
        // theta = 0, rope rotating up at 1 rad/s: hook rises at L * 1 m/s.
        let v = glider_speed_ms(0.0, 0.0, 0.0, 1.0_f32.to_degrees(), 5.0);
        assert!((v - 5.0).abs() < 1e-4, "{v}");
    }

    #[test]
    fn rotation_at_vertical_rope_slows_horizontal_component() {
        let v = glider_speed_ms(10.0, 0.0, 90.0, 1.0_f32.to_degrees(), 5.0);
        assert!((v - 5.0).abs() < 1e-4, "{v}");
    }
}
