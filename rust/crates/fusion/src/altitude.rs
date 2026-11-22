//! ISA barometric altitude, total over its whole input range (glitched sensor
//! values are clamped, never produce NaN).

const ISA_SCALE_M: f32 = 44_330.0;
const ISA_EXPONENT: f32 = 5.255;
const ALT_MIN_M: f32 = -1_000.0;
const ALT_MAX_M: f32 = 11_000.0;
const P_MIN_HPA: f32 = 100.0;
const P_MAX_HPA: f32 = 1_100.0;

/// Altitude in metres for `pressure_hpa`, given the sea-level reference (QNH).
#[must_use]
pub fn pressure_to_altitude_m(pressure_hpa: f32, qnh_hpa: f32) -> f32 {
    let p = pressure_hpa.clamp(P_MIN_HPA, P_MAX_HPA);
    let qnh = qnh_hpa.clamp(P_MIN_HPA, P_MAX_HPA);
    ISA_SCALE_M * (1.0 - libm::powf(p / qnh, 1.0 / ISA_EXPONENT))
}

/// Sea-level reference (QNH) such that [`pressure_to_altitude_m`] maps
/// `pressure_hpa` to `altitude_m`. Used to calibrate the baro against GPS.
#[must_use]
pub fn qnh_from_altitude_hpa(pressure_hpa: f32, altitude_m: f32) -> f32 {
    let p = pressure_hpa.clamp(P_MIN_HPA, P_MAX_HPA);
    let alt = altitude_m.clamp(ALT_MIN_M, ALT_MAX_M);
    p / libm::powf(1.0 - alt / ISA_SCALE_M, ISA_EXPONENT)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn standard_atmosphere_sea_level_is_zero() {
        assert!(pressure_to_altitude_m(1013.25, 1013.25).abs() < 1e-3);
    }

    #[test]
    fn known_isa_point() {
        // ISA: 898.75 hPa at ~1000 m.
        let h = pressure_to_altitude_m(898.75, 1013.25);
        assert!((h - 1000.0).abs() < 5.0, "{h}");
    }

    #[test]
    fn lower_pressure_is_higher() {
        assert!(pressure_to_altitude_m(950.0, 1013.25) > pressure_to_altitude_m(960.0, 1013.25));
    }

    #[test]
    fn qnh_roundtrip() {
        for &(p, alt) in &[(950.0_f32, 512.0_f32), (1000.0, 0.0), (850.0, 1500.0)] {
            let qnh = qnh_from_altitude_hpa(p, alt);
            let back = pressure_to_altitude_m(p, qnh);
            assert!((back - alt).abs() < 0.5, "{p} {alt} -> {back}");
        }
    }

    #[test]
    fn total_on_garbage_inputs() {
        for &(p, q) in &[
            (0.0_f32, 1013.0_f32),
            (-5.0, 1013.0),
            (5000.0, 0.0),
            (f32::MAX, 1.0),
        ] {
            assert!(pressure_to_altitude_m(p, q).is_finite(), "{p} {q}");
            assert!(qnh_from_altitude_hpa(p, 1e9).is_finite(), "{p}");
            assert!(qnh_from_altitude_hpa(p, -1e9).is_finite(), "{p}");
        }
    }
}
