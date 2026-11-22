//! Float -> fixed-point quantisation with saturation (NaN maps to 0).

/// Rounds `value * scale` to the nearest `i16`, saturating.
#[must_use]
pub fn to_i16(value: f32, scale: f32) -> i16 {
    let x = libm_round_f32(value * scale);
    if x.is_nan() {
        0
    } else {
        x.clamp(f32::from(i16::MIN), f32::from(i16::MAX)) as i16
    }
}

/// Rounds `value * scale` to the nearest `u16`, saturating.
#[must_use]
pub fn to_u16(value: f32, scale: f32) -> u16 {
    let x = libm_round_f32(value * scale);
    if x.is_nan() {
        0
    } else {
        x.clamp(f32::from(u16::MIN), f32::from(u16::MAX)) as u16
    }
}

/// Rounds `value * scale` to the nearest `i32`, saturating.
#[must_use]
pub fn to_i32(value: f64, scale: f64) -> i32 {
    let x = libm_round_f64(value * scale);
    if x.is_nan() {
        0
    } else {
        x.clamp(f64::from(i32::MIN), f64::from(i32::MAX)) as i32
    }
}

/// Rounds `value * scale` to the nearest `u8`, saturating.
#[must_use]
pub fn to_u8(value: f32, scale: f32) -> u8 {
    let x = libm_round_f32(value * scale);
    if x.is_nan() {
        0
    } else {
        x.clamp(f32::from(u8::MIN), f32::from(u8::MAX)) as u8
    }
}

/// Rounds `value * scale` to the nearest `i8`, saturating.
#[must_use]
pub fn to_i8(value: f32, scale: f32) -> i8 {
    let x = libm_round_f32(value * scale);
    if x.is_nan() {
        0
    } else {
        x.clamp(f32::from(i8::MIN), f32::from(i8::MAX)) as i8
    }
}

/// Round half away from zero without `std` (exact for |x| < 2^23).
fn libm_round_f32(x: f32) -> f32 {
    if x.is_nan() || x.abs() >= 8_388_608.0 {
        return x;
    }
    let t = (x.abs() + 0.5) as i32 as f32;
    if x < 0.0 { -t } else { t }
}

/// Round half away from zero without `std` (exact for |x| < 2^52).
fn libm_round_f64(x: f64) -> f64 {
    if x.is_nan() || x.abs() >= 4_503_599_627_370_496.0 {
        return x;
    }
    let t = (x.abs() + 0.5) as i64 as f64;
    if x < 0.0 { -t } else { t }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rounds_to_nearest() {
        assert_eq!(to_i16(12.344, 100.0), 1234);
        assert_eq!(to_i16(12.346, 100.0), 1235);
        assert_eq!(to_i16(-12.346, 100.0), -1235);
        assert_eq!(to_u16(2.55, 10.0), 26);
        assert_eq!(to_i32(48.123_456_78, 1e7), 481_234_568);
        assert_eq!(to_u8(0.44, 10.0), 4);
        assert_eq!(to_i8(-3.6, 1.0), -4);
    }

    #[test]
    fn saturates() {
        assert_eq!(to_i16(1e9, 1.0), i16::MAX);
        assert_eq!(to_i16(-1e9, 1.0), i16::MIN);
        assert_eq!(to_u16(-5.0, 1.0), 0);
        assert_eq!(to_u16(1e9, 1.0), u16::MAX);
        assert_eq!(to_i32(1e12, 1.0), i32::MAX);
        assert_eq!(to_i32(-1e12, 1.0), i32::MIN);
        assert_eq!(to_u8(300.0, 1.0), u8::MAX);
        assert_eq!(to_i8(-300.0, 1.0), i8::MIN);
    }

    #[test]
    fn nan_is_zero() {
        assert_eq!(to_i16(f32::NAN, 1.0), 0);
        assert_eq!(to_u16(f32::NAN, 1.0), 0);
        assert_eq!(to_i32(f64::NAN, 1.0), 0);
        assert_eq!(to_u8(f32::NAN, 1.0), 0);
        assert_eq!(to_i8(f32::NAN, 1.0), 0);
    }
}
