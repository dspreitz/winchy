use super::*;

// One degree of latitude on the 6371 km sphere.
const M_PER_DEG: f64 = EARTH_RADIUS_M * core::f64::consts::PI / 180.0;
const WINCH: GeoPoint = GeoPoint::new(48.5, 11.5);

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

#[test]
fn enu_of_same_point_is_zero() {
    let o = enu_offset(WINCH, WINCH);
    assert!(close(o.east_m, 0.0, 1e-9) && close(o.north_m, 0.0, 1e-9));
}

#[test]
fn enu_north_offset() {
    let o = enu_offset(WINCH, GeoPoint::new(48.501, 11.5));
    assert!(close(o.north_m, 0.001 * M_PER_DEG, 1e-6), "{o:?}");
    assert!(close(o.east_m, 0.0, 1e-9));
}

#[test]
fn enu_east_offset_shrinks_with_latitude() {
    let o = enu_offset(WINCH, GeoPoint::new(48.5, 11.501));
    let expected = 0.001 * M_PER_DEG * 48.5_f64.to_radians().cos();
    assert!(close(o.east_m, expected, 1e-6), "{o:?}");
    assert!(close(o.north_m, 0.0, 1e-9));
}

#[test]
fn distance_is_symmetric_and_pythagorean() {
    let p = GeoPoint::new(48.503, 11.504);
    let d1 = horizontal_distance_m(WINCH, p);
    let d2 = horizontal_distance_m(p, WINCH);
    let o = enu_offset(WINCH, p);
    assert!(close(d1, d2, 1e-6));
    assert!(close(d1, o.east_m.hypot(o.north_m), 1e-6));
}

#[test]
fn bearing_cardinal_directions() {
    assert!(close(
        bearing_deg(WINCH, GeoPoint::new(48.51, 11.5)),
        0.0,
        1e-9
    ));
    assert!(close(
        bearing_deg(WINCH, GeoPoint::new(48.5, 11.51)),
        90.0,
        1e-9
    ));
    assert!(close(
        bearing_deg(WINCH, GeoPoint::new(48.49, 11.5)),
        180.0,
        1e-9
    ));
    assert!(close(
        bearing_deg(WINCH, GeoPoint::new(48.5, 11.49)),
        270.0,
        1e-9
    ));
}

#[test]
fn bearing_is_in_range() {
    let b = bearing_deg(WINCH, GeoPoint::new(48.51, 11.49));
    assert!(
        (0.0..360.0).contains(&b) && close(b, 360.0 - 33.5, 1.0),
        "{b}"
    );
}

#[test]
fn elevation_angle() {
    assert!(close(elevation_angle_deg(100.0, 0.0), 0.0, 1e-12));
    assert!(close(elevation_angle_deg(100.0, 100.0), 45.0, 1e-9));
    assert!(close(elevation_angle_deg(0.0, 50.0), 90.0, 1e-9));
    assert!(close(elevation_angle_deg(100.0, -100.0), -45.0, 1e-9));
}

#[test]
fn slant_distance() {
    assert!(close(slant_distance_m(300.0, 400.0), 500.0, 1e-9));
}

#[test]
fn winch_relative_on_ground_at_launch_start() {
    // Glider 1000 m north, same height: cable = horizontal + hook offset.
    let rope = GeoPoint::new(48.5 + 1000.0 / M_PER_DEG, 11.5);
    let g = winch_relative(WINCH, 500.0, rope, 500.0, 5.0);
    assert!(close(g.horizontal_m, 1000.0, 1e-6));
    assert!(close(g.dh_m, 0.0, 1e-12));
    assert!(close(g.slant_m, 1000.0, 1e-6));
    assert!(close(g.elevation_deg, 0.0, 1e-9));
    assert!(close(g.bearing_deg, 0.0, 1e-9));
    assert!(close(g.cable_length_m, 1005.0, 1e-6));
}

#[test]
fn winch_relative_in_climb() {
    let rope = GeoPoint::new(48.5 + 300.0 / M_PER_DEG, 11.5);
    let g = winch_relative(WINCH, 500.0, rope, 900.0, 5.0);
    assert!(close(g.dh_m, 400.0, 1e-9));
    assert!(close(g.slant_m, 500.0, 1e-6));
    assert!(close(g.elevation_deg, 53.130_102, 1e-5));
    assert!(close(g.cable_length_m, 505.0, 1e-6));
}
