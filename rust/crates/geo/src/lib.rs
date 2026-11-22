//! Local-tangent-plane geometry between two GNSS fixes.
//!
//! Everything here works on an equirectangular approximation around the mean
//! latitude, which is exact to centimetres over the < 2 km of a winch launch.
//! Positions use `f64`: an `f32` latitude only resolves ~0.4 m.
#![cfg_attr(not(test), no_std)]

/// Mean Earth radius in metres.
pub const EARTH_RADIUS_M: f64 = 6_371_000.0;

/// A geodetic position in decimal degrees (WGS84).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeoPoint {
    /// Latitude, degrees north.
    pub lat_deg: f64,
    /// Longitude, degrees east.
    pub lon_deg: f64,
}

impl GeoPoint {
    /// Creates a point from latitude and longitude in degrees.
    #[must_use]
    pub const fn new(lat_deg: f64, lon_deg: f64) -> Self {
        Self { lat_deg, lon_deg }
    }
}

/// East/north offset in metres on the local tangent plane.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Enu {
    /// Metres east.
    pub east_m: f64,
    /// Metres north.
    pub north_m: f64,
}

/// East/north offset of `p` relative to `reference`.
#[must_use]
pub fn enu_offset(reference: GeoPoint, p: GeoPoint) -> Enu {
    let mean_lat = f64::midpoint(reference.lat_deg, p.lat_deg).to_radians();
    Enu {
        east_m: (p.lon_deg - reference.lon_deg).to_radians() * libm::cos(mean_lat) * EARTH_RADIUS_M,
        north_m: (p.lat_deg - reference.lat_deg).to_radians() * EARTH_RADIUS_M,
    }
}

/// Ground distance between two points in metres.
#[must_use]
pub fn horizontal_distance_m(a: GeoPoint, b: GeoPoint) -> f64 {
    let o = enu_offset(a, b);
    libm::hypot(o.east_m, o.north_m)
}

/// Initial bearing from `from` to `to`, degrees clockwise from north, `[0, 360)`.
#[must_use]
pub fn bearing_deg(from: GeoPoint, to: GeoPoint) -> f64 {
    let o = enu_offset(from, to);
    let deg = libm::atan2(o.east_m, o.north_m).to_degrees();
    if deg < 0.0 { deg + 360.0 } else { deg }
}

/// Elevation angle (degrees) of a point `dh_m` above and `horizontal_m` away.
#[must_use]
pub fn elevation_angle_deg(horizontal_m: f64, dh_m: f64) -> f64 {
    libm::atan2(dh_m, horizontal_m).to_degrees()
}

/// Straight-line distance from a horizontal separation and a height delta.
#[must_use]
pub fn slant_distance_m(horizontal_m: f64, dh_m: f64) -> f64 {
    libm::hypot(horizontal_m, dh_m)
}

/// Geometry of the rope segment as seen from the winch.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WinchRelative {
    /// Ground distance winch -> rope segment, m.
    pub horizontal_m: f64,
    /// Rope segment height above the winch, m.
    pub dh_m: f64,
    /// Straight-line distance winch -> rope segment, m.
    pub slant_m: f64,
    /// Elevation of the rope segment seen from the winch, degrees.
    pub elevation_deg: f64,
    /// Bearing winch -> rope segment (ground-roll heading at launch), degrees.
    pub bearing_deg: f64,
    /// Winch -> glider hook distance (slant + hook offset), m.
    pub cable_length_m: f64,
}

/// Bundles the winch-relative geometry from both fixes and altitudes.
///
/// `hook_distance_m` is the rope length between the segment and the glider's
/// CG hook. At force onset (both ends on the ground) `cable_length_m` is the
/// initial cable length; during the climb it is the live winch->glider distance.
#[must_use]
pub fn winch_relative(
    winch: GeoPoint,
    winch_alt_m: f64,
    rope: GeoPoint,
    rope_alt_m: f64,
    hook_distance_m: f64,
) -> WinchRelative {
    let horizontal_m = horizontal_distance_m(winch, rope);
    let dh_m = rope_alt_m - winch_alt_m;
    let slant_m = slant_distance_m(horizontal_m, dh_m);
    WinchRelative {
        horizontal_m,
        dh_m,
        slant_m,
        elevation_deg: elevation_angle_deg(horizontal_m, dh_m),
        bearing_deg: bearing_deg(winch, rope),
        cable_length_m: slant_m + hook_distance_m,
    }
}

#[cfg(test)]
mod tests;
