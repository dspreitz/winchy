//! Rope segment position relative to the winch.

use winchy_geo::{GeoPoint, WinchRelative, winch_relative};

use crate::state::RopeState;

/// Rope length between the segment and the glider's CG hook, m.
pub const HOOK_DISTANCE_M: f64 = 5.0;

/// Winch-relative geometry, once the winch position and our own fix exist.
#[must_use]
pub fn winch_geometry(s: &RopeState) -> Option<WinchRelative> {
    let w = s.winch.filter(|_| s.gnss.fix)?.pos;
    Some(winch_relative(
        GeoPoint::new(f64::from(w.lat_e7) * 1e-7, f64::from(w.lon_e7) * 1e-7),
        f64::from(w.altitude_dm) / 10.0,
        GeoPoint::new(s.gnss.lat_deg, s.gnss.lon_deg),
        f64::from(s.gnss.alt_m),
        HOOK_DISTANCE_M,
    ))
}

/// Distance prior for ADR: the last known slant distance to the winch (kept
/// while the link is down - the winch is parked).
#[must_use]
pub fn adr_distance_m(s: &RopeState) -> Option<f32> {
    s.geometry.map(|g| g.slant_m as f32).filter(|&d| d > 0.0)
}

#[cfg(test)]
mod tests {
    use winchy_protocol::{WinchPos, WinchStatus};

    use super::*;
    use crate::state::Winch;

    fn with_winch() -> RopeState {
        let mut s = RopeState::new(14);
        s.winch = Some(Winch {
            pos: WinchPos {
                lat_e7: 480_000_000,
                lon_e7: 110_000_000,
                altitude_dm: 5_000,
                hacc_dm: 10,
                status: WinchStatus(3),
            },
            received_ms: 0,
        });
        s.gnss.lat_deg = 48.01;
        s.gnss.lon_deg = 11.0;
        s.gnss.alt_m = 800.0;
        s
    }

    #[test]
    fn needs_winch_and_own_fix() {
        assert_eq!(winch_geometry(&RopeState::new(14)), None);
        assert_eq!(winch_geometry(&with_winch()), None, "no own fix");
        let mut no_winch = with_winch();
        no_winch.gnss.fix = true;
        no_winch.winch = None;
        assert_eq!(winch_geometry(&no_winch), None);
    }

    #[test]
    fn matches_the_geo_crate() {
        let mut s = with_winch();
        s.gnss.fix = true;
        let expected = winch_relative(
            GeoPoint::new(48.0, 11.0),
            500.0,
            GeoPoint::new(48.01, 11.0),
            800.0,
            HOOK_DISTANCE_M,
        );
        let got = winch_geometry(&s).unwrap();
        assert!((got.slant_m - expected.slant_m).abs() < 1e-6);
        assert!((got.cable_length_m - expected.cable_length_m).abs() < 1e-6);
    }

    #[test]
    fn adr_distance_from_the_last_geometry() {
        let mut s = with_winch();
        assert_eq!(adr_distance_m(&s), None);
        s.gnss.fix = true;
        s.geometry = winch_geometry(&s);
        let d = adr_distance_m(&s).unwrap();
        assert!((f64::from(d) - s.geometry.unwrap().slant_m).abs() < 0.01);
        s.gnss.fix = false; // link down / fix lost: keep the last distance
        assert!(adr_distance_m(&s).is_some());
    }
}
