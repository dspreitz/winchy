//! Barometric sea-level reference (QNH): seeded from the last known field
//! elevation at boot, calibrated against confident GNSS fixes while idle.

use winchy_fusion::altitude::qnh_from_altitude_hpa;

use crate::last_fix::plausible_altitude;

/// Tuning of [`Qnh`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QnhConfig {
    /// EMA weight per barometer sample once calibrated (~30 s at 1 Hz).
    pub alpha: f32,
    /// Minimum satellites for a calibrating fix.
    pub min_sats: u8,
    /// Maximum horizontal accuracy for a calibrating fix, m.
    pub max_hacc_m: f32,
    /// Maximum age of the fix, ms.
    pub max_age_ms: u64,
    /// A fix this far from the baro altitude is a first-lock outlier, m...
    pub max_jump_m: f32,
    /// ...unless it persists this many samples (the unit really moved).
    pub far_hits: u32,
    /// Reference before any seed or fix, hPa.
    pub default_hpa: f32,
}

impl Default for QnhConfig {
    fn default() -> Self {
        Self {
            alpha: 0.1,
            min_sats: 5,
            max_hacc_m: 5.0,
            max_age_ms: 5_000,
            max_jump_m: 100.0,
            far_hits: 10,
            default_hpa: 1_013.25,
        }
    }
}

/// A GNSS altitude offered for calibration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GnssAltitude {
    /// Height above mean sea level, m.
    pub alt_m: f32,
    /// Satellites used.
    pub sats: u8,
    /// Horizontal accuracy, m.
    pub hacc_m: f32,
    /// Age of the solution, ms.
    pub age_ms: u64,
}

/// Outcome of one [`Qnh::update`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QnhUpdate {
    /// Reference unchanged.
    Unchanged,
    /// First GNSS calibration: the reference jumped; reset the altitude filter
    /// so the jump is not read as a climb.
    Snapped,
    /// Reference nudged towards the GNSS altitude.
    Blended,
}

/// QNH estimator.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Qnh {
    cfg: QnhConfig,
    reference: Option<f32>,
    from_gnss: bool,
    far: u32,
}

impl Qnh {
    /// Estimator without a reference (uses `default_hpa`).
    #[must_use]
    pub const fn new(cfg: QnhConfig) -> Self {
        Self {
            cfg,
            reference: None,
            from_gnss: false,
            far: 0,
        }
    }

    /// Seeds the reference from a stored field elevation (only before any
    /// reference exists; implausible altitudes are refused).
    pub fn seed(&mut self, pressure_hpa: f32, alt_m: f32) -> bool {
        if self.reference.is_some() || !plausible_altitude(alt_m) {
            return false;
        }
        self.reference = Some(qnh_from_altitude_hpa(pressure_hpa, alt_m));
        true
    }

    /// Offers one barometer sample with the latest GNSS fix.
    pub fn update(
        &mut self,
        pressure_hpa: f32,
        fix: Option<GnssAltitude>,
        baro_alt_m: Option<f32>,
        idle: bool,
    ) -> QnhUpdate {
        let c = self.cfg;
        let Some(fix) = fix.filter(|f| {
            idle && f.age_ms <= c.max_age_ms && f.sats >= c.min_sats && f.hacc_m <= c.max_hacc_m
        }) else {
            return QnhUpdate::Unchanged;
        };
        let far = baro_alt_m.is_some_and(|b| (fix.alt_m - b).abs() > c.max_jump_m);
        if far && self.far < c.far_hits {
            self.far += 1;
            return QnhUpdate::Unchanged;
        }
        self.far = 0;
        let reference = qnh_from_altitude_hpa(pressure_hpa, fix.alt_m);
        match self.reference {
            Some(q) if self.from_gnss => {
                self.reference = Some(q + c.alpha * (reference - q));
                QnhUpdate::Blended
            }
            _ => {
                self.reference = Some(reference);
                self.from_gnss = true;
                QnhUpdate::Snapped
            }
        }
    }

    /// Reference to use now, hPa.
    #[must_use]
    pub fn hpa(&self) -> f32 {
        self.reference.unwrap_or(self.cfg.default_hpa)
    }

    /// True once a GNSS fix calibrated the reference.
    #[must_use]
    pub const fn from_gnss(&self) -> bool {
        self.from_gnss
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const P: f32 = 965.0;

    #[expect(
        clippy::unnecessary_wraps,
        reason = "tests pass it where an Option is taken"
    )]
    fn good(alt_m: f32) -> Option<GnssAltitude> {
        Some(GnssAltitude {
            alt_m,
            sats: 9,
            hacc_m: 2.0,
            age_ms: 100,
        })
    }

    fn close(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-3
    }

    #[test]
    fn standard_pressure_until_seeded() {
        let q = Qnh::new(QnhConfig::default());
        assert!(close(q.hpa(), 1_013.25));
        assert!(!q.from_gnss());
    }

    #[test]
    fn seed_uses_the_stored_elevation_once() {
        let mut q = Qnh::new(QnhConfig::default());
        assert!(q.seed(P, 400.0));
        assert!(close(q.hpa(), qnh_from_altitude_hpa(P, 400.0)));
        assert!(!q.seed(P, 0.0), "only the first seed counts");
        assert!(!q.from_gnss());
    }

    #[test]
    fn implausible_seed_is_refused() {
        let mut q = Qnh::new(QnhConfig::default());
        assert!(!q.seed(P, 58_316.0));
        assert!(close(q.hpa(), 1_013.25));
    }

    #[test]
    fn first_good_fix_snaps_then_blends() {
        let mut q = Qnh::new(QnhConfig::default());
        assert_eq!(
            q.update(P, good(400.0), Some(390.0), true),
            QnhUpdate::Snapped
        );
        let first = qnh_from_altitude_hpa(P, 400.0);
        assert!(close(q.hpa(), first));
        assert!(q.from_gnss());

        assert_eq!(
            q.update(P, good(410.0), Some(400.0), true),
            QnhUpdate::Blended
        );
        let target = qnh_from_altitude_hpa(P, 410.0);
        assert!(close(q.hpa(), first + 0.1 * (target - first)));
    }

    #[test]
    fn weak_stale_or_missing_fixes_and_towing_change_nothing() {
        let cfg = QnhConfig::default();
        let cases = [
            (None, true),
            (good(400.0).map(|f| GnssAltitude { sats: 4, ..f }), true),
            (good(400.0).map(|f| GnssAltitude { hacc_m: 5.1, ..f }), true),
            (
                good(400.0).map(|f| GnssAltitude { age_ms: 5_001, ..f }),
                true,
            ),
            (good(400.0), false),
        ];
        for (fix, idle) in cases {
            let mut q = Qnh::new(cfg);
            assert_eq!(
                q.update(P, fix, Some(400.0), idle),
                QnhUpdate::Unchanged,
                "{fix:?} {idle}"
            );
            assert!(!q.from_gnss());
        }
    }

    #[test]
    fn outlier_is_ignored_unless_it_persists() {
        let mut q = Qnh::new(QnhConfig::default());
        for i in 0..10 {
            assert_eq!(
                q.update(P, good(600.0), Some(400.0), true),
                QnhUpdate::Unchanged,
                "{i}"
            );
        }
        assert_eq!(
            q.update(P, good(600.0), Some(400.0), true),
            QnhUpdate::Snapped
        );
    }

    #[test]
    fn a_close_fix_resets_the_outlier_count() {
        let mut q = Qnh::new(QnhConfig::default());
        for _ in 0..9 {
            q.update(P, good(600.0), Some(400.0), true);
        }
        assert_eq!(
            q.update(P, good(401.0), Some(400.0), true),
            QnhUpdate::Snapped
        );
        for i in 0..10 {
            assert_eq!(
                q.update(P, good(600.0), Some(400.0), true),
                QnhUpdate::Unchanged,
                "{i}"
            );
        }
    }

    #[test]
    fn without_a_baro_altitude_there_is_no_outlier_check() {
        let mut q = Qnh::new(QnhConfig::default());
        assert_eq!(q.update(P, good(600.0), None, true), QnhUpdate::Snapped);
    }
}
