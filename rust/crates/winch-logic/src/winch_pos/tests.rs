use super::*;
use winchy_geo::GeoPoint;
use winchy_gnss::survey::SurveyConfig;

fn survey_with(points: &[(f64, f64)]) -> SurveyIn {
    let mut s = SurveyIn::new(SurveyConfig::default());
    for &(lat, lon) in points {
        s.add(GeoPoint::new(lat, lon), 420.36);
    }
    s
}

#[test]
fn nothing_without_a_position() {
    assert_eq!(build(&survey_with(&[]), false), None);
}

#[test]
fn single_fix_has_maximum_accuracy_and_fix_flag() {
    let p = build(&survey_with(&[(48.746_886_7, 11.449_150_2)]), true).unwrap();
    assert_eq!(p.lat_e7, 487_468_867);
    assert_eq!(p.lon_e7, 114_491_502);
    assert_eq!(p.altitude_dm, 4204); // 420.36 m rounded to dm
    assert_eq!(p.hacc_dm, u16::MAX);
    assert_eq!(p.status, WinchStatus(WinchStatus::FIX));
}

#[test]
fn converged_survey_reports_accuracy_and_done() {
    let pts: Vec<_> = (0..40)
        .map(|i| (48.0 + f64::from(i % 2) * 1e-6, 11.0))
        .collect();
    let s = survey_with(&pts);
    assert!(s.is_converged());
    let p = build(&s, false).unwrap();
    assert_eq!(p.hacc_dm, 5); // floored at 0.5 m
    assert_eq!(p.status, WinchStatus(WinchStatus::SURVEY_DONE));
}

#[test]
fn schedule_is_due_at_once_then_every_period() {
    let mut s = Schedule::new();
    assert!(s.due(0));
    s.sent(1_000);
    assert!(!s.due(1_000 + PERIOD_MS - 1));
    assert!(s.due(1_000 + PERIOD_MS));
}

#[test]
fn failed_send_retries_soon() {
    let mut s = Schedule::new();
    s.failed(5_000);
    assert!(!s.due(5_000 + RETRY_MS - 1));
    assert!(s.due(5_000 + RETRY_MS));
}
