use super::*;

const STILL: [f32; 3] = [0.5, -1.0, 0.8];

#[test]
fn opens_after_lying_still_long_enough() {
    let mut g = EraseGate::new();
    assert!(!g.update(1_000, STILL, true));
    assert!(!g.update(1_000 + STILL_FOR_MS - 1, STILL, true));
    assert!(g.update(1_000 + STILL_FOR_MS, STILL, true));
}

#[test]
fn motion_restarts_the_wait() {
    let mut g = EraseGate::new();
    g.update(0, STILL, true);
    assert!(g.update(STILL_FOR_MS, STILL, true));
    assert!(!g.update(STILL_FOR_MS + 10, [0.0, 0.0, 4.0], true));
    assert!(!g.update(STILL_FOR_MS + 20, STILL, true));
    assert!(g.update(2 * STILL_FOR_MS + 20, STILL, true));
}

#[test]
fn magnitude_counts_not_single_axes() {
    let mut g = EraseGate::new();
    g.update(0, [2.0, 2.0, 2.0], true); // |w| = 3.46 dps
    assert!(!g.update(STILL_FOR_MS, [2.0, 2.0, 2.0], true));
}

#[test]
fn never_during_a_tow() {
    let mut g = EraseGate::new();
    g.update(0, STILL, true);
    assert!(!g.update(STILL_FOR_MS, STILL, false));
    assert!(!g.update(2 * STILL_FOR_MS, STILL, true)); // wait restarts after the tow
}
