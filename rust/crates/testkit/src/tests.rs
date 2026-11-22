use super::Rng;

#[test]
fn same_seed_same_sequence() {
    let mut a = Rng::new(42);
    let mut b = Rng::new(42);
    for _ in 0..100 {
        assert_eq!(a.next_u64(), b.next_u64());
    }
}

#[test]
fn different_seeds_differ() {
    assert_ne!(Rng::new(1).next_u64(), Rng::new(2).next_u64());
}

#[test]
fn zero_seed_is_usable() {
    let mut r = Rng::new(0);
    assert_ne!(r.next_u64(), r.next_u64());
}

#[test]
fn uniform_stays_in_unit_interval() {
    let mut r = Rng::new(7);
    for _ in 0..10_000 {
        let u = r.uniform();
        assert!((0.0..1.0).contains(&u));
    }
}

#[test]
fn gauss_has_requested_moments() {
    let mut r = Rng::new(3);
    let n = 20_000;
    let samples: Vec<f64> = (0..n).map(|_| r.gauss(5.0, 2.0)).collect();
    let mean = samples.iter().sum::<f64>() / f64::from(n);
    let var = samples.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / f64::from(n);
    assert!((mean - 5.0).abs() < 0.05, "mean {mean}");
    assert!((var.sqrt() - 2.0).abs() < 0.05, "sigma {}", var.sqrt());
}
