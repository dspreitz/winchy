//! Deterministic test helpers for Winchy host tests.
//!
//! Simulation tests (Kalman filters, survey-in, bias tracking) need repeatable
//! noise. [`Rng`] is a seeded xorshift64* generator with a Box-Muller Gaussian,
//! so every test run sees exactly the same samples.

/// Seeded pseudo-random generator (xorshift64*), reproducible across runs.
#[derive(Debug, Clone)]
pub struct Rng {
    state: u64,
}

impl Rng {
    /// Creates a generator from `seed` (a zero seed is remapped internally).
    #[must_use]
    pub fn new(seed: u64) -> Self {
        // xorshift has an all-zero fixed point; splitmix the seed away from it.
        let mixed = seed.wrapping_add(0x9E37_79B9_7F4A_7C15);
        Self {
            state: if mixed == 0 { 1 } else { mixed },
        }
    }

    /// Next raw 64-bit value.
    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.state = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// Uniform sample in `[0, 1)`.
    pub fn uniform(&mut self) -> f64 {
        // Top 53 bits -> exact f64 mantissa in [0, 1).
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Gaussian sample with the given mean and standard deviation.
    pub fn gauss(&mut self, mean: f64, sigma: f64) -> f64 {
        // Box-Muller; 1 - u keeps the log argument in (0, 1].
        let u1 = 1.0 - self.uniform();
        let u2 = self.uniform();
        let z = (-2.0 * u1.ln()).sqrt() * (core::f64::consts::TAU * u2).cos();
        mean + sigma * z
    }
}

#[cfg(test)]
mod tests;
