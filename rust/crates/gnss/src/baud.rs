//! Receiver baud-rate discovery and raise (u-blox keeps its baud in BBR,
//! which is lost on a cold start without the backup cell).
//!
//! A pure state machine: the firmware executes each [`ProbeAction`] and feeds
//! the outcome back. Strategy (field-proven): probe the high baud first (warm
//! case); only when the receiver is found at the low baud send the raise
//! command (sending it at the wrong baud injects noise), then confirm.

/// What the caller should do next.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProbeAction {
    /// Open the UART at `baud` and report whether valid framing arrives
    /// within `timeout_ms` (see [`crate::detect::contains_gnss_frame`]).
    Listen {
        /// UART baud.
        baud: u32,
        /// Listen window, ms.
        timeout_ms: u32,
    },
    /// At the current (low) baud, send the CFG-VALSET that raises the
    /// receiver to `to`, then report back (outcome ignored).
    Raise {
        /// Target baud.
        to: u32,
    },
    /// Wait, then report back (outcome ignored).
    Sleep {
        /// Duration, ms.
        ms: u32,
    },
    /// Receiver confirmed at `baud`. Terminal.
    Found {
        /// Working baud.
        baud: u32,
    },
    /// No answer after all attempts; continue assuming `baud`. Terminal.
    GaveUp {
        /// Assumed baud.
        baud: u32,
    },
}

/// Listen window per probe, ms.
pub const LISTEN_MS: u32 = 600;
/// Listen window after a raise, ms.
pub const CONFIRM_MS: u32 = 900;
/// Settling time after the raise command, ms.
pub const SETTLE_MS: u32 = 400;
/// Back-off between attempts while the receiver boots, ms.
pub const BACKOFF_MS: u32 = 500;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Start,
    ListenHigh,
    ListenLow,
    Raised,
    Settled,
    Confirm,
    Backoff,
    Done(ProbeAction),
}

/// Baud discovery state machine.
#[derive(Debug, Clone)]
pub struct BaudProbe {
    low: u32,
    high: u32,
    attempts_left: u32,
    state: State,
}

impl BaudProbe {
    /// Creates a probe for a receiver that boots at `low` and should run at
    /// `high`, trying `attempts` rounds.
    #[must_use]
    pub const fn new(low: u32, high: u32, attempts: u32) -> Self {
        Self {
            low,
            high,
            attempts_left: attempts,
            state: State::Start,
        }
    }

    /// Reports the outcome of the previous action (`alive` matters only after
    /// a `Listen`) and returns the next action. Call with `false` first.
    pub fn step(&mut self, alive: bool) -> ProbeAction {
        let listen = |baud, timeout_ms| ProbeAction::Listen { baud, timeout_ms };
        let (next, action) = match self.state {
            State::Done(a) => (State::Done(a), a),
            State::Start | State::Backoff => {
                if self.attempts_left == 0 {
                    let a = ProbeAction::GaveUp { baud: self.high };
                    (State::Done(a), a)
                } else {
                    self.attempts_left -= 1;
                    (State::ListenHigh, listen(self.high, LISTEN_MS))
                }
            }
            State::ListenHigh | State::Confirm if alive => {
                let a = ProbeAction::Found { baud: self.high };
                (State::Done(a), a)
            }
            State::ListenHigh => (State::ListenLow, listen(self.low, LISTEN_MS)),
            State::ListenLow if alive => (State::Raised, ProbeAction::Raise { to: self.high }),
            State::ListenLow | State::Confirm => {
                (State::Backoff, ProbeAction::Sleep { ms: BACKOFF_MS })
            }
            State::Raised => (State::Settled, ProbeAction::Sleep { ms: SETTLE_MS }),
            State::Settled => (State::Confirm, listen(self.high, CONFIRM_MS)),
        };
        self.state = next;
        action
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ProbeAction::{Found, GaveUp, Listen, Raise, Sleep};

    const LO: u32 = 9_600;
    const HI: u32 = 115_200;

    fn probe(attempts: u32) -> BaudProbe {
        BaudProbe::new(LO, HI, attempts)
    }

    #[test]
    fn warm_receiver_found_at_high_baud() {
        let mut p = probe(12);
        assert_eq!(
            p.step(false),
            Listen {
                baud: HI,
                timeout_ms: LISTEN_MS
            }
        );
        assert_eq!(p.step(true), Found { baud: HI });
    }

    #[test]
    fn cold_receiver_is_raised_and_confirmed() {
        let mut p = probe(12);
        assert_eq!(
            p.step(false),
            Listen {
                baud: HI,
                timeout_ms: LISTEN_MS
            }
        );
        assert_eq!(
            p.step(false),
            Listen {
                baud: LO,
                timeout_ms: LISTEN_MS
            }
        );
        assert_eq!(p.step(true), Raise { to: HI });
        assert_eq!(p.step(false), Sleep { ms: SETTLE_MS });
        assert_eq!(
            p.step(false),
            Listen {
                baud: HI,
                timeout_ms: CONFIRM_MS
            }
        );
        assert_eq!(p.step(true), Found { baud: HI });
    }

    #[test]
    fn unconfirmed_raise_retries_from_high() {
        let mut p = probe(12);
        p.step(false);
        p.step(false);
        p.step(true); // raise
        p.step(false); // settle
        assert_eq!(
            p.step(false),
            Listen {
                baud: HI,
                timeout_ms: CONFIRM_MS
            }
        );
        assert_eq!(p.step(false), Sleep { ms: BACKOFF_MS });
        assert_eq!(
            p.step(false),
            Listen {
                baud: HI,
                timeout_ms: LISTEN_MS
            }
        );
        assert_eq!(p.step(true), Found { baud: HI });
    }

    #[test]
    fn silent_receiver_gives_up_assuming_high() {
        let mut p = probe(2);
        let seq: Vec<ProbeAction> = (0..8).map(|_| p.step(false)).collect();
        assert_eq!(
            seq,
            vec![
                Listen {
                    baud: HI,
                    timeout_ms: LISTEN_MS
                },
                Listen {
                    baud: LO,
                    timeout_ms: LISTEN_MS
                },
                Sleep { ms: BACKOFF_MS },
                Listen {
                    baud: HI,
                    timeout_ms: LISTEN_MS
                },
                Listen {
                    baud: LO,
                    timeout_ms: LISTEN_MS
                },
                Sleep { ms: BACKOFF_MS },
                GaveUp { baud: HI },
                GaveUp { baud: HI },
            ]
        );
    }

    #[test]
    fn zero_attempts_gives_up_immediately() {
        assert_eq!(probe(0).step(false), GaveUp { baud: HI });
    }

    #[test]
    fn terminal_state_is_sticky() {
        let mut p = probe(12);
        p.step(false);
        assert_eq!(p.step(true), Found { baud: HI });
        assert_eq!(p.step(false), Found { baud: HI });
        assert_eq!(p.step(true), Found { baud: HI });
    }
}
