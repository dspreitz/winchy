//! Ring position over the flash partition: after boot, continue after the
//! block with the highest sequence number.
//!
//! Erasing a flash sector stops every task for up to ~80 ms, so [`Ring`]
//! keeps a reserve of already erased slots ahead of the write position. The
//! firmware refills it only while that stall is harmless (unit idle); a
//! write uses a reserved slot, and only an empty reserve forces an erase
//! right before the write.

/// Where to write next.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Resume {
    /// Block index to write next.
    pub index: usize,
    /// Sequence number for that block.
    pub seq: u32,
}

/// Resume point from the sequence numbers found in each block slot (None =
/// erased or foreign).
#[must_use]
pub fn resume(seqs: impl IntoIterator<Item = Option<u32>>) -> Resume {
    let mut slots = 0;
    let mut newest: Option<(usize, u32)> = None;
    for (i, s) in seqs.into_iter().enumerate() {
        slots = i + 1;
        if let Some(s) = s
            && newest.is_none_or(|(_, n)| s > n)
        {
            newest = Some((i, s));
        }
    }
    match newest {
        Some((i, s)) => Resume {
            index: (i + 1) % slots,
            seq: s.wrapping_add(1),
        },
        None => Resume { index: 0, seq: 0 },
    }
}

/// Number of consecutive erased slots starting at the write position (each
/// item: is that slot's header erased?), at most `limit`.
#[must_use]
pub fn leading_erased(erased: impl IntoIterator<Item = bool>, limit: usize) -> usize {
    erased.into_iter().take(limit).take_while(|e| *e).count()
}

/// One planned block write.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Write {
    /// Slot to write.
    pub index: usize,
    /// Sequence number of the block.
    pub seq: u32,
    /// The slot is not erased yet: erase it first.
    pub erase: bool,
}

/// Write position plus the reserve of erased slots ahead of it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Ring {
    slots: usize,
    index: usize,
    seq: u32,
    ready: usize,
    reserve: usize,
}

impl Ring {
    /// Ring of `slots` slots continuing at `at`, of which `ready` slots from
    /// the write position on are known to be erased; the reserve target is
    /// `reserve` slots (capped below `slots`, so the newest block is never
    /// erased ahead).
    #[must_use]
    pub fn new(slots: usize, at: Resume, ready: usize, reserve: usize) -> Self {
        let reserve = reserve.min(slots.saturating_sub(1));
        Self {
            slots,
            index: at.index,
            seq: at.seq,
            ready: ready.min(reserve),
            reserve,
        }
    }

    /// Plans the next block write and advances past it.
    pub fn next_write(&mut self) -> Write {
        let w = Write {
            index: self.index,
            seq: self.seq,
            erase: self.ready == 0,
        };
        self.ready = self.ready.saturating_sub(1);
        self.index = (self.index + 1) % self.slots.max(1);
        self.seq = self.seq.wrapping_add(1);
        w
    }

    /// Slot to erase next to refill the reserve; `None` if it is full.
    #[must_use]
    pub fn erase_ahead(&self) -> Option<usize> {
        (self.ready < self.reserve).then(|| (self.index + self.ready) % self.slots)
    }

    /// The slot returned by [`erase_ahead`](Self::erase_ahead) was erased.
    pub fn erased_ahead(&mut self) {
        self.ready = (self.ready + 1).min(self.reserve);
    }

    /// Erased slots ahead of the write position.
    #[must_use]
    pub fn ready(&self) -> usize {
        self.ready
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(index: usize, seq: u32) -> Resume {
        Resume { index, seq }
    }

    #[test]
    fn leading_erased_counts_from_the_write_position_up_to_limit() {
        assert_eq!(leading_erased([true, true, false, true], 10), 2);
        assert_eq!(leading_erased([false, true], 10), 0);
        assert_eq!(leading_erased([true; 8], 5), 5);
    }

    #[test]
    fn without_reserve_every_write_erases() {
        let mut r = Ring::new(4, at(2, 7), 0, 0);
        assert_eq!(
            r.next_write(),
            Write {
                index: 2,
                seq: 7,
                erase: true
            }
        );
        assert_eq!(
            r.next_write(),
            Write {
                index: 3,
                seq: 8,
                erase: true
            }
        );
        assert_eq!(
            r.next_write(),
            Write {
                index: 0,
                seq: 9,
                erase: true
            }
        );
        assert_eq!(r.erase_ahead(), None);
    }

    #[test]
    fn known_erased_slots_are_used_without_erasing() {
        let mut r = Ring::new(10, at(8, 0), 2, 4);
        assert_eq!(r.ready(), 2);
        assert_eq!(
            r.next_write(),
            Write {
                index: 8,
                seq: 0,
                erase: false
            }
        );
        assert_eq!(
            r.next_write(),
            Write {
                index: 9,
                seq: 1,
                erase: false
            }
        );
        assert_eq!(
            r.next_write(),
            Write {
                index: 0,
                seq: 2,
                erase: true
            }
        );
    }

    #[test]
    fn refill_erases_the_slots_after_the_reserve_and_wraps() {
        let mut r = Ring::new(10, at(8, 0), 1, 3);
        assert_eq!(r.erase_ahead(), Some(9));
        r.erased_ahead();
        assert_eq!(r.erase_ahead(), Some(0));
        r.erased_ahead();
        assert_eq!(r.erase_ahead(), None); // 3 ready
        assert_eq!(
            r.next_write(),
            Write {
                index: 8,
                seq: 0,
                erase: false
            }
        );
        assert_eq!(r.erase_ahead(), Some(1));
    }

    #[test]
    fn reserve_never_reaches_the_newest_block() {
        let mut r = Ring::new(4, at(0, 0), 0, 100);
        for _ in 0..3 {
            assert!(r.erase_ahead().is_some());
            r.erased_ahead();
        }
        assert_eq!(r.ready(), 3);
        assert_eq!(r.erase_ahead(), None);
        // A known-erased count beyond the cap is clamped too.
        assert_eq!(Ring::new(4, at(0, 0), 4, 100).ready(), 3);
    }

    #[test]
    fn sequence_wraps() {
        let mut r = Ring::new(2, at(0, u32::MAX), 0, 0);
        assert_eq!(r.next_write().seq, u32::MAX);
        assert_eq!(r.next_write().seq, 0);
    }

    #[test]
    fn empty_partition_starts_at_zero() {
        assert_eq!(resume([None, None, None]), Resume { index: 0, seq: 0 });
        assert_eq!(resume([]), Resume { index: 0, seq: 0 });
    }

    #[test]
    fn continues_after_the_newest_block() {
        assert_eq!(
            resume([Some(5), Some(6), None]),
            Resume { index: 2, seq: 7 }
        );
    }

    #[test]
    fn wrapped_ring_overwrites_the_oldest() {
        assert_eq!(
            resume([Some(10), Some(8), Some(9)]),
            Resume { index: 1, seq: 11 }
        );
    }

    #[test]
    fn newest_in_the_last_slot_wraps_to_zero() {
        assert_eq!(
            resume([Some(1), Some(2), Some(3)]),
            Resume { index: 0, seq: 4 }
        );
    }
}
