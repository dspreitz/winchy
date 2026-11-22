//! Ring position over the flash partition: after boot, continue after the
//! block with the highest sequence number.

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

#[cfg(test)]
mod tests {
    use super::*;

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
