//! 4 KiB log blocks: a CRC-protected header (sequence, base time, UTC
//! offset) followed by records `kind u8 | dt_us i32 | payload`.

use crate::record::{MAX_PAYLOAD, Record};

/// Block size (one flash sector).
pub const BLOCK_SIZE: usize = 4096;
/// Header size.
pub const HEADER_SIZE: usize = 32;
/// Header magic ("WLG1").
pub const MAGIC: u32 = 0x3147_4C57;
/// A UTC offset differing by more than this (clock stepped) starts a new block.
pub const OFFSET_TOLERANCE_US: i64 = 1_000;
/// Per-record overhead (kind + time delta).
pub const RECORD_OVERHEAD: usize = 5;
/// Header value for "UTC offset unknown".
const NO_OFFSET: i64 = i64::MIN;

/// Decoded block header.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BlockHeader {
    /// Ring sequence number.
    pub seq: u32,
    /// Monotonic time the record deltas refer to, µs since boot.
    pub base_mono_us: u64,
    /// UTC minus monotonic, µs (None: clock not set while writing).
    pub utc_offset_us: Option<i64>,
    /// Record bytes after the header.
    pub used: u16,
}

impl BlockHeader {
    /// Sequence number if `header` starts with the magic (cheap ring scan;
    /// the CRC is checked by [`entries`]).
    #[must_use]
    pub fn peek_seq(header: &[u8]) -> Option<u32> {
        let magic = u32::from_le_bytes(header.get(..4)?.try_into().ok()?);
        (magic == MAGIC).then(|| u32::from_le_bytes([header[4], header[5], header[6], header[7]]))
    }
}

/// One decoded measurement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Entry {
    /// Measurement time, µs since boot.
    pub mono_us: u64,
    /// Measurement time, µs since the Unix epoch (None: clock not set).
    pub utc_us: Option<i64>,
    /// The measurement.
    pub record: Record,
}

/// Iterator over the records of a verified block.
#[derive(Debug, Clone)]
pub struct Entries<'a> {
    header: BlockHeader,
    data: &'a [u8],
}

impl Iterator for Entries<'_> {
    type Item = Entry;

    fn next(&mut self) -> Option<Entry> {
        let (&kind, rest) = self.data.split_first()?;
        let len = Record::payload_len(kind)?;
        let (dt, rest) = rest.split_at_checked(4)?;
        let (payload, rest) = rest.split_at_checked(len)?;
        let record = Record::decode(kind, payload)?;
        self.data = rest;
        let dt = i64::from(i32::from_le_bytes([dt[0], dt[1], dt[2], dt[3]]));
        let mono = i64::try_from(self.header.base_mono_us).unwrap_or(i64::MAX) + dt;
        Some(Entry {
            mono_us: u64::try_from(mono).unwrap_or(0),
            utc_us: self.header.utc_offset_us.map(|o| mono + o),
            record,
        })
    }
}

/// Verifies a block and iterates its records; None for erased, foreign or
/// torn blocks.
#[must_use]
pub fn entries(block: &[u8]) -> Option<(BlockHeader, Entries<'_>)> {
    let h = block.get(..HEADER_SIZE)?;
    let seq = BlockHeader::peek_seq(h)?;
    let u64_at = |o: usize| {
        let mut b = [0u8; 8];
        b.copy_from_slice(&h[o..o + 8]);
        b
    };
    let used = u16::from_le_bytes([h[24], h[25]]);
    let data = block.get(HEADER_SIZE..HEADER_SIZE + usize::from(used))?;
    let crc = u32::from_le_bytes([h[28], h[29], h[30], h[31]]);
    if crc32(&[&h[..28], data]) != crc {
        return None;
    }
    let offset = i64::from_le_bytes(u64_at(16));
    let header = BlockHeader {
        seq,
        base_mono_us: u64::from_le_bytes(u64_at(8)),
        utc_offset_us: (offset != NO_OFFSET).then_some(offset),
        used,
    };
    Some((header, Entries { header, data }))
}

/// Packs records into one block.
#[derive(Debug, Clone)]
pub struct BlockWriter {
    buf: [u8; BLOCK_SIZE],
    used: usize,
    base_mono_us: u64,
    utc_offset_us: Option<i64>,
}

impl Default for BlockWriter {
    fn default() -> Self {
        Self::new()
    }
}

impl BlockWriter {
    /// Empty writer.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            buf: [0xFF; BLOCK_SIZE],
            used: 0,
            base_mono_us: 0,
            utc_offset_us: None,
        }
    }

    /// True if no record is buffered.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.used == 0
    }

    /// Appends a record measured at `mono_us` while UTC = mono + offset.
    /// Returns false (record not added) if the block must be sealed first:
    /// full, clock stepped, or time delta out of range.
    pub fn push(&mut self, mono_us: u64, utc_offset_us: Option<i64>, record: &Record) -> bool {
        if self.is_empty() {
            self.base_mono_us = mono_us;
            self.utc_offset_us = utc_offset_us;
        }
        let same_clock = match (self.utc_offset_us, utc_offset_us) {
            (None, None) => true,
            (Some(a), Some(b)) => (a - b).abs() <= OFFSET_TOLERANCE_US,
            _ => false,
        };
        let dt = i128::from(mono_us) - i128::from(self.base_mono_us);
        let Ok(dt) = i32::try_from(dt) else {
            return false;
        };
        let mut payload = [0u8; MAX_PAYLOAD];
        let len = record.encode_payload(&mut payload);
        let start = HEADER_SIZE + self.used;
        let end = start + RECORD_OVERHEAD + len;
        if !same_clock || end > BLOCK_SIZE {
            return false;
        }
        self.buf[start] = record.kind();
        self.buf[start + 1..start + RECORD_OVERHEAD].copy_from_slice(&dt.to_le_bytes());
        self.buf[start + RECORD_OVERHEAD..end].copy_from_slice(&payload[..len]);
        self.used += RECORD_OVERHEAD + len;
        true
    }

    /// Finishes the block with sequence `seq` and returns it (None if empty);
    /// the writer is empty afterwards.
    pub fn seal(&mut self, seq: u32) -> Option<&[u8; BLOCK_SIZE]> {
        if self.is_empty() {
            return None;
        }
        let used = u16::try_from(self.used).unwrap_or(u16::MAX);
        let h = &mut self.buf[..HEADER_SIZE];
        h[0..4].copy_from_slice(&MAGIC.to_le_bytes());
        h[4..8].copy_from_slice(&seq.to_le_bytes());
        h[8..16].copy_from_slice(&self.base_mono_us.to_le_bytes());
        h[16..24].copy_from_slice(&self.utc_offset_us.unwrap_or(NO_OFFSET).to_le_bytes());
        h[24..26].copy_from_slice(&used.to_le_bytes());
        h[26..28].copy_from_slice(&[0xFF, 0xFF]);
        let crc = crc32(&[
            &self.buf[..28],
            &self.buf[HEADER_SIZE..HEADER_SIZE + self.used],
        ]);
        self.buf[28..32].copy_from_slice(&crc.to_le_bytes());
        self.used = 0;
        Some(&self.buf)
    }
}

/// CRC-32 (IEEE, as zlib).
#[must_use]
pub fn crc32(parts: &[&[u8]]) -> u32 {
    let mut crc = !0u32;
    for &byte in parts.iter().flat_map(|p| p.iter()) {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

#[cfg(test)]
mod tests;
