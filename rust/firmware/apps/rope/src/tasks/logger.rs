//! Flash data log: every task hands its measurements (stamped with the
//! monotonic time of the measurement) to this task, which packs them into
//! blocks (`winchy-datalog`) and writes them round-robin to the `storage`
//! partition. The block header carries the UTC offset of the PPS-disciplined
//! system clock, so each sample gets an exact UTC time.

use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::mpsc::{Receiver, RecvTimeoutError, SyncSender, TrySendError, sync_channel};
use std::time::Duration;

use winchy_datalog::block::{BLOCK_SIZE, BlockHeader, BlockWriter, HEADER_SIZE};
use winchy_datalog::record::{Record, event};
use winchy_datalog::ring::resume;
use winchy_gnss::time::ClockSource;
use winchy_storage::RawPartition;

use crate::shared::{State, lock, now_us, unix_now_us};

/// Queue depth (~3 s of all sensors; covers a sector erase).
const QUEUE: usize = 256;

/// Producer handle, cloned into every task.
#[derive(Clone)]
pub struct Logger {
    tx: SyncSender<(i64, Record)>,
    dropped: Arc<AtomicU32>,
}

impl Logger {
    /// Logs `record` measured at monotonic `mono_us`; never blocks (a full
    /// queue drops the record and counts it).
    pub fn log(&self, mono_us: i64, record: Record) {
        if let Err(TrySendError::Full(_) | TrySendError::Disconnected(_)) =
            self.tx.try_send((mono_us, record))
        {
            self.dropped.fetch_add(1, Ordering::Relaxed);
        }
    }

    /// Logs `record` measured now.
    pub fn log_now(&self, record: Record) {
        self.log(now_us(), record);
    }

    /// Logs an event now.
    pub fn event(&self, code: u8, value: i32) {
        self.log_now(Record::Event { code, value });
    }
}

/// Consumer side, moved into the writer task.
pub struct Sink {
    rx: Receiver<(i64, Record)>,
    dropped: Arc<AtomicU32>,
}

/// Creates the logger pair.
#[must_use]
pub fn channel() -> (Logger, Sink) {
    let (tx, rx) = sync_channel(QUEUE);
    let dropped = Arc::new(AtomicU32::new(0));
    (
        Logger {
            tx,
            dropped: dropped.clone(),
        },
        Sink { rx, dropped },
    )
}

/// Ring writer over the raw partition.
struct Ring {
    part: RawPartition,
    slots: usize,
    index: usize,
    seq: u32,
}

impl Ring {
    fn open() -> Option<Self> {
        let part = RawPartition::find(c"storage")
            .map_err(|e| log::error!("datalog: storage partition {e:?}"))
            .ok()?;
        let slots = part.size() / BLOCK_SIZE;
        let mut header = [0u8; HEADER_SIZE];
        let r = resume((0..slots).map(|i| {
            part.read(i * BLOCK_SIZE, &mut header).ok()?;
            BlockHeader::peek_seq(&header)
        }));
        log::info!(
            "datalog: {slots} blocks of {BLOCK_SIZE} B, continuing at block {} (seq {})",
            r.index,
            r.seq
        );
        Some(Self {
            part,
            slots,
            index: r.index,
            seq: r.seq,
        })
    }

    fn write(&mut self, writer: &mut BlockWriter) {
        let Some(block) = writer.seal(self.seq) else {
            return;
        };
        let at = self.index * BLOCK_SIZE;
        if let Err(e) = self
            .part
            .erase(at, BLOCK_SIZE)
            .and_then(|()| self.part.write(at, block))
        {
            log::warn!("datalog: block {} write {e:?}", self.index);
        }
        self.index = (self.index + 1) % self.slots;
        self.seq = self.seq.wrapping_add(1);
    }
}

/// UTC minus monotonic, if the system clock is set.
fn utc_offset(state: &State) -> Option<i64> {
    (lock(state).clock != ClockSource::Unset).then(|| unix_now_us() - now_us())
}

/// Task body.
pub fn run(sink: &Sink, state: &State) {
    let Some(mut ring) = Ring::open() else {
        return;
    };
    let mut writer = BlockWriter::new();
    let push = |ring: &mut Ring, writer: &mut BlockWriter, mono: i64, rec: &Record| {
        let mono = u64::try_from(mono).unwrap_or(0);
        let offset = utc_offset(state);
        if !writer.push(mono, offset, rec) {
            ring.write(writer);
            let _ = writer.push(mono, offset, rec);
        }
    };
    loop {
        match sink.rx.recv_timeout(Duration::from_secs(1)) {
            Ok((mono, rec)) => push(&mut ring, &mut writer, mono, &rec),
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => return,
        }
        let dropped = sink.dropped.swap(0, Ordering::Relaxed);
        if dropped > 0 {
            let rec = Record::Event {
                code: event::DROPPED,
                value: i32::try_from(dropped).unwrap_or(i32::MAX),
            };
            push(&mut ring, &mut writer, now_us(), &rec);
        }
    }
}
