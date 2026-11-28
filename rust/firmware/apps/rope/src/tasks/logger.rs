//! Flash data log: every task hands its measurements (stamped with the
//! monotonic time of the measurement) to this task, which packs them into
//! blocks (`winchy-datalog`) and writes them round-robin to the `storage`
//! partition. The block header carries the UTC offset of the PPS-disciplined
//! system clock, so each sample gets an exact UTC time.
//!
//! A sector erase stops every task for up to ~80 ms. So the writer keeps a
//! reserve of erased slots and refills it only while the unit lies still and
//! no tow runs ([`EraseGate`]); during a tow blocks go into the reserve
//! without any erase, and only an exhausted reserve erases inline.

use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::mpsc::{Receiver, RecvTimeoutError, SyncSender, TrySendError, sync_channel};
use std::time::{Duration, Instant};

use winchy_datalog::block::{BLOCK_SIZE, BlockHeader, BlockWriter, HEADER_SIZE};
use winchy_datalog::record::{Record, event};
use winchy_datalog::ring::{Ring, leading_erased, resume};
use winchy_gnss::time::ClockSource;
use winchy_rope_logic::erase_gate::EraseGate;
use winchy_storage::RawPartition;

use crate::shared::{State, lock, now_ms, now_us, unix_now_us};

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

/// Erased slots kept ahead of the write position (~3 min of log).
const RESERVE: usize = 64;
/// At most one erase-ahead per this interval (each stalls all tasks).
const REFILL_INTERVAL: Duration = Duration::from_millis(500);

/// Ring writer over the raw partition.
struct Flash {
    part: RawPartition,
    plan: Ring,
    /// "Full" is reported once per fill-up (after boot or after the reserve
    /// ran dry), "exhausted" once per run dry.
    reported_full: bool,
    reported_empty: bool,
}

impl Flash {
    fn open() -> Option<Self> {
        let part = RawPartition::find(c"storage")
            .map_err(|e| log::error!("datalog: storage partition {e:?}"))
            .ok()?;
        let slots = part.size() / BLOCK_SIZE;
        let mut header = [0u8; HEADER_SIZE];
        let mut read_header =
            |i: usize| part.read(i * BLOCK_SIZE, &mut header).ok().map(|()| header);
        let at = resume((0..slots).map(|i| BlockHeader::peek_seq(&read_header(i)?)));
        let ready = leading_erased(
            (0..RESERVE).map(|k| {
                read_header((at.index + k) % slots).is_some_and(|h| h.iter().all(|b| *b == 0xFF))
            }),
            RESERVE,
        );
        let plan = Ring::new(slots, at, ready, RESERVE);
        log::info!(
            "datalog: {slots} blocks of {BLOCK_SIZE} B, continuing at block {} (seq {}), {} erased ahead",
            at.index,
            at.seq,
            plan.ready()
        );
        Some(Self {
            part,
            plan,
            reported_full: false,
            reported_empty: false,
        })
    }

    fn write(&mut self, writer: &mut BlockWriter) {
        if writer.is_empty() {
            return;
        }
        let w = self.plan.next_write();
        let Some(block) = writer.seal(w.seq) else {
            return;
        };
        let at = w.index * BLOCK_SIZE;
        let erased = if w.erase {
            if !self.reported_empty {
                log::info!("datalog: erase reserve exhausted, erasing inline");
                self.reported_empty = true;
                self.reported_full = false;
            }
            self.part.erase(at, BLOCK_SIZE)
        } else {
            Ok(())
        };
        if let Err(e) = erased.and_then(|()| self.part.write(at, block)) {
            log::warn!("datalog: block {} write {e:?}", w.index);
        }
    }

    /// Erases one slot of the reserve, if it is not full.
    fn refill(&mut self) {
        let Some(i) = self.plan.erase_ahead() else {
            return;
        };
        match self.part.erase(i * BLOCK_SIZE, BLOCK_SIZE) {
            Ok(()) => {
                self.plan.erased_ahead();
                self.reported_empty = false;
                let full = self.plan.erase_ahead().is_none();
                if full && !self.reported_full {
                    log::info!("datalog: erase reserve full ({} blocks)", self.plan.ready());
                    self.reported_full = true;
                }
            }
            Err(e) => log::warn!("datalog: erase ahead {i} {e:?}"),
        }
    }
}

/// UTC minus monotonic, if the system clock is set.
fn utc_offset(state: &State) -> Option<i64> {
    (lock(state).clock != ClockSource::Unset).then(|| unix_now_us() - now_us())
}

/// Task body.
pub fn run(sink: &Sink, state: &State) {
    let Some(mut ring) = Flash::open() else {
        return;
    };
    let mut writer = BlockWriter::new();
    let mut gate = EraseGate::new();
    let mut last_refill = Instant::now();
    let push = |ring: &mut Flash, writer: &mut BlockWriter, mono: i64, rec: &Record| {
        let mono = u64::try_from(mono).unwrap_or(0);
        let offset = utc_offset(state);
        if !writer.push(mono, offset, rec) {
            ring.write(writer);
            let _ = writer.push(mono, offset, rec);
        }
    };
    loop {
        match sink.rx.recv_timeout(REFILL_INTERVAL) {
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
        let (gyro, idle) = {
            let s = lock(state);
            (s.attitude.gyro_dps, s.is_idle())
        };
        if gate.update(now_ms(), gyro, idle) && last_refill.elapsed() >= REFILL_INTERVAL {
            last_refill = Instant::now();
            ring.refill();
        }
    }
}
