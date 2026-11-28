//! 1PPS edge capture. The ISR only timestamps the rising edge (low 32 bits of
//! the µs clock, the widest atomic) and counts it; the GNSS task polls for new
//! edges and labels them with a UTC second (`winchy_gnss::time::PpsDiscipline`).

use std::sync::atomic::{AtomicU32, Ordering};

use esp_idf_svc::hal::gpio::{Input, InterruptType, PinDriver};

use crate::now_us;

/// Edges seen (ISR). One PPS input per board.
static EDGES: AtomicU32 = AtomicU32::new(0);
/// Low 32 bits of the monotonic µs timestamp of the last edge (ISR).
static EDGE_US: AtomicU32 = AtomicU32::new(0);

/// Low 32 bits of a µs timestamp (wraps every ~71 min; only differences of a
/// few ms are taken, so the wrap is harmless).
fn low32(us: i64) -> u32 {
    u32::try_from(us.rem_euclid(1 << 32)).unwrap_or(0)
}

/// The armed 1PPS input.
pub struct PpsInput {
    pin: PinDriver<'static, Input>,
    seen: u32,
}

impl PpsInput {
    /// Arms the rising-edge interrupt on `pin`.
    #[must_use]
    pub fn start(mut pin: PinDriver<'static, Input>) -> Self {
        let armed = pin
            .set_interrupt_type(InterruptType::PosEdge)
            .and_then(|()| {
                // SAFETY: the callback only touches atomics and esp_timer_get_time,
                // both ISR-safe; it lives as long as the pin driver (forever).
                unsafe {
                    pin.subscribe(|| {
                        EDGE_US.store(low32(now_us()), Ordering::Release);
                        EDGES.fetch_add(1, Ordering::AcqRel);
                    })
                }
            });
        match armed.and_then(|()| pin.enable_interrupt()) {
            Ok(()) => log::info!("gnss: 1PPS interrupt armed"),
            Err(e) => log::error!("gnss: 1PPS interrupt {e:?}"),
        }
        Self {
            pin,
            seen: EDGES.load(Ordering::Acquire),
        }
    }

    /// Monotonic µs of the latest edge, if one came since the last call.
    pub fn poll(&mut self) -> Option<i64> {
        let edges = EDGES.load(Ordering::Acquire);
        if edges == self.seen {
            return None;
        }
        self.seen = edges;
        let _ = self.pin.enable_interrupt(); // the HAL disables it after each edge
        let now = now_us();
        let elapsed = low32(now).wrapping_sub(EDGE_US.load(Ordering::Acquire));
        Some(now - i64::from(elapsed))
    }
}
