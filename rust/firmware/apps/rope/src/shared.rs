//! Shared state handle, clocks and task spawning.

use std::ffi::CStr;
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};

use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use winchy_rope_logic::state::RopeState;

/// The rope state shared by all tasks.
pub type State = Arc<Mutex<RopeState>>;

/// Locks a mutex; a task that panicked while holding it must not take the
/// others down, so poisoning is ignored.
pub fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Monotonic time since boot, µs.
pub fn now_us() -> i64 {
    // SAFETY: esp_timer_get_time has no preconditions once the system runs.
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

/// Monotonic time since boot, ms.
pub fn now_ms() -> u64 {
    u64::try_from(now_us() / 1_000).unwrap_or(0)
}

/// System (UTC) clock, µs since the epoch.
pub fn unix_now_us() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| i64::try_from(d.as_micros()).unwrap_or(i64::MAX))
}

/// Sets the system (UTC) clock.
pub fn set_unix_us(us: i64) {
    let tv = esp_idf_svc::sys::timeval {
        tv_sec: us.div_euclid(1_000_000),
        tv_usec: us.rem_euclid(1_000_000).try_into().unwrap_or(0),
    };
    // SAFETY: tv is a valid timeval for the duration of the call; no timezone.
    let rc = unsafe { esp_idf_svc::sys::settimeofday(&raw const tv, core::ptr::null()) };
    if rc != 0 {
        log::warn!("clock: settimeofday failed ({rc})");
    }
}

/// Spawns a FreeRTOS-backed std thread with a name, stack and priority.
pub fn spawn(name: &'static CStr, stack: usize, priority: u8, f: impl FnOnce() + Send + 'static) {
    let cfg = ThreadSpawnConfiguration {
        name: Some(name),
        stack_size: stack,
        priority,
        ..ThreadSpawnConfiguration::default()
    };
    if let Err(e) = cfg.set() {
        log::error!("spawn {name:?}: config {e:?}");
    }
    if let Err(e) = std::thread::Builder::new().stack_size(stack).spawn(f) {
        log::error!("spawn {name:?}: {e}");
    }
    let _ = ThreadSpawnConfiguration::default().set();
}
