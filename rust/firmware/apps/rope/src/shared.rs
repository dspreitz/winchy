//! Shared state handle, clocks and task spawning.

use std::sync::{Arc, Mutex};

use winchy_rope_logic::state::RopeState;

/// The rope state shared by all tasks.
pub type State = Arc<Mutex<RopeState>>;

pub use winchy_rt::{lock, now_ms, now_us, set_unix_us, spawn, unix_now_us};
