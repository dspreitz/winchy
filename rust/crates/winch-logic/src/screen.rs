//! What the OLED shows, as plain data (drawn by `winchy-winch-ui`).
//!
//! While rope telemetry is fresh: the tow phase in the title bar and the two
//! numbers the winch driver steers by (glider speed, rope force) in large
//! type, plus a status line that alternates with a battery warning. Without
//! telemetry: the winch's own GPS lock and survey progress.

use core::fmt::Write as _;

use heapless::String;
use winchy_protocol::{TelemetryFlags, TowPhase};

use crate::state::WinchState;

/// Telemetry older than this hands the screen back to the GPS view, ms.
pub const TELEMETRY_TIMEOUT_MS: u64 = 3_000;
/// Frames per state of the battery warning blink (~2 s at 2 Hz).
pub const WARN_BLINK_FRAMES: u32 = 4;

/// Title bar: title on the left, status on the right.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Header {
    /// Title (tow phase or screen name).
    pub title: String<12>,
    /// Winch receiver satellites.
    pub sats: u8,
    /// Winch receiver has a fix (the GPS icon is struck through otherwise).
    pub fix: bool,
    /// Log server reachable ("I" shown).
    pub online: bool,
}

/// One OLED frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Screen {
    /// Title bar.
    pub header: Header,
    /// Large lines (empty = not drawn).
    pub big: [String<12>; 2],
    /// Small line under the large ones (empty = not drawn).
    pub small: String<21>,
    /// Small bottom line.
    pub bottom: String<21>,
}

/// Display name of a tow phase (as on the MicroPython winch).
#[must_use]
pub fn phase_name(phase: TowPhase) -> &'static str {
    match phase {
        TowPhase::Idle => "IDLE",
        TowPhase::SlackOut => "SLACK",
        TowPhase::GroundRoll => "ROLL",
        TowPhase::Rotation => "ROTATE",
        TowPhase::Climb => "CLIMB",
        TowPhase::Top => "TOP",
        TowPhase::Released => "RELEASED",
        TowPhase::LinkBreak => "BREAK!",
    }
}

/// True while the latest telemetry is fresh enough to show.
#[must_use]
pub fn telemetry_fresh(state: &WinchState, now_ms: u64) -> bool {
    state
        .telemetry
        .is_some_and(|t| now_ms.saturating_sub(t.at_ms) < TELEMETRY_TIMEOUT_MS)
}

/// Builds the frame for `now_ms`; `frame` counts rendered frames (blink).
#[must_use]
pub fn build(state: &WinchState, now_ms: u64, frame: u32) -> Screen {
    match state.telemetry {
        Some(rx) if telemetry_fresh(state, now_ms) => telemetry_screen(state, &rx, frame),
        _ => gps_screen(state),
    }
}

fn header(state: &WinchState, title: &str) -> Header {
    Header {
        title: text(format_args!("{title}")),
        sats: state.sats,
        fix: state.fix,
        online: state.online,
    }
}

/// Formats into a fixed-capacity line (too long = left empty, never panics).
fn text<const N: usize>(args: core::fmt::Arguments<'_>) -> String<N> {
    let mut s = String::new();
    if s.write_fmt(args).is_err() {
        s.clear();
    }
    s
}

/// `a / b` rounded half away from zero (`b > 0`).
pub(crate) fn round_div(a: i32, b: i32) -> i32 {
    if a >= 0 {
        (a + b / 2) / b
    } else {
        (a - b / 2) / b
    }
}

fn telemetry_screen(state: &WinchState, rx: &crate::state::RxTelemetry, frame: u32) -> Screen {
    let t = &rx.telemetry;
    let kmh = f32::from(t.glider_speed_cms) * 0.036;
    // Large type fits ~8 characters: calibrated force (0.1 N) as newtons, raw
    // ADC counts (six digits) in thousands, floored like MicroPython's //.
    let force: String<12> = if t.flags.contains(TelemetryFlags::FORCE_UNCALIBRATED) {
        text(format_args!("{}k", t.force.div_euclid(1000)))
    } else {
        text(format_args!("{}N", round_div(t.force, 10)))
    };
    let warn =
        t.flags.contains(TelemetryFlags::BATTERY_LOW) && (frame / WARN_BLINK_FRAMES) % 2 == 0;
    let bottom = if warn {
        text(format_args!(
            "!BATT LOW {:.1}V",
            f32::from(t.battery_mv) / 1000.0
        ))
    } else {
        text(format_args!(
            "A{:.0} H{} {}dB",
            f32::from(t.rope_angle_cdeg) / 100.0,
            round_div(t.altitude_dm, 10),
            rx.rssi_dbm
        ))
    };
    Screen {
        header: header(state, phase_name(t.phase)),
        big: [text(format_args!("{kmh:.0}km/h")), force],
        small: String::new(),
        bottom,
    }
}

fn gps_screen(state: &WinchState) -> Screen {
    let survey = &state.survey;
    let (big, small, bottom) = if survey.position().is_none() {
        (
            text(format_args!("NO FIX")),
            String::new(),
            text(format_args!("waiting for fix")),
        )
    } else {
        let acc = survey.accuracy_m();
        let big = if acc.is_finite() {
            text(format_args!("{acc:.1} m"))
        } else {
            text(format_args!("-- m"))
        };
        let bottom = if survey.is_converged() {
            text(format_args!("SURVEYED"))
        } else {
            text(format_args!("surveying..."))
        };
        (big, text(format_args!("n={}", survey.samples())), bottom)
    };
    Screen {
        header: header(state, "WINCH GPS"),
        big: [big, String::new()],
        small,
        bottom,
    }
}

#[cfg(test)]
mod tests;
