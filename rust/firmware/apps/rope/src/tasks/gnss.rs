//! GNSS: baud discovery, NAV-PVT at 10 Hz, defensive time adoption, 1PPS
//! clock discipline, winch geometry and field-elevation persistence.
//!
//! The 1PPS ISR only timestamps the edge; labelling it with a UTC second and
//! stepping the system clock happens here, in task context.

use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};

use esp_idf_svc::hal::delay::{FreeRtos, TickType};
use esp_idf_svc::hal::gpio::{Input, InterruptType, PinDriver};
use esp_idf_svc::hal::uart::UartDriver;
use esp_idf_svc::hal::units::Hertz;
use winchy_gnss::baud::{BaudProbe, ProbeAction};
use winchy_gnss::time::{ClockSource, PpsDiscipline, TimeAction, TimeFix};
use winchy_gnss::ubx::{
    CfgValue, FixType, Framer, NavPvt, key, layer, msg, receiver_config, write_valset,
};
use winchy_rope_logic::clock::{STEP_TOLERANCE_US, step_us, utc_now_us};
use winchy_rope_logic::geometry::winch_geometry;
use winchy_rope_logic::last_fix::LastFixSaver;
use winchy_settings::NvsStore;
use winchy_storage::MgaCache;

use crate::assist;
use crate::shared::{State, lock, now_ms, now_us, set_unix_us, unix_now_us};
use crate::tasks::baro::NVS_LAST_ALT;
use crate::tasks::logger::Logger;
use winchy_datalog::record::{Record, event};

/// GNSS UART, shared with the AssistNow upload.
pub type GnssUart = Arc<Mutex<UartDriver<'static>>>;

/// Navigation rate, Hz.
const NAV_RATE_HZ: u16 = 10;
/// No NAV-PVT for this long: re-detect the baud (receiver lost its config).
const SILENCE_MS: u64 = 8_000;
/// Idle poll interval of the UART, ms.
const POLL_MS: u32 = 10;

/// 1PPS edges seen (ISR).
static PPS_EDGES: AtomicU32 = AtomicU32::new(0);
/// Low 32 bits of the monotonic µs timestamp of the last edge (ISR).
static PPS_EDGE_US: AtomicU32 = AtomicU32::new(0);

/// Low 32 bits of a µs timestamp (wraps every ~71 min; only differences of
/// a few ms are taken, so the wrap is harmless).
fn low32(us: i64) -> u32 {
    u32::try_from(us.rem_euclid(1 << 32)).unwrap_or(0)
}

fn listen(uart: &UartDriver<'static>, baud: u32, ms: u32) -> bool {
    if uart.change_baudrate(Hertz(baud)).is_err() {
        return false;
    }
    let _ = uart.clear_rx();
    let mut window = [0u8; 600];
    let mut len = 0;
    let start = now_ms();
    while now_ms() - start < u64::from(ms) {
        let mut chunk = [0u8; 128];
        let n = uart
            .read(&mut chunk, TickType::new_millis(20).ticks())
            .unwrap_or(0);
        for &b in &chunk[..n] {
            if len == window.len() {
                window.copy_within(1.., 0);
                len -= 1;
            }
            window[len] = b;
            len += 1;
        }
        if winchy_gnss::detect::contains_gnss_frame(&window[..len]) {
            return true;
        }
    }
    false
}

fn send_valset(uart: &UartDriver<'static>, items: &[(u32, CfgValue)]) {
    let mut frame = [0u8; 160];
    match write_valset(layer::RAM | layer::BBR, items, &mut frame) {
        Ok(n) => {
            let _ = uart.write(&frame[..n]);
        }
        Err(e) => log::error!("gnss: VALSET encode {e:?}"),
    }
}

/// Finds the receiver's baud, raises it to 115200 and configures NAV-PVT.
fn bring_up(uart: &UartDriver<'static>) {
    let mut probe = BaudProbe::new(9_600, 115_200, 12);
    let mut outcome = false;
    let baud = loop {
        match probe.step(outcome) {
            ProbeAction::Listen { baud, timeout_ms } => outcome = listen(uart, baud, timeout_ms),
            ProbeAction::Raise { to } => {
                send_valset(uart, &[(key::UART1_BAUDRATE, CfgValue::U32(to))]);
                FreeRtos::delay_ms(150);
                outcome = false;
            }
            ProbeAction::Sleep { ms } => {
                FreeRtos::delay_ms(ms);
                outcome = false;
            }
            ProbeAction::Found { baud } => break baud,
            ProbeAction::GaveUp { baud } => {
                log::warn!("gnss: baud probe gave up, assuming {baud}");
                break baud;
            }
        }
    };
    let _ = uart.change_baudrate(Hertz(baud));
    send_valset(uart, &receiver_config(NAV_RATE_HZ));
    log::info!("gnss: {baud} baud, NAV-PVT {NAV_RATE_HZ} Hz");
}

fn start_pps(pps: &mut PinDriver<'static, Input>) {
    let armed = pps
        .set_interrupt_type(InterruptType::PosEdge)
        .and_then(|()| {
            // SAFETY: the callback only touches atomics and esp_timer_get_time,
            // both ISR-safe; it lives as long as the pin driver (forever).
            unsafe {
                pps.subscribe(|| {
                    PPS_EDGE_US.store(low32(now_us()), Ordering::Release);
                    PPS_EDGES.fetch_add(1, Ordering::AcqRel);
                })
            }
        });
    match armed.and_then(|()| pps.enable_interrupt()) {
        Ok(()) => log::info!("gnss: 1PPS interrupt armed"),
        Err(e) => log::error!("gnss: 1PPS interrupt {e:?}"),
    }
}

/// Data-log record of a solution, with the receiver's own epoch time.
fn gnss_record(nav: &NavPvt) -> Record {
    let sat_i16 =
        |v: i32| i16::try_from(v.clamp(i32::from(i16::MIN), i32::from(i16::MAX))).unwrap_or(0);
    Record::Gnss {
        fix: match nav.fix {
            FixType::None => 0,
            FixType::Fix2D => 2,
            FixType::Fix3D => 3,
        },
        sats: nav.num_sv,
        lat_e7: nav.lat_e7,
        lon_e7: nav.lon_e7,
        hmsl_mm: nav.hmsl_mm,
        hacc_cm: u16::try_from(nav.hacc_mm / 10).unwrap_or(u16::MAX),
        gspeed_cms: sat_i16(nav.gspeed_mms / 10),
        climb_cms: sat_i16(-nav.vel_d_mms / 10),
        utc_ms: nav
            .utc
            .and_then(|u| u.to_unix())
            .map_or(0, |s| s * 1_000 + i64::from(nav.nano_ns) / 1_000_000),
    }
}

/// GNSS time handling across NAV-PVT solutions and PPS edges.
struct Clock {
    fix: TimeFix,
    pps: PpsDiscipline,
    seen_edges: u32,
    /// The clock has been aligned to a PPS edge since the last coarse set.
    pps_locked: bool,
    reject_logged_ms: Option<u64>,
}

impl Clock {
    fn on_nav(&mut self, nav: &NavPvt, now: u64, state: &State, logger: &Logger) {
        let Some(utc_s) = nav.utc.and_then(|u| u.to_unix()) else {
            return;
        };
        let source = lock(state).clock;
        let clock_s = unix_now_us().div_euclid(1_000_000);
        match self.fix.decide(source, utc_s, clock_s, nav.tacc_ns) {
            TimeAction::Set => {
                set_unix_us(utc_s * 1_000_000 + i64::from(nav.nano_ns) / 1_000);
                lock(state).clock = ClockSource::Gnss;
                logger.event(event::CLOCK, event::CLOCK_GNSS);
                self.pps_locked = false;
                log::info!("clock: set from GNSS ({:?}, was {source:?})", nav.utc);
            }
            TimeAction::Reject
                if source == ClockSource::Ntp
                    && self.reject_logged_ms.is_none_or(|t| now - t > 30_000) =>
            {
                self.reject_logged_ms = Some(now);
                log::warn!("clock: GNSS time rejected, {:+} s vs NTP", utc_s - clock_s);
            }
            _ => {}
        }
        if lock(state).clock == ClockSource::Gnss {
            self.pps.arm(utc_s, nav.nano_ns, now);
        }
    }

    fn on_poll(&mut self, pps: &mut PinDriver<'static, Input>, logger: &Logger) {
        let edges = PPS_EDGES.load(Ordering::Acquire);
        if edges == self.seen_edges {
            return;
        }
        self.seen_edges = edges;
        let _ = pps.enable_interrupt(); // the HAL disables it after each edge
        let now = now_us();
        let elapsed = low32(now).wrapping_sub(PPS_EDGE_US.load(Ordering::Acquire));
        let edge_us = now - i64::from(elapsed);
        let Some(second) = self
            .pps
            .on_edge(u64::try_from(edge_us / 1_000).unwrap_or(0))
        else {
            return;
        };
        let truth = utc_now_us(second, edge_us, now_us());
        if let Some(d) = step_us(unix_now_us(), truth, STEP_TOLERANCE_US) {
            set_unix_us(utc_now_us(second, edge_us, now_us()));
            if d.abs() > 50_000 {
                log::info!("clock: PPS step {d:+} us");
            }
        }
        if !self.pps_locked {
            self.pps_locked = true;
            logger.event(event::CLOCK, event::CLOCK_PPS);
            log::info!("clock: locked to 1PPS");
        }
    }
}

/// Task body.
pub fn run(
    uart: &GnssUart,
    mut pps: PinDriver<'static, Input>,
    state: &State,
    store: &Mutex<NvsStore>,
    logger: &Logger,
) {
    bring_up(&lock(uart));
    match MgaCache::open() {
        Ok(cache) => assist::feed_cached(uart, &cache),
        Err(e) => log::warn!("assistnow: cache partition {e:?}"),
    }
    // Solutions queued while feeding are seconds old; they must not set the
    // clock or arm the PPS label.
    let _ = lock(uart).clear_rx();
    start_pps(&mut pps);

    let mut clock = Clock {
        fix: TimeFix::default(),
        pps: PpsDiscipline::new(),
        seen_edges: PPS_EDGES.load(Ordering::Acquire),
        pps_locked: false,
        reject_logged_ms: None,
    };
    let mut saver = LastFixSaver::default();
    let mut framer = Framer::new();
    let mut last_nav = now_ms();
    let mut had_fix = false;
    let mut buf = [0u8; 256];
    loop {
        // Non-blocking read so the AssistNow upload can take the UART.
        let n = lock(uart).read(&mut buf, 0).unwrap_or(0);
        clock.on_poll(&mut pps, logger);
        if n == 0 {
            if now_ms() - last_nav > SILENCE_MS {
                log::warn!(
                    "gnss: no NAV-PVT for {} s, re-detecting",
                    SILENCE_MS / 1_000
                );
                bring_up(&lock(uart));
                last_nav = now_ms();
            }
            FreeRtos::delay_ms(POLL_MS);
            continue;
        }
        for &b in &buf[..n] {
            let Some(p) = framer.push(b) else { continue };
            if (p.class, p.id) != msg::NAV_PVT {
                continue;
            }
            let Some(nav) = NavPvt::parse(p.payload) else {
                continue;
            };
            let now = now_ms();
            logger.log_now(gnss_record(&nav));
            last_nav = now;
            clock.on_nav(&nav, now, state, logger);
            let fix = nav.fix != FixType::None;
            if fix != had_fix {
                had_fix = fix;
                log::info!(
                    "gnss: {} ({} sats, hAcc {:.1} m)",
                    if fix { "fix" } else { "fix lost" },
                    nav.num_sv,
                    nav.hacc_mm as f32 / 1_000.0
                );
            }
            let (idle, alt) = {
                let mut s = lock(state);
                s.gnss.fix = fix;
                s.gnss.sats = nav.num_sv;
                s.gnss.hacc_m = nav.hacc_mm as f32 / 1_000.0;
                if fix {
                    s.gnss.lat_deg = nav.lat_deg();
                    s.gnss.lon_deg = nav.lon_deg();
                    s.gnss.alt_m = nav.hmsl_mm as f32 / 1_000.0;
                    s.gnss.ground_speed_ms = nav.gspeed_mms as f32 / 1_000.0;
                    s.gnss.climb_ms = nav.climb_ms();
                } else {
                    s.gnss.ground_speed_ms = 0.0; // no fix -> no speed
                }
                s.gnss.updated_ms = Some(now);
                if let Some(g) = winch_geometry(&s) {
                    s.geometry = Some(g); // kept when the fix drops (ADR prior)
                }
                (s.is_idle(), s.gnss.alt_m)
            };
            if saver.should_save(now, idle, fix, nav.num_sv, alt) {
                let _ = lock(store).set_internal(NVS_LAST_ALT, &format!("{alt:.1}"));
            }
        }
    }
}
