//! GNSS (interim u-blox 7, NMEA at 9600 baud): fixed-base receiver setup,
//! survey-in of the parked winch from GGA, the system clock from RMC with
//! defensive adoption, and 1PPS discipline of that clock.

use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::uart::UartDriver;
use winchy_geo::GeoPoint;
use winchy_gnss::clock::{STEP_TOLERANCE_US, step_us, utc_now_us};
use winchy_gnss::nmea::{Sentence, parse};
use winchy_gnss::time::{ClockSource, PpsDiscipline, TimeAction, TimeFix};
use winchy_gnss::ublox7;
use winchy_rt::pps::PpsInput;

use crate::shared::{State, lock, now_ms, now_us, set_unix_us, unix_now_us};

/// Longest NMEA sentence kept (the standard allows 82 characters).
const MAX_LINE: usize = 120;
/// Idle poll interval of the UART, ms.
const POLL_MS: u32 = 10;
/// Pause between configuration messages, ms.
const CFG_GAP_MS: u32 = 80;

/// Tells the receiver it is a fixed base (Stationary + static hold, SBAS) and
/// saves that, as the MicroPython winch did.
fn configure(uart: &UartDriver<'static>) {
    let msgs: [&[u8]; 3] = [
        &ublox7::cfg_nav5_stationary(),
        &ublox7::cfg_sbas(),
        &ublox7::cfg_save(),
    ];
    for m in msgs {
        if let Err(e) = uart.write(m) {
            log::warn!("gnss: config write {e:?}");
        }
        FreeRtos::delay_ms(CFG_GAP_MS);
    }
    log::info!("gnss: Stationary model + static hold + SBAS set");
}

/// Clock handling across RMC sentences and PPS edges.
struct Clock {
    fix: TimeFix,
    pps: PpsDiscipline,
    pps_locked: bool,
}

impl Clock {
    fn on_rmc_time(&mut self, utc_s: i64, state: &State) {
        let source = lock(state).clock;
        let clock_s = unix_now_us().div_euclid(1_000_000);
        // NMEA carries no time accuracy: 0 skips that gate.
        match self.fix.decide(source, utc_s, clock_s, 0) {
            TimeAction::Set => {
                set_unix_us(utc_s * 1_000_000);
                lock(state).clock = ClockSource::Gnss;
                self.pps_locked = false;
                log::info!("clock: set from GNSS (was {source:?})");
            }
            TimeAction::Reject if source == ClockSource::Ntp => {
                log::warn!("clock: GNSS time rejected, {:+} s vs NTP", utc_s - clock_s);
            }
            _ => {}
        }
        if lock(state).clock == ClockSource::Gnss {
            // RMC time is the epoch's whole second (1 Hz): arm the next edge.
            self.pps.arm(utc_s, 0, now_ms());
        }
    }

    fn on_poll(&mut self, pps: &mut PpsInput) {
        let Some(edge_us) = pps.poll() else {
            return;
        };
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
            log::info!("clock: locked to 1PPS");
        }
    }
}

/// Applies one sentence to the shared state.
fn on_sentence(sentence: Sentence, clock: &mut Clock, state: &State, logged: &mut (bool, bool)) {
    match sentence {
        Sentence::Gga(g) => {
            let fix = g.quality > 0;
            let mut s = lock(state);
            s.sats = g.sats;
            s.fix = fix;
            if let (true, Some(lat), Some(lon)) = (fix, g.lat_deg, g.lon_deg) {
                s.survey
                    .add(GeoPoint::new(lat, lon), f64::from(g.alt_m.unwrap_or(0.0)));
            }
            let converged = s.survey.is_converged();
            let (n, acc) = (s.survey.samples(), s.survey.accuracy_m());
            drop(s);
            if fix != logged.0 {
                logged.0 = fix;
                log::info!(
                    "gnss: {} ({} sats)",
                    if fix { "fix" } else { "fix lost" },
                    g.sats
                );
            }
            if converged && !logged.1 {
                logged.1 = true;
                log::info!("gnss: survey converged (n={n}, {acc:.1} m)");
            }
        }
        Sentence::Rmc(r) => {
            if let Some(utc_s) = r.utc.and_then(|u| u.to_unix()) {
                clock.on_rmc_time(utc_s, state);
            }
        }
    }
}

/// Task body.
pub fn run(gnss: winchy_board::GnssPins, state: &State) {
    let uart = gnss.uart;
    configure(&uart);
    let mut pps = PpsInput::start(gnss.pps);
    let mut clock = Clock {
        fix: TimeFix::default(),
        pps: PpsDiscipline::new(),
        pps_locked: false,
    };
    let mut logged = (false, false);
    let mut line = [0u8; MAX_LINE];
    let mut len = 0;
    let mut buf = [0u8; 128];
    loop {
        clock.on_poll(&mut pps);
        let n = uart.read(&mut buf, 0).unwrap_or(0);
        if n == 0 {
            FreeRtos::delay_ms(POLL_MS);
            continue;
        }
        for &b in &buf[..n] {
            if b == b'\n' {
                if let Some(s) = parse(&line[..len]) {
                    on_sentence(s, &mut clock, state, &mut logged);
                }
                len = 0;
            } else if len < MAX_LINE {
                line[len] = b;
                len += 1;
            } else {
                len = 0; // overlong garbage: drop and resync on the next '\n'
            }
        }
    }
}
