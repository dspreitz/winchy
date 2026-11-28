//! The winch's WiFi dashboard: one HTML page that opens a WebSocket and
//! renders the JSON snapshot the winch pushes twice a second. Page and JSON
//! fields are the MicroPython winch's, so the phone view stays the same.
//! The page sends "hello" when the socket opens: ESP-IDF 5.5 does not call
//! the server's handler for the handshake itself.

use core::fmt::Write as _;

use heapless::String;
use winchy_gnss::time::ClockSource;
use winchy_protocol::TelemetryFlags;

use crate::screen::{phase_name, round_div, telemetry_fresh};
use crate::state::WinchState;

/// Push interval of the snapshot, ms.
pub const PUSH_MS: u32 = 500;

/// Dashboard page (served at `/`).
pub const PAGE: &str = r#"<!DOCTYPE html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>Winchy</title><style>
body{font-family:sans-serif;background:#111;color:#eee;margin:0;text-align:center}
#phase{font-size:13vw;font-weight:bold;padding:8px;background:#223}
.big{font-size:10vw;font-weight:bold}.lbl{font-size:4vw;color:#8ac}
.row{display:flex}.cell{flex:1;padding:8px}
#warn{background:#c00;color:#fff;font-size:6vw;padding:10px;display:none}
.stale{opacity:.35}
.dot{display:inline-block;width:7vw;height:7vw;border-radius:50%;background:#555;vertical-align:middle;margin-right:2vw}
</style></head><body>
<div id=warn>! ROPE BATTERY LOW</div>
<div id=phase>--</div>
<div class=row><div class=cell><span id=ropedot class=dot></span><span class=lbl>ROPE</span></div>
<div class=cell><span id=tdot class=dot></span><span class=lbl>TIME</span></div></div>
<div id=clock class=lbl>--:--:--</div>
<div class=cell><div class=lbl>SPEED km/h</div><div id=speed class=big>--</div></div>
<div class=row><div class=cell><div class=lbl>FORCE</div><div id=force class=big>--</div></div>
<div class=cell><div class=lbl>ANGLE</div><div id=angle class=big>--</div></div></div>
<div class=row><div class=cell><div class=lbl>ALT m</div><div id=alt class=big>--</div></div>
<div class=cell><div class=lbl>BATT</div><div id=batt class=big>--</div><div id=battsub class=lbl>--</div></div></div>
<div id=link class=lbl>link --</div>
<div id=winch class=lbl>winch GPS --</div>
<script>
var last=Date.now();var ws;
function render(d){
 last=Date.now();document.body.className='';
 phase.textContent=d.phase;
 speed.textContent=(d.speed*3.6).toFixed(0);
 force.textContent=d.force+(d.uncal?' c':' N');
 angle.textContent=d.angle.toFixed(1);
 alt.textContent=d.alt;
 batt.textContent=(d.battpct>100?'--':d.battpct+'%')+(d.charging?' CHG':'');
 battsub.textContent=d.batt.toFixed(1)+' V';
 link.textContent='link '+d.rssi+' dBm   rx'+d.rx+' lost'+d.lost;
 winch.textContent='winch '+(d.wfix?'fix':'no fix')+' '+(d.wsats||0)+'sat'+(d.wacc!=null?'  '+d.wacc+'m':'')+(d.wconv?'  SURVEYED':'');
 warn.style.display=d.battlow?'block':'none';
 ropedot.style.background=d.rope?'#1c1':'#c33';
 tdot.style.background=d.tsync?'#1c1':'#c33';
 clock.textContent=d.time+' UTC';
}
function connect(){
 ws=new WebSocket('ws://'+location.host+'/ws');
 ws.onopen=function(){ws.send('hello');};
 ws.onmessage=function(ev){try{render(JSON.parse(ev.data));}catch(e){}};
 ws.onclose=function(){setTimeout(connect,1000);};
 ws.onerror=function(){try{ws.close();}catch(e){}};
}
connect();
setInterval(function(){
 if(Date.now()-last>3000){document.body.className='stale';
  ropedot.style.background='#555';tdot.style.background='#555';}
},1000);
</script></body></html>"#;

/// UTC wall-clock time of day.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimeOfDay {
    /// Hour.
    pub hour: u8,
    /// Minute.
    pub minute: u8,
    /// Second.
    pub second: u8,
}

impl TimeOfDay {
    /// Time of day of a Unix timestamp (UTC, no leap seconds).
    #[must_use]
    pub fn from_unix(unix_s: i64) -> Self {
        let day_s = unix_s.rem_euclid(86_400);
        let part = |v: i64| u8::try_from(v).unwrap_or(0);
        Self {
            hour: part(day_s / 3_600),
            minute: part(day_s / 60 % 60),
            second: part(day_s % 60),
        }
    }
}

/// JSON snapshot for the page. `time` is the UTC time of day if the clock is
/// set. Calibrated force is sent in newtons, raw counts as they are.
#[must_use]
pub fn json(state: &WinchState, now_ms: u64, time: Option<TimeOfDay>) -> String<640> {
    let mut j = String::new();
    // Capacity covers the longest possible snapshot; a failed write would
    // only truncate the JSON, which the page then ignores.
    let _ = write_json(&mut j, state, now_ms, time);
    j
}

fn write_json(
    j: &mut String<640>,
    state: &WinchState,
    now_ms: u64,
    time: Option<TimeOfDay>,
) -> core::fmt::Result {
    match state.telemetry.map(|rx| (rx, rx.telemetry)) {
        Some((rx, t)) => {
            let uncal = t.flags.contains(TelemetryFlags::FORCE_UNCALIBRATED);
            let force = if uncal { t.force } else { round_div(t.force, 10) };
            write!(
                j,
                r#"{{"phase":"{}","force":{force},"uncal":{uncal},"angle":{:.1},"alt":{},"speed":{:.2},"rssi":{},"batt":{:.1},"battlow":{},"battpct":{},"charging":{},"gps":{},"#,
                phase_name(t.phase),
                f32::from(t.rope_angle_cdeg) / 100.0,
                round_div(t.altitude_dm, 10),
                f32::from(t.glider_speed_cms) / 100.0,
                rx.rssi_dbm,
                f32::from(t.battery_mv) / 1000.0,
                t.flags.contains(TelemetryFlags::BATTERY_LOW),
                t.battery_pct,
                t.flags.contains(TelemetryFlags::CHARGING),
                t.flags.contains(TelemetryFlags::GNSS_FIX),
            )?;
        }
        None => j.push_str(
            r#"{"phase":"--","force":0,"uncal":true,"angle":0.0,"alt":0,"speed":0.00,"rssi":0,"batt":0.0,"battlow":false,"battpct":255,"charging":false,"gps":false,"#,
        ).map_err(|()| core::fmt::Error)?,
    }
    write!(
        j,
        r#""rx":{},"lost":{},"wsats":{},"wfix":{},"#,
        state.received, state.lost, state.sats, state.fix
    )?;
    match state.survey.position() {
        Some((p, _)) => write!(j, r#""wlat":{:.6},"wlon":{:.6},"#, p.lat_deg, p.lon_deg)?,
        None => j
            .push_str(r#""wlat":null,"wlon":null,"#)
            .map_err(|()| core::fmt::Error)?,
    }
    let acc = state.survey.accuracy_m();
    if acc.is_finite() {
        write!(j, r#""wacc":{acc:.1},"#)?;
    } else {
        j.push_str(r#""wacc":null,"#)
            .map_err(|()| core::fmt::Error)?;
    }
    write!(
        j,
        r#""wconv":{},"rope":{},"tsync":{},"#,
        state.survey.is_converged(),
        telemetry_fresh(state, now_ms),
        state.clock != ClockSource::Unset
    )?;
    match time {
        Some(t) => write!(
            j,
            r#""time":"{:02}:{:02}:{:02}","#,
            t.hour, t.minute, t.second
        )?,
        None => j
            .push_str(r#""time":"--:--:--","#)
            .map_err(|()| core::fmt::Error)?,
    }
    j.push_str(r#""upstatus":""}"#)
        .map_err(|()| core::fmt::Error)
}

#[cfg(test)]
mod tests;
