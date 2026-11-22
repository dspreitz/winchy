//! u-blox AssistNow (Thingstream) logic: when to download, the one-time ZTP
//! registration request/response, the data URL and the HTTP `Date` header
//! (coarse time aiding even without a GNSS fix). No I/O here.

use crate::time::UtcDateTime;

/// Default ZTP registration endpoint.
pub const ZTP_URL: &str = "https://api.thingstream.io/ztp/assistnow/credentials";
/// Constellations requested.
pub const GNSS: &str = "gps";
/// Data requested: 7-day predicted orbits + almanac.
pub const DATA: &str = "uporb_7,ualm";
/// Re-download when the cache is older than this.
pub const MAX_AGE_S: i64 = 3 * 86_400;
/// MGA blob is streamed to the receiver in chunks of this size...
pub const FEED_CHUNK: usize = 128;
/// ...with this pause between chunks, ms.
pub const FEED_GAP_MS: u32 = 15;

/// State of the on-flash blob cache.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cache {
    /// No cached blob.
    Missing,
    /// Cached blob, with its download time if known (Unix seconds).
    Present {
        /// Download time.
        downloaded_at_s: Option<i64>,
    },
}

/// Download if there is no cache, or (once the time is known) it is stale.
#[must_use]
pub fn should_download(cache: Cache, now_s: Option<i64>) -> bool {
    let Cache::Present { downloaded_at_s } = cache else {
        return true; // no cache
    };
    let Some(now) = now_s else {
        return false; // have a cache, cannot judge its age yet
    };
    // Age unknown but time known: refresh; otherwise refresh when stale.
    downloaded_at_s.is_none_or(|t| now - t > MAX_AGE_S)
}

/// Request-building error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RequestError {
    /// Output buffer too small.
    BufferTooSmall,
    /// Token empty or contains characters outside `[A-Za-z0-9._-]`.
    BadToken,
}

/// Builds the ZTP registration JSON body from the token and the two full UBX
/// frames (sync..checksum), which are sent as lowercase hex.
///
/// # Errors
/// [`RequestError`].
pub fn register_body(
    token: &str,
    uniqid_frame: &[u8],
    monver_frame: &[u8],
    out: &mut [u8],
) -> Result<usize, RequestError> {
    if token.is_empty()
        || !token
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b"._-".contains(&b))
    {
        return Err(RequestError::BadToken);
    }
    let mut w = Writer::new(out);
    w.str("{\"token\":\"")?;
    w.str(token)?;
    w.str("\",\"messages\":{\"UBX-SEC-UNIQID\":\"")?;
    w.hex(uniqid_frame)?;
    w.str("\",\"UBX-MON-VER\":\"")?;
    w.hex(monver_frame)?;
    w.str("\"}}")?;
    Ok(w.len)
}

/// Extracts the string value of `"key"` from a flat JSON object into `out`
/// (escapes decoded; `\uXXXX` only for ASCII). `None` if absent or malformed.
#[must_use]
pub fn json_string_field<'o>(json: &[u8], key: &str, out: &'o mut [u8]) -> Option<&'o str> {
    let mut i = 0;
    while i < json.len() {
        if json[i] != b'"' {
            i += 1;
            continue;
        }
        let end = string_end(json, i)?; // index of the closing quote
        let raw = &json[i + 1..end];
        let mut j = skip_ws(json, end + 1);
        if json.get(j) == Some(&b':') {
            if raw == key.as_bytes() {
                j = skip_ws(json, j + 1);
                if json.get(j) != Some(&b'"') {
                    return None; // value is not a string
                }
                let vend = string_end(json, j)?;
                return unescape(&json[j + 1..vend], out);
            }
            i = j + 1; // a key that doesn't match; its value is scanned next
        } else {
            i = end + 1; // a string value
        }
    }
    None
}

/// Builds the data URL `<service_url>?chipcode=<pct-encoded>&gnss=..&data=..`.
///
/// # Errors
/// [`RequestError::BufferTooSmall`].
pub fn download_url<'o>(
    service_url: &str,
    chipcode: &str,
    out: &'o mut [u8],
) -> Result<&'o str, RequestError> {
    let mut w = Writer::new(out);
    w.str(service_url)?;
    w.str("?chipcode=")?;
    for b in chipcode.bytes() {
        if b.is_ascii_alphanumeric() || b"-._~".contains(&b) {
            w.bytes(&[b])?;
        } else {
            w.bytes(&[
                b'%',
                HEX_UPPER[usize::from(b >> 4)],
                HEX_UPPER[usize::from(b & 0x0F)],
            ])?;
        }
    }
    w.str("&gnss=")?;
    w.str(GNSS)?;
    w.str("&data=")?;
    w.str(DATA)?;
    let len = w.len;
    core::str::from_utf8(&out[..len]).map_err(|_| RequestError::BufferTooSmall)
}

/// Parses an RFC 7231 date (`Mon, 22 Jun 2026 20:43:21 GMT`); years outside
/// 2024..=2050 are rejected as implausible.
#[must_use]
pub fn parse_http_date(s: &str) -> Option<UtcDateTime> {
    const MONTHS: [&str; 12] = [
        "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec",
    ];
    let mut it = s.split_whitespace();
    let _weekday = it.next()?;
    let day: u8 = it.next()?.parse().ok()?;
    let month_name = it.next()?;
    let month = MONTHS.iter().position(|m| *m == month_name)?;
    let year: u16 = it.next()?.parse().ok()?;
    let mut hms = it.next()?.split(':');
    let (hour, minute, second) = (
        hms.next()?.parse().ok()?,
        hms.next()?.parse().ok()?,
        hms.next()?.parse().ok()?,
    );
    if !(2024..=2050).contains(&year) {
        return None;
    }
    let t = UtcDateTime {
        year,
        month: month as u8 + 1,
        day,
        hour,
        minute,
        second,
    };
    t.to_unix().map(|_| t)
}

const HEX_LOWER: &[u8; 16] = b"0123456789abcdef";
const HEX_UPPER: &[u8; 16] = b"0123456789ABCDEF";

struct Writer<'a> {
    out: &'a mut [u8],
    len: usize,
}

impl<'a> Writer<'a> {
    fn new(out: &'a mut [u8]) -> Self {
        Self { out, len: 0 }
    }

    fn bytes(&mut self, b: &[u8]) -> Result<(), RequestError> {
        let dst = self
            .out
            .get_mut(self.len..self.len + b.len())
            .ok_or(RequestError::BufferTooSmall)?;
        dst.copy_from_slice(b);
        self.len += b.len();
        Ok(())
    }

    fn str(&mut self, s: &str) -> Result<(), RequestError> {
        self.bytes(s.as_bytes())
    }

    fn hex(&mut self, data: &[u8]) -> Result<(), RequestError> {
        for b in data {
            self.bytes(&[
                HEX_LOWER[usize::from(b >> 4)],
                HEX_LOWER[usize::from(b & 0x0F)],
            ])?;
        }
        Ok(())
    }
}

fn skip_ws(json: &[u8], mut i: usize) -> usize {
    while json.get(i).is_some_and(u8::is_ascii_whitespace) {
        i += 1;
    }
    i
}

/// Index of the closing quote of the string starting at `json[start] == '"'`.
fn string_end(json: &[u8], start: usize) -> Option<usize> {
    let mut i = start + 1;
    while i < json.len() {
        match json[i] {
            b'\\' => i += 2,
            b'"' => return Some(i),
            _ => i += 1,
        }
    }
    None
}

fn unescape<'o>(raw: &[u8], out: &'o mut [u8]) -> Option<&'o str> {
    let mut n = 0;
    let mut i = 0;
    while i < raw.len() {
        let c = if raw[i] == b'\\' {
            i += 1;
            match *raw.get(i)? {
                b'"' => b'"',
                b'\\' => b'\\',
                b'/' => b'/',
                b'n' => b'\n',
                b't' => b'\t',
                b'r' => b'\r',
                b'u' => {
                    let hex = core::str::from_utf8(raw.get(i + 1..i + 5)?).ok()?;
                    let v = u8::try_from(u16::from_str_radix(hex, 16).ok()?).ok()?;
                    if !v.is_ascii() {
                        return None;
                    }
                    i += 4;
                    v
                }
                _ => return None,
            }
        } else {
            raw[i]
        };
        *out.get_mut(n)? = c;
        n += 1;
        i += 1;
    }
    core::str::from_utf8(&out[..n]).ok()
}

#[cfg(test)]
mod tests;
