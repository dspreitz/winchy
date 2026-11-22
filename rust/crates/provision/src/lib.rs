//! Device provisioning over the USB console.
//!
//! Secrets (WiFi passwords, API tokens) never live in the firmware image:
//! they are typed once into the serial console and stored in NVS. This crate
//! defines which keys exist, validates values, and parses console commands.
//! Secret values are never echoed back, only "set, N chars".
#![cfg_attr(not(test), no_std)]

/// A provisionable setting.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Key {
    /// u-blox Thingstream AssistNow ZTP device-profile token.
    ZtpToken,
    /// Optional override of the ZTP endpoint URL.
    ZtpUrl,
    /// GitHub token for log uploads.
    GithubToken,
    /// WiFi network SSID, priority slot 1..=4 (1 = most preferred).
    WifiSsid(u8),
    /// WiFi password for the same slot.
    WifiPass(u8),
}

/// Number of WiFi slots.
pub const WIFI_SLOTS: u8 = 4;

/// Why a key or value was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ValueError {
    /// Length outside the allowed range.
    Length,
    /// Contains a character that is not allowed for this key.
    Charset,
}

impl Key {
    /// Every key, in display order.
    pub const ALL: [Key; 11] = [
        Key::ZtpToken,
        Key::ZtpUrl,
        Key::GithubToken,
        Key::WifiSsid(1),
        Key::WifiPass(1),
        Key::WifiSsid(2),
        Key::WifiPass(2),
        Key::WifiSsid(3),
        Key::WifiPass(3),
        Key::WifiSsid(4),
        Key::WifiPass(4),
    ];

    /// NVS key name (at most 15 characters).
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Key::ZtpToken => "ztp_token",
            Key::ZtpUrl => "ztp_url",
            Key::GithubToken => "github_token",
            Key::WifiSsid(1) => "wifi1_ssid",
            Key::WifiSsid(2) => "wifi2_ssid",
            Key::WifiSsid(3) => "wifi3_ssid",
            Key::WifiSsid(4) => "wifi4_ssid",
            Key::WifiPass(1) => "wifi1_pass",
            Key::WifiPass(2) => "wifi2_pass",
            Key::WifiPass(3) => "wifi3_pass",
            Key::WifiPass(4) => "wifi4_pass",
            Key::WifiSsid(_) | Key::WifiPass(_) => "invalid_slot",
        }
    }

    /// Parses a console key name.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Self> {
        Key::ALL.into_iter().find(|k| k.name() == name)
    }

    /// True if the value must never be shown.
    #[must_use]
    pub const fn is_secret(self) -> bool {
        matches!(self, Key::ZtpToken | Key::GithubToken | Key::WifiPass(_))
    }

    /// Validates a value for this key.
    ///
    /// # Errors
    /// [`ValueError`] if the length or a character is not allowed.
    pub fn validate(self, value: &str) -> Result<(), ValueError> {
        let printable = |v: &str| v.bytes().all(|b| (0x20..=0x7E).contains(&b));
        let (len_ok, chars_ok) = match self {
            Key::ZtpToken | Key::GithubToken => (
                (1..=255).contains(&value.len()),
                value
                    .bytes()
                    .all(|b| b.is_ascii_alphanumeric() || b"-_.".contains(&b)),
            ),
            Key::ZtpUrl => (
                (9..=200).contains(&value.len()),
                value.starts_with("https://")
                    && value
                        .bytes()
                        .all(|b| b.is_ascii_graphic() && !b"\"'<>\\".contains(&b)),
            ),
            Key::WifiSsid(_) => ((1..=32).contains(&value.len()), printable(value)),
            Key::WifiPass(_) => (
                value.is_empty() || (8..=63).contains(&value.len()),
                printable(value),
            ),
        };
        if !chars_ok {
            return Err(ValueError::Charset);
        }
        if !len_ok {
            return Err(ValueError::Length);
        }
        Ok(())
    }
}

/// A parsed console command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Command<'a> {
    /// `set <key> <value>` (the value is the rest of the line, may contain spaces).
    Set(Key, &'a str),
    /// `get <key>`.
    Get(Key),
    /// `clear <key>`.
    Clear(Key),
    /// `list`.
    List,
    /// `help`.
    Help,
}

/// Why a command line was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandError<'a> {
    /// Empty line.
    Empty,
    /// Unknown verb.
    UnknownCommand(&'a str),
    /// Unknown key name.
    UnknownKey(&'a str),
    /// Key or value missing.
    Missing,
    /// Value rejected for the key.
    Invalid(Key, ValueError),
}

/// Parses one console line (trailing CR/LF ignored).
///
/// # Errors
/// [`CommandError`] describing the problem.
pub fn parse(line: &str) -> Result<Command<'_>, CommandError<'_>> {
    let line = line.trim();
    let (verb, rest) = line.split_once(' ').unwrap_or((line, ""));
    let rest = rest.trim_start();
    match verb {
        "" => Err(CommandError::Empty),
        "list" => Ok(Command::List),
        "help" => Ok(Command::Help),
        "set" | "get" | "clear" => {
            let (name, value) = rest.split_once(' ').unwrap_or((rest, ""));
            if name.is_empty() {
                return Err(CommandError::Missing);
            }
            let key = Key::from_name(name).ok_or(CommandError::UnknownKey(name))?;
            match verb {
                "set" => {
                    let value = value.trim_start();
                    if value.is_empty() && !matches!(key, Key::WifiPass(_)) {
                        return Err(CommandError::Missing);
                    }
                    key.validate(value)
                        .map_err(|e| CommandError::Invalid(key, e))?;
                    Ok(Command::Set(key, value))
                }
                "get" => Ok(Command::Get(key)),
                _ => Ok(Command::Clear(key)),
            }
        }
        other => Err(CommandError::UnknownCommand(other)),
    }
}

/// How a stored value may be shown: secrets only as their length.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Shown<'a> {
    /// Not stored.
    Unset,
    /// Secret, stored with this many characters.
    Secret(usize),
    /// Non-secret value.
    Plain(&'a str),
}

/// Display policy for a stored value.
#[must_use]
pub fn show(key: Key, stored: Option<&str>) -> Shown<'_> {
    match stored {
        None => Shown::Unset,
        Some(v) if key.is_secret() => Shown::Secret(v.len()),
        Some(v) => Shown::Plain(v),
    }
}

/// True if the next typed character belongs to a secret value and must be
/// echoed as `*`: `prefix` (the line typed so far) is `set <secret-key> ...`.
#[must_use]
pub fn masks_input(prefix: &str) -> bool {
    let Some(rest) = prefix.strip_prefix("set ") else {
        return false;
    };
    let rest = rest.trim_start();
    rest.split_once(' ')
        .and_then(|(name, _)| Key::from_name(name))
        .is_some_and(Key::is_secret)
}

/// Persistent key-value storage (NVS on the device, a map in tests).
pub trait Store {
    /// Storage error.
    type Error: core::fmt::Debug;

    /// Reads a value into `buf`; `Ok(None)` if unset.
    ///
    /// # Errors
    /// Storage failure.
    fn get<'b>(&mut self, key: Key, buf: &'b mut [u8]) -> Result<Option<&'b str>, Self::Error>;

    /// Stores a value.
    ///
    /// # Errors
    /// Storage failure.
    fn set(&mut self, key: Key, value: &str) -> Result<(), Self::Error>;

    /// Removes a value (no error if unset).
    ///
    /// # Errors
    /// Storage failure.
    fn remove(&mut self, key: Key) -> Result<(), Self::Error>;
}

/// Longest value any key accepts (buffer size for reads).
pub const MAX_VALUE_LEN: usize = 255;

/// Executes one console line against `store`, writing the reply (one or more
/// lines, each ending in `\n`) to `out`. Secrets are never written to `out`.
///
/// # Errors
/// Only formatting errors from `out`.
pub fn execute<S: Store, W: core::fmt::Write>(
    line: &str,
    store: &mut S,
    out: &mut W,
) -> core::fmt::Result {
    let cmd = match parse(line) {
        Ok(cmd) => cmd,
        Err(CommandError::Empty) => return Ok(()),
        Err(CommandError::UnknownCommand(v)) => {
            return writeln!(out, "error: unknown command '{v}' (try 'help')");
        }
        Err(CommandError::UnknownKey(k)) => {
            return writeln!(out, "error: unknown key '{k}' (try 'list')");
        }
        Err(CommandError::Missing) => return writeln!(out, "error: missing key or value"),
        Err(CommandError::Invalid(key, e)) => {
            let why = match e {
                ValueError::Length => "length",
                ValueError::Charset => "characters",
            };
            return writeln!(out, "error: invalid value for {} ({why})", key.name());
        }
    };
    match cmd {
        Command::Set(key, value) => match store.set(key, value) {
            Ok(()) if key.is_secret() => {
                writeln!(out, "ok: {} set ({} chars)", key.name(), value.len())
            }
            Ok(()) => writeln!(out, "ok: {} = {value}", key.name()),
            Err(_) => writeln!(out, "error: storage failure"),
        },
        Command::Get(key) => show_key(key, store, out),
        Command::Clear(key) => match store.remove(key) {
            Ok(()) => writeln!(out, "ok: {} cleared", key.name()),
            Err(_) => writeln!(out, "error: storage failure"),
        },
        Command::List => {
            for key in Key::ALL {
                show_key(key, store, out)?;
            }
            Ok(())
        }
        Command::Help => out.write_str(HELP),
    }
}

const HELP: &str = "commands:\n  set <key> <value>   store a value (rest of the line, spaces allowed)\n  get <key>           show a value (secrets only as their length)\n  clear <key>         remove a value\n  list                show all keys\n";

fn show_key<S: Store, W: core::fmt::Write>(
    key: Key,
    store: &mut S,
    out: &mut W,
) -> core::fmt::Result {
    let mut buf = [0u8; MAX_VALUE_LEN];
    match store.get(key, &mut buf) {
        Ok(stored) => match show(key, stored) {
            Shown::Unset => writeln!(out, "{}: unset", key.name()),
            Shown::Secret(n) => writeln!(out, "{}: set ({n} chars)", key.name()),
            Shown::Plain(v) => writeln!(out, "{}: {v}", key.name()),
        },
        Err(_) => writeln!(out, "error: storage failure"),
    }
}

#[cfg(test)]
mod tests;
