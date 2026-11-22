//! NVS-backed settings and the USB provisioning console.
//!
//! Values live in the NVS namespace [`NAMESPACE`]; which keys exist and how
//! commands behave is defined (and host-tested) in `winchy-provision`.

use std::io::Write as _;

use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};
use esp_idf_svc::sys::EspError;
use winchy_provision::{Key, Store, execute, masks_input};

/// NVS namespace for all Winchy settings.
pub const NAMESPACE: &str = "winchy";

/// Longest console line accepted.
const MAX_LINE: usize = 320;

/// Settings stored in NVS.
pub struct NvsStore {
    nvs: EspNvs<NvsDefault>,
}

impl NvsStore {
    /// Opens (creating if needed) the Winchy namespace on the default partition.
    ///
    /// # Errors
    /// NVS errors.
    pub fn open(partition: EspDefaultNvsPartition) -> Result<Self, EspError> {
        Ok(Self {
            nvs: EspNvs::new(partition, NAMESPACE, true)?,
        })
    }
}

impl NvsStore {
    /// Reads a device-internal value (not a console key, e.g. the AssistNow
    /// chipcode the receiver registration returned).
    ///
    /// # Errors
    /// NVS errors.
    pub fn get_internal<'b>(
        &self,
        name: &str,
        buf: &'b mut [u8],
    ) -> Result<Option<&'b str>, EspError> {
        self.nvs.get_str(name, buf)
    }

    /// Stores a device-internal value.
    ///
    /// # Errors
    /// NVS errors.
    pub fn set_internal(&self, name: &str, value: &str) -> Result<(), EspError> {
        self.nvs.set_str(name, value)
    }
}

impl Store for NvsStore {
    type Error = EspError;

    fn get<'b>(&mut self, key: Key, buf: &'b mut [u8]) -> Result<Option<&'b str>, EspError> {
        self.nvs.get_str(key.name(), buf)
    }

    fn set(&mut self, key: Key, value: &str) -> Result<(), EspError> {
        self.nvs.set_str(key.name(), value)
    }

    fn remove(&mut self, key: Key) -> Result<(), EspError> {
        self.nvs.remove(key.name()).map(|_| ())
    }
}

/// Line console on the system console (native USB-Serial-JTAG).
///
/// Reads through the console's own VFS without installing the USB-Serial-JTAG
/// driver: installing it while the port is the system console hangs in
/// `esp_intr_alloc` (interrupt watchdog, rope board, ESP-IDF 5.5.5).
///
/// Without the driver, ESP-IDF 5.5.5's `read()` in BLOCKING mode makes exactly
/// one attempt and returns one byte or -1 immediately; in `O_NONBLOCK` mode it
/// never returns data (`usb_serial_jtag_get_read_bytes_available()` is 0
/// without the driver). So stdin stays in its default mode and is polled.
pub struct Console {
    line: String,
}

impl Console {
    /// Creates the console (stdin stays in its default mode, see type docs).
    #[must_use]
    pub fn new() -> Self {
        Self {
            line: String::with_capacity(MAX_LINE),
        }
    }

    /// Writes `text` (the console VFS already turns `\n` into `\r\n`).
    fn write(text: &str) {
        let mut out = std::io::stdout().lock();
        let _ = out.write_all(text.as_bytes());
        let _ = out.flush();
        drop(out);
        // Without the driver the USB-Serial-JTAG VFS only pushes its TX FIFO
        // on '\n'; fsync sends single echoed characters immediately.
        // SAFETY: fsync on fd 1 (stdout) has no memory-safety preconditions.
        unsafe { esp_idf_svc::sys::fsync(1) };
    }

    /// Reads pending input (sleeping `idle_ms` if there is none) and executes
    /// every completed line.
    pub fn poll<S: Store>(&mut self, store: &mut S, idle_ms: u32) {
        let mut buf = [0u8; 64];
        let mut n = 0;
        while n < buf.len() {
            let mut byte = 0u8;
            // SAFETY: reads at most one byte into a local variable.
            let r = unsafe { esp_idf_svc::sys::read(0, (&raw mut byte).cast(), 1) };
            if r != 1 {
                break;
            }
            buf[n] = byte;
            n += 1;
        }
        if n == 0 {
            FreeRtos::delay_ms(idle_ms);
            return;
        }
        for &b in &buf[..n] {
            match b {
                b'\r' | b'\n' => {
                    if self.line.is_empty() {
                        continue;
                    }
                    Self::write("\n");
                    let line = core::mem::take(&mut self.line);
                    let mut reply = String::new();
                    let _ = execute(&line, store, &mut reply);
                    Self::write(&reply);
                    Self::write("> ");
                }
                0x08 | 0x7F => {
                    if self.line.pop().is_some() {
                        Self::write("\x08 \x08");
                    }
                }
                0x20..=0x7E if self.line.len() < MAX_LINE => {
                    let echo = if masks_input(&self.line) {
                        '*'
                    } else {
                        char::from(b)
                    };
                    self.line.push(char::from(b));
                    let mut tmp = [0u8; 4];
                    Self::write(echo.encode_utf8(&mut tmp));
                }
                _ => {}
            }
        }
    }

    /// Prints the banner and prompt.
    pub fn greet(&self) {
        Self::write("\nwinchy console - type 'help'\n> ");
    }
}

impl Default for Console {
    fn default() -> Self {
        Self::new()
    }
}
