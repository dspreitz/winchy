//! `winchy-console`: serial console for provisioning a Winchy unit.
//!
//! Usage: `cargo run -p winchy-console -- /dev/cu.usbmodemXXXX`
//! Type `exit` (or `quit`) to leave. `set <secret-key>` without a value asks
//! for the value with hidden input, so it never shows on screen or in history.
//!
//! DTR/RTS are kept low: on the ESP32-S3 native USB port an asserted DTR
//! silences the unit's console output.
#![expect(
    clippy::print_stdout,
    clippy::print_stderr,
    reason = "interactive CLI tool"
)]

use std::io::{BufRead as _, Read as _, Write as _};
use std::process::ExitCode;
use std::time::Duration;

use winchy_console::{Action, classify, secret_command};

/// Espressif USB vendor id (native USB-Serial-JTAG).
const ESPRESSIF_VID: u16 = 0x303A;

fn list_ports() {
    match serialport::available_ports() {
        Ok(ports) => {
            for p in ports {
                if let serialport::SerialPortType::UsbPort(u) = &p.port_type
                    && u.vid == ESPRESSIF_VID
                {
                    println!("{}", p.port_name);
                }
            }
        }
        Err(e) => eprintln!("cannot list ports: {e}"),
    }
}

fn main() -> ExitCode {
    let Some(port_name) = std::env::args().nth(1) else {
        eprintln!("usage: winchy-console <port>\nEspressif ports:");
        list_ports();
        return ExitCode::FAILURE;
    };
    let mut port = match serialport::new(&port_name, 115_200)
        .timeout(Duration::from_millis(100))
        .open()
    {
        Ok(p) => p,
        Err(e) => {
            eprintln!("cannot open {port_name}: {e} (is another monitor still open?)");
            return ExitCode::FAILURE;
        }
    };
    let _ = port.write_data_terminal_ready(false);
    let _ = port.write_request_to_send(false);

    let Ok(mut reader) = port.try_clone() else {
        eprintln!("cannot clone port handle");
        return ExitCode::FAILURE;
    };
    std::thread::spawn(move || {
        let mut buf = [0u8; 512];
        let mut out = std::io::stdout();
        loop {
            match reader.read(&mut buf) {
                Ok(n) if n > 0 => {
                    let _ = out.write_all(&buf[..n]);
                    let _ = out.flush();
                }
                Ok(_) => {}
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                Err(e) => {
                    eprintln!("\nport closed: {e}");
                    std::process::exit(1);
                }
            }
        }
    });

    println!("connected to {port_name} - type 'help' for commands, 'exit' to quit");
    for line in std::io::stdin().lock().lines() {
        let Ok(line) = line else { break };
        let command = match classify(&line) {
            Action::Exit => break,
            Action::Nothing => continue,
            Action::Send(cmd) => cmd.to_owned(),
            Action::PromptSecret(key) => {
                match rpassword::prompt_password(format!("{} (hidden): ", key.name())) {
                    Ok(value) => secret_command(key, value.trim()),
                    Err(e) => {
                        eprintln!("input error: {e}");
                        continue;
                    }
                }
            }
        };
        if let Err(e) = port.write_all(format!("{command}\r").as_bytes()) {
            eprintln!("write failed: {e}");
            break;
        }
    }
    println!("bye");
    ExitCode::SUCCESS
}
