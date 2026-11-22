//! Line handling of the host console (pure, tested).

use winchy_provision::Key;

/// What to do with one line typed by the user.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Action<'a> {
    /// Leave the console.
    Exit,
    /// `set <secret-key>` without a value: ask for it with hidden input.
    PromptSecret(Key),
    /// Send the line to the unit as typed.
    Send(&'a str),
    /// Nothing to send (blank line).
    Nothing,
}

/// Classifies a typed line.
#[must_use]
pub fn classify(line: &str) -> Action<'_> {
    let line = line.trim();
    match line {
        "" => Action::Nothing,
        "exit" | "quit" => Action::Exit,
        _ => match line
            .strip_prefix("set ")
            .map(str::trim)
            .and_then(Key::from_name)
        {
            Some(key) if key.is_secret() => Action::PromptSecret(key),
            _ => Action::Send(line),
        },
    }
}

/// The command sent for a secret entered via the hidden prompt.
#[must_use]
pub fn secret_command(key: Key, value: &str) -> String {
    format!("set {} {value}", key.name())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exit_and_quit_leave() {
        assert_eq!(classify("exit"), Action::Exit);
        assert_eq!(classify("  quit \n"), Action::Exit);
    }

    #[test]
    fn bare_secret_set_prompts() {
        assert_eq!(
            classify("set ztp_token"),
            Action::PromptSecret(Key::ZtpToken)
        );
        assert_eq!(
            classify("set wifi2_pass  "),
            Action::PromptSecret(Key::WifiPass(2))
        );
    }

    #[test]
    fn everything_else_is_sent_trimmed() {
        assert_eq!(
            classify("set wifi1_ssid My Net\n"),
            Action::Send("set wifi1_ssid My Net")
        );
        assert_eq!(
            classify("set ztp_token abc"),
            Action::Send("set ztp_token abc")
        );
        assert_eq!(
            classify("set wifi1_ssid"),
            Action::Send("set wifi1_ssid"),
            "not secret"
        );
        assert_eq!(classify("list"), Action::Send("list"));
        assert_eq!(classify("exitnow"), Action::Send("exitnow"));
    }

    #[test]
    fn blank_lines_send_nothing() {
        assert_eq!(classify(""), Action::Nothing);
        assert_eq!(classify("   \r\n"), Action::Nothing);
    }

    #[test]
    fn secret_command_format() {
        assert_eq!(secret_command(Key::ZtpToken, "abc"), "set ztp_token abc");
        assert_eq!(
            secret_command(Key::WifiPass(1), "pass word"),
            "set wifi1_pass pass word"
        );
    }
}
