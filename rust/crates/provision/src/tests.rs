use super::*;

#[test]
fn key_names_fit_nvs_and_roundtrip() {
    for k in Key::ALL {
        assert!(k.name().len() <= 15, "{}", k.name());
        assert_eq!(Key::from_name(k.name()), Some(k));
    }
    assert_eq!(Key::ZtpToken.name(), "ztp_token");
    assert_eq!(Key::WifiSsid(2).name(), "wifi2_ssid");
    assert_eq!(Key::WifiPass(4).name(), "wifi4_pass");
}

#[test]
fn unknown_and_out_of_range_names() {
    assert_eq!(Key::from_name("wifi5_ssid"), None);
    assert_eq!(Key::from_name("wifi0_pass"), None);
    assert_eq!(Key::from_name("ZTP_TOKEN"), None);
    assert_eq!(Key::from_name(""), None);
}

#[test]
fn secrets() {
    assert!(Key::ZtpToken.is_secret());
    assert!(Key::GithubToken.is_secret());
    assert!(Key::WifiPass(1).is_secret());
    assert!(!Key::WifiSsid(1).is_secret());
    assert!(!Key::ZtpUrl.is_secret());
}

#[test]
fn token_validation_blocks_injection() {
    assert_eq!(Key::ZtpToken.validate("a1B2-c3_d4.e5"), Ok(()));
    assert_eq!(Key::ZtpToken.validate(""), Err(ValueError::Length));
    assert_eq!(Key::ZtpToken.validate("abc\"}"), Err(ValueError::Charset));
    assert_eq!(Key::ZtpToken.validate("ab cd"), Err(ValueError::Charset));
    assert_eq!(Key::GithubToken.validate("ghp_0123456789abcdef"), Ok(()));
}

#[test]
fn wifi_validation() {
    assert_eq!(Key::WifiSsid(1).validate("Home Net 5G"), Ok(()));
    assert_eq!(Key::WifiSsid(1).validate(""), Err(ValueError::Length));
    assert_eq!(
        Key::WifiSsid(1).validate(&"x".repeat(33)),
        Err(ValueError::Length)
    );
    assert_eq!(Key::WifiPass(1).validate(""), Ok(()), "open network");
    assert_eq!(Key::WifiPass(1).validate("short"), Err(ValueError::Length));
    assert_eq!(Key::WifiPass(1).validate("pass word with spaces"), Ok(()));
    assert_eq!(
        Key::WifiPass(1).validate(&"x".repeat(64)),
        Err(ValueError::Length)
    );
    assert_eq!(
        Key::WifiSsid(1).validate("tab\there"),
        Err(ValueError::Charset)
    );
}

#[test]
fn url_validation() {
    assert_eq!(
        Key::ZtpUrl.validate("https://api.thingstream.io/ztp/assistnow/credentials"),
        Ok(())
    );
    assert_eq!(
        Key::ZtpUrl.validate("http://insecure.example"),
        Err(ValueError::Charset)
    );
    assert_eq!(
        Key::ZtpUrl.validate("https://a b"),
        Err(ValueError::Charset)
    );
}

#[test]
fn parse_set_keeps_spaces_in_value() {
    assert_eq!(
        parse("set wifi1_ssid My Home Net\r\n"),
        Ok(Command::Set(Key::WifiSsid(1), "My Home Net"))
    );
    assert_eq!(
        parse("set ztp_token abc123"),
        Ok(Command::Set(Key::ZtpToken, "abc123"))
    );
}

#[test]
fn parse_other_commands() {
    assert_eq!(parse("get ztp_token"), Ok(Command::Get(Key::ZtpToken)));
    assert_eq!(
        parse("clear wifi2_pass\n"),
        Ok(Command::Clear(Key::WifiPass(2)))
    );
    assert_eq!(parse("list"), Ok(Command::List));
    assert_eq!(parse("  help  "), Ok(Command::Help));
}

#[test]
fn parse_errors() {
    assert_eq!(parse("\r\n"), Err(CommandError::Empty));
    assert_eq!(parse("reboot"), Err(CommandError::UnknownCommand("reboot")));
    assert_eq!(parse("set nope 1"), Err(CommandError::UnknownKey("nope")));
    assert_eq!(parse("set ztp_token"), Err(CommandError::Missing));
    assert_eq!(parse("get"), Err(CommandError::Missing));
    assert_eq!(
        parse("set ztp_token bad\"value"),
        Err(CommandError::Invalid(Key::ZtpToken, ValueError::Charset))
    );
}

#[test]
fn show_never_reveals_secrets() {
    assert_eq!(show(Key::ZtpToken, Some("supersecret")), Shown::Secret(11));
    assert_eq!(show(Key::WifiPass(1), None), Shown::Unset);
    assert_eq!(show(Key::WifiSsid(1), Some("Home")), Shown::Plain("Home"));
}

#[test]
fn input_masking() {
    assert!(!masks_input(""));
    assert!(!masks_input("set ztp_tok"));
    assert!(!masks_input("set ztp_token"), "key not finished yet");
    assert!(masks_input("set ztp_token "));
    assert!(masks_input("set ztp_token abc"));
    assert!(masks_input("set wifi2_pass my pass"));
    assert!(!masks_input("set wifi2_ssid "), "SSID is not secret");
    assert!(!masks_input("get ztp_token "));
}

mod console {
    use super::super::*;
    use std::collections::HashMap;

    #[derive(Default)]
    struct MapStore {
        map: HashMap<&'static str, String>,
        fail: bool,
    }

    impl Store for MapStore {
        type Error = &'static str;
        fn get<'b>(&mut self, key: Key, buf: &'b mut [u8]) -> Result<Option<&'b str>, Self::Error> {
            if self.fail {
                return Err("io");
            }
            Ok(self.map.get(key.name()).map(|v| {
                buf[..v.len()].copy_from_slice(v.as_bytes());
                core::str::from_utf8(&buf[..v.len()]).unwrap()
            }))
        }
        fn set(&mut self, key: Key, value: &str) -> Result<(), Self::Error> {
            if self.fail {
                return Err("io");
            }
            self.map.insert(key.name(), value.to_string());
            Ok(())
        }
        fn remove(&mut self, key: Key) -> Result<(), Self::Error> {
            if self.fail {
                return Err("io");
            }
            self.map.remove(key.name());
            Ok(())
        }
    }

    fn run(store: &mut MapStore, line: &str) -> String {
        let mut out = String::new();
        execute(line, store, &mut out).unwrap();
        out
    }

    #[test]
    fn set_secret_is_stored_but_never_echoed() {
        let mut s = MapStore::default();
        let reply = run(&mut s, "set ztp_token abcdefghijklmnop");
        assert_eq!(reply, "ok: ztp_token set (16 chars)\n");
        assert_eq!(s.map["ztp_token"], "abcdefghijklmnop");
        assert!(!run(&mut s, "get ztp_token").contains("abcdef"));
        assert_eq!(run(&mut s, "get ztp_token"), "ztp_token: set (16 chars)\n");
    }

    #[test]
    fn plain_values_are_shown() {
        let mut s = MapStore::default();
        assert_eq!(
            run(&mut s, "set wifi1_ssid Home Net"),
            "ok: wifi1_ssid = Home Net\n"
        );
        assert_eq!(run(&mut s, "get wifi1_ssid"), "wifi1_ssid: Home Net\n");
        assert_eq!(run(&mut s, "get wifi2_ssid"), "wifi2_ssid: unset\n");
    }

    #[test]
    fn clear_removes() {
        let mut s = MapStore::default();
        run(&mut s, "set github_token ghp_x");
        assert_eq!(
            run(&mut s, "clear github_token"),
            "ok: github_token cleared\n"
        );
        assert!(!s.map.contains_key("github_token"));
    }

    #[test]
    fn list_shows_every_key_without_secrets() {
        let mut s = MapStore::default();
        run(&mut s, "set wifi1_pass verysecretpw");
        let reply = run(&mut s, "list");
        assert_eq!(reply.lines().count(), Key::ALL.len());
        assert!(reply.contains("wifi1_pass: set (12 chars)\n"));
        assert!(!reply.contains("verysecretpw"));
    }

    #[test]
    fn errors_are_explained() {
        let mut s = MapStore::default();
        assert_eq!(
            run(&mut s, "reboot"),
            "error: unknown command 'reboot' (try 'help')\n"
        );
        assert_eq!(
            run(&mut s, "get nope"),
            "error: unknown key 'nope' (try 'list')\n"
        );
        assert_eq!(
            run(&mut s, "set ztp_token"),
            "error: missing key or value\n"
        );
        assert_eq!(
            run(&mut s, "set wifi1_pass short"),
            "error: invalid value for wifi1_pass (length)\n"
        );
        assert_eq!(run(&mut s, ""), "");
    }

    #[test]
    fn storage_failures_are_reported() {
        let mut s = MapStore {
            fail: true,
            ..MapStore::default()
        };
        assert_eq!(run(&mut s, "set ztp_token abc"), "error: storage failure\n");
    }

    #[test]
    fn help_lists_commands() {
        let mut s = MapStore::default();
        let reply = run(&mut s, "help");
        for verb in ["set", "get", "clear", "list"] {
            assert!(reply.contains(verb), "{verb}");
        }
    }
}
