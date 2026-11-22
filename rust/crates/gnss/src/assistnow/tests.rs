use super::*;

const DAY: i64 = 86_400;

#[test]
fn download_decision() {
    assert!(should_download(Cache::Missing, None));
    assert!(should_download(Cache::Missing, Some(1_000)));
    let fresh = Cache::Present {
        downloaded_at_s: Some(10 * DAY),
    };
    assert!(
        !should_download(fresh, None),
        "cannot judge age without time"
    );
    assert!(!should_download(fresh, Some(12 * DAY)));
    assert!(should_download(fresh, Some(13 * DAY + 1)));
    assert!(should_download(
        Cache::Present {
            downloaded_at_s: None
        },
        Some(DAY)
    ));
    assert!(!should_download(
        Cache::Present {
            downloaded_at_s: None
        },
        None
    ));
}

fn body(token: &str, a: &[u8], b: &[u8]) -> Result<String, RequestError> {
    let mut out = [0u8; 256];
    register_body(token, a, b, &mut out).map(|n| String::from_utf8(out[..n].to_vec()).unwrap())
}

#[test]
fn register_body_format() {
    assert_eq!(
        body("tok-1_2.3", &[0xB5, 0x62, 0x27], &[0x0A, 0xFF]).unwrap(),
        r#"{"token":"tok-1_2.3","messages":{"UBX-SEC-UNIQID":"b56227","UBX-MON-VER":"0aff"}}"#
    );
}

#[test]
fn register_body_rejects_unsafe_tokens() {
    assert_eq!(body("", &[], &[]), Err(RequestError::BadToken));
    assert_eq!(body("a\",\"x\":\"y", &[], &[]), Err(RequestError::BadToken));
    assert_eq!(body("a b", &[], &[]), Err(RequestError::BadToken));
}

#[test]
fn register_body_small_buffer() {
    let mut out = [0u8; 20];
    assert_eq!(
        register_body("tok", &[1], &[2], &mut out),
        Err(RequestError::BufferTooSmall)
    );
}

#[test]
fn json_field_extraction() {
    let json = br#"{ "allowedData" : ["uporb_7"], "chipcode":"Ab+c/d==", "serviceUrl" : "https:\/\/x.example\/a?b" }"#;
    let mut out = [0u8; 64];
    assert_eq!(
        json_string_field(json, "chipcode", &mut out),
        Some("Ab+c/d==")
    );
    assert_eq!(
        json_string_field(json, "serviceUrl", &mut out),
        Some("https://x.example/a?b")
    );
    assert_eq!(json_string_field(json, "missing", &mut out), None);
}

#[test]
fn json_field_escapes_and_errors() {
    let mut out = [0u8; 32];
    assert_eq!(
        json_string_field(br#"{"k":"a\"b\\cA"}"#, "k", &mut out),
        Some("a\"b\\cA")
    );
    assert_eq!(
        json_string_field(br#"{"k":"unterminated}"#, "k", &mut out),
        None
    );
    assert_eq!(json_string_field(br#"{"k":123}"#, "k", &mut out), None);
    let mut tiny = [0u8; 2];
    assert_eq!(json_string_field(br#"{"k":"abc"}"#, "k", &mut tiny), None);
    // A key name appearing inside another value must not match.
    assert_eq!(
        json_string_field(br#"{"a":"k","k":"v"}"#, "k", &mut out),
        Some("v")
    );
}

#[test]
fn data_url() {
    let mut out = [0u8; 128];
    assert_eq!(
        download_url("https://x.example/data", "Ab+c/d==", &mut out),
        Ok("https://x.example/data?chipcode=Ab%2Bc%2Fd%3D%3D&gnss=gps&data=uporb_7,ualm")
    );
    let mut small = [0u8; 10];
    assert_eq!(
        download_url("https://x.example", "c", &mut small),
        Err(RequestError::BufferTooSmall)
    );
}

#[test]
fn http_date() {
    assert_eq!(
        parse_http_date("Mon, 22 Jun 2026 20:43:21 GMT"),
        Some(UtcDateTime {
            year: 2026,
            month: 6,
            day: 22,
            hour: 20,
            minute: 43,
            second: 21
        })
    );
    assert_eq!(parse_http_date("Mon, 22 Jun 2070 20:43:21 GMT"), None);
    assert_eq!(parse_http_date("Mon, 22 Jun 1999 20:43:21 GMT"), None);
    assert_eq!(parse_http_date("Mon, 22 Foo 2026 20:43:21 GMT"), None);
    assert_eq!(parse_http_date("garbage"), None);
    assert_eq!(parse_http_date(""), None);
    assert_eq!(
        parse_http_date("Mon, 31 Feb 2026 20:43:21 GMT"),
        None,
        "invalid day"
    );
}
