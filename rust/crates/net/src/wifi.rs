//! WiFi network selection.

/// One access point seen in a scan.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScanHit<'a> {
    /// SSID.
    pub ssid: &'a str,
    /// Signal, dBm.
    pub rssi: i8,
}

/// Minimum signal for roaming away from a working link, dBm.
pub const MIN_ROAM_RSSI: i8 = -75;

/// Strongest signal of `ssid` in the scan, if seen.
#[must_use]
pub fn best_rssi(seen: &[ScanHit<'_>], ssid: &str) -> Option<i8> {
    seen.iter().filter(|h| h.ssid == ssid).map(|h| h.rssi).max()
}

/// Join order as indices into `networks` (priority order): visible networks
/// first in list order, then unseen ones (hidden/edge APs) in list order.
pub fn order_candidates<'a>(
    networks: &'a [&'a str],
    seen: &'a [ScanHit<'a>],
) -> impl Iterator<Item = usize> + 'a {
    let visible = move |i: &usize| best_rssi(seen, networks[*i]).is_some();
    let all = 0..networks.len();
    all.clone()
        .filter(visible)
        .chain(all.filter(move |i| !visible(i)))
}

/// The network (index) the device should switch to from `current`: the first
/// configured network above `current` that is visible with at least
/// `min_rssi`. An unknown `current` ranks below everything.
#[must_use]
pub fn preferred(
    networks: &[&str],
    current: &str,
    seen: &[ScanHit<'_>],
    min_rssi: i8,
) -> Option<usize> {
    for (i, ssid) in networks.iter().enumerate() {
        if *ssid == current {
            return None;
        }
        if best_rssi(seen, ssid).is_some_and(|r| r >= min_rssi) {
            return Some(i);
        }
    }
    None
}

/// True if a roaming scan can pay off (not already on the top network).
#[must_use]
pub fn roam_worth_scanning(networks: &[&str], current: &str) -> bool {
    networks.first().is_some_and(|top| *top != current)
}

#[cfg(test)]
mod tests {
    use super::*;

    const NETWORKS: [&str; 3] = ["Home", "Hotspot", "Site"];

    fn hit(ssid: &str, rssi: i8) -> ScanHit<'_> {
        ScanHit { ssid, rssi }
    }

    fn order(seen: &[ScanHit<'_>]) -> Vec<&'static str> {
        order_candidates(&NETWORKS, seen)
            .map(|i| NETWORKS[i])
            .collect()
    }

    #[test]
    fn best_rssi_takes_strongest_of_several_aps() {
        let seen = [hit("Site", -80), hit("Site", -55), hit("Home", -70)];
        assert_eq!(best_rssi(&seen, "Site"), Some(-55));
        assert_eq!(best_rssi(&seen, "Hotspot"), None);
    }

    #[test]
    fn field_regression_priority_beats_stronger_site_ap() {
        assert_eq!(
            order(&[hit("Site", -48), hit("Hotspot", -77)])[0],
            "Hotspot"
        );
    }

    #[test]
    fn top_priority_wins_even_when_weaker() {
        assert_eq!(order(&[hit("Hotspot", -40), hit("Home", -68)])[0], "Home");
    }

    #[test]
    fn order_is_list_order_with_unseen_last() {
        assert_eq!(
            order(&[hit("Site", -40), hit("Hotspot", -80)]),
            ["Hotspot", "Site", "Home"]
        );
        assert_eq!(order(&[hit("Site", -50)]), ["Site", "Home", "Hotspot"]);
        assert_eq!(order(&[]), ["Home", "Hotspot", "Site"]);
    }

    #[test]
    fn preferred_flags_hotspot_over_site() {
        let seen = [hit("Hotspot", -66), hit("Site", -50)];
        assert_eq!(preferred(&NETWORKS, "Site", &seen, MIN_ROAM_RSSI), Some(1));
    }

    #[test]
    fn weak_ap_never_steals_a_working_link() {
        let seen = [hit("Hotspot", -85)];
        assert_eq!(preferred(&NETWORKS, "Site", &seen, MIN_ROAM_RSSI), None);
    }

    #[test]
    fn preferred_none_on_top_priority() {
        let seen = [hit("Hotspot", -40), hit("Site", -40)];
        assert_eq!(preferred(&NETWORKS, "Home", &seen, MIN_ROAM_RSSI), None);
    }

    #[test]
    fn unknown_current_ranks_below_everything() {
        let seen = [hit("Site", -60)];
        assert_eq!(
            preferred(&NETWORKS, "Neighbour", &seen, MIN_ROAM_RSSI),
            Some(2)
        );
    }

    #[test]
    fn roam_scan_only_when_not_on_top() {
        assert!(!roam_worth_scanning(&NETWORKS, "Home"));
        assert!(roam_worth_scanning(&NETWORKS, "Site"));
        assert!(!roam_worth_scanning(&[], "Site"));
    }
}
