//! Network policy for both units, pure and host-tested.
//!
//! WiFi: the configured list order IS the priority (field lesson: joining the
//! strongest AP picked an unreachable site network over the operator's phone
//! hotspot). Visible networks are tried first, unseen ones last; roaming only
//! moves to a higher-priority network with a usable signal.
#![cfg_attr(not(test), no_std)]

pub mod dns;
pub mod wifi;
