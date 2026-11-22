//! One task per sensor rate. Each owns its driver, writes its own section of
//! the shared state and never blocks another task's bus.

pub mod baro;
pub mod force;
pub mod gnss;
pub mod imu;
pub mod net;
pub mod power;
pub mod radio;
