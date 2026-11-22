//! Propagates the ESP-IDF linker arguments from esp-idf-sys.
fn main() {
    embuild::espidf::sysenv::output();
}
