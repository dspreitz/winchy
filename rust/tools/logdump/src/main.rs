//! `winchy-logdump <dump.bin> [out-dir]`
//!
//! Read the rope's `storage` partition first, e.g.
//! `espflash read-flash --port <port> 0x350000 0x4B0000 dump.bin`.
#![expect(clippy::print_stdout, clippy::print_stderr, reason = "CLI tool")]

use std::collections::BTreeMap;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::PathBuf;

use winchy_datalog::block::{Entry, entries};
use winchy_logdump::{backfill_utc, csv_header, csv_row, file_stem, ordered_blocks};

fn main() {
    let mut args = std::env::args().skip(1);
    let Some(input) = args.next() else {
        eprintln!("usage: winchy-logdump <dump.bin> [out-dir]");
        std::process::exit(2);
    };
    let out_dir = PathBuf::from(args.next().unwrap_or_else(|| ".".to_owned()));
    let dump = match std::fs::read(&input) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("{input}: {e}");
            std::process::exit(1);
        }
    };
    if let Err(e) = std::fs::create_dir_all(&out_dir) {
        eprintln!("{}: {e}", out_dir.display());
        std::process::exit(1);
    }
    let blocks = ordered_blocks(&dump);
    let mut all: Vec<Entry> = blocks
        .iter()
        .filter_map(|b| entries(b))
        .flat_map(|(_, it)| it)
        .collect();
    backfill_utc(&mut all);
    let mut files: BTreeMap<&str, (BufWriter<File>, usize)> = BTreeMap::new();
    for e in &all {
        let stem = file_stem(&e.record);
        if !files.contains_key(stem) {
            let path = out_dir.join(format!("{stem}.csv"));
            let mut w = match File::create(&path) {
                Ok(f) => BufWriter::new(f),
                Err(err) => {
                    eprintln!("{}: {err}", path.display());
                    std::process::exit(1);
                }
            };
            let _ = writeln!(w, "{}", csv_header(&e.record));
            files.insert(stem, (w, 0));
        }
        if let Some((w, n)) = files.get_mut(stem) {
            let _ = writeln!(w, "{}", csv_row(e));
            *n += 1;
        }
    }
    println!("{} blocks", blocks.len());
    for (stem, (mut w, n)) in files {
        let _ = w.flush();
        println!("{stem}.csv: {n} rows");
    }
}
