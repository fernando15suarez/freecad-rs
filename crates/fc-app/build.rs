//! Embeds every SVG in assets/icons (FreeCAD's icons, see assets/icons/NOTICE).
use std::{env, fs, path::Path};

fn main() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("assets/icons");
    println!("cargo:rerun-if-changed={}", dir.display());
    let mut names: Vec<String> = fs::read_dir(&dir)
        .unwrap()
        .filter_map(|e| e.ok())
        .map(|e| e.file_name().to_string_lossy().into_owned())
        .filter(|n| n.ends_with(".svg"))
        .collect();
    names.sort();
    let mut src = String::from("pub static ICONS: &[(&str, &[u8])] = &[\n");
    for n in names {
        let stem = n.trim_end_matches(".svg");
        src.push_str(&format!("    ({stem:?}, include_bytes!({:?})),\n", dir.join(&n).display().to_string()));
    }
    src.push_str("];\n");
    fs::write(Path::new(&env::var("OUT_DIR").unwrap()).join("icons.rs"), src).unwrap();
}
