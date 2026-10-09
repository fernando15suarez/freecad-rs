//! Import a FreeCAD .FCStd file and print its features, import notes, recompute errors
//! and the body's volume.
//!
//!     cargo run -p fc-part --example fcstd_info -- part.FCStd

use fc_part::fcstd::{self, NoteKind};
use fc_part::FeatureKind;

fn main() {
    let Some(path) = std::env::args().nth(1) else {
        eprintln!("usage: fcstd_info FILE.FCStd");
        std::process::exit(2);
    };
    let (mut doc, report) = match fcstd::import_with_report(&path) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("{path}: {e}");
            std::process::exit(1);
        }
    };
    println!("{path}: FreeCAD {} — body {}", report.program_version, report.body);
    doc.recompute();
    println!("\nfeatures:");
    for (f, st) in doc.features.iter().zip(&doc.states) {
        let mut line = format!("  {:<20} {:<16}", f.name, f.kind.type_name());
        if f.label != f.name {
            line += &format!(" \"{}\"", f.label);
        }
        if let FeatureKind::Sketch(s) = &f.kind {
            line += &format!(" {} geo, {} constraints", s.sketch.geometry.len(), s.sketch.constraints.len());
            if let Some(r) = &st.solve {
                line += &format!(", dof {} {:?}", r.dof, r.status);
                let map = report.sketches.get(&f.name);
                let fc = |v: &[usize]| map.map(|m| m.to_freecad(v)).unwrap_or_else(|| v.to_vec());
                if !r.conflicting.is_empty() {
                    line += &format!(", conflicting (FreeCAD index) {:?}", fc(&r.conflicting));
                }
                if !r.redundant.is_empty() {
                    line += &format!(", redundant (FreeCAD index) {:?}", fc(&r.redundant));
                }
            }
        } else if let Some(s) = &st.shape {
            line += &format!(" volume {:.6}", s.volume());
        }
        if f.suppressed {
            line += " [suppressed]";
        }
        if let Some(e) = &st.error {
            line += &format!("  ERROR: {e}");
        }
        println!("{line}");
    }
    if !doc.parameters.is_empty() || !doc.expressions.is_empty() {
        println!("\nparameters / expressions:");
        for (k, v) in &doc.parameters {
            println!("  {k} = {v}");
        }
        for (k, v) in &doc.expressions {
            println!("  {k} = {v}");
        }
    }
    if !report.notes.is_empty() {
        println!("\nimport notes:");
        for n in &report.notes {
            let tag = match n.kind {
                NoteKind::Skipped => "skipped",
                NoteKind::Approximated => "approx ",
                NoteKind::Info => "info   ",
            };
            println!("  [{tag}] {}: {}", n.object, n.message);
        }
    }
    match doc.result() {
        Some(s) => {
            let b = s.bbox();
            println!("\nbody volume {:.6} mm^3, area {:.6} mm^2, {} faces", s.volume(), s.area(), s.face_count());
            println!("bbox [{:.4}, {:.4}, {:.4}] .. [{:.4}, {:.4}, {:.4}]", b[0], b[1], b[2], b[3], b[4], b[5]);
        }
        None => println!("\nno solid"),
    }
}
