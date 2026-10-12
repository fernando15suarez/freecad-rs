mod common;

/// Writes tools/parity/fem/bracket.step for the FreeCAD reference script.
/// `cargo test -p fc-fem --test bracket_step -- --ignored`
#[test]
#[ignore]
fn write_bracket_step() {
    let s = common::bracket();
    println!("bracket bbox {:?} volume {}", s.bbox(), s.volume());
    s.write_step(&common::parity_dir().join("bracket.step").to_string_lossy()).unwrap();
}

#[test]
fn bracket_is_a_solid() {
    let s = common::bracket();
    assert_eq!(s.solid_count(), 1);
    let v = s.volume();
    let expect = (60.0 * 8.0 + 8.0 * 42.0) * 30.0 - std::f64::consts::PI * 25.0 * 8.0;
    assert!((v - expect).abs() < 1e-6 * expect, "{v} vs {expect}");
}
