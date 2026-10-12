//! Tool lookup errors (own test binary because it changes environment variables).

#[test]
fn missing_tool_gives_clear_error() {
    std::env::set_var("FCRS_GMSH", "/nonexistent/gmsh");
    std::env::set_var("FCRS_CCX", "/nonexistent/ccx");
    let e = fc_fem::tools::gmsh().unwrap_err().to_string();
    assert!(e.contains("gmsh not found") && e.contains("FCRS_GMSH=/nonexistent/gmsh"), "{e}");
    let a = fc_fem::Analysis::new(fc_fem::ShapeSource::Brep("x.brep".into()), fc_fem::Material::pla());
    let e = fc_fem::solve(&a, &Default::default()).unwrap_err().to_string();
    assert!(e.contains("not found"), "{e}");
    std::env::remove_var("FCRS_GMSH");
    std::env::remove_var("FCRS_CCX");
}
