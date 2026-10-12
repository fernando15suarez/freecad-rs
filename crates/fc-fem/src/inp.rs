//! CalculiX input (.inp) writer following FreeCAD's `femsolver/calculix/writer.py` section by
//! section (same set names, comments, ordering, number formats and blank lines), so a file
//! written here diffs cleanly against one written by FreeCAD for the same mesh.

use std::fmt::Write as _;

use crate::fmt::{fmt_g, g13, py};
use crate::mesh::FemMesh;
use crate::model::{Analysis, AnalysisType, ConstraintKind, HeatFluxKind};
use crate::resolve::{ConstraintData, Resolved};

const STARS: &str = "***********************************************************";

const UNITS_INFORMATION: &str = "***********************************************************
**  About units:
**  See ccx manual, ccx does not know about any unit.
**  Golden rule: The user must make sure that the numbers they provide have consistent units.
**  The user is the FreeCAD calculix writer module ;-)
**
**  The unit system which is used at Guido Dhondt's company: mm, N, s, K
**  Since Length and Mass are connected by Force, if Length is mm the Mass is in t to get N
**  The following units are used to write to inp file:
**
**  Length: mm (this includes the mesh geometry)
**  Mass: t
**  TimeSpan: s
**  Temperature: K
**
**  This leads to:
**  Force: N
**  Pressure: N/mm^2 == MPa (Young's Modulus has unit Pressure)
**  Density: t/mm^3
**  Gravity: mm/s^2
**  Thermal conductivity: t*mm/K/s^3 == as W/m/K == kW/mm/K
**  Specific Heat: mm^2/s^2/K = J/kg/K == kJ/t/K
";

/// Options for the parts of the file that are not analysis data.
#[derive(Debug, Clone)]
pub struct InpOptions {
    /// "written by" footer value.
    pub written_by: String,
    /// "written on" footer value (FreeCAD: `time.ctime()`).
    pub written_on: String,
    /// "file name" footer value (FreeCAD: the .FCStd file name).
    pub file_name: String,
}

impl Default for InpOptions {
    fn default() -> Self {
        let secs = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
        InpOptions {
            written_by: format!("freecad-rs fc-fem {}", env!("CARGO_PKG_VERSION")),
            written_on: format!("{secs} (unix time)"),
            file_name: String::new(),
        }
    }
}

#[derive(Clone, Copy, PartialEq)]
enum Ty {
    Static,
    Frequency,
    Thermomech,
}

fn ty(a: &Analysis) -> Ty {
    match a.analysis_type {
        AnalysisType::Static => Ty::Static,
        AnalysisType::Frequency { .. } => Ty::Frequency,
        AnalysisType::Thermomech { .. } => Ty::Thermomech,
    }
}

/// Write the mesh part (FreeCAD's `FemMesh.writeABAQUS`, highest dimension elements only).
pub fn write_mesh(out: &mut String, mesh: &FemMesh) {
    out.push_str("** written by FreeCAD inp file writer for CalculiX,Abaqus meshes\n");
    out.push_str("** highest dimension mesh elements only.\n\n");
    out.push_str("** Nodes\n*Node, NSET=Nall\n");
    for (id, p) in mesh.node_ids.iter().zip(&mesh.coords) {
        let _ = writeln!(out, "{id}, {}, {}, {}", fmt_g(p[0], 13, false), fmt_g(p[1], 13, false), fmt_g(p[2], 13, false));
    }
    out.push_str("\n\n** Volume elements\n");
    let mut els: Vec<_> = mesh.elements.iter().collect();
    els.sort_by_key(|e| e.id);
    let _ = writeln!(out, "*Element, TYPE={}, ELSET=Evolumes", els.first().map(|e| e.ccx_type()).unwrap_or("C3D4"));
    for e in els {
        out.push_str(&e.id.to_string());
        for n in &e.nodes {
            let _ = write!(out, ", {n}");
        }
        out.push('\n');
    }
    out.push_str("\n** Define element set Eall\n*ELSET, ELSET=Eall\nEvolumes\n");
    // write_mesh.py reopens the file and appends two newlines
    out.push_str("\n\n");
}

fn section(out: &mut String, title: &str) {
    let _ = write!(out, "\n{STARS}\n** {title}\n");
}

/// Write the complete CalculiX input file.
pub fn write_inp(a: &Analysis, mesh: &FemMesh, r: &Resolved, opts: &InpOptions) -> String {
    let t = ty(a);
    let mut f = String::with_capacity(mesh.node_count() * 60 + mesh.element_count() * 70 + 4096);
    write_mesh(&mut f, mesh);

    // ---- element sets for materials
    section(&mut f, "Element sets for materials and FEM element type (solid, shell, beam, fluid)");
    let elset_name = |i: usize| -> String {
        let long = format!("{}Solid", a.materials[i].name);
        if long.len() < 81 {
            long
        } else {
            format!("M{i}Solid")
        }
    };
    for (i, _) in a.materials.iter().enumerate() {
        let _ = writeln!(f, "*ELSET,ELSET={}", elset_name(i));
        if a.materials.len() == 1 {
            f.push_str("Evolumes\n");
        } else {
            for id in &r.material_elements[i] {
                let _ = writeln!(f, "{id},");
            }
        }
    }

    let cons = |pred: &dyn Fn(&ConstraintKind) -> bool| -> Vec<usize> {
        a.constraints.iter().enumerate().filter(|(_, c)| pred(&c.kind)).map(|(i, _)| i).collect()
    };
    let fixed = cons(&|k| matches!(k, ConstraintKind::Fixed));
    let disp = cons(&|k| matches!(k, ConstraintKind::Displacement { .. }));
    let temp = cons(&|k| matches!(k, ConstraintKind::Temperature { .. } | ConstraintKind::HeatSource { .. }));
    let itemp = cons(&|k| matches!(k, ConstraintKind::InitialTemperature { .. }));
    let force = cons(&|k| matches!(k, ConstraintKind::Force { .. }));
    let pressure = cons(&|k| matches!(k, ConstraintKind::Pressure { .. }));
    let selfweight = cons(&|k| matches!(k, ConstraintKind::SelfWeight { .. }));
    let heatflux = cons(&|k| matches!(k, ConstraintKind::HeatFlux(_)));
    let nodes_of = |i: usize| -> Vec<u32> {
        match &r.data[i] {
            ConstraintData::Nodes(s) => s.iter().copied().collect(),
            _ => Vec::new(),
        }
    };
    let write_nset = |f: &mut String, i: usize| {
        let _ = writeln!(f, "*NSET,NSET={}", a.constraints[i].name);
        for n in nodes_of(i) {
            let _ = writeln!(f, "{n},");
        }
    };

    // ---- node sets
    if !fixed.is_empty() {
        section(&mut f, "constraints fixed node sets");
        for &i in &fixed {
            let _ = writeln!(f, "** {}", a.constraints[i].name);
            write_nset(&mut f, i);
        }
    }
    if !disp.is_empty() {
        section(&mut f, "constraints displacement node sets");
        for &i in &disp {
            let _ = writeln!(f, "** {}", a.constraints[i].name);
            write_nset(&mut f, i);
        }
    }
    if t == Ty::Thermomech && !temp.is_empty() {
        section(&mut f, "constraints temperature node sets");
        for &i in &temp {
            let _ = writeln!(f, "** {}", a.constraints[i].name);
            write_nset(&mut f, i);
        }
    }
    if matches!(t, Ty::Thermomech | Ty::Static) && !itemp.is_empty() {
        section(&mut f, "constraints initial temperature node sets");
        for &i in &itemp {
            let _ = writeln!(f, "** {}", a.constraints[i].name);
        }
    }

    // ---- materials
    let steady = matches!(a.analysis_type, AnalysisType::Thermomech { steady_state: true });
    let density_needed = t == Ty::Frequency || (t == Ty::Thermomech && !steady) || !selfweight.is_empty();
    f.push_str("\n** Physical constants for SI(mm) unit system with Kelvins\n");
    f.push_str("*PHYSICAL CONSTANTS, ABSOLUTE ZERO=0, STEFAN BOLTZMANN=5.670374419e-11\n");
    section(&mut f, "Materials");
    f.push_str("** see information about units at file end\n");
    for m in &a.materials {
        let mat = &m.material;
        let _ = writeln!(f, "** FreeCAD material name: {}", mat.name);
        let _ = writeln!(f, "** {}", m.name);
        let _ = writeln!(f, "*MATERIAL, NAME={}", m.name);
        let _ = writeln!(f, "*ELASTIC\n{},{}", g13(mat.youngs_modulus), g13(mat.poisson_ratio));
        if density_needed {
            let _ = writeln!(f, "*DENSITY\n{}", g13(mat.density * 1e-12));
        }
        let rt = mat.expansion_reference_temperature.unwrap_or(0.0);
        if t == Ty::Thermomech {
            let _ = writeln!(f, "*CONDUCTIVITY\n{}", g13(mat.thermal_conductivity));
            let _ = writeln!(f, "*EXPANSION, ZERO={}\n{}", g13(rt), g13(mat.thermal_expansion));
            let _ = writeln!(f, "*SPECIFIC HEAT\n{}", g13(mat.specific_heat * 1e6));
        }
        if t == Ty::Static {
            let _ = writeln!(f, "*EXPANSION, ZERO={}\n{}", g13(rt), g13(mat.thermal_expansion));
        }
    }
    if matches!(t, Ty::Thermomech | Ty::Static) && !itemp.is_empty() {
        section(&mut f, "Initial temperature constraint");
        f.push_str("*INITIAL CONDITIONS,TYPE=TEMPERATURE\n");
        for &i in &itemp {
            let _ = writeln!(f, "** {}", a.constraints[i].name);
            if let ConstraintKind::InitialTemperature { temperature } = a.constraints[i].kind {
                let _ = writeln!(f, "Nall,{}", py(temperature));
            }
        }
    }
    section(&mut f, "Sections");
    for (i, m) in a.materials.iter().enumerate() {
        let _ = writeln!(f, "*SOLID SECTION, ELSET={}, MATERIAL={}", elset_name(i), m.name);
    }

    // ---- step
    section(&mut f, "At least one step is needed to run an CalculiX analysis of FreeCAD");
    let s = &a.solver;
    let mut step = String::from("*STEP");
    if t != Ty::Frequency {
        let _ = write!(step, ", INC={}", s.increments_maximum);
    }
    let _ = writeln!(f, "{step}");
    let mut proc = match &a.analysis_type {
        AnalysisType::Static => "*STATIC".to_string(),
        AnalysisType::Frequency { .. } => "*FREQUENCY".to_string(),
        AnalysisType::Thermomech { steady_state } => {
            format!("*COUPLED TEMPERATURE-DISPLACEMENT{}", if *steady_state { ", STEADY STATE" } else { "" })
        }
    };
    if let Some(ms) = &s.matrix_solver {
        let _ = write!(proc, ", SOLVER={ms}");
    }
    let param = match &a.analysis_type {
        AnalysisType::Frequency { modes, low_hz, high_hz } => {
            if *low_hz == 0.0 && *high_hz == 0.0 {
                format!("{modes}\n")
            } else {
                format!("{modes},{},{}\n", py(*low_hz), py(*high_hz))
            }
        }
        _ => format!(
            "{},{},{},{}",
            py(s.time_initial_increment),
            py(s.time_period),
            py(s.time_minimum_increment),
            py(s.time_maximum_increment)
        ),
    };
    let _ = writeln!(f, "{proc}\n{param}");

    // ---- step constraints
    if !fixed.is_empty() {
        section(&mut f, "Fixed Constraints");
        for &i in &fixed {
            let n = &a.constraints[i].name;
            let _ = writeln!(f, "** {n}\n*BOUNDARY\n{n},1\n{n},2\n{n},3\n");
        }
    }
    if !disp.is_empty() {
        section(&mut f, "Displacement constraint applied");
        for &i in &disp {
            let c = &a.constraints[i];
            let _ = writeln!(f, "** {}\n*BOUNDARY", c.name);
            if let ConstraintKind::Displacement { x, y, z } = c.kind {
                for (k, v) in [(1, x), (2, y), (3, z)] {
                    if let Some(v) = v {
                        let _ = writeln!(f, "{},{k},{k},{}", c.name, py(v));
                    }
                }
            }
        }
        f.push('\n');
    }
    if t != Ty::Frequency && !selfweight.is_empty() {
        section(&mut f, "Self weight Constraint");
        for &i in &selfweight {
            let c = &a.constraints[i];
            if let ConstraintKind::SelfWeight { gravity, direction } = c.kind {
                let l = crate::mesh::norm(direction).max(1e-300);
                let _ = writeln!(
                    f,
                    "** {}\n*DLOAD\nEall,GRAV,{},{},{},{}\n",
                    c.name,
                    g13(gravity),
                    g13(direction[0] / l),
                    g13(direction[1] / l),
                    g13(direction[2] / l)
                );
            }
        }
    }
    if t != Ty::Frequency && !force.is_empty() {
        section(&mut f, "constraints force node loads");
        for &i in &force {
            let c = &a.constraints[i];
            let _ = writeln!(f, "** {}\n*CLOAD", c.name);
            if let ConstraintData::NodeLoads { tables, direction, vectors } = &r.data[i] {
                for (ti, (label, table)) in tables.iter().enumerate() {
                    let kind = if label.starts_with("Vertex") { "node load" } else { "node loads" };
                    let _ = writeln!(f, "** {kind} on shape: {}:{label}", a.shape_name);
                    for (node, load) in table {
                        let v = match direction {
                            Some(d) => d.map(|c| c * load),
                            None => vectors[ti].get(node).copied().unwrap_or([0.0; 3]),
                        };
                        let d = direction.unwrap_or(v);
                        for k in 0..3 {
                            if d[k].abs() > 1e-15 {
                                let _ = writeln!(f, "{node},{},{}", k + 1, g13(v[k]));
                            }
                        }
                    }
                    f.push('\n');
                }
            }
            f.push('\n');
        }
    }
    if t != Ty::Frequency && !pressure.is_empty() {
        section(&mut f, "constraints pressure element face loads");
        for &i in &pressure {
            let c = &a.constraints[i];
            let _ = writeln!(f, "** {}\n*DLOAD", c.name);
            if let (ConstraintKind::Pressure { pressure, reversed }, ConstraintData::ElementFaces(faces)) = (&c.kind, &r.data[i]) {
                let p = if *reversed { -pressure } else { *pressure };
                for (label, list) in faces {
                    let _ = writeln!(f, "** {}.{label}", a.shape_name);
                    for (e, fno) in list {
                        let _ = writeln!(f, "{e},P{fno},{}", py(p));
                    }
                }
            }
        }
    }
    if t == Ty::Thermomech && !temp.is_empty() {
        section(&mut f, "Fixed temperature constraint applied");
        for &i in &temp {
            let c = &a.constraints[i];
            let _ = writeln!(f, "** {}", c.name);
            match c.kind {
                ConstraintKind::Temperature { temperature } => {
                    let _ = writeln!(f, "*BOUNDARY\n{},11,11,{}\n", c.name, py(temperature));
                }
                ConstraintKind::HeatSource { power } => {
                    let n = nodes_of(i).len().max(1) as f64;
                    let _ = writeln!(f, "*CFLUX\n{},11,{}\n", c.name, py(power * 1000.0 / n));
                }
                _ => {}
            }
        }
    }
    if t == Ty::Thermomech && !heatflux.is_empty() {
        section(&mut f, "constraints heatflux element face heatflux");
        for &i in &heatflux {
            let c = &a.constraints[i];
            let _ = writeln!(f, "** {}", c.name);
            if let (ConstraintKind::HeatFlux(kind), ConstraintData::ElementFaces(faces)) = (&c.kind, &r.data[i]) {
                let (kw, ft, vals) = match kind {
                    HeatFluxKind::Flux(q) => ("DFLUX", "S", g13(q * 1e-3)),
                    HeatFluxKind::Convection { film_coefficient, ambient_temperature } => {
                        ("FILM", "F", format!("{},{}", g13(*ambient_temperature), g13(film_coefficient * 1e-3)))
                    }
                };
                let _ = writeln!(f, "*{kw}");
                for (label, list) in faces {
                    let _ = writeln!(f, "** {}.{label}", a.shape_name);
                    for (e, fno) in list {
                        let _ = writeln!(f, "{e},{ft}{fno},{vals}");
                    }
                }
            }
        }
    }

    // ---- output
    section(&mut f, "Outputs --> frd file");
    f.push_str("*NODE FILE\n");
    f.push_str(if t == Ty::Thermomech { "U, NT\n" } else { "U\n" });
    f.push_str("*EL FILE\n");
    f.push_str(if t == Ty::Thermomech { "S, E, HFL\n" } else { "S, E\n" });
    if !fixed.is_empty() || !disp.is_empty() {
        f.push_str("** outputs --> dat file\n");
    }
    if !fixed.is_empty() {
        f.push_str("** reaction forces for Constraint fixed\n");
        for &i in &fixed {
            let _ = writeln!(f, "*NODE PRINT, NSET={}, TOTALS=ONLY\nRF", a.constraints[i].name);
        }
    }
    if !disp.is_empty() {
        f.push_str("** reaction forces for Constraint displacement constraining translation\n");
        for &i in &disp {
            if let ConstraintKind::Displacement { x, y, z } = a.constraints[i].kind {
                if x.is_some() || y.is_some() || z.is_some() {
                    let _ = writeln!(f, "*NODE PRINT, NSET={}, TOTALS=ONLY\nRF", a.constraints[i].name);
                }
            }
        }
    }
    if !fixed.is_empty() || !disp.is_empty() {
        f.push('\n');
    }
    f.push_str("*OUTPUT, FREQUENCY=1");
    let _ = write!(f, "\n{STARS}\n*END STEP\n");

    // ---- footer
    section(&mut f, "CalculiX Input file");
    let _ = writeln!(f, "**   written by    --> {}", opts.written_by);
    let _ = writeln!(f, "**   written on    --> {}", opts.written_on);
    let _ = writeln!(f, "**   file name     --> {}", opts.file_name);
    let _ = writeln!(f, "**   analysis name --> {}", a.name);
    f.push_str("**\n**\n");
    f.push_str(UNITS_INFORMATION);
    f.push_str("**\n");
    f
}
