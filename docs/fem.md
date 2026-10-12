# FEM (crate `fc-fem`)

`fc-fem` is the backend of the FEM workbench. It does what FreeCAD's FEM workbench does with
**Gmsh** (mesher) and **CalculiX `ccx`** (solver), using the same external programs: it writes the
geometry as BREP, writes a Gmsh `.geo` script the way FreeCAD's `femmesh/gmshtools.py` writes it,
reads the mesh back (`.msh` 4.1), writes the CalculiX input the way
`femsolver/calculix/writer.py` writes it, runs `ccx`, and reads `.frd` / `.dat` results.

On the reference cases the `.geo` and `.inp` files are **line for line identical** to FreeCAD
1.1.3's, apart from the footer lines that carry the date and program name. Meshes have
identical node and element numbering, and results are identical to the last digit of the `.frd`
file.

## Tools

| tool | lookup order |
|------|--------------|
| Gmsh | `$FCRS_GMSH`, `gmsh` on `PATH`, `/Applications/FreeCAD.app/Contents/Resources/bin/gmsh` (and Linux/Windows FreeCAD install dirs) |
| ccx  | `$FCRS_CCX`, `ccx` on `PATH`, `/Applications/FreeCAD.app/Contents/Resources/bin/ccx` (same) |

If an environment variable points to a missing file, that is an error. The lookup does not fall
back to the other locations. When a tool is missing, `solve` fails before doing any work, with
`FemError::ToolMissing`, which names every location it tried. `fc_fem::tools::available()` lets
the UI grey out the solve command.

## Public API

```rust
use fc_fem::*;
let mut a = Analysis::new(ShapeSource::Shape(body_tip.clone()), Material::steel_generic());
a.mesh.max_size = 4.0;                          // CharacteristicLengthMax, mm (order 2 by default)
a.mesh.refinements.push(MeshRefinement { refs: vec![SubRef::face(7)], size: 1.5 }); // MeshRegion
a.fixed(vec![SubRef::face_of(&body_tip, 2).unwrap()]);          // Face2 + geometric signature
a.force(vec![SubRef::face(5)], 500.0, ForceDirection::Vector([0.0, 0.0, -1.0]));  // N
a.pressure(vec![SubRef::face(6)], 0.1, false);                  // MPa, into the face
a.self_weight([0.0, 0.0, -1.0]);                                // 9806.65 mm/s^2
let sol = solve(&a, &RunOptions::default())?;
let r = &sol.results;
r.max_displacement();            // Extreme { value, node, index, position }
r.max_von_mises();
r.scalar(Field::VonMises);       // per node, also displacement components, principal, temperature...
r.deformed(r.auto_scale(0.1));   // node positions scaled
r.surface_mesh(Field::VonMises, scale); // (positions, outer triangles, per-vertex scalar)
r.surface.tri_face;              // B-rep face index of every display triangle (picking)
r.reactions["CONSTRAINTFIXED"];  // reaction force totals from the .dat file
r.modes;                         // frequency analysis: Hz + mode shapes
```

* **Model** (`model.rs`, all serde): `Analysis { name, shape_name, shape: ShapeSource
  (Brep path | Step path | in-memory fc_occt::Shape), analysis_type (Static | Frequency{modes,
  low_hz, high_hz} | Thermomech{steady_state}), materials: Vec<MaterialAssignment>,
  mesh: MeshSettings, constraints: Vec<Constraint>, solver: SolverSettings }`.
  * Constraints: `Fixed`, `Force { force, direction: Vector | Normal{reversed} }`, `Pressure {
    pressure, reversed }`, `Displacement { x, y, z: Option<f64> }`, `SelfWeight`, `Temperature`,
    `HeatSource` (CFLUX), `HeatFlux(Flux | Convection)` and `InitialTemperature`. Names follow
    FreeCAD (`ConstraintFixed`, `ConstraintFixed001`, ...) and become the CalculiX set names.
  * References use FreeCAD's `Face3` / `Edge5` / `Vertex2` / `Solid1` (`SubRef::parse`) and can
    carry a `Signature` (centre, area or length, plane normal). If the indexed entity does not
    match the signature, `FemMesh::resolve` uses the best match. This works around FreeCAD's
    toponaming problem.
* **Materials** (`material.rs`): FreeCAD card fields (E in MPa, ν, density in kg/m³,
  conductivity, expansion, specific heat). Built-in cards are `Steel-Generic`, `S235JR`,
  `Aluminum 6061-T6`, `PLA`, `ABS`, `Copper Generic`, `Ti-6Al-4V` and `CalculiX-Steel`, looked up
  with `Material::by_name`. Several materials are supported when each is assigned to `Solid<n>`
  references.
* **Stages**, each callable separately:
  `prepare_geometry`, `mesh_analysis` (Gmsh), `write_input` (`resolve` + `write_inp`), `run_ccx`,
  and `read_results` (`parse_frd`, `parse_dat_totals`, `parse_dat_frequencies`). Use
  `RunOptions { work_dir, keep_files }` to keep `shape.brep`, `shape2mesh.geo`, `mesh.msh` and
  `Mesh.{inp,frd,dat}`.

## How meshing works

1. The solid is written to `shape.brep`. FreeCAD meshes a BREP too, and Gmsh's OCC kernel numbers
   faces, edges and vertices in `TopExp::MapShapes` order, which is the same order as FreeCAD's
   `FaceN`.
2. `shape2mesh.geo` is written with the same statements as FreeCAD:
   * CharacteristicLengthMax/Min (0 → 1e+22) and MeshSizeFromCurvature 12.
   * Optimize, ElementOrder and SecondOrderLinear.
   * Algorithm 2 for 2D and algorithm 1 for 3D.
   * `Coherence Mesh` and `Mesh.SaveAll = 1`.
   * A physical group for every solid, face and edge, as FreeCAD 1.1 does. The counts come from
     the BREP's TShapes. These groups also affect Gmsh's output numbering.
   * FreeCAD's `MeshRegion` becomes `Characteristic Length { PointsOf { Surface{n}; } } = size;`.
     FreeCAD lists the face's vertex numbers instead, which gives the same mesh.

   The differences from FreeCAD are:
   * We save `.msh` 4.1 instead of `.unv`.
   * We use `General.NumThreads = 1` because Gmsh's threaded meshing is not deterministic. FreeCAD
     uses all cores, so two FreeCAD runs of the same model give slightly different meshes.
3. `parse_msh` keeps the entity classification of every element. Surface triangles of face *n*
   give the node sets (Fixed, Displacement, Temperature) and the element faces (Pressure,
   HeatFlux), which are found through the tetrahedra's face table. FreeCAD finds them with a
   geometric search, and both give the same sets. Tet node order is converted to CalculiX order
   with the same corner rotation FreeCAD's UNV import produces, so element lines are identical.

## How solving works

`write_inp` reproduces FreeCAD's writer section by section:
* Section order: mesh, material element sets, node sets, physical constants, materials, initial
  temperature, sections, `*STEP` with the procedure line, step constraints, outputs, then the
  footer with the units notes.
* Number formats are reproduced exactly: `%.13g` for coordinates, Python `{:.13G}` for loads and
  material constants, and Python `repr` where FreeCAD writes a plain float.
* Forces use FreeCAD's `get_force_obj_*_nodeload_table`:
  * Face loads are force divided by the exact B-rep area, times the node area of each mesh face.
    Tri3 faces share the area 1/3 per corner. Tri6 faces put 0 on the corners and 1/3 on each
    midside node.
  * Edge loads are weighted by length: 1/2 per node for 2-node segments, and 1/6, 1/6, 2/3 for
    3-node segments.
  * Vertex loads are split equally.
  * A "normal" direction on planar faces gives one direction vector, like FreeCAD. Curved faces
    use one normal per element face.
* `ccx -i Mesh` runs in the working directory with `OMP_NUM_THREADS` set, as `ccxtools` does. A
  run counts as failed if the output contains `*ERROR`, the exit code is non-zero, or no `.frd`
  file was written.
* Results:
  * Displacement (`DISP`), stress (`STRESS`, nodal and extrapolated by ccx), total strain
    (`TOSTRAIN`), and for thermomechanical runs temperature (`NDTEMP`) and heat flux (`FLUX`).
  * We compute von Mises and the principal stresses (closed-form eigenvalues).
  * From the `.dat` file: reaction totals (`*NODE PRINT ... TOTALS=ONLY`) and eigenfrequencies.

## Parity with FreeCAD 1.1.3

`tools/parity/fem/freecad_reference.py` runs with `freecadcmd`. For each case it builds the
geometry with Part and meshes it with FreeCAD's own `GmshTools`. The only change is that the
`.geo` is pinned to one thread; FreeCAD's preferences are not touched. It then writes and runs
ccx with `ccxtools`.

The script saves the following for each case:
* `<case>.brep` and `<case>.geo`
* `<case>.json`, with node and element counts, max |u| and its node, max von Mises, principal
  extremes, reaction totals, temperatures and frequencies
* for the two snapshot cases, also FreeCAD's `Mesh.inp`, `.frd` and `.dat`

`cargo test -p fc-fem --test parity` re-solves every case on the same BREP with the same mesh
settings. The material is Steel-Generic (E = 200 GPa, ν = 0.3, ρ = 7900 kg/m³) with quadratic
tets (C3D10).

| case | setup | nodes ours / FC | max \|u\| mm ours / FC | max von Mises MPa ours / FC | other | theory |
|------|-------|-----------------|------------------------|-----------------------------|-------|--------|
| cantilever | 1000×100×100 box, Face1 fixed, 1000 N −z on Face2, h=25 | 6488 / 6488 | 0.200517 / 0.200517 | 5.9122 / 5.9122 | RF_z 1000.0 / 1000.0 N | tip u_z 0.19995 vs Euler–Bernoulli 0.20000 / Timoshenko 0.20156 (−0.8 %); σ_xx(x=500, top) 2.998 vs Mc/I 3.000 |
| selfweight | same beam, gravity −z | 6488 / 6488 | 0.058219 / 0.058219 | 2.2791 / 2.2791 | RF_z 771.993 / 771.993 N | ρgV = 774.725 N (−0.35 %: load on clamped nodes is not a reaction) |
| frequency | same beam, 6 modes | 6488 / 6488 | – | – | f1..f6 = 81.069, 81.073, 486.220, 486.243, 719.912, 1261.80 Hz (identical) | f1 Euler–Bernoulli 81.280 Hz (−0.26 %) |
| plate_hole | 200×100×10, Ø20 hole, Face1 fixed, 10 kN along face normal on Face6, h=5, hole region 1.5 | 38875 / 38875 | 0.010454 / 0.010454 | 32.008 / 32.008 | RF_x −10000 / −10000 N | max σ_xx at hole 32.88 vs Kt·σ_net = 2.512·12.5 = 31.40 (+4.7 %) |
| pressure | 200×50×10 plate, both ends fixed, 0.1 MPa on top, h=5 | 8801 / 8801 | 0.025149 / 0.025149 | 17.5536 / 17.5536 | RF_z 993.33 / 993.33 N | |
| thermal_bar | 100×10×10, Face1 fixed, T = 293 K / 393 K at the ends, steady thermomech, h=2.5 | 6610 / 6610 | 0.060411 / 0.060411 | 15.1697 / 15.1697 | T_max 393 / 393 K | end u_x 0.05998 vs α·ΔT_mean·L 0.06000 |
| bracket | fc-part L bracket (Pad of an L sketch + through-all Pocket), STEP → FreeCAD, back face fixed, 500 N −z on the arm end, h=4 | 5776 / 5776 | 0.120015 / 0.120015 | 90.336 / 90.336 | RF_z 500 / 500 N | |
| bracket (native) | the same bracket as an in-memory fc-part shape, faces picked by signature (deliberately wrong index) | 5736 / 5776 | 0.120155 / 0.120015 (+0.12 %) | 91.17 / 90.34 (+0.9 %) | | |
| cantilever (native) | fc-occt prism, faces by signature | 6597 / 6488 | 0.200521 / 0.200517 | | | |

The "native" rows mesh our own OCCT shape rather than FreeCAD's BREP. The vertex and face order
differs, so Gmsh produces a slightly different mesh. Displacements still agree within 0.2 %. The
peak von Mises stress sits at a re-entrant corner and is mesh sensitive, so it agrees within 1 %.

Snapshot tests on the stored small reference files:
* `snapshot_static` covers Fixed, Displacement, Force, Pressure and SelfWeight.
* `snapshot_thermo` covers Fixed, InitialTemperature, Temperature, DFLUX and FILM.
* The `.geo` and `.inp` we write equal FreeCAD's line for line (`tests/snapshot.rs`, which needs
  gmsh). The tool-free `stages::inp_writer_snapshot` test checks the same thing from the stored
  `snapshot_static.msh`.
* The `.frd` and `.dat` readers reproduce FreeCAD's result numbers (`stages::frd_and_dat_parsing`).

Regenerate the references with:

```
cargo test -p fc-fem --test bracket_step -- --ignored        # bracket.step for FreeCAD
FEM_PARITY_OUT=tools/parity/fem FEM_PARITY_WORK=/tmp/femref \
  /Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd tools/parity/fem/freecad_reference.py
cargo test -p fc-fem --test stages -- --ignored write_snapshot_msh
```

## Tests

`OCCT_DIR=… CARGO_TARGET_DIR=target-fem cargo test -p fc-fem` runs the following. Solver
tests print `SKIP` and pass when gmsh or ccx is missing.

* Unit tests: Python and C number formatting, the material library, `SubRef` parsing, and stress
  invariants (von Mises, principal stresses).
* `stages`:
  * `.msh` classification and C3D10 node order (midside positions, positive orientation)
  * boundary extraction (closed, outward; checked with the divergence theorem)
  * signature re-resolution
  * pressure element faces
  * the tool-free inp snapshot
  * `.frd` / `.dat` parsing for static and thermomechanical output (the modal `.frd` is covered by `parity`)
  * analysis JSON round trip
* `snapshot`: geo and inp parity with FreeCAD.
* `parity`: the end-to-end table above, plus the theory checks.
* `tools_env`: a clear error when the tools are missing.
* `bracket_step`: the fc-part bracket is a valid solid.

## Gaps and differences

* Only solid tetrahedral meshes are supported: no shells or beams, and no hexahedra or
  recombination. Boundary layers and the Netgen mesher are not supported either.
* Constraints that are not implemented: contact, tie, transform, plane rotation, rigid body,
  spring, centrifugal and body heat source. Amplitudes, buckling, electromagnetic and nonlinear
  material or geometry are not implemented either. Time-dependent thermomechanical runs write
  the same step as FreeCAD but are not parity-tested.
* The footer (`written by` / `written on`) names fc-fem and uses a unix timestamp instead of
  `time.ctime()`.
* FreeCAD's `CharacteristicLengthMap` lists vertex numbers; we write the equivalent
  `PointsOf { Surface{n}; }`. On plate_hole the mesh is identical.
* Edge forces use mesh edge lengths for the total. FreeCAD uses B-rep lengths, which is the same
  for straight edges. Face forces use exact B-rep areas when the shape is in memory or read from
  STEP, and mesh areas for a BREP file path; these match exactly for planar faces.
* FreeCAD's frequency result import crashes headless `freecadcmd` 1.1.3. For the modal case the
  reference therefore takes its frequencies from `Mesh.dat`, and we compare against those.
