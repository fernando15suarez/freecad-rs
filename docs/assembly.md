# Assemblies (`crates/fc-assembly`)

Reference behaviour: FreeCAD 1.1's Assembly workbench (Ondsel solver) and Onshape's mates
on mate connectors.

## Data model

- `Workspace` (the `.fcrs` v2 project file): `parts: Vec<PartEntry { id, name, doc: fc_part::Document }>`,
  `assemblies: Vec<Assembly>`. `Workspace::from_json` also loads an old single-body document (v1),
  which becomes a one-part workspace.
- `PartEntry` also carries optional `meta: PartMeta { part_number, description, material, density (kg/m³),
  mass (kg), color }` for the BOM and STEP colours, and `brep` for parts imported from STEP (no features;
  `PartEntry::shape()` returns the body's result or the imported solid).
- `Assembly`: `instances` (`Instance { id, name, of: InstanceOf::Part | Assembly, placement: Frame, grounded, visible }`),
  `connectors` (mate connectors created in the assembly, anchored to an instance), `joints`, optional
  `exploded_views`.
- `Frame { pos, rot: [x, y, z, w] }`: FreeCAD's `Base::Placement`, with the quaternion in the same order as
  `Rotation.Q`.
- Joint sides (`JointRef`):
  - `PartConnector { instance, connector }`: a `FeatureKind::MateConnector` of the part, resolved with
    `Document::connector_frame`.
  - `AssemblyConnector(id)`.
  - `Geometry { instance, element, implicit }`: a geometry pick. `implicit` is `Center`, `Vertex(name)` or
    `EdgeCenter(name)` for FreeCAD-style picks; these use our port of `UtilsAssembly.findPlacement`.
    `Infer(fc_part::Infer)` gives Onshape-style inference through `fc_part::mate::infer_frame`, the same
    code mate connectors use.
  - `Frame { instance, local }` (FreeCAD's detached JCS) and `Origin { instance }`.
- Geometry comes from a `PartResolver`. `Workspace` implements it over its recomputed part documents, and
  `MapResolver` / `NoGeometry` exist for tests and caches.

## Joints

The equations are listed in the module docs of `joint.rs`. They are Ondsel's equations for the joint
FreeCAD maps each type to (`AssemblyObject::makeMbdJointOfType` / `makeMbdJointDistance`):

| Joint | removes | | Joint | removes |
|---|---|---|---|---|
| Fastened (Fixed) | 6 | | Parallel | 2 |
| Revolute | 5 | | Perpendicular, Angle | 1 |
| Slider | 5 | | Distance | 1–3 by geometry |
| Cylindrical, PinSlot | 4 | | RackPinion, Screw | 1 |
| Planar, Ball | 3 | | Gears, Belt | 1 |

- **Offsets.** `offset_a` / `offset_b` correspond to FreeCAD's `Offset1` / `Offset2`.
  `Joint::with_offset(z)` and `Joint::with_rotation(deg)` do what the task panel's Offset and Rotation
  fields do.
- **Limits.** `length_min/max` bounds `rIJ·zI`, and `angle_min/max` bounds the rotation about zI. They are
  enforced as an active set.
- **Sense.** `Auto` uses the root nearest to the current placements, which is Ondsel's behaviour.
  `Aligned` and `Flipped` give Onshape-like explicit flips, and FreeCAD's "Reverse" is `Flipped`.
- **Coupling joints.** Gears, Belt, Screw and RackPinion take their constant from the placements at solve
  start, as Ondsel does (`aConstant`). A static solve therefore never moves a part because of them, and
  drags keep the coupling.
- **Pre-solve.** `Assembly::presolve_joint` is FreeCAD's "match JCS". When one side is not connected to
  ground, that side moves together with its downstream parts.

## Solver

`Assembly::solve` and `Assembly::drag` use a port of the OndselSolver position solve that FreeCAD 1.1 runs
(`SolveMethod::Ondsel`, the default; `crates/fc-assembly/src/ondsel.rs`). Our original Levenberg–Marquardt
solver is still available as `SolveMethod::LevenbergMarquardt`, and it is the fallback when the Newton
iteration does not converge.

### The Ondsel solve

FreeCAD's call chain is `AssemblyObject::solve` → `ASMTAssembly::runPreDrag` → `System::runPreDrag` →
`SystemSolver::runPreDrag` → `runPosIC` → `PosICNewtonRaphson::run`. What it computes, and where the port
mirrors it:

| Ondsel / FreeCAD | what it does | our port |
|---|---|---|
| `AssemblyObject::makeMbdPart` | every part gets mass 1 and inertias (1, 1, 1); its frame is the part placement | same weights for every instance |
| `PartFrame::qX`, `qE`; `EulerParameters::calcABC` | unknowns per part: position and four Euler parameters (vector part first), rotation matrix `A(E) = (e3² − e·e) I + 2 e eᵀ + 2 e3 [e]×`, not normalised while iterating | `ondsel::rot_apply`, 7 coordinates per instance (`dual::NL` = 14 slots per joint) |
| `EulerConstraint` | `E·E − 1 = 0` per part, second derivative 2I | one Euler row per instance |
| `AssemblyObject::fixGroundedPart`, `ASMTFixedJoint` | grounded parts are pinned to the assembly by a fixed joint | grounded instances are constants |
| `Part::fillqsuWeights` | `wX = 1e6·m/m_max + 1e3`, `wE[0..3] = 1e6·J/J_max + 1e3`, `wE[3] = 1e3` | `OndselOptions::inertia_ratio` (see below) |
| `AnyPosICNewtonRaphson::fillY`, `fillPyPx` | Newton on the stationarity of `½ Σ (q − q_old)ᵀ W (q − q_old)` subject to all constraints: matrix `[−W + Σλ∂²G, Jᵀ; J, 0]` | same system (sign flipped). `λ∂²G` uses central differences of the exact joint gradients |
| constraints' `fillPosICJacob` | closed-form `λ`-weighted second derivatives (`ppGpEpE`, ...) | same terms, numerically |
| `GESpMatParPvMarkoFast` / `GESpMatParPvPrecise` | sparse Gaussian elimination | envelope LDLᵀ in body order (breadth first from the ground) |
| `PosICNewtonRaphson::run`, `handleSingularMatrix`, `GESpMatFullPvPosIC` | equations found dependent are removed (`removeRedundantConstraints`) and the solve restarts | zero pivots of equations are skipped (multiplier step 0) |
| `NewtonRaphson::isConvergedToNumericalLimit`, `iterMax` = 100 | iterate to the numerical limit, no line search | the same stopping idea, 100 iterations |
| `ASMTAssembly::createMbD` | parts and joints sorted by name | (the stationary point does not depend on the order) |

Our joint equations (`joint.rs`) are not Ondsel's in form, but they define the same constraint manifold.
The stationary point of the weighted problem depends only on that manifold, so the result is the same.

**Weights of FreeCAD 1.1.** FreeCAD 1.1.3 ships OndselSolver 1.0.1. Its results are reproduced exactly with
`J/J_max = 1/3`, i.e. rotation weights `1e6/3 + 1e3`. The current OndselSolver sources compute `J/J_max = 1`.

- `OndselOptions::FREECAD_1_1` (`inertia_ratio` = 1/3) is the default.
- `OndselOptions::ONDSEL_MASTER` (`inertia_ratio` = 1) reproduces the current OndselSolver sources.

We found the 1/3 by building OndselSolver from FreeCAD's sources and running it on FreeCAD's own model of
each case (`tools/parity/assembly/ondsel/`, below). With the sources as they are, it agrees with FreeCAD 1.1.3
in the determined cases but not in the free DOF (the hinge by 6 µm, the 4-bar by 2e-5 mm). Black-box probes
of FreeCAD's solve near the hinge solution gave the ratio of rotation to translation weight as about a third
of the sources' value. With `aJi/aJiMax = 1/3` in `Part::fillqsuWeights`, the source build matches FreeCAD
1.1.3 in all 8 cases to 1e-15.

### Islands, limits, diagnosis

- **Islands.** Islands of free instances connected by joints are solved separately. This gives the same
  result as one system, because the objective and the constraints separate. Unlike FreeCAD, an island
  with no grounded part is still solved (FreeCAD drops joints that are not connected to ground).
- **Limits.** FreeCAD's static solve ignores limits; only its drag enforces them
  (`PosICDragLimitNewtonRaphson`). We enforce them in both, the way Ondsel's drag does: violated limits are
  activated as equations and the solve is repeated from where the parts are.
- **Diagnosis.** The LM Jacobian in 6-DOF tangent coordinates (a scaled rotation vector and a translation per
  instance) is factored by a pivoted Cholesky of JJᵀ at the end. That gives the rank, so DOF = 6n − rank.
  Each dependent equation is reported as *redundant* if the system converged, or *conflicting* (together
  with its dependency group) if it did not. A failed solve leaves the parts where they were, like FreeCAD.

### Drag

FreeCAD's interactive drag (`AssemblyObject::preDrag` / `doDragStep` / `postDrag`) runs in three stages:

1. The GUI's drag mode computes the dragged part's new placement.
2. `ASMTAssembly::runDragStep` seeds that placement into the solver and runs `PosICDragNewtonRaphson`, the
   same weighted Newton iteration, followed by `PosICDragLimitNewtonRaphson`. On failure it retries at
   1/2, 1/4, 1/8 and 1/16 of the way (position lerp, Euler-parameter slerp), then restores the placements.
3. `runPostDrag` runs the solve again.

`Assembly::drag(instance, DragTarget::Frame | Point { local, world })` follows the same steps:

1. **New placement of the dragged part.** We find the placement that brings the dragged part (or its
   grabbed point) as close to the target as the joints allow. This is a constrained least-squares problem:
   - a Schur complement is applied on the linearised joints;
   - H⁻¹ is a 6×6 block plus a scaled identity.
2. **Drag step.** The dragged part is seeded at that placement and held there; the other parts move by the
   Ondsel minimum-movement solve. If the hold cannot be met exactly, the part is held by weights ×1e9
   instead. The step is retried at 1/2 … 1/16 of the way, and the placements are restored if every try
   fails.
3. **Post-drag solve.** A solve from the result, which also produces the report.

FreeCAD's own drag step writes per-part drag weights (1e6 for the dragged part's position, 1e3 elsewhere),
but `Part::fillqsuWeights` then overwrites them with the normal weights. As a result, its single drag step
leaves the dragged part between the cursor and the other parts. We hold the dragged part, which is the
limit of repeated drag steps toward a fixed cursor.

### Speed

Release build, with 200 parts (a 100-link revolute chain plus 100 fastened parts, one island of 700
unknowns + 600 equations):

| | Ondsel port | before (LM) |
|---|---|---|
| solve | about 30 ms | about 80 ms |
| 20 mm drag of the chain's tip | about 110 ms | about 170 ms |

The full Newton converges in about 11 iterations on the chain.

## FreeCAD parity

`tools/parity/assembly/make_assemblies.py` (run with `freecadcmd`) builds 8 reference assemblies:

1. It creates the joints the FreeCAD way (`setJointConnectors`).
2. It puts the moving parts at a chosen start pose and runs one `assembly.solve()`.
3. It saves `<case>.FCStd` plus `assemblies.json` (JCS, element geometry, start and solved placements).

`tests/parity.rs` checks these things:

- **JCS inference.** For every joint side, `infer_frame(element, vertex)` ∘ offset equals FreeCAD's
  `Placement1/2` to 1e-9.
- **Solve.** We solve the same joints on the recorded JCS from the same start. The default solver must
  land on FreeCAD's placements to 1e-9 (mm and degrees) in every case, free DOF included. The old LM solver
  is run as well, for comparison.
- **OndselSolver itself.** `tools/parity/assembly/ondsel/` produces reference runs of OndselSolver:
  1. `export_start.py` (freecadcmd) exports FreeCAD's Ondsel model of every case at its start pose
     (`exportAsASMT`). FreeCAD 1.1.3 writes truncated joint type names, which the script repairs.
  2. `run.sh` builds OndselSolver from FreeCAD's sources with a small driver (`drv.cpp`) that runs
     `runPreDrag`.
  3. It writes `ondsel_master.json` (the sources as they are) and `ondsel_r3.json`
     (`aJi/aJiMax = 1/3`).

  `ondsel_solver_parity` checks two things: `ondsel_r3.json` equals FreeCAD's results, and our port with
  `OndselOptions::ONDSEL_MASTER` equals `ondsel_master.json` to 1e-9.

| case | DOF | Ondsel port: max Δpos (mm) | max Δrot (°) | before (LM): max Δpos (mm) | max Δrot (°) |
|---|---|---|---|---|---|
| hinge (revolute on an edge) | 1 | 3e-14 | 4e-13 | 0.17 | 4.9 |
| slider (face + vertex) | 1 | 0 | 6e-30 | 3e-11 | 5e-10 |
| fourbar (4 revolutes, 3 redundant eq.) | 1 | 3e-35 | 1e-14 | 0.16 | 0.45 |
| ball | 3 | 0 | 3e-15 | 4e-12 | 3e-15 |
| fixed_stack (offset + rotation) | 0 | 4e-15 | 6e-15 | 2e-13 | 4e-13 |
| angle (revolute + Angle 30°) | 0 | 1e-27 | 1e-14 | 5e-12 | 1e-10 |
| distance (plane-plane 5 + point-point 15) | 2 | 2e-15 | 4e-22 | 2e-14 | 4e-3 |
| cylindrical (cylinder faces) | 2 | 4e-22 | 2e-15 | 3e-14 | 9e-4 |

Before the port, the hinge and the 4-bar stopped at a different angle of the free rotation:
FreeCAD turned the hinge plate from 40° to 44.9°, while LM's minimum-norm steps in length-scaled coordinates
kept it nearer 40°. The DOF counts agree with FreeCAD's joint semantics; the 4-bar is 18 − rank 17 = 1.

`freecad_fcstd_import` imports every `.FCStd`, then checks that solving moves nothing and gives the same
DOF (`fc_assembly::fcstd`).

## Known differences and gaps

- **Orientation roots.** Ondsel's direction-cosine equations also accept flipped poses. We use rotation
  vectors with an explicit `Sense` (Auto reproduces the nearest root). Results are the same unless a part
  starts within about 90° of a flip.
- **Planar faces (`Workspace` resolver).** The JCS Z is the outward normal from fc-occt, whereas FreeCAD
  uses the surface axis. They differ only on reversed faces.
- **Circular edges.** fc-occt's edge info has no X axis, so OCCT's automatic X is used.
- **Gears.** Ondsel's carrier-marker variant for coaxial gears is not implemented, and Screw/RackPinion
  angles are unwrapped from the solve-start value.
- **Distance joint variants.** Torus variants are mapped onto the plane, line and point cases. A Distance
  of 0 for point-line becomes an in-line constraint, where Ondsel uses a singular √.
- **FCStd import.** Members that are PartDesign Bodies (directly, via App::Link or inside an App::Part)
  import their bodies; other members (Part::Box, ...) are placeholders without geometry. The Distance
  variant is guessed from the element names.
- **Not done yet.** Flexible sub-assemblies, simulations and motions.

## Sub-assemblies

An instance can place another assembly (`InstanceOf::Assembly`, added with
`Workspace::add_sub_assembly(parent, name, sub, placement)`, which refuses cycles). It is posed **rigidly**:
the solver treats it as one body, with its own instances at their stored placements. FreeCAD 1.1 also
has flexible sub-assemblies; we do not have them yet.

- **Geometry references** go through the sub-assembly by instance names, FreeCAD's sub-name style:
  `JointRef::geometry(sub_instance, "Bolt.Face6")`, nested as `"Inner.Bolt.Face6"`. The second pick of a
  two-element selection may carry the same prefix. Part mate connectors inside a sub-assembly cannot be
  referenced (use an assembly connector or a geometry path).
- `Workspace::leaves(asm)` flattens the tree to `LeafInstance { path, name: "Sub.Bolt", part, placement, visible }`
  with placements in the top assembly's coordinates.
- `leaf_shape` and `instance_shape(asm, path)` give posed solids. A sub-assembly instance gives the
  compound of its parts.
- BOM, STEP, STL and interference all recurse through sub-assemblies.

## STEP and STL export/import

`Workspace::export_step(asm, path, &StepExportOptions { schema: Ap214 | Ap242, colors, visible_only })`
builds an XDE document (`XCAFApp_Application`, "MDTV-XCAF") through fc-occt's `XdeDoc`:

- each part is written once as a shape label (`XCAFDoc_ShapeTool::AddShape`);
- each (sub-)assembly becomes an assembly label whose components (`AddComponent` with a `TopLoc_Location`)
  place its instances;
- product names are the part and assembly names, and the NAUO names are the instance names;
- the part colour is written when `meta.color` is set.

The file is written with `STEPCAFControl_Writer`.

`Workspace::import_step(path)` reads with `STEPCAFControl_Reader`:

- every part label becomes an imported part (its solid is stored as BREP text in the project);
- every assembly label becomes an `Assembly`, with one instance per component;
- a nested assembly becomes an `InstanceOf::Assembly` instance;
- a file without an assembly, or with several roots, is wrapped in an assembly named after the file.

Nothing is grounded and there are no joints.

`Workspace::export_stl(asm, path, deflection, visible_only)` writes the posed, flattened assembly as one
binary STL.

**FreeCAD parity** (`tools/parity/assembly/step_assembly.py`):

- **FreeCAD reads our files.** Our AP214 and AP242 exports of the test assembly import in FreeCAD 1.1.3
  (`Import.insert`) as `App::Part "Main"` containing the plate, two `App::Part "BoltPair"` (one turned
  90°) and a loose bolt. That is 6 solids, 27500 mm³ in total, with every bounding box equal to ours.
- **We read FreeCAD's files.** The script also writes `freecad_asm.step` (App::Part with App::Links to a
  shared sub-part). `tests/tools.rs::freecad_step_import` reads it back with the right tree
  (Top → Plate, Pair1, Pair2 → PinA, PinB), one shared Pin part, and the right volumes and boxes.

## Interference and clearance

`Workspace::interferences(asm, &InterferenceOptions { min_volume, tolerance, visible_only })` checks every
pair of posed part instances, with sub-assemblies flattened:

1. A bounding-box prefilter skips the pair unless the boxes overlap by more than `tolerance` on every
   axis.
2. A boolean common runs on the pairs that pass.
3. A pair is reported when the common's volume is above `min_volume` (1e-6 mm³ by default), so solids
   that only touch at a face are ignored.

Each `Interference` has the two leaves, the overlap volume and the common solid (in assembly coordinates)
for highlighting.

`Workspace::clearance(asm, path_a, path_b)` is the minimum distance (`BRepExtrema_DistShapeShape`) between
two instances or sub-assemblies, with the closest points. It is 0 when they touch or overlap. FreeCAD has
no built-in equivalent; the behaviour follows Onshape and SolidWorks.

## Bill of materials

`Workspace::bom(asm, &BomOptions { detail_sub_assemblies, flatten, visible_only })` follows FreeCAD's
`Assembly::BomObject`:

- Siblings that place the same part or sub-assembly are grouped into one row with a quantity.
- Rows are numbered `1`, `2`, `2.1`, ...
- With `detail_sub_assemblies` (the default), a sub-assembly's contents are listed indented under it, with
  quantities per sub-assembly.
- `flatten` gives one row per distinct part with its total count across the whole tree.

Each row has `name`, `part_number`, `description`, `material`, `quantity` and `unit_mass`. The unit mass is
`meta.mass`, or volume × `meta.density`; for a sub-assembly it is the sum over its parts.

`bom_csv(&rows)` writes the CSV with these columns: Index, Name, Part Number, Description, Material,
Quantity, Unit Mass (kg) and Total Mass (kg). Fields are quoted per RFC 4180. FreeCAD's spreadsheet export
has only Index, Name, Description, File Name and Quantity.

## Exploded views

`Assembly::exploded_views` holds `ExplodedView { name, steps }`. Each `ExplodeStep { instances, kind, transform }`
mirrors FreeCAD's `ExplodedViewStep`:

- **`Normal`** sets `placement = transform * placement`, where `transform` is FreeCAD's
  `MovementTransform` in world coordinates.
  - `ExplodeStep::translate(ids, dir, distance)` builds a move.
  - `ExplodeStep::rotate(ids, center, axis, degrees)` builds a rotation as
    `Placement(center, rot) * Placement(center)⁻¹`, like `setMovementAngle`.
- **`Radial`**, built with `ExplodeStep::radial(ids, distance)`, moves each instance's bounding-box
  centre away from the assembly's centre: `base += (centre − assembly centre) · 4·distance / diagonal`.

Steps apply in order, each starting from the result of the previous ones.

`Workspace::exploded(asm, view, t)` evaluates a view at animation time `t ∈ [0, 1]` using the real bounding
boxes:

- The steps play one after another, and a partial step follows the transform's screw motion
  (`Frame::pow`), so a rotation keeps its pivot.
- It returns `Exploded { placements, lines }`; the lines are trails of the moved centres.
- It never changes the solved placements.

`Assembly::exploded_placements(view, t)` is the geometry-free variant: instance origins stand in for the
centres, and the radial size falls back to 100 (FreeCAD's headless fallback).
