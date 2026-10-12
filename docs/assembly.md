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

- **Unknowns.** Each free instance has 6 unknowns: a scaled incremental rotation vector (about the
  instance origin) and a translation. The system is re-linearised at zero every iteration, and the
  Jacobians are exact (forward AD over the 12 unknowns of a joint).
- **Iteration.** Islands of free instances connected by joints are solved separately with
  Levenberg–Marquardt minimum-norm steps. This gives minimal movement: underconstrained parts stay as close
  as possible to where they are.
- **Diagnosis.** A pivoted Cholesky of JJᵀ at the end gives the rank, so DOF = 6n − rank. Each dependent
  equation is reported as *redundant* if the system converged, or *conflicting* (together with its
  dependency group) if it did not. A failed solve leaves the parts where they were, like FreeCAD.
- **Drag.** `Assembly::drag(instance, DragTarget::Frame | Point { local, world })` minimises the distance to
  the target subject to the linearised joints. It uses a Schur complement on the constraints, where H⁻¹ is
  a 6×6 block plus a scaled identity. A final exact joint solve then polishes the result.
- **Speed (release build).** 200 parts (a 100-link revolute chain plus 100 fastened parts, 600 unknowns in
  one island) solve in about 80 ms. A 20 mm drag of the chain's tip takes about 170 ms.

## FreeCAD parity

`tools/parity/assembly/make_assemblies.py` (run with `freecadcmd`) builds 8 reference assemblies:

1. It creates the joints the FreeCAD way (`setJointConnectors`).
2. It puts the moving parts at a chosen start pose and runs one `assembly.solve()`.
3. It saves `<case>.FCStd` plus `assemblies.json` (JCS, element geometry, start and solved placements).

`tests/parity.rs` checks two things:

- **JCS inference.** For every joint side, `infer_frame(element, vertex)` ∘ offset equals FreeCAD's
  `Placement1/2` to 1e-9.
- **Solve.** We solve the same joints on the recorded JCS from the same start and compare with FreeCAD:

| case | DOF | max Δpos (mm) | max Δrot (°) | |
|---|---|---|---|---|
| hinge (revolute on an edge) | 1 | 0.17 | 4.9 | free angle differs |
| slider (face + vertex) | 1 | 3e-11 | 5e-10 | |
| fourbar (4 revolutes, 3 redundant eq.) | 1 | 0.16 | 0.45 | free angle differs |
| ball | 3 | 4e-12 | 3e-15 | |
| fixed_stack (offset + rotation) | 0 | 2e-13 | 4e-13 | |
| angle (revolute + Angle 30°) | 0 | 5e-12 | 1e-10 | |
| distance (plane-plane 5 + point-point 15) | 2 | 2e-14 | 4e-3 | |
| cylindrical (cylinder faces) | 2 | 3e-14 | 9e-4 | |

The determined cases match exactly, and so do the mechanisms whose free DOF both solvers resolve by pure
projection. For the hinge and the 4-bar, both results satisfy the joints but stop at a different value of
the free rotation:

- FreeCAD's Newton iteration in Ondsel's coordinates turned the hinge plate on from 40° to 44.9°.
- Our minimum-norm step, in length-scaled coordinates, keeps it nearer the start.

The DOF counts agree with FreeCAD's joint semantics; the 4-bar is 18 − rank 17 = 1.

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
