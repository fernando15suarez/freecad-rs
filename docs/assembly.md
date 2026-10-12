# Assemblies (`crates/fc-assembly`)

Reference behaviour: FreeCAD 1.1's Assembly workbench (Ondsel solver) and Onshape's mates
on mate connectors.

## Data model

- `Workspace` (the `.fcrs` v2 project file): `parts: Vec<PartEntry { id, name, doc: fc_part::Document }>`,
  `assemblies: Vec<Assembly>`. `Workspace::from_json` also loads an old single-body document (v1),
  which becomes a one-part workspace.
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
- **Not done yet.** Sub-assemblies as instances (geometry references into them), simulations and motions,
  BOM, and exploded-view animation.
