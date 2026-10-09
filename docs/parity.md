# FreeCAD parity

fc-part is checked against FreeCAD 1.1.3 itself: reference models are built **and
measured** by FreeCAD, committed under `tools/parity/models/`, then imported with
`fc_part::fcstd`, recomputed by fc-part and compared.

```
tools/parity/run.sh                         # rebuild every model with FreeCAD (needs FreeCAD.app)
PARITY_ONLY=pad_rect,groove_ring tools/parity/run.sh
tools/parity/run.sh measure some.FCStd      # FreeCAD's measurements of any file, as JSON
cargo test -p fc-part --test parity         # the comparison (no FreeCAD needed)
cargo test -p fc-part --test parity -- --nocapture parity_table   # ours vs FreeCAD table
cargo run -p fc-part --example fcstd_info -- part.FCStd           # inspect any file
```

* `tools/parity/fcparity.py` – model-building helpers and `measure()` (volume, area,
  bounding box, solid/face/edge/vertex counts, per-feature volume and error, per-sketch
  DOF, conflicting / redundant / partially redundant / malformed constraints, 0-based,
  and the sketch placement).
* `tools/parity/make_models.py` – one function per model (`@model`).
* `models/<name>.FCStd` + `models/<name>.json` – FreeCAD's file and its numbers.
* `crates/fc-part/tests/parity.rs` – one test per model. Checks: no import note of kind
  *Skipped*; feature errors agree (FreeCAD refusing a sketch with redundant constraints is
  tolerated, see F5); every sketch's DOF (when FreeCAD's is >= 0) and its
  conflicting/redundant/partially-redundant/malformed lists, mapped back to FreeCAD's
  constraint numbering; solving the imported sketch does not move FreeCAD's solved
  geometry by more than 1e-7; body volume and area within 1e-6 relative; bbox within
  1e-6 mm; face, edge and solid counts exact. `perturbed_sketches_resolve_to_freecad_geometry`
  nudges every fully constrained sketch (45 of them) and requires our solver to land back on
  FreeCAD's solution within 1e-6. `all_models_listed` makes sure every model has a test.

## Results (FreeCAD 1.1.3 vs fc-part)

| model | covers | FreeCAD volume | fc-part | result |
|---|---|---:|---:|---|
| pad_rect | constrained rectangle, Pad | 1000.000000 | 1000.000000 | match |
| pad_two_lengths | SideType Two sides | 1600.000000 | 1600.000000 | match |
| pad_symmetric_xz | XZ plane, Symmetric | 576.000000 | 576.000000 | match |
| pad_reversed_yz | YZ plane, Reversed | 168.000000 | 168.000000 | match |
| pad_map_reversed | MapReversed + AttachmentOffset on XY and XZ | 126.000000 | 126.000000 | match |
| pad_up_to_first | UpToFirst, reversed, offset sketch | 6480.000000 | 6479.999998 | match (3e-10 rel) |
| pocket_two_lengths | Pocket two sides | 5858.628331 | 5858.628331 | match |
| pocket_through_all_face | sketch on `Pad.FaceN`, ThroughAll | 5497.345175 | 5497.345175 | match |
| pocket_length_offset_face | sketch on the -Y side face | 5840.000000 | 5840.000000 | match |
| face_frames | sketches on -Z, +Y, -X faces (frame rotation) | 5835.000000 | 5835.000000 | match |
| revolution_v_axis | V_Axis, 360 | 4712.388980 | 4712.388980 | match |
| revolution_construction_line | axis = construction line (`Axis0`), 270 | 209.926219 | 209.926219 | match |
| revolution_symmetric_h_axis | H_Axis, symmetric 120 | 418.879020 | 418.879020 | match |
| revolution_two_angles | TwoAngles 60 + 30 | 659.734457 | 659.734457 | match |
| groove_ring | Groove 360 | 5830.795965 | 5830.795965 | match |
| groove_symmetric_partial | Groove symmetric 90, reversed | 6082.123377 | 6082.123377 | match |
| hole_counterbore | Hole, counterbore, angled drill point | 5613.088531 | 5613.088531 | match |
| hole_countersink | Hole, countersink | 5700.267727 | 5700.549794 | **F1** (ignored) |
| hole_through_flat | Hole ThroughAll, flat | 5717.256661 | 5717.256661 | match |
| loft_rect_to_circle | loft between offset sketches | 3059.219533 | 3059.219533 | match |
| loft_ruled_rects | ruled loft | 3480.000000 | 3480.000000 | match |
| pipe_l_path | pipe along an L, Transition = Right corner | 628.318531 | 628.318531 | match |
| pipe_l_path_transformed | same, FreeCAD's default Transition = Transformed | 376.991118 | 628.318531 | **F2** (ignored) |
| pipe_arc_path | pipe along line-arc-line (tangent) | 637.215060 | 637.215060 | match |
| slot_tangent_arcs | slot: endpoint tangent arcs, Equal, Distance | 1514.159265 | 1514.159265 | match |
| sketch_hole_loop | outer loop with a circular and a rectangular hole | 4808.761102 | 4808.761102 | match |
| expressions_named | named constraints, sketch + Pad/Pocket expressions, Spreadsheet aliases, `<<label>>` | 1978.300888 | 1978.300888 | match |
| sketch_constraint_zoo | 49 constraints of most kinds, clockwise arc, DOF 0 | – | – | match (DOF, geometry) |
| sketch_conflicting | conflicting constraints | – (error) | – (error) | match: conflicting list identical (12 constraints) |
| sketch_redundant | redundant constraints | – (FreeCAD refuses) | 600.000000 | match: redundant list identical [11, 12]; see F5 |
| sketch_underconstrained | DOF 10 | 374.867259 | 374.867259 | match (DOF 10) |
| multi_feature_part | pad, pocket on face, pad on pocket face, hole | 9675.143150 | 9675.143150 | match |
| multi_solid_body | two disjoint pads | 1000.000000 | error / 500 | **F3** (ignored) |

33 parity tests pass, 3 are `#[ignore = "see docs/parity.md"]`.

## Findings for code outside the importer

### F1 — Countersink cone is distorted by the "lift" above the face
* **Where:** `crates/fc-part/src/lib.rs`, `hole_tool`.
* **Repro:** `cargo test -p fc-part --test parity -- --ignored hole_countersink`
  (box 30x20x10, Hole Ø6 depth 6, angled 118°, countersink Ø12 / 90°).
* **Expected (FreeCAD):** 5700.267727 mm³ — the cone runs from r = 6 at the face to r = 3
  at depth 3. **Actual:** 5700.549794 mm³ (0.282 mm³ less material removed, area off 1e-5).
* **Cause:** every profile point with z == 0 is lifted by `lift = 1e-3 * diameter` above
  the face (`w` closure). For the countersink the top point `(rc, 0)` is lifted but the
  next point `(r, (rc - r)/tan(half))` is not, so the cone becomes steeper: its radius at
  the face is `rc - lift * (rc - r) / (depth_cs + lift)` ≈ 5.994 instead of 6. The analytic
  volume error of that cone is 0.012·π·7.5 = 0.2827 mm³, exactly the observed difference.
  (Counterbores are unaffected: their walls are vertical.)
* **Fix:** extend the cone along its own slope when lifting, i.e. use
  `(rc + lift * half.tan(), -lift)` as the top point (or add the lift as a separate
  cylinder of radius `rc` above z = 0).

### F2 — Pipe transition mode is hard-wired to Right corner
* **Where:** `crates/fc-occt/cpp/shim.cpp` (sweep: `SetTransitionMode(BRepBuilderAPI_RightCorner)`),
  `fc_part::Pipe` (no transition field).
* **Repro:** `cargo test -p fc-part --test parity -- --ignored pipe_l_path_transformed`
  (circle r = 2 swept along an L: 30 mm up, 20 mm along X; FreeCAD default Transition).
* **Expected (FreeCAD, `Transformed`):** 376.991118 mm³, 4 faces / 5 edges (the profile is
  not rotated at the corner, so the second leg degenerates to zero volume).
  **Actual:** 628.318531 mm³ (mitred elbow). With `Transition = Right corner` both agree
  (`pipe_l_path`), and on smooth spines all modes agree (`pipe_arc_path`).
* **Fix:** add `transition: Transformed | RightCorner | RoundCorner` to `Pipe` and pass it
  to `BRepOffsetAPI_MakePipeShell::SetTransitionMode` (FreeCAD: 0 → `BRepBuilderAPI_Transformed`,
  1 → `RightCorner`, 2 → `RoundCorner`). The importer already reads `Transition`; it
  currently reports non-right-corner transitions as an Info note.

### F3 — Bodies with several solids are rejected
* **Where:** `crates/fc-part/src/lib.rs`, `combine` ("a Body must be a single contiguous solid").
* **Repro:** `cargo test -p fc-part --test parity -- --ignored multi_solid_body`
  (two disjoint 10x10x5 pads).
* **Expected (FreeCAD 1.1, `Body.AllowCompound = true` by default):** a body of 2 solids,
  1000 mm³. **Actual:** `Pad001` errors, body = first pad, 500 mm³.
* **Note:** decide whether to
  follow 1.1 (keep a compound) — the importer could read `AllowCompound`.

### F4 — Sketch frames on faces and reversed/offset attachments differ from FreeCAD
The importer compensates (re-expresses the sketch in fc-part's frame; exact for quarter
turns), so nothing fails, but a sketch drawn in fc-part has different coordinates than the
same sketch drawn in FreeCAD, and a document round-tripped to FreeCAD would need the
inverse transform.
* **Where:** `crates/fc-part/src/lib.rs`, `Document::resolve_support`; `placement.rs`,
  `Placement::reversed`.
* **FreeCAD (Attacher, legacy FlatFace orientation used for feature faces):** normal
  closest to ±Z → x = +X; closest to +Y → x = −X, −Y → x = +X; closest to +X → x = +Y,
  −X → x = −Y; origin = projection of the body origin. fc-part uses x = +Y for both ±X and
  x = +X for both ±Y, so sketches on **+Y and −X faces are rotated 180°** relative to
  FreeCAD (`face_frames`: `Sketch002`, `Sketch003`). For faces not aligned with an axis
  the frames differ by an arbitrary angle (fc-part projects world X).
* **MapReversed:** FreeCAD flips the normal and **X** (`ZReverse` + `XReverse`, i.e. a 180°
  turn about Y); `Placement::reversed` flips the normal and **Y** — a 180° in-plane rotation
  apart (`pad_map_reversed`).
* **AttachmentOffset with MapReversed:** FreeCAD applies the offset in the reversed frame
  (`z = -2` for offset 2 on reversed XY); fc-part applies `offset` along the support normal
  before reversing (`z = +2`). The importer stores `offset = -Pz` in that case.
* **Fix (optional):** follow FreeCAD's rules in `resolve_support` and `Placement::reversed`
  (and apply `offset` after reversing). The importer would then see identity frames.

### F5 — Sketches with redundant constraints
FreeCAD 1.1 refuses to recompute a sketch with redundant constraints ("Sketch with
redundant constraints", the Pad then has no profile); fc-part solves it and pads it
(`sketch_redundant`: FreeCAD no solid, fc-part 600 mm³). The redundant lists agree exactly.
Design choice rather than a bug; listed so it is a conscious one.

### F6 — Over-constrained DOF
For `sketch_conflicting` FreeCAD reports DoF = −2 (parameters − equations), fc-part 0
(parameters − rank). The conflicting lists agree; the suite skips DOF comparison when
FreeCAD's is negative.

## What the importer handles (`fc_part::fcstd`)

API: `import(path) -> Result<Document, ImportError>`,
`import_with_report(path) -> Result<(Document, ImportReport), ImportError>`,
`import_bytes(&[u8])`, `import_document_xml(&str)`. `ImportReport` has `notes`
(`Skipped` / `Approximated` / `Info`, per FreeCAD object), `sketches[name]`
(`SketchMap`: FreeCAD geometry/constraint index → ours, `to_freecad(&[usize])` to map
diagnose lists back, and the frame rotation/offset applied), `program_version`, `body`,
`other_bodies`. Feature names and labels are FreeCAD's.

* **Sketch geometry:** LineSegment, Circle, ArcOfCircle (any normal/AngleXU; clockwise arcs
  become CCW arcs, endpoint numbering kept as FreeCAD's sketcher emulates CCW), Point.
  Construction from `geometryModeFlags` (bit 1) or `<Construction>`; internal-alignment
  geometry counts as construction. Other curves (ellipses, B-splines, …) are skipped with a
  note; constraints on them are dropped. External geometry: skipped with a note.
* **Constraints:** Coincident, Horizontal/Vertical (line, or two points), Parallel,
  Tangent (edge-edge; endpoint-endpoint → `TangentAt`; endpoint-edge → `TangentAt(p, edge)`),
  Distance (line length; point-point; point-line/circle), DistanceX/Y (line; single point =
  lock against the origin; two points, signed), Angle (line; arc sweep; two lines; two
  lines with endpoints — direction flips add π), Perpendicular (edge-edge; with endpoints
  → Coincident/PointOnObject + Perpendicular, noted), Radius, Diameter, Equal,
  PointOnObject, Symmetric (about a point or a line), Block. FreeCAD's root point (−1, 1)
  maps to `ROOT`, −1/−2 to `H_AXIS`/`V_AXIS`. Names, reference (non-driving) state;
  deactivated constraints are dropped. ElementIds/ElementPositions or the legacy
  First/Second/Third attributes.
* **Not supported (noted):** tangent/perpendicular/angle *via point* (Third element),
  edge-to-edge Distance, arc-length Distance, InternalAlignment, SnellsLaw, Weight,
  Group/Text.
* **Attachment:** origin planes (`AttachmentSupport` or old `Support`, FlatFace/ObjectXY)
  with MapReversed and AttachmentOffset; planar faces `Feature.FaceN` (matched on
  fc-part's solid by FreeCAD's face plane, preferring the same index); anything else
  (datum planes, other map modes) from the stored `Placement`, noted.
* **Pad/Pocket:** Length, Length2, Type (Length, ThroughAll, UpToFirst, TwoLengths;
  UpToLast → ThroughAll and UpToFace/UpToShape → Length, noted), SideType or old Midplane,
  Reversed. Taper, offset, custom direction: noted and ignored.
* **Revolution/Groove:** ReferenceAxis V_Axis/H_Axis/`EdgeN`/`AxisN` of the profile or the
  Origin's X/Y/Z axes, Angle/Angle2, Type TwoAngles, SideType/Midplane, Reversed.
* **Hole:** Diameter, Depth, DepthType, DrillPoint/DrillPointAngle, HoleCutType
  None/Counterbore/Countersink (others → counterbore, noted), HoleCutDiameter/Depth/
  CountersinkAngle. Threads, taper, DrillForDepth: noted.
* **Loft:** Profile, Sections, Ruled (Closed noted). **Pipe:** Profile, Spine, Mode
  Standard/Frenet (others noted), Transition (see F2), multi-section noted.
* **Expressions:** every ExpressionEngine entry of the imported features
  (`Length`, `Constraints.name`, `Constraints[i]`, `AttachmentOffset.Base.z`, hole
  properties) is rewritten (`.Constraints.x` → `Sketch.Constraints.x`, `<<Label>>`,
  labels as names, `Sheet.alias` → `alias`, constraint indices renumbered, values of
  constraints re-expressed in a rotated frame wrapped as `(expr) * s + a`). Spreadsheet
  cells become document parameters (alias, else `Sheet_A1`). Expressions that do not
  parse or evaluate in fc-part are dropped with a note (the stored value stays).
* **Body:** first `PartDesign::Body` (others noted), Group order, Tip, Suppressed;
  BaseFeature noted. Unsupported feature types (fillets, patterns, datums, …) are skipped
  with a note, and features consuming them are skipped too.

## Resolution (lead)
* F1 fixed: the countersink rim is lifted along its cone (`hole_tool`).
* F2: `Pipe` has a `transition` field (Transformed default, Right corner, Round corner) passed
  to `SetTransitionMode`. `pipe_l_path_transformed` stays ignored: OCCT's Transformed mode at a
  sharp corner yields a degenerate solid in both programs and the numbers differ.
* F3 fixed: bodies may hold several solids, like FreeCAD 1.1 (AllowCompound).
* F4 fixed: face attachments use FreeCAD's legacy FlatFace X rule, MapReversed flips X (not
  Y), and AttachmentOffset applies in the reversed frame. Imports now need no re-expression.
* F5: redundant sketches are a warning (feature state `warning`), and their features still compute, as FreeCAD's do.
