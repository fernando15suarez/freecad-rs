# freecad-rs roadmap

A Rust reimplementation of FreeCAD's **Sketcher** and the **Part Design** features that turn
sketches into solids. Geometry kernel: OpenCASCADE 7.8 (built from source via `occt-sys`,
wrapped by a thin C++ shim in `crates/fc-occt`). FreeCAD's own source (LGPL, incl. planegcs)
is the behavioral reference.

Out of scope for now: FEM, CAM, Arch, TechDraw, assemblies, Python.

## Crates
- `fc-sketch` — sketch geometry, constraints, solver (DOF, conflicting/redundant reports), profile loops. Pure Rust.
- `fc-occt` — C++ shim over OCCT: faces from profiles, prism/revol/boolean, loft/sweep, meshing, STEP/STL.
- `fc-part` — document model: Body, Sketch placement, features, dependency-ordered recompute, save/load.
- `fc-app` — native Mac app (eframe): 3D view, feature tree, property panel, sketch editor.

## Milestones
- [x] **M1 Kernel spike** — OCCT builds statically on macOS arm64; pad a rectangle; STEP + STL export.
- [x] **M2 Solver** — points, lines, circles, arcs, construction geometry; coincident, horizontal, vertical,
      parallel, perpendicular, tangent, equal, point-on-object, symmetric, distance (+X/Y), radius,
      diameter, angle, lock; DOF count; conflicting/redundant constraint reporting like FreeCAD.
- [x] **M3 Part Design core** — Body + feature tree; Pad, Pocket, Revolution, Groove from sketches;
      named dimensions + expressions (sketch constraints and feature lengths can be named and referenced, e.g. `width / 2 + 3`);
      deleting named values/dimensions shows their users and can freeze them to plain values; index references renumber;
      changing a sketch dimension recomputes the solid (headless, tested).
- [x] **M4 App** — 3D view (orbit/pan/zoom, standard views), feature tree, property editing, STEP/STL export.
- [x] **M5 Sketch editor** — draw tools (line, polyline, rectangle, circle, arc, point), snapping,
      constraint tools, dimension editing, DOF/over-constraint coloring like FreeCAD.
- [ ] **M6 More features** — sketch on face of an existing solid, Loft, Sweep (Pipe), Hole.
- [x] **M7 Files + parity** — save/load document, comparison suite against FreeCAD on sample models.

## Testing
- `cargo test` — solver (48), Part Design (19), FreeCAD parity suite (36 models built with FreeCAD 1.1.3, `tools/parity/`).
- `tools/e2e/run.sh` — drives the real app with scripted mouse/keyboard input (`FCRS_INPUT`), e.g. new sketch ->
  rectangle -> dimensions -> Pad -> formula in Length -> sketch on the top face -> circle -> Pocket -> undo/redo.
- Sketch editor scripts: `crates/fc-app/src/sketcher/scripts/` (`FCRS_SKETCH_SCRIPT`).

## Phase 2: Assembly + FEM (requested 2026-10-12)
- [x] **A1 Multi-part workspaces** — several parts + assemblies per document, instances, v2 project files.
- [x] **A2 Mate connectors** — Onshape-style connectors on parts and in assemblies, rebuild-safe anchors.
- [x] **A3 Joints + solver** — FreeCAD 1.1 joints + Onshape planar/pin-slot; port of FreeCAD's Ondsel solver (exact parity); drag; DOF/conflict reports.
- [x] **A4 Assembly tools** — structured STEP export/import (FreeCAD reads it), STL, interference/clearance, BOM + CSV, exploded views, sub-assemblies.
- [x] **F1-F3 FEM** — Gmsh meshing + CalculiX (FreeCAD's bundled tools), static/modal/thermal, identical to FreeCAD FEM on reference cases; FEM workbench UI with result color maps.
- [x] **F4 Assembly FEA** — multi-part analyses (bonded/ties/contact), contact detection, per-part results, FreeCAD parity.
- [ ] Next candidates: gear/belt/screw joint tools, flexible sub-assemblies, shells/beams, nonlinear materials, Part workbench (booleans, fillets).

FreeCAD source reference: `~/Projects/rust-gaming/freecad-src` (full checkout incl. OndselSolver).
