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
- [ ] **M3 Part Design core** — Body + feature tree; Pad, Pocket, Revolution, Groove from sketches;
      named dimensions + expressions (sketch constraints and feature lengths can be named and referenced, e.g. `width / 2 + 3`);
      deleting named values/dimensions shows their users and can freeze them to plain values; index references renumber;
      changing a sketch dimension recomputes the solid (headless, tested).
- [ ] **M4 App** — 3D view (orbit/pan/zoom, standard views), feature tree, property editing, STEP/STL export.
- [ ] **M5 Sketch editor** — draw tools (line, polyline, rectangle, circle, arc, point), snapping,
      constraint tools, dimension editing, DOF/over-constraint coloring like FreeCAD.
- [ ] **M6 More features** — sketch on face of an existing solid, Loft, Sweep (Pipe), Hole.
- [ ] **M7 Files + parity** — save/load document, comparison suite against FreeCAD on sample models.
- [ ] **M8 MBD / 3D annotations** — SolidWorks-MBD-style PMI on the model: linear/radial/angular dimensions
      between faces and edges, notes with leaders, datums, GD&T frames, annotation views; attached to topology
      so they follow model changes; measurements are temporary unless kept; STEP AP242 PMI export
      (needs OCCT's ApplicationFramework/XCAF toolkits).
