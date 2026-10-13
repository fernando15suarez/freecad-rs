# freecad-rs

A Rust reimplementation of FreeCAD's Sketcher and the Part Design features that turn sketches
into solids, on OpenCASCADE 7.8 (built from source through `occt-sys`). See ROADMAP.md for what
is done and how it is tested.

Calc Canvas links freecad-rs models live: their named parameters are driven from the board and
their named measurements flow back (`fc_part::measure`).

## Licence

LGPL-2.1-or-later (see LICENSE). freecad-rs follows FreeCAD's behaviour and ports parts of it
(FreeCAD is LGPL-2.1-or-later). `crates/fc-occt/occt-patch` holds a patched OpenCASCADE source
file under OCCT's own licence: LGPL 2.1 with the exception in
`crates/fc-occt/occt-patch/OCCT_LGPL_EXCEPTION.txt`.

The parity models in `tools/parity/models` were built with FreeCAD 1.1.3 by `tools/parity/make_models.py`.
