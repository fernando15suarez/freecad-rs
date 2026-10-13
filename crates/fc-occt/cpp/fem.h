#pragma once
// FEM services for freecad-rs (fc-fem): BooleanFragments (OCCT general fuse) of several parts
// with the history needed to map parts to the solids and faces of the result.
#include "rust/cxx.h"
#include "shim.h"
#include <memory>

namespace fcocc {

/// Splits the direct children of `args` (a compound, one child per part) against each other.
/// `fuse` = true: BOPAlgo general fuse (FreeCAD's BooleanFragments, "Standard" mode), touching
/// parts then share their common faces. `fuse` = false: the children are just collected in a
/// compound. `history` receives records of 4 integers:
///   [0, part, local solid index, result solid index] and [1, part, local face index, result face index]
/// (indices 1-based, TopExp::MapShapes order of the part and of the result).
std::unique_ptr<Shape> fem_fragments(const Shape &args, bool fuse, double fuzzy, rust::Vec<int64_t> &history);

/// 1-based result face indices of every face of solid `solid` (1-based) of `s`.
rust::Vec<size_t> fem_solid_faces(const Shape &s, size_t solid);

} // namespace fcocc
