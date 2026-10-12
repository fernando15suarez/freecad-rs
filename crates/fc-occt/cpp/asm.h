#pragma once
// Assembly-level OCCT services: XDE (XCAF) documents for structured STEP export/import,
// located shapes, minimum distance and BREP strings. Separate from shim.h so the core
// shim does not depend on the OCAF toolkits.
#include "rust/cxx.h"
#include "shim.h"
#include <memory>
#include <vector>
#include <array>
#include <TDocStd_Document.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <TDF_Label.hxx>

namespace fcocc {

struct AsmNode;
struct AsmComp;

/// An XDE document plus the labels we created or read, addressed by index.
struct AsmDoc {
    Handle(TDocStd_Document) doc;
    Handle(XCAFDoc_ShapeTool) shapes;
    Handle(XCAFDoc_ColorTool) colors;
    std::vector<TDF_Label> labels;
};

std::unique_ptr<AsmDoc> asm_new();
size_t asm_add_part(AsmDoc &d, const Shape &s, rust::Str name, std::array<double, 4> rgba);
size_t asm_add_assembly(AsmDoc &d, rust::Str name);
size_t asm_add_component(AsmDoc &d, size_t parent, size_t child, std::array<double, 12> m, rust::Str name);
void asm_write_step(AsmDoc &d, rust::Str path, int schema);

std::unique_ptr<AsmDoc> asm_read_step(rust::Str path);
rust::Vec<AsmNode> asm_nodes(const AsmDoc &d);
rust::Vec<AsmComp> asm_components(const AsmDoc &d);
rust::Vec<size_t> asm_roots(const AsmDoc &d);
std::unique_ptr<Shape> asm_node_shape(const AsmDoc &d, size_t node);

std::unique_ptr<Shape> shape_moved(const Shape &s, std::array<double, 12> m);
std::array<double, 7> min_distance(const Shape &a, const Shape &b);
rust::String shape_to_brep(const Shape &s);
std::unique_ptr<Shape> shape_from_brep(rust::Str text);

} // namespace fcocc
