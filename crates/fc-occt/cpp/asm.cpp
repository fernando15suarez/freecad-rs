// Assembly services for freecad-rs: XDE documents (structured STEP export/import),
// located shapes, minimum distance and BREP strings. Errors become std::runtime_error.
#include "asm.h"
#include "fc-occt/src/asm.rs.h"

#include <Standard_Failure.hxx>
#include <Message.hxx>
#include <Message_Messenger.hxx>
#include <gp_Trsf.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <TDataStd_Name.hxx>
#include <TDF_LabelSequence.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TCollection_AsciiString.hxx>
#include <Quantity_Color.hxx>
#include <Quantity_ColorRGBA.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <STEPControl_Controller.hxx>
#include <Interface_Static.hxx>

#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>

namespace fcocc {

namespace {

[[noreturn]] void fail(const Standard_Failure &e, const char *where) {
    std::ostringstream os;
    os << where << ": " << e.DynamicType()->Name();
    if (e.GetMessageString() && *e.GetMessageString()) os << ": " << e.GetMessageString();
    throw std::runtime_error(os.str());
}

#define ASM_TRY try {
#define ASM_CATCH(where)                                                                   \
    }                                                                                      \
    catch (const Standard_Failure &e) { fail(e, where); }

std::unique_ptr<Shape> wrap(const TopoDS_Shape &s) {
    auto r = std::make_unique<Shape>();
    r->s = s;
    return r;
}

void quiet() {
    static bool done = false;
    if (!done) {
        Message::DefaultMessenger()->ChangePrinters().Clear();
        done = true;
    }
}

// Row-major 3x4 [R | t].
gp_Trsf trsf(const std::array<double, 12> &m) {
    gp_Trsf t;
    t.SetValues(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10], m[11]);
    return t;
}

std::array<double, 12> matrix(const gp_Trsf &t) {
    std::array<double, 12> m{};
    for (int r = 1; r <= 3; r++)
        for (int c = 1; c <= 4; c++) m[(r - 1) * 4 + (c - 1)] = t.Value(r, c);
    return m;
}

void set_name(const TDF_Label &l, rust::Str name) {
    std::string n(name);
    if (!n.empty()) TDataStd_Name::Set(l, TCollection_ExtendedString(n.c_str(), Standard_True));
}

rust::String get_name(const TDF_Label &l) {
    Handle(TDataStd_Name) n;
    if (!l.FindAttribute(TDataStd_Name::GetID(), n)) return rust::String();
    // ExtendedString -> UTF-8.
    Standard_Integer len = n->Get().LengthOfCString();
    std::string buf(len + 1, '\0');
    Standard_PCharacter p = buf.data();
    n->Get().ToUTF8CString(p);
    buf.resize(std::strlen(buf.c_str()));
    return rust::String(buf);
}

Handle(TDocStd_Document) new_doc() {
    Handle(XCAFApp_Application) app = XCAFApp_Application::GetApplication();
    Handle(TDocStd_Document) doc;
    app->NewDocument("MDTV-XCAF", doc);
    return doc;
}

std::unique_ptr<AsmDoc> make(Handle(TDocStd_Document) doc) {
    auto d = std::make_unique<AsmDoc>();
    d->doc = doc;
    d->shapes = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    d->colors = XCAFDoc_DocumentTool::ColorTool(doc->Main());
    return d;
}

const TDF_Label &label(const AsmDoc &d, size_t i, const char *what) {
    if (i >= d.labels.size()) throw std::runtime_error(std::string(what) + ": bad label index");
    return d.labels[i];
}

} // namespace

std::unique_ptr<AsmDoc> asm_new() {
    ASM_TRY
    return make(new_doc());
    ASM_CATCH("asm_new")
}

size_t asm_add_part(AsmDoc &d, const Shape &s, rust::Str name, std::array<double, 4> rgba) {
    ASM_TRY
    if (s.s.IsNull()) throw std::runtime_error("asm_add_part: null shape");
    TDF_Label l = d.shapes->AddShape(s.s, Standard_False, Standard_False);
    set_name(l, name);
    if (rgba[3] >= 0.0) {
        Quantity_Color c(rgba[0], rgba[1], rgba[2], Quantity_TOC_sRGB);
        d.colors->SetColor(l, Quantity_ColorRGBA(c, (float)rgba[3]), XCAFDoc_ColorGen);
    }
    d.labels.push_back(l);
    return d.labels.size() - 1;
    ASM_CATCH("asm_add_part")
}

size_t asm_add_assembly(AsmDoc &d, rust::Str name) {
    ASM_TRY
    TDF_Label l = d.shapes->NewShape();
    set_name(l, name);
    d.labels.push_back(l);
    return d.labels.size() - 1;
    ASM_CATCH("asm_add_assembly")
}

size_t asm_add_component(AsmDoc &d, size_t parent, size_t child, std::array<double, 12> m, rust::Str name) {
    ASM_TRY
    TDF_Label p = label(d, parent, "asm_add_component");
    TDF_Label c = label(d, child, "asm_add_component");
    TDF_Label l = d.shapes->AddComponent(p, c, TopLoc_Location(trsf(m)));
    if (l.IsNull()) throw std::runtime_error("asm_add_component: parent is not an assembly");
    set_name(l, name);
    d.labels.push_back(l);
    return d.labels.size() - 1;
    ASM_CATCH("asm_add_component")
}

void asm_write_step(AsmDoc &d, rust::Str path, int schema) {
    ASM_TRY
    quiet();
    d.shapes->UpdateAssemblies();
    // The schema must be set before the writer creates its model (the header's FILE_SCHEMA).
    STEPControl_Controller::Init();
    Interface_Static::SetCVal("write.step.schema", schema == 242 ? "AP242DIS" : "AP214IS");
    Interface_Static::SetCVal("write.step.unit", "MM");
    STEPCAFControl_Writer w;
    w.SetNameMode(Standard_True);
    w.SetColorMode(Standard_True);
    if (!w.Transfer(d.doc, STEPControl_AsIs)) throw std::runtime_error("STEP transfer failed");
    std::string p(path);
    if (w.Write(p.c_str()) != IFSelect_RetDone) throw std::runtime_error("STEP write failed: " + p);
    // Leave the global schema as the plain writer expects it.
    Interface_Static::SetCVal("write.step.schema", "AP214IS");
    ASM_CATCH("asm_write_step")
}

std::unique_ptr<AsmDoc> asm_read_step(rust::Str path) {
    ASM_TRY
    quiet();
    auto d = make(new_doc());
    STEPCAFControl_Reader r;
    r.SetNameMode(Standard_True);
    r.SetColorMode(Standard_True);
    std::string p(path);
    if (r.ReadFile(p.c_str()) != IFSelect_RetDone) throw std::runtime_error("STEP read failed: " + p);
    if (!r.Transfer(d->doc)) throw std::runtime_error("STEP transfer failed: " + p);
    TDF_LabelSequence seq;
    d->shapes->GetShapes(seq);
    for (TDF_LabelSequence::Iterator it(seq); it.More(); it.Next()) d->labels.push_back(it.Value());
    return d;
    ASM_CATCH("asm_read_step")
}

rust::Vec<AsmNode> asm_nodes(const AsmDoc &d) {
    ASM_TRY
    rust::Vec<AsmNode> out;
    for (const TDF_Label &l : d.labels) {
        AsmNode n;
        n.is_assembly = XCAFDoc_ShapeTool::IsAssembly(l);
        n.name = get_name(l);
        n.rgba = {0.0, 0.0, 0.0, -1.0};
        Quantity_ColorRGBA c;
        if (d.colors->GetColor(l, XCAFDoc_ColorGen, c) || d.colors->GetColor(l, XCAFDoc_ColorSurf, c)) {
            Standard_Real r, g, b;
            c.GetRGB().Values(r, g, b, Quantity_TOC_sRGB);
            n.rgba = {r, g, b, c.Alpha()};
        }
        out.push_back(std::move(n));
    }
    return out;
    ASM_CATCH("asm_nodes")
}

rust::Vec<AsmComp> asm_components(const AsmDoc &d) {
    ASM_TRY
    rust::Vec<AsmComp> out;
    auto index = [&](const TDF_Label &l) -> long {
        for (size_t i = 0; i < d.labels.size(); i++)
            if (d.labels[i] == l) return (long)i;
        return -1;
    };
    for (size_t i = 0; i < d.labels.size(); i++) {
        const TDF_Label &l = d.labels[i];
        if (!XCAFDoc_ShapeTool::IsAssembly(l)) continue;
        TDF_LabelSequence comps;
        XCAFDoc_ShapeTool::GetComponents(l, comps, Standard_False);
        for (TDF_LabelSequence::Iterator it(comps); it.More(); it.Next()) {
            TDF_Label ref;
            if (!XCAFDoc_ShapeTool::GetReferredShape(it.Value(), ref)) continue;
            long j = index(ref);
            if (j < 0) continue;
            AsmComp c;
            c.parent = i;
            c.child = (size_t)j;
            c.name = get_name(it.Value());
            c.matrix = matrix(XCAFDoc_ShapeTool::GetLocation(it.Value()).Transformation());
            out.push_back(std::move(c));
        }
    }
    return out;
    ASM_CATCH("asm_components")
}

rust::Vec<size_t> asm_roots(const AsmDoc &d) {
    ASM_TRY
    rust::Vec<size_t> out;
    TDF_LabelSequence seq;
    d.shapes->GetFreeShapes(seq);
    for (TDF_LabelSequence::Iterator it(seq); it.More(); it.Next())
        for (size_t i = 0; i < d.labels.size(); i++)
            if (d.labels[i] == it.Value()) out.push_back(i);
    return out;
    ASM_CATCH("asm_roots")
}

std::unique_ptr<Shape> asm_node_shape(const AsmDoc &d, size_t node) {
    ASM_TRY
    return wrap(XCAFDoc_ShapeTool::GetShape(label(d, node, "asm_node_shape")));
    ASM_CATCH("asm_node_shape")
}

std::unique_ptr<Shape> shape_moved(const Shape &s, std::array<double, 12> m) {
    ASM_TRY
    return wrap(s.s.Moved(TopLoc_Location(trsf(m))));
    ASM_CATCH("shape_moved")
}

std::array<double, 7> min_distance(const Shape &a, const Shape &b) {
    ASM_TRY
    if (a.s.IsNull() || b.s.IsNull()) throw std::runtime_error("min_distance: null shape");
    BRepExtrema_DistShapeShape ds(a.s, b.s);
    if (!ds.IsDone() || ds.NbSolution() < 1) throw std::runtime_error("min_distance: no solution");
    gp_Pnt p = ds.PointOnShape1(1), q = ds.PointOnShape2(1);
    return {ds.Value(), p.X(), p.Y(), p.Z(), q.X(), q.Y(), q.Z()};
    ASM_CATCH("min_distance")
}

rust::String shape_to_brep(const Shape &s) {
    ASM_TRY
    std::ostringstream os;
    BRepTools::Write(s.s, os);
    return rust::String(os.str());
    ASM_CATCH("shape_to_brep")
}

std::unique_ptr<Shape> shape_from_brep(rust::Str text) {
    ASM_TRY
    std::string t(text);
    if (t.find("CASCADE Topology") == std::string::npos) throw std::runtime_error("shape_from_brep: not a BREP text");
    std::istringstream is{t};
    TopoDS_Shape s;
    BRep_Builder bb;
    BRepTools::Read(s, is, bb);
    if (s.IsNull()) throw std::runtime_error("shape_from_brep: no shape");
    return wrap(s);
    ASM_CATCH("shape_from_brep")
}

} // namespace fcocc
