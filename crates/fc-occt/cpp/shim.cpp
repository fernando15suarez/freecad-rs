// Thin C++ layer over OpenCASCADE for freecad-rs. Every entry point converts OCCT's
// Standard_Failure into std::runtime_error so cxx can surface it as a Rust Err.
#include "shim.h"
#include "fc-occt/src/ffi.rs.h"

#include <Standard_Version.hxx>
#include <Standard_Failure.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp_Trsf.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GC_MakeSegment.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepOffsetAPI_MakePipeShell.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <BRep_Builder.hxx>
#include <BRepTools.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepLib_FindSurface.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <Poly_Triangulation.hxx>
#include <Poly_PolygonOnTriangulation.hxx>
#include <Poly_Polygon3D.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>
#include <TopLoc_Location.hxx>
#include <ShapeFix_Face.hxx>
#include <ShapeFix_Shape.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <STEPControl_Writer.hxx>
#include <STEPControl_Reader.hxx>
#include <StlAPI_Writer.hxx>
#include <Interface_Static.hxx>
#include <GCPnts_TangentialDeflection.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <Precision.hxx>
#include <TopoDS_Vertex.hxx>
#include <Message.hxx>
#include <Message_Messenger.hxx>

#include <stdexcept>
#include <sstream>
#include <string>

namespace fcocc {

namespace {

[[noreturn]] void rethrow_occt(const Standard_Failure &e, const char *where) {
    std::ostringstream os;
    os << where << ": " << e.DynamicType()->Name();
    if (e.GetMessageString() && *e.GetMessageString())
        os << ": " << e.GetMessageString();
    throw std::runtime_error(os.str());
}

#define OCC_TRY try {
#define OCC_CATCH(where)                                                                   \
    }                                                                                      \
    catch (const Standard_Failure &e) { rethrow_occt(e, where); }

gp_Pnt P(std::array<double, 3> a) { return gp_Pnt(a[0], a[1], a[2]); }
gp_Dir D(std::array<double, 3> a) { return gp_Dir(a[0], a[1], a[2]); }
gp_Vec V(std::array<double, 3> a) { return gp_Vec(a[0], a[1], a[2]); }
std::array<double, 3> A(const gp_XYZ &p) { return {p.X(), p.Y(), p.Z()}; }

std::unique_ptr<Shape> wrap(const TopoDS_Shape &s) {
    auto r = std::make_unique<Shape>();
    r->s = s;
    return r;
}

// OCCT's data exchange prints progress banners to stdout; keep the app's output clean.
void quiet() {
    static bool done = false;
    if (!done) {
        Message::DefaultMessenger()->ChangePrinters().Clear();
        done = true;
    }
}

void need(const Shape &s, const char *what) {
    if (s.s.IsNull()) throw std::runtime_error(std::string(what) + ": null shape");
}

} // namespace

int occt_version() { return OCC_VERSION_HEX; }

std::unique_ptr<WireBuilder> new_wire_builder() { return std::make_unique<WireBuilder>(); }

void WireBuilder::add_line(std::array<double, 3> a, std::array<double, 3> b) {
    OCC_TRY
    if (P(a).Distance(P(b)) < Precision::Confusion()) throw std::runtime_error("add_line: zero-length line");
    edges.push_back(BRepBuilderAPI_MakeEdge(P(a), P(b)).Edge());
    OCC_CATCH("add_line")
}

void WireBuilder::add_arc(std::array<double, 3> c, std::array<double, 3> n, std::array<double, 3> x, double r,
                          double a0, double a1) {
    OCC_TRY
    gp_Circ circ(gp_Ax2(P(c), D(n), D(x)), r);
    Handle(Geom_TrimmedCurve) arc = GC_MakeArcOfCircle(circ, a0, a1, true).Value();
    edges.push_back(BRepBuilderAPI_MakeEdge(arc).Edge());
    OCC_CATCH("add_arc")
}

void WireBuilder::add_circle(std::array<double, 3> c, std::array<double, 3> n, std::array<double, 3> x, double r) {
    OCC_TRY
    gp_Circ circ(gp_Ax2(P(c), D(n), D(x)), r);
    edges.push_back(BRepBuilderAPI_MakeEdge(circ).Edge());
    OCC_CATCH("add_circle")
}

void WireBuilder::end_wire() {
    OCC_TRY
    if (edges.empty()) throw std::runtime_error("end_wire: no edges");
    BRepBuilderAPI_MakeWire mk;
    for (auto &e : edges) mk.Add(e);
    if (!mk.IsDone()) throw std::runtime_error("end_wire: edges do not connect into a wire");
    TopoDS_Wire w = mk.Wire();
    edges.clear();
    if (faces.empty()) loose.push_back(w);
    else faces.back().wires.push_back(w);
    OCC_CATCH("end_wire")
}

void WireBuilder::begin_face() { faces.emplace_back(); }

std::unique_ptr<Shape> WireBuilder::build_faces() {
    OCC_TRY
    BRep_Builder bb;
    TopoDS_Compound comp;
    bb.MakeCompound(comp);
    int n = 0;
    for (auto &fg : faces) {
        if (fg.wires.empty()) continue;
        if (!fg.wires[0].Closed() && !BRep_Tool::IsClosed(fg.wires[0]))
            throw std::runtime_error("build_faces: outer wire is not closed");
        BRepBuilderAPI_MakeFace mk(fg.wires[0], Standard_True);
        if (!mk.IsDone()) throw std::runtime_error("build_faces: wire is not planar");
        TopoDS_Face outer = mk.Face();
        BRepBuilderAPI_MakeFace mkh(outer);
        for (size_t i = 1; i < fg.wires.size(); ++i) mkh.Add(fg.wires[i]);
        TopoDS_Face f = mkh.Face();
        // Let ShapeFix orient inner wires as holes regardless of their drawing direction.
        ShapeFix_Face fix(f);
        fix.FixOrientation();
        fix.Perform();
        bb.Add(comp, fix.Face());
        ++n;
    }
    if (n == 0) throw std::runtime_error("build_faces: no closed profile");
    faces.clear();
    return wrap(comp);
    OCC_CATCH("build_faces")
}

std::unique_ptr<Shape> WireBuilder::build_wires() {
    BRep_Builder bb;
    TopoDS_Compound comp;
    bb.MakeCompound(comp);
    for (auto &w : loose) bb.Add(comp, w);
    for (auto &fg : faces)
        for (auto &w : fg.wires) bb.Add(comp, w);
    loose.clear();
    faces.clear();
    return wrap(comp);
}

std::unique_ptr<Shape> prism(const Shape &s, std::array<double, 3> d) {
    OCC_TRY
    need(s, "prism");
    BRepPrimAPI_MakePrism mk(s.s, V(d));
    if (!mk.IsDone()) throw std::runtime_error("prism failed");
    return wrap(mk.Shape());
    OCC_CATCH("prism")
}

std::unique_ptr<Shape> revol(const Shape &s, std::array<double, 3> origin, std::array<double, 3> dir, double angle) {
    OCC_TRY
    need(s, "revol");
    BRepPrimAPI_MakeRevol mk(s.s, gp_Ax1(P(origin), D(dir)), angle);
    if (!mk.IsDone()) throw std::runtime_error("revolution failed");
    return wrap(mk.Shape());
    OCC_CATCH("revol")
}

template <class Op> std::unique_ptr<Shape> boolean(const Shape &a, const Shape &b, const char *name) {
    need(a, name);
    need(b, name);
    Op op(a.s, b.s);
    if (!op.IsDone() || op.HasErrors()) throw std::runtime_error(std::string(name) + " failed");
    return wrap(op.Shape());
}

std::unique_ptr<Shape> fuse(const Shape &a, const Shape &b) {
    OCC_TRY return boolean<BRepAlgoAPI_Fuse>(a, b, "fuse");
    OCC_CATCH("fuse")
}
std::unique_ptr<Shape> cut(const Shape &a, const Shape &b) {
    OCC_TRY return boolean<BRepAlgoAPI_Cut>(a, b, "cut");
    OCC_CATCH("cut")
}
std::unique_ptr<Shape> common(const Shape &a, const Shape &b) {
    OCC_TRY return boolean<BRepAlgoAPI_Common>(a, b, "common");
    OCC_CATCH("common")
}

std::unique_ptr<Shape> translated(const Shape &s, std::array<double, 3> d) {
    OCC_TRY
    gp_Trsf t;
    t.SetTranslation(V(d));
    return wrap(BRepBuilderAPI_Transform(s.s, t, true).Shape());
    OCC_CATCH("translated")
}

std::unique_ptr<Shape> rotated(const Shape &s, std::array<double, 3> origin, std::array<double, 3> dir, double angle) {
    OCC_TRY
    gp_Trsf t;
    t.SetRotation(gp_Ax1(P(origin), D(dir)), angle);
    return wrap(BRepBuilderAPI_Transform(s.s, t, true).Shape());
    OCC_CATCH("rotated")
}

std::unique_ptr<Shape> loft(const Shape &sections, bool solid, bool ruled) {
    OCC_TRY
    BRepOffsetAPI_ThruSections mk(solid, ruled);
    int n = 0;
    for (TopoDS_Iterator it(sections.s); it.More(); it.Next()) {
        const TopoDS_Shape &w = it.Value();
        if (w.ShapeType() == TopAbs_WIRE) { mk.AddWire(TopoDS::Wire(w)); ++n; }
        else if (w.ShapeType() == TopAbs_VERTEX) { mk.AddVertex(TopoDS::Vertex(w)); ++n; }
        else if (w.ShapeType() == TopAbs_COMPOUND) {
            TopExp_Explorer ex(w, TopAbs_WIRE);
            if (ex.More()) { mk.AddWire(TopoDS::Wire(ex.Current())); ++n; }
        }
    }
    if (n < 2) throw std::runtime_error("loft needs at least two sections");
    mk.CheckCompatibility(Standard_True);
    mk.Build();
    if (!mk.IsDone()) throw std::runtime_error("loft failed");
    return wrap(mk.Shape());
    OCC_CATCH("loft")
}

std::unique_ptr<Shape> sweep(const Shape &profile, const Shape &spine, bool frenet) {
    OCC_TRY
    TopExp_Explorer sp(spine.s, TopAbs_WIRE);
    if (!sp.More()) throw std::runtime_error("sweep: spine has no wire");
    TopoDS_Wire spineWire = TopoDS::Wire(sp.Current());
    // Run the spine from the end nearest the profile (FreeCAD lets the profile sit at either end).
    {
        Bnd_Box pb;
        BRepBndLib::Add(profile.s, pb);
        gp_Pnt c = pb.IsVoid() ? gp_Pnt() : gp_Pnt((pb.CornerMin().XYZ() + pb.CornerMax().XYZ()) / 2.0);
        TopoDS_Vertex v1, v2;
        TopExp::Vertices(spineWire, v1, v2);
        gp_Pnt a = BRep_Tool::Pnt(v1);
        gp_Pnt b = BRep_Tool::Pnt(v2);
        if (c.Distance(b) < c.Distance(a)) spineWire = TopoDS::Wire(spineWire.Reversed());
    }
    BRepOffsetAPI_MakePipeShell mk(spineWire);
    mk.SetMode(frenet);
    // Sharp corners in the path get mitred corners instead of failing (FreeCAD: Transition = Right corner).
    mk.SetTransitionMode(BRepBuilderAPI_RightCorner);
    int n = 0;
    for (TopExp_Explorer ex(profile.s, TopAbs_WIRE); ex.More(); ex.Next()) { mk.Add(ex.Current()); ++n; }
    if (n == 0) throw std::runtime_error("sweep: profile has no wire");
    mk.Build();
    if (!mk.IsDone()) throw std::runtime_error("sweep failed");
    mk.MakeSolid();
    TopoDS_Shape out = mk.Shape();
    GProp_GProps gp;
    BRepGProp::VolumeProperties(out, gp);
    if (gp.Mass() < 0) out.Reverse();
    return wrap(out);
    OCC_CATCH("sweep")
}

std::unique_ptr<Shape> make_compound(const Shape &parts, const Shape &add) {
    BRep_Builder bb;
    TopoDS_Compound comp;
    bb.MakeCompound(comp);
    if (!parts.s.IsNull()) {
        if (parts.s.ShapeType() == TopAbs_COMPOUND)
            for (TopoDS_Iterator it(parts.s); it.More(); it.Next()) bb.Add(comp, it.Value());
        else bb.Add(comp, parts.s);
    }
    if (!add.s.IsNull()) bb.Add(comp, add.s);
    return wrap(comp);
}

std::unique_ptr<Shape> empty_shape() { return std::make_unique<Shape>(); }
std::unique_ptr<Shape> clone_shape(const Shape &s) { return wrap(s.s); }

std::unique_ptr<Shape> refine(const Shape &s) {
    OCC_TRY
    need(s, "refine");
    ShapeUpgrade_UnifySameDomain u(s.s, Standard_True, Standard_True, Standard_False);
    u.Build();
    return wrap(u.Shape());
    OCC_CATCH("refine")
}

bool is_null(const Shape &s) { return s.s.IsNull(); }

bool is_valid(const Shape &s) {
    if (s.s.IsNull()) return false;
    try {
        BRepCheck_Analyzer an(s.s);
        return an.IsValid();
    } catch (const Standard_Failure &) { return false; }
}

double volume(const Shape &s) {
    if (s.s.IsNull()) return 0;
    GProp_GProps p;
    BRepGProp::VolumeProperties(s.s, p);
    return p.Mass();
}

double area(const Shape &s) {
    if (s.s.IsNull()) return 0;
    GProp_GProps p;
    BRepGProp::SurfaceProperties(s.s, p);
    return p.Mass();
}

std::array<double, 6> bbox(const Shape &s) {
    if (s.s.IsNull()) return {0, 0, 0, 0, 0, 0};
    Bnd_Box b;
    BRepBndLib::Add(s.s, b);
    if (b.IsVoid()) return {0, 0, 0, 0, 0, 0};
    double x0, y0, z0, x1, y1, z1;
    b.Get(x0, y0, z0, x1, y1, z1);
    return {x0, y0, z0, x1, y1, z1};
}

static size_t count(const Shape &s, TopAbs_ShapeEnum t) {
    if (s.s.IsNull()) return 0;
    TopTools_IndexedMapOfShape m;
    TopExp::MapShapes(s.s, t, m);
    return (size_t)m.Extent();
}
size_t count_solids(const Shape &s) { return count(s, TopAbs_SOLID); }
size_t count_faces(const Shape &s) { return count(s, TopAbs_FACE); }
size_t count_edges(const Shape &s) { return count(s, TopAbs_EDGE); }

FaceInfo face_info(const Shape &s, size_t index) {
    OCC_TRY
    TopTools_IndexedMapOfShape m;
    TopExp::MapShapes(s.s, TopAbs_FACE, m);
    if (index < 1 || (int)index > m.Extent()) throw std::runtime_error("face_info: index out of range");
    TopoDS_Face f = TopoDS::Face(m.FindKey((int)index));
    FaceInfo fi;
    BRepAdaptor_Surface ad(f, true);
    bool reversed = f.Orientation() == TopAbs_REVERSED;
    switch (ad.GetType()) {
    case GeomAbs_Plane: {
        fi.kind = 0;
        gp_Ax3 ax = ad.Plane().Position();
        gp_Dir n = ax.Direction();
        gp_Dir x = ax.XDirection();
        if (reversed) n.Reverse();
        if (!ax.Direct() != reversed) { /* keep x as-is; the editor builds y = n x x */ }
        fi.origin = A(ax.Location().XYZ());
        fi.normal = A(n.XYZ());
        fi.xdir = A(x.XYZ());
        break;
    }
    case GeomAbs_Cylinder: {
        fi.kind = 1;
        gp_Ax3 ax = ad.Cylinder().Position();
        fi.origin = A(ax.Location().XYZ());
        fi.normal = A(ax.Direction().XYZ());
        fi.xdir = A(ax.XDirection().XYZ());
        fi.radius = ad.Cylinder().Radius();
        break;
    }
    case GeomAbs_Cone: fi.kind = 2; break;
    case GeomAbs_Sphere: fi.kind = 3; fi.radius = ad.Sphere().Radius(); fi.origin = A(ad.Sphere().Location().XYZ()); break;
    case GeomAbs_Torus: fi.kind = 4; break;
    default: fi.kind = 5; break;
    }
    GProp_GProps p;
    BRepGProp::SurfaceProperties(f, p);
    fi.area = p.Mass();
    fi.center = A(p.CentreOfMass().XYZ());
    Bnd_Box b;
    BRepBndLib::Add(f, b);
    double x0, y0, z0, x1, y1, z1;
    b.Get(x0, y0, z0, x1, y1, z1);
    fi.bbox_min = {x0, y0, z0};
    fi.bbox_max = {x1, y1, z1};
    return fi;
    OCC_CATCH("face_info")
}

MeshData mesh(const Shape &s, double lin, double ang) {
    OCC_TRY
    MeshData md;
    if (s.s.IsNull()) return md;
    BRepMesh_IncrementalMesh im(s.s, lin, false, ang, true);
    TopTools_IndexedMapOfShape faces;
    TopExp::MapShapes(s.s, TopAbs_FACE, faces);
    for (int fi = 1; fi <= faces.Extent(); ++fi) {
        TopoDS_Face f = TopoDS::Face(faces(fi));
        TopLoc_Location loc;
        Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(f, loc);
        if (tri.IsNull()) continue;
        gp_Trsf tr = loc.Transformation();
        bool rev = f.Orientation() == TopAbs_REVERSED;
        uint32_t base = (uint32_t)(md.positions.size() / 3);
        if (!tri->HasNormals()) tri->ComputeNormals();
        for (int i = 1; i <= tri->NbNodes(); ++i) {
            gp_Pnt p = tri->Node(i).Transformed(tr);
            md.positions.push_back((float)p.X());
            md.positions.push_back((float)p.Y());
            md.positions.push_back((float)p.Z());
            gp_Dir n = tri->Normal(i);
            n.Transform(tr);
            if (rev) n.Reverse();
            md.normals.push_back((float)n.X());
            md.normals.push_back((float)n.Y());
            md.normals.push_back((float)n.Z());
        }
        for (int i = 1; i <= tri->NbTriangles(); ++i) {
            int a, b, c;
            tri->Triangle(i).Get(a, b, c);
            if (rev) std::swap(b, c);
            md.indices.push_back(base + a - 1);
            md.indices.push_back(base + b - 1);
            md.indices.push_back(base + c - 1);
            md.tri_face.push_back((uint32_t)fi);
        }
    }
    TopTools_IndexedMapOfShape edges;
    TopExp::MapShapes(s.s, TopAbs_EDGE, edges);
    md.edge_offsets.push_back(0);
    for (int ei = 1; ei <= edges.Extent(); ++ei) {
        TopoDS_Edge e = TopoDS::Edge(edges(ei));
        if (!BRep_Tool::Degenerated(e)) {
            try {
                BRepAdaptor_Curve c(e);
                GCPnts_TangentialDeflection td(c, ang, lin);
                for (int i = 1; i <= td.NbPoints(); ++i) {
                    gp_Pnt p = td.Value(i);
                    md.edge_points.push_back((float)p.X());
                    md.edge_points.push_back((float)p.Y());
                    md.edge_points.push_back((float)p.Z());
                }
            } catch (const Standard_Failure &) {
            }
        }
        md.edge_offsets.push_back((uint32_t)(md.edge_points.size() / 3));
    }
    return md;
    OCC_CATCH("mesh")
}

void write_step(const Shape &s, rust::Str path) {
    OCC_TRY
    quiet();
    need(s, "write_step");
    STEPControl_Writer w;
    Interface_Static::SetCVal("write.step.schema", "AP214IS");
    if (w.Transfer(s.s, STEPControl_AsIs) != IFSelect_RetDone) throw std::runtime_error("STEP transfer failed");
    std::string p(path);
    if (w.Write(p.c_str()) != IFSelect_RetDone) throw std::runtime_error("STEP write failed: " + p);
    OCC_CATCH("write_step")
}

void write_stl(const Shape &s, rust::Str path, double lin) {
    OCC_TRY
    need(s, "write_stl");
    BRepMesh_IncrementalMesh im(s.s, lin, false, 0.5, true);
    StlAPI_Writer w;
    w.ASCIIMode() = false;
    std::string p(path);
    if (!w.Write(s.s, p.c_str())) throw std::runtime_error("STL write failed: " + p);
    OCC_CATCH("write_stl")
}

void write_brep(const Shape &s, rust::Str path) {
    OCC_TRY
    std::string p(path);
    if (!BRepTools::Write(s.s, p.c_str())) throw std::runtime_error("BREP write failed: " + p);
    OCC_CATCH("write_brep")
}

std::unique_ptr<Shape> read_step(rust::Str path) {
    OCC_TRY
    quiet();
    STEPControl_Reader r;
    std::string p(path);
    if (r.ReadFile(p.c_str()) != IFSelect_RetDone) throw std::runtime_error("STEP read failed: " + p);
    r.TransferRoots();
    return wrap(r.OneShape());
    OCC_CATCH("read_step")
}

} // namespace fcocc
