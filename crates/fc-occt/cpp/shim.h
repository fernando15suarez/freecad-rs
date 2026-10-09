#pragma once
#include "rust/cxx.h"
#include <memory>
#include <vector>
#include <array>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>

namespace fcocc {

struct MeshData;
struct FaceInfo;

struct Shape {
    TopoDS_Shape s;
};

struct FaceGroup {
    std::vector<TopoDS_Wire> wires;
};

struct WireBuilder {
    std::vector<TopoDS_Edge> edges;
    std::vector<FaceGroup> faces;
    std::vector<TopoDS_Wire> loose;
    void add_line(std::array<double, 3> a, std::array<double, 3> b);
    void add_arc(std::array<double, 3> c, std::array<double, 3> n, std::array<double, 3> x, double r, double a0, double a1);
    void add_circle(std::array<double, 3> c, std::array<double, 3> n, std::array<double, 3> x, double r);
    void end_wire();
    void begin_face();
    std::unique_ptr<Shape> build_faces();
    std::unique_ptr<Shape> build_wires();
};

int occt_version();
std::unique_ptr<WireBuilder> new_wire_builder();

std::unique_ptr<Shape> prism(const Shape &s, std::array<double, 3> d);
std::unique_ptr<Shape> revol(const Shape &s, std::array<double, 3> origin, std::array<double, 3> dir, double angle);
std::unique_ptr<Shape> fuse(const Shape &a, const Shape &b);
std::unique_ptr<Shape> cut(const Shape &a, const Shape &b);
std::unique_ptr<Shape> common(const Shape &a, const Shape &b);
std::unique_ptr<Shape> translated(const Shape &s, std::array<double, 3> d);
std::unique_ptr<Shape> rotated(const Shape &s, std::array<double, 3> origin, std::array<double, 3> dir, double angle);
std::unique_ptr<Shape> loft(const Shape &sections, bool solid, bool ruled);
std::unique_ptr<Shape> sweep(const Shape &profile, const Shape &spine, bool frenet, int transition);
std::unique_ptr<Shape> make_compound(const Shape &parts, const Shape &add);
std::unique_ptr<Shape> empty_shape();
std::unique_ptr<Shape> clone_shape(const Shape &s);
std::unique_ptr<Shape> refine(const Shape &s);

bool is_null(const Shape &s);
bool is_valid(const Shape &s);
double volume(const Shape &s);
double area(const Shape &s);
std::array<double, 6> bbox(const Shape &s);
size_t count_solids(const Shape &s);
size_t count_faces(const Shape &s);
size_t count_edges(const Shape &s);
FaceInfo face_info(const Shape &s, size_t index);
MeshData mesh(const Shape &s, double lin, double ang);

void write_step(const Shape &s, rust::Str path);
void write_stl(const Shape &s, rust::Str path, double lin);
void write_brep(const Shape &s, rust::Str path);
std::unique_ptr<Shape> read_step(rust::Str path);

} // namespace fcocc
