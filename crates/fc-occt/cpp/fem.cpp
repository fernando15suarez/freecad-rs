// FEM services for freecad-rs: BooleanFragments with part history. Errors become
// std::runtime_error.
#include "fem.h"
#include "fc-occt/src/fem.rs.h"

#include <Standard_Failure.hxx>
#include <BRepAlgoAPI_BuilderAlgo.hxx>
#include <BRep_Builder.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>

#include <sstream>
#include <stdexcept>

namespace fcocc {

namespace {
[[noreturn]] void fem_fail(const Standard_Failure &e, const char *where) {
    std::ostringstream os;
    os << where << ": " << e.DynamicType()->Name();
    if (e.GetMessageString() && *e.GetMessageString()) os << ": " << e.GetMessageString();
    throw std::runtime_error(os.str());
}
std::unique_ptr<Shape> fem_wrap(const TopoDS_Shape &s) {
    auto r = std::make_unique<Shape>();
    r->s = s;
    return r;
}
} // namespace

std::unique_ptr<Shape> fem_fragments(const Shape &args, bool fuse, double fuzzy, rust::Vec<int64_t> &history) {
    try {
        if (args.s.IsNull()) throw std::runtime_error("fem_fragments: null shape");
        TopTools_ListOfShape parts;
        if (args.s.ShapeType() == TopAbs_COMPOUND) {
            for (TopoDS_Iterator it(args.s); it.More(); it.Next()) parts.Append(it.Value());
        } else {
            parts.Append(args.s);
        }
        if (parts.IsEmpty()) throw std::runtime_error("fem_fragments: no parts");
        TopoDS_Shape result;
        BRepAlgoAPI_BuilderAlgo bop;
        if (fuse) {
            bop.SetArguments(parts);
            if (fuzzy > 0.0) bop.SetFuzzyValue(fuzzy);
            bop.SetRunParallel(Standard_False);
            bop.SetNonDestructive(Standard_True);
            bop.Build();
            if (!bop.IsDone() || bop.HasErrors()) {
                std::ostringstream os;
                bop.DumpErrors(os);
                throw std::runtime_error("BooleanFragments failed: " + os.str());
            }
            result = bop.Shape();
        } else {
            BRep_Builder bb;
            TopoDS_Compound c;
            bb.MakeCompound(c);
            for (TopTools_ListIteratorOfListOfShape it(parts); it.More(); it.Next()) bb.Add(c, it.Value());
            result = c;
        }
        TopTools_IndexedMapOfShape rsol, rface;
        TopExp::MapShapes(result, TopAbs_SOLID, rsol);
        TopExp::MapShapes(result, TopAbs_FACE, rface);
        int64_t pi = 0;
        for (TopTools_ListIteratorOfListOfShape it(parts); it.More(); it.Next(), ++pi) {
            for (int kind = 0; kind < 2; ++kind) {
                TopAbs_ShapeEnum t = kind == 0 ? TopAbs_SOLID : TopAbs_FACE;
                const TopTools_IndexedMapOfShape &rm = kind == 0 ? rsol : rface;
                TopTools_IndexedMapOfShape local;
                TopExp::MapShapes(it.Value(), t, local);
                for (int i = 1; i <= local.Extent(); ++i) {
                    const TopoDS_Shape &sub = local(i);
                    TopTools_ListOfShape images;
                    if (fuse) {
                        images = bop.Modified(sub);
                        if (images.IsEmpty() && !bop.IsDeleted(sub)) images.Append(sub);
                    } else {
                        images.Append(sub);
                    }
                    for (TopTools_ListIteratorOfListOfShape im(images); im.More(); im.Next()) {
                        int k = rm.FindIndex(im.Value());
                        if (k > 0) {
                            history.push_back(kind);
                            history.push_back(pi);
                            history.push_back(i);
                            history.push_back(k);
                        }
                    }
                }
            }
        }
        return fem_wrap(result);
    } catch (const Standard_Failure &e) {
        fem_fail(e, "fem_fragments");
    }
}

rust::Vec<size_t> fem_solid_faces(const Shape &s, size_t solid) {
    try {
        TopTools_IndexedMapOfShape sol, fac;
        TopExp::MapShapes(s.s, TopAbs_SOLID, sol);
        TopExp::MapShapes(s.s, TopAbs_FACE, fac);
        if (solid < 1 || (int)solid > sol.Extent()) throw std::runtime_error("fem_solid_faces: index out of range");
        TopTools_IndexedMapOfShape own;
        TopExp::MapShapes(sol((int)solid), TopAbs_FACE, own);
        rust::Vec<size_t> out;
        for (int i = 1; i <= own.Extent(); ++i) {
            int k = fac.FindIndex(own(i));
            if (k > 0) out.push_back((size_t)k);
        }
        return out;
    } catch (const Standard_Failure &e) {
        fem_fail(e, "fem_solid_faces");
    }
}

} // namespace fcocc
