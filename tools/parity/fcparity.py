# Helpers shared by the FreeCAD parity scripts (run inside FreeCAD's Python:
# `freecadcmd make_models.py`, see tools/parity/run.sh).
#
# Building blocks for PartDesign reference models plus `measure()`, which records what
# FreeCAD computes (volume, area, bbox, topology counts, per-sketch DOF and
# conflicting/redundant lists) so the Rust parity test can compare against it.

import json
import math

import FreeCAD as App
import Part
import Sketcher

V = App.Vector


def new_body(name):
    doc = App.newDocument(name)
    body = doc.addObject("PartDesign::Body", "Body")
    return doc, body


def origin_feature(body, role):
    for o in body.Origin.OriginFeatures:
        if o.Role == role:
            return o
    raise KeyError(role)


def new_sketch(body, name="Sketch", plane="XY_Plane", face=None, offset=0.0, reversed=False):
    """Sketch attached to an Origin plane, or to `face` = (feature, "FaceN")."""
    sk = body.newObject("Sketcher::SketchObject", name)
    if face is None:
        sk.AttachmentSupport = [(origin_feature(body, plane), "")]
    else:
        sk.AttachmentSupport = [face]
    sk.MapMode = "FlatFace"
    sk.MapReversed = reversed
    if offset:
        sk.AttachmentOffset = App.Placement(V(0, 0, offset), App.Rotation())
    return sk


def face_toward(feature, normal, extreme=True):
    """(feature, 'FaceN') of the planar face whose normal is `normal`, the outermost one."""
    feature.Document.recompute()
    n = V(*normal)
    best = None
    for i, f in enumerate(feature.Shape.Faces):
        if f.Surface.__class__.__name__ != "Plane":
            continue
        fn = f.normalAt(0, 0)
        if (fn - n).Length > 1e-6:
            continue
        h = f.CenterOfMass.dot(n)
        if best is None or h > best[0]:
            best = (h, i + 1)
    return (feature, "Face%d" % best[1])


def line(sk, a, b, construction=False):
    return sk.addGeometry(Part.LineSegment(V(a[0], a[1], 0), V(b[0], b[1], 0)), construction)


def circle(sk, c, r, construction=False):
    return sk.addGeometry(Part.Circle(V(c[0], c[1], 0), V(0, 0, 1), r), construction)


def arc(sk, c, r, a0, a1, construction=False):
    return sk.addGeometry(Part.ArcOfCircle(Part.Circle(V(c[0], c[1], 0), V(0, 0, 1), r), a0, a1), construction)


def point(sk, p):
    return sk.addGeometry(Part.Point(V(p[0], p[1], 0)), False)


def C(*args):
    return Sketcher.Constraint(*args)


def rect(sk, x0, y0, w, h, dims=True, anchor=True):
    """Closed rectangle of 4 lines (bottom, right, top, left), fully constrained when
    `dims` and `anchor` are set: corner (x0, y0) is locked relative to the origin."""
    p = [(x0, y0), (x0 + w, y0), (x0 + w, y0 + h), (x0, y0 + h)]
    g = [line(sk, p[i], p[(i + 1) % 4]) for i in range(4)]
    for i in range(4):
        sk.addConstraint(C("Coincident", g[i], 2, g[(i + 1) % 4], 1))
    sk.addConstraint(C("Horizontal", g[0]))
    sk.addConstraint(C("Horizontal", g[2]))
    sk.addConstraint(C("Vertical", g[1]))
    sk.addConstraint(C("Vertical", g[3]))
    if dims:
        sk.addConstraint(C("DistanceX", g[0], 1, g[0], 2, w))
        sk.addConstraint(C("DistanceY", g[1], 1, g[1], 2, h))
    if anchor:
        if x0 == 0 and y0 == 0:
            sk.addConstraint(C("Coincident", g[0], 1, -1, 1))
        else:
            sk.addConstraint(C("DistanceX", -1, 1, g[0], 1, x0))
            sk.addConstraint(C("DistanceY", -1, 1, g[0], 1, y0))
    return g


def anchored_circle(sk, c, r, construction=False):
    g = circle(sk, c, r, construction)
    sk.addConstraint(C("Radius", g, r))
    if c[0] == 0 and c[1] == 0:
        sk.addConstraint(C("Coincident", g, 3, -1, 1))
    else:
        sk.addConstraint(C("DistanceX", -1, 1, g, 3, c[0]))
        sk.addConstraint(C("DistanceY", -1, 1, g, 3, c[1]))
    return g


def _err(o):
    try:
        if not o.isValid():
            return o.getStatusString()
    except Exception as e:  # noqa
        return str(e)
    return None


def measure(doc, body):
    doc.recompute()
    out = {"freecad": ".".join(App.Version()[:3]), "features": [], "sketches": {}}
    s = body.Shape
    if s.isNull() or not s.Solids:
        out["solid"] = None
    else:
        bb = s.BoundBox
        out["solid"] = {
            "volume": s.Volume,
            "area": s.Area,
            "bbox": [bb.XMin, bb.YMin, bb.ZMin, bb.XMax, bb.YMax, bb.ZMax],
            "solids": len(s.Solids),
            "faces": len(s.Faces),
            "edges": len(s.Edges),
            "vertices": len(s.Vertexes),
            "valid": s.isValid(),
        }
    for o in body.Group:
        f = {"name": o.Name, "type": o.TypeId, "error": _err(o)}
        if o.TypeId.startswith("PartDesign::") and hasattr(o, "Shape") and not o.Shape.isNull():
            f["volume"] = o.Shape.Volume
        if o.TypeId == "Sketcher::SketchObject":
            rc = o.solve()
            pl = o.Placement
            out["sketches"][o.Name] = {
                "solve": rc,
                "dof": o.DoF,
                # FreeCAD reports 1-based constraint numbers; store 0-based indices.
                "conflicting": sorted(i - 1 for i in o.ConflictingConstraints),
                "redundant": sorted(i - 1 for i in o.RedundantConstraints),
                "partially_redundant": sorted(i - 1 for i in o.PartiallyRedundantConstraints),
                "malformed": sorted(i - 1 for i in o.MalformedConstraints),
                "constraints": len(o.Constraints),
                "geometry": len(o.Geometry),
                "placement": {
                    "origin": list(pl.Base),
                    "xdir": list(pl.Rotation.multVec(V(1, 0, 0))),
                    "ydir": list(pl.Rotation.multVec(V(0, 1, 0))),
                },
            }
        out["features"].append(f)
    return out


def save(doc, body, outdir, name):
    import os

    m = measure(doc, body)
    path = os.path.join(outdir, name + ".FCStd")
    if os.path.exists(path):
        os.remove(path)  # saveAs would leave a .FCBak backup next to it
    doc.saveAs(path)
    with open(os.path.join(outdir, name + ".json"), "w") as fh:
        json.dump(m, fh, indent=1, sort_keys=True)
    App.closeDocument(doc.Name)
    return m
