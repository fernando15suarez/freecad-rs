# Builds the parity reference models with FreeCAD and records FreeCAD's measurements.
# Run through tools/parity/run.sh (sets PARITY_DIR); optional PARITY_ONLY=name1,name2.
#
# Each model function returns (doc, body); the driver saves models/<name>.FCStd and
# models/<name>.json (see fcparity.measure for the JSON fields).

import math
import os
import sys

HERE = os.environ.get("PARITY_DIR") or os.getcwd()
sys.dont_write_bytecode = True
sys.path.insert(0, HERE)

import FreeCAD as App  # noqa: E402
import Spreadsheet  # noqa: E402,F401

from fcparity import (  # noqa: E402
    C, V, anchored_circle, arc, circle, face_toward, line, new_body, new_sketch, point, rect, save,
)

MODELS = []


def model(fn):
    MODELS.append(fn)
    return fn


def pad_of(body, sk, name="Pad", length=5.0, **props):
    pad = body.newObject("PartDesign::Pad", name)
    pad.Profile = sk
    pad.Length = length
    for k, v in props.items():
        setattr(pad, k, v)
    return pad


def base_box(body, w=30, h=20, t=10):
    sk = new_sketch(body, "Sketch")
    rect(sk, 0, 0, w, h)
    return pad_of(body, sk, length=t)


@model
def pad_rect():
    doc, body = new_body("pad_rect")
    sk = new_sketch(body)
    rect(sk, 0, 0, 20, 10)
    pad_of(body, sk, length=5)
    return doc, body


@model
def pad_two_lengths():
    doc, body = new_body("pad_two_lengths")
    sk = new_sketch(body)
    rect(sk, -5, -3, 20, 10)
    pad_of(body, sk, length=5, SideType="Two sides", Length2=3)
    return doc, body


@model
def pad_symmetric_xz():
    doc, body = new_body("pad_symmetric_xz")
    sk = new_sketch(body, plane="XZ_Plane")
    rect(sk, 2, 1, 12, 6)
    pad_of(body, sk, length=8, SideType="Symmetric")
    return doc, body


@model
def pad_reversed_yz():
    doc, body = new_body("pad_reversed_yz")
    sk = new_sketch(body, plane="YZ_Plane")
    rect(sk, 1, 2, 7, 4)
    pad_of(body, sk, length=6, Reversed=True)
    return doc, body


@model
def pocket_through_all_face():
    doc, body = new_body("pocket_through_all_face")
    pad = base_box(body)
    sk = new_sketch(body, "Sketch001", face=face_toward(pad, (0, 0, 1)))
    anchored_circle(sk, (15, 10), 4)
    pk = body.newObject("PartDesign::Pocket", "Pocket")
    pk.Profile = sk
    pk.Type = "ThroughAll"
    return doc, body


@model
def pocket_length_offset_face():
    # Pocket of a rectangle on the side face (normal -Y) of a box, 4 mm deep.
    doc, body = new_body("pocket_length_offset_face")
    pad = base_box(body)
    sk = new_sketch(body, "Sketch001", face=face_toward(pad, (0, -1, 0)))
    g = rect(sk, 0, 0, 1, 1, dims=False, anchor=False)
    sk.Geometry  # noqa
    # Constrain in the face's own frame: place with Distance constraints from the origin.
    sk.addConstraint(C("DistanceX", g[0], 1, g[0], 2, 10))
    sk.addConstraint(C("DistanceY", g[1], 1, g[1], 2, 4))
    sk.addConstraint(C("DistanceX", -1, 1, g[0], 1, 5))
    sk.addConstraint(C("DistanceY", -1, 1, g[0], 1, 3))
    pk = body.newObject("PartDesign::Pocket", "Pocket")
    pk.Profile = sk
    pk.Length = 4
    return doc, body


@model
def face_frames():
    # Sketches on faces whose normals are -Z, +Y and -X: FreeCAD's FlatFace frame
    # (x = Z x n, or x = +-X when n is along Z) and asymmetric profiles expose any
    # in-plane rotation between FreeCAD's sketch frame and ours.
    doc, body = new_body("face_frames")
    pad = base_box(body)
    sk = new_sketch(body, "Sketch001", face=face_toward(pad, (0, 0, -1)))
    rect(sk, 5, -15, 6, 4)  # frame: x = +X, y = -Y
    p1 = body.newObject("PartDesign::Pocket", "Pocket")
    p1.Profile = sk
    p1.Length = 2
    sk = new_sketch(body, "Sketch002", face=face_toward(p1, (0, 1, 0)))
    rect(sk, -12, 2, 5, 3)  # frame: x = -X, y = +Z
    p2 = body.newObject("PartDesign::Pocket", "Pocket001")
    p2.Profile = sk
    p2.Length = 3
    sk = new_sketch(body, "Sketch003", face=face_toward(p2, (-1, 0, 0)))
    rect(sk, -15, 1, 3, 6)  # frame: x = -Y, y = +Z
    p3 = body.newObject("PartDesign::Pocket", "Pocket002")
    p3.Profile = sk
    p3.Length = 4
    return doc, body


@model
def pad_map_reversed():
    # MapReversed flips FreeCAD's sketch X and normal (Y kept), plus an attachment offset.
    doc, body = new_body("pad_map_reversed")
    sk = new_sketch(body, offset=2, reversed=True)
    rect(sk, 2, 1, 8, 3)
    line(sk, (0, 0), (5, 9), construction=True)
    pad_of(body, sk, length=4)
    # XZ reversed: x = -X, y = +Z, normal = +Y; overlaps the first pad.
    sk2 = new_sketch(body, "Sketch001", plane="XZ_Plane", reversed=True)
    rect(sk2, 1, -6, 4, 2)
    pad_of(body, sk2, "Pad001", length=6)
    return doc, body


@model
def multi_solid_body():
    # Two disjoint pads: FreeCAD 1.1 keeps a multi-solid Body (AllowCompound).
    doc, body = new_body("multi_solid_body")
    sk = new_sketch(body)
    rect(sk, 0, 0, 10, 10)
    pad_of(body, sk, length=5)
    sk2 = new_sketch(body, "Sketch001")
    rect(sk2, 20, 0, 10, 10)
    pad_of(body, sk2, "Pad001", length=5)
    return doc, body


@model
def pad_up_to_first():
    # Pad from a sketch 20 mm above a box, reversed (downwards) up to the box's top face.
    doc, body = new_body("pad_up_to_first")
    base_box(body)
    sk = new_sketch(body, "Sketch001", offset=20)
    rect(sk, 5, 5, 8, 6)
    pad_of(body, sk, "Pad001", Type="UpToFirst", Reversed=True)
    return doc, body


@model
def pocket_two_lengths():
    doc, body = new_body("pocket_two_lengths")
    base_box(body)
    sk = new_sketch(body, "Sketch001", offset=5)
    anchored_circle(sk, (10, 10), 3)
    pk = body.newObject("PartDesign::Pocket", "Pocket")
    pk.Profile = sk
    if hasattr(pk, "SideType"):
        pk.SideType = "Two sides"
    else:
        pk.Type = "TwoLengths"
    pk.Length = 2
    pk.Length2 = 3
    return doc, body


@model
def revolution_two_angles():
    doc, body = new_body("revolution_two_angles")
    sk = new_sketch(body, plane="XZ_Plane")
    rect(sk, 4, 0, 6, 10)
    rv = body.newObject("PartDesign::Revolution", "Revolution")
    rv.Profile = sk
    rv.ReferenceAxis = (sk, ["V_Axis"])
    if hasattr(rv, "SideType"):
        rv.SideType = "Two sides"
    else:
        rv.Type = "TwoAngles"
    rv.Angle = 60
    rv.Angle2 = 30
    return doc, body


@model
def groove_symmetric_partial():
    doc, body = new_body("groove_symmetric_partial")
    sk = new_sketch(body)
    anchored_circle(sk, (0, 0), 10)
    pad_of(body, sk, length=20)
    sk2 = new_sketch(body, "Sketch001", plane="XZ_Plane")
    rect(sk2, 6, 5, 6, 4)
    gr = body.newObject("PartDesign::Groove", "Groove")
    gr.Profile = sk2
    gr.ReferenceAxis = (sk2, ["V_Axis"])
    gr.Angle = 90
    if hasattr(gr, "SideType"):
        gr.SideType = "Symmetric"
    else:
        gr.Midplane = True
    gr.Reversed = True
    return doc, body


@model
def revolution_v_axis():
    doc, body = new_body("revolution_v_axis")
    sk = new_sketch(body, plane="XZ_Plane")
    rect(sk, 5, 0, 5, 20)
    rv = body.newObject("PartDesign::Revolution", "Revolution")
    rv.Profile = sk
    rv.ReferenceAxis = (sk, ["V_Axis"])
    rv.Angle = 360
    return doc, body


@model
def revolution_construction_line():
    doc, body = new_body("revolution_construction_line")
    sk = new_sketch(body)
    # Construction axis: line from (0,0) to (10,10) at 45 deg.
    ax = line(sk, (0, 0), (10, 10), construction=True)
    sk.addConstraint(C("Coincident", ax, 1, -1, 1))
    sk.addConstraint(C("Angle", ax, math.radians(45)))
    sk.addConstraint(C("Distance", ax, math.hypot(10, 10)))
    # Triangle profile to the right of the axis.
    a, b, c = (6, 2), (12, 4), (9, 0)
    g = [line(sk, a, b), line(sk, b, c), line(sk, c, a)]
    for i in range(3):
        sk.addConstraint(C("Coincident", g[i], 2, g[(i + 1) % 3], 1))
    sk.addConstraint(C("DistanceX", -1, 1, g[0], 1, a[0]))
    sk.addConstraint(C("DistanceY", -1, 1, g[0], 1, a[1]))
    sk.addConstraint(C("DistanceX", -1, 1, g[1], 1, b[0]))
    sk.addConstraint(C("DistanceY", -1, 1, g[1], 1, b[1]))
    sk.addConstraint(C("PointOnObject", g[2], 1, -1))
    sk.addConstraint(C("DistanceX", -1, 1, g[2], 1, c[0]))
    rv = body.newObject("PartDesign::Revolution", "Revolution")
    rv.Profile = sk
    rv.ReferenceAxis = (sk, ["Axis0"])  # construction lines are exposed as Axis<n>
    rv.Angle = 270
    return doc, body


@model
def revolution_symmetric_h_axis():
    doc, body = new_body("revolution_symmetric_h_axis")
    sk = new_sketch(body)
    rect(sk, 0, 3, 10, 4)
    rv = body.newObject("PartDesign::Revolution", "Revolution")
    rv.Profile = sk
    rv.ReferenceAxis = (sk, ["H_Axis"])
    rv.Angle = 120
    if hasattr(rv, "SideType"):
        rv.SideType = "Symmetric"
    else:
        rv.Midplane = True
    return doc, body


@model
def groove_ring():
    doc, body = new_body("groove_ring")
    sk = new_sketch(body)
    anchored_circle(sk, (0, 0), 10)
    pad_of(body, sk, length=20)
    sk2 = new_sketch(body, "Sketch001", plane="XZ_Plane")
    rect(sk2, 8, 8, 4, 4)
    gr = body.newObject("PartDesign::Groove", "Groove")
    gr.Profile = sk2
    gr.ReferenceAxis = (sk2, ["V_Axis"])
    gr.Angle = 360
    return doc, body


def hole_model(name, cut):
    doc, body = new_body(name)
    pad = base_box(body)
    sk = new_sketch(body, "Sketch001", face=face_toward(pad, (0, 0, 1)))
    anchored_circle(sk, (15, 10), 3)
    h = body.newObject("PartDesign::Hole", "Hole")
    h.Profile = sk
    h.ThreadType = "None"
    h.Diameter = 6
    h.DepthType = "Dimension"
    h.Depth = 6
    h.DrillPoint = "Angled"
    h.DrillPointAngle = 118
    h.HoleCutType = cut
    h.HoleCutCustomValues = True
    if cut == "Counterbore":
        h.HoleCutDiameter = 11
        h.HoleCutDepth = 3
    elif cut == "Countersink":
        h.HoleCutDiameter = 12
        h.HoleCutCountersinkAngle = 90
    return doc, body


@model
def hole_counterbore():
    return hole_model("hole_counterbore", "Counterbore")


@model
def hole_countersink():
    return hole_model("hole_countersink", "Countersink")


@model
def hole_through_flat():
    doc, body = hole_model("hole_through_flat", "None")
    h = doc.getObject("Hole")
    h.DepthType = "ThroughAll"
    h.DrillPoint = "Flat"
    return doc, body


@model
def loft_rect_to_circle():
    doc, body = new_body("loft_rect_to_circle")
    sk = new_sketch(body)
    rect(sk, -10, -6, 20, 12)
    sk2 = new_sketch(body, "Sketch001", offset=20)
    anchored_circle(sk2, (0, 0), 5)
    lo = body.newObject("PartDesign::AdditiveLoft", "AdditiveLoft")
    lo.Profile = sk
    lo.Sections = [sk2]
    lo.Ruled = False
    return doc, body


@model
def loft_ruled_rects():
    doc, body = new_body("loft_ruled_rects")
    sk = new_sketch(body)
    rect(sk, -10, -10, 20, 20)
    sk2 = new_sketch(body, "Sketch001", offset=15)
    rect(sk2, -4, -6, 8, 12)
    lo = body.newObject("PartDesign::AdditiveLoft", "AdditiveLoft")
    lo.Profile = sk
    lo.Sections = [sk2]
    lo.Ruled = True
    return doc, body


def l_pipe(name, transition):
    doc, body = new_body(name)
    prof = new_sketch(body, "Profile")
    anchored_circle(prof, (0, 0), 2)
    sp = new_sketch(body, "Spine", plane="XZ_Plane")
    a = line(sp, (0, 0), (0, 30))
    b = line(sp, (0, 30), (20, 30))
    sp.addConstraint(C("Coincident", a, 1, -1, 1))
    sp.addConstraint(C("PointOnObject", a, 2, -2))
    sp.addConstraint(C("Coincident", a, 2, b, 1))
    sp.addConstraint(C("Horizontal", b))
    sp.addConstraint(C("DistanceY", a, 1, a, 2, 30))
    sp.addConstraint(C("DistanceX", b, 1, b, 2, 20))
    pp = body.newObject("PartDesign::AdditivePipe", "AdditivePipe")
    pp.Profile = prof
    pp.Spine = (sp, ["Edge1", "Edge2"])
    pp.Transition = transition
    return doc, body


@model
def pipe_l_path():
    return l_pipe("pipe_l_path", "Right corner")


@model
def pipe_l_path_transformed():
    # FreeCAD's default transition at a sharp corner.
    return l_pipe("pipe_l_path_transformed", "Transformed")


@model
def pipe_arc_path():
    # Smooth path: straight, quarter arc (tangent), straight.
    doc, body = new_body("pipe_arc_path")
    prof = new_sketch(body, "Profile")
    anchored_circle(prof, (0, 0), 2)
    sp = new_sketch(body, "Spine", plane="XZ_Plane")
    a = line(sp, (0, 0), (0, 20))
    r = arc(sp, (10, 20), 10, math.pi / 2, math.pi)
    b = line(sp, (10, 30), (25, 30))
    sp.addConstraint(C("Coincident", a, 1, -1, 1))
    sp.addConstraint(C("PointOnObject", a, 2, -2))
    sp.addConstraint(C("Tangent", a, 2, r, 2))
    sp.addConstraint(C("Tangent", r, 1, b, 1))
    sp.addConstraint(C("Radius", r, 10))
    sp.addConstraint(C("Horizontal", b))
    sp.addConstraint(C("DistanceY", a, 1, a, 2, 20))
    sp.addConstraint(C("DistanceX", b, 1, b, 2, 15))
    pp = body.newObject("PartDesign::AdditivePipe", "AdditivePipe")
    pp.Profile = prof
    pp.Spine = (sp, ["Edge1", "Edge2", "Edge3"])
    return doc, body


@model
def slot_tangent_arcs():
    doc, body = new_body("slot_tangent_arcs")
    sk = new_sketch(body)
    # Slot: two horizontal lines joined by two semicircles, endpoint-tangent.
    l1 = line(sk, (0, -5), (30, -5))
    a1 = arc(sk, (30, 0), 5, -math.pi / 2, math.pi / 2)
    l2 = line(sk, (30, 5), (0, 5))
    a2 = arc(sk, (0, 0), 5, math.pi / 2, 3 * math.pi / 2)
    sk.addConstraint(C("Tangent", l1, 2, a1, 1))
    sk.addConstraint(C("Tangent", a1, 2, l2, 1))
    sk.addConstraint(C("Tangent", l2, 2, a2, 1))
    sk.addConstraint(C("Tangent", a2, 2, l1, 1))
    sk.addConstraint(C("Horizontal", l1))
    sk.addConstraint(C("Equal", a1, a2))
    sk.addConstraint(C("Coincident", a2, 3, -1, 1))
    sk.addConstraint(C("Radius", a1, 5))
    sk.addConstraint(C("Distance", a2, 3, a1, 3, 30))
    pad_of(body, sk, length=4)
    return doc, body


@model
def sketch_hole_loop():
    doc, body = new_body("sketch_hole_loop")
    sk = new_sketch(body)
    rect(sk, 0, 0, 40, 25)
    anchored_circle(sk, (12, 12.5), 5)
    g = rect(sk, 24, 6, 10, 12)
    pad_of(body, sk, length=6)
    return doc, body


@model
def expressions_named():
    doc, body = new_body("expressions_named")
    sheet = doc.addObject("Spreadsheet::Sheet", "Spreadsheet")
    sheet.set("A1", "4")
    sheet.setAlias("A1", "thick")
    sheet.set("A2", "=thick * 2 + 1")
    sheet.setAlias("A2", "depth")
    sk = new_sketch(body)
    g = rect(sk, 0, 0, 24, 10)
    # rect adds: 4 coincident, 2 horizontal, 2 vertical, DistanceX (#8), DistanceY (#9), root (#10)
    sk.renameConstraint(8, "width")
    sk.renameConstraint(9, "height")
    sk.setExpression("Constraints.height", ".Constraints.width / 2")
    pad = pad_of(body, sk, length=5)
    pad.setExpression("Length", "Spreadsheet.thick * 1 mm + Sketch.Constraints.width / 8")
    sk2 = new_sketch(body, "Sketch001", face=face_toward(pad, (0, 0, 1)))
    anchored_circle(sk2, (12, 6), 2)
    pk = body.newObject("PartDesign::Pocket", "Pocket")
    pk.Profile = sk2
    pk.setExpression("Length", "<<Spreadsheet>>.depth * 1mm / 3")
    return doc, body


@model
def sketch_constraint_zoo():
    # Many constraint kinds in one fully constrained sketch: DOF + solved geometry parity.
    doc, body = new_body("sketch_constraint_zoo")
    sk = new_sketch(body)
    # Locked point (FreeCAD "Lock" = DistanceX + DistanceY with a single element).
    p = point(sk, (-10, -10))
    sk.addConstraint(C("DistanceX", p, 1, -10))
    sk.addConstraint(C("DistanceY", p, 1, -10))
    # Line with length (Distance on one edge) and absolute angle.
    l0 = line(sk, (0, 0), (10, 5))
    sk.addConstraint(C("Coincident", l0, 1, -1, 1))
    sk.addConstraint(C("Distance", l0, 12))
    sk.addConstraint(C("Angle", l0, math.radians(30)))
    # Perpendicular line from l0's end, given length; reference (non-driving) length too.
    l1 = line(sk, (10, 5), (7, 12))
    sk.addConstraint(C("Coincident", l0, 2, l1, 1))
    sk.addConstraint(C("Perpendicular", l0, l1))
    sk.addConstraint(C("Distance", l1, 1, l1, 2, 6))
    ref = sk.addConstraint(C("DistanceX", l1, 1, l1, 2, -3))
    sk.setDriving(ref, False)
    # Parallel + equal line, positioned by a point-to-line distance and DistanceY.
    l2 = line(sk, (2, -6), (13, 0))
    sk.addConstraint(C("Parallel", l0, l2))
    sk.addConstraint(C("Equal", l0, l2))
    sk.addConstraint(C("Distance", l2, 1, l0, 5))
    sk.addConstraint(C("DistanceX", -1, 1, l2, 1, 3))
    # Circle tangent to l2 (edge-edge), center on V axis... use PointOnObject on H axis.
    c0 = circle(sk, (20, -8), 3)
    sk.addConstraint(C("Tangent", l2, c0))
    sk.addConstraint(C("Diameter", c0, 6))
    sk.addConstraint(C("PointOnObject", c0, 3, -1))
    # Arc with endpoint tangent to l1's end, radius and sweep angle.
    a0 = arc(sk, (3, 11), 4, 0.1, 2.0)
    sk.addConstraint(C("Tangent", l1, 2, a0, 1))
    sk.addConstraint(C("Radius", a0, 4))
    sk.addConstraint(C("Angle", a0, math.radians(100)))
    # Two lines at an angle (GUI style: angle via endpoints), symmetric endpoints.
    l3 = line(sk, (-20, 0), (-12, 2))
    l4 = line(sk, (-20, 0), (-14, 7))
    sk.addConstraint(C("Coincident", l3, 1, l4, 1))
    sk.addConstraint(C("Angle", l3, 1, l4, 1, math.radians(35)))
    sk.addConstraint(C("DistanceX", -1, 1, l3, 1, -20))
    sk.addConstraint(C("DistanceY", -1, 1, l3, 1, 0))
    sk.addConstraint(C("Horizontal", l3))
    sk.addConstraint(C("Distance", l3, 9))
    sk.addConstraint(C("Distance", l4, 9))
    # Two points symmetric about l4, and two about the origin.
    q0 = point(sk, (-13, 0))
    q1 = point(sk, (-17, 6))
    sk.addConstraint(C("Symmetric", q0, 1, q1, 1, l4))
    sk.addConstraint(C("DistanceX", -1, 1, q0, 1, -13))
    sk.addConstraint(C("PointOnObject", q0, 1, l3))
    s0 = point(sk, (5, 20))
    s1 = point(sk, (-5, -20))
    sk.addConstraint(C("Symmetric", s0, 1, s1, 1, -1, 1))
    sk.addConstraint(C("Vertical", s0, 1, l1, 2))
    sk.addConstraint(C("DistanceY", -1, 1, s0, 1, 20))
    # A blocked circle.
    b0 = circle(sk, (30, 20), 2)
    sk.addConstraint(C("Block", b0))
    # Endpoint perpendicularity (FreeCAD: one constraint; fc-part: coincident + perpendicular).
    l5 = line(sk, (25, 0), (30, 0))
    l6 = line(sk, (30, 0), (30, 6))
    sk.addConstraint(C("Perpendicular", l5, 2, l6, 1))
    sk.addConstraint(C("Horizontal", l5))
    sk.addConstraint(C("DistanceX", -1, 1, l5, 1, 25))
    sk.addConstraint(C("DistanceY", -1, 1, l5, 1, 0))
    sk.addConstraint(C("Distance", l5, 5))
    sk.addConstraint(C("Distance", l6, 6))
    # Endpoint-to-curve tangency.
    l7 = line(sk, (34, 7), (40, 7))
    c1 = circle(sk, (40, 5), 2)
    sk.addConstraint(C("Tangent", l7, 2, c1))
    sk.addConstraint(C("Horizontal", l7))
    sk.addConstraint(C("DistanceX", -1, 1, l7, 1, 34))
    sk.addConstraint(C("DistanceY", -1, 1, l7, 1, 7))
    sk.addConstraint(C("Distance", l7, 6))
    sk.addConstraint(C("Radius", c1, 2))
    # A clockwise arc (normal -Z) with points on its start and end.
    import Part
    cw = sk.addGeometry(Part.ArcOfCircle(Part.Circle(V(50, 0, 0), V(0, 0, -1), 3), 0.2, 1.4), False)
    sk.addConstraint(C("Block", cw))
    ps = point(sk, (52, 1))
    pe = point(sk, (51, 2))
    sk.addConstraint(C("Coincident", ps, 1, cw, 1))
    sk.addConstraint(C("Coincident", pe, 1, cw, 2))
    return doc, body


@model
def sketch_conflicting():
    doc, body = new_body("sketch_conflicting")
    sk = new_sketch(body)
    g = rect(sk, 0, 0, 20, 10)
    sk.addConstraint(C("DistanceX", g[2], 2, g[2], 1, 25))  # top width 25 vs bottom 20
    sk.addConstraint(C("Distance", g[3], 12))  # left height 12 vs right 10
    return doc, body


@model
def sketch_redundant():
    doc, body = new_body("sketch_redundant")
    sk = new_sketch(body)
    g = rect(sk, 0, 0, 20, 10)
    sk.addConstraint(C("Parallel", g[0], g[2]))  # already both horizontal
    sk.addConstraint(C("DistanceX", g[2], 2, g[2], 1, 20))  # consistent duplicate width
    pad_of(body, sk, length=3)
    return doc, body


@model
def sketch_underconstrained():
    doc, body = new_body("sketch_underconstrained")
    sk = new_sketch(body)
    g = rect(sk, 0, 0, 20, 10, dims=False)
    circle(sk, (10, 5), 2)
    arc(sk, (30, 0), 3, 0, 1, construction=True)
    pad_of(body, sk, length=2)
    return doc, body


@model
def multi_feature_part():
    # Pad, pocket on top face, revolution-free stack: pad on top face, then a hole.
    doc, body = new_body("multi_feature_part")
    pad = base_box(body, 40, 30, 8)
    sk2 = new_sketch(body, "Sketch001", face=face_toward(pad, (0, 0, 1)))
    rect(sk2, 5, 5, 15, 20)
    pk = body.newObject("PartDesign::Pocket", "Pocket")
    pk.Profile = sk2
    pk.Length = 3
    sk3 = new_sketch(body, "Sketch002", face=face_toward(pk, (0, 0, 1)))
    anchored_circle(sk3, (30, 15), 6)
    pad2 = pad_of(body, sk3, "Pad001", length=10)
    sk4 = new_sketch(body, "Sketch003", face=face_toward(pad2, (0, 0, 1)))
    anchored_circle(sk4, (30, 15), 2)
    h = body.newObject("PartDesign::Hole", "Hole")
    h.Profile = sk4
    h.ThreadType = "None"
    h.Diameter = 4
    h.DepthType = "Dimension"
    h.Depth = 12
    h.DrillPoint = "Angled"
    return doc, body


def main():
    outdir = os.path.join(HERE, "models")
    os.makedirs(outdir, exist_ok=True)
    only = [s for s in os.environ.get("PARITY_ONLY", "").split(",") if s]
    for fn in MODELS:
        if only and fn.__name__ not in only:
            continue
        doc, body = fn()
        m = save(doc, body, outdir, fn.__name__)
        sol = m["solid"]
        vol = "%.6f" % sol["volume"] if sol else "-"
        errs = [f["name"] + ": " + f["error"] for f in m["features"] if f["error"]]
        print("PARITY %-32s volume=%s %s" % (fn.__name__, vol, "; ".join(errs)))


main()
