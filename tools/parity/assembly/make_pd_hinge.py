import FreeCAD as App, Part, Sketcher
import JointObject, UtilsAssembly
import os
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pd_hinge.FCStd")
doc = App.newDocument("pd_hinge")
def body(name, w, h, t):
    b = doc.addObject("PartDesign::Body", name)
    sk = b.newObject("Sketcher::SketchObject", name + "Sketch")
    sk.AttachmentSupport = [(b.Origin.OriginFeatures[3], "")]
    sk.MapMode = "FlatFace"
    pts = [App.Vector(0,0,0), App.Vector(w,0,0), App.Vector(w,h,0), App.Vector(0,h,0)]
    for i in range(4):
        sk.addGeometry(Part.LineSegment(pts[i], pts[(i+1)%4]))
    sk.addGeometry(Part.Circle(App.Vector(5, h/2, 0), App.Vector(0,0,1), 3))
    pad = b.newObject("PartDesign::Pad", name + "Pad"); pad.Profile = sk; pad.Length = t
    return b
a = body("Base", 60, 30, 10)
c = body("Lever", 50, 10, 5)
doc.recompute()
asm = doc.addObject("Assembly::AssemblyObject", "Assembly")
jg = asm.newObject("Assembly::JointGroup", "Joints")
la = asm.newObject("App::Link", "Base"); la.LinkedObject = a
lc = asm.newObject("App::Link", "Lever"); lc.LinkedObject = c
lc.Placement = App.Placement(App.Vector(20, 40, 12), App.Rotation(30, 0, 0))
doc.recompute()
g = jg.newObject("App::FeaturePython", "GroundedJoint")
JointObject.GroundedJoint(g, la)
j = jg.newObject("App::FeaturePython", "Revolute")
JointObject.Joint(j, 1)
# hole edges: find circular edge on top of Base and bottom of Lever
def circ(shape, z):
    for i, e in enumerate(shape.Edges):
        if hasattr(e.Curve, "Radius") and abs(e.Curve.Center.z - z) < 1e-6:
            return "Edge%d" % (i + 1)
ea = circ(a.Shape, 10); ec = circ(c.Shape, 0)
print("edges", ea, ec)
j.Proxy.setJointConnectors(j, [[la, [ea, ea]], [lc, [ec, ec]]])
asm.solve()
doc.recompute()
doc.saveAs(OUT)
print("saved")
