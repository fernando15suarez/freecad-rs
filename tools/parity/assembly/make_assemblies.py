# Build reference assemblies with FreeCAD 1.1's Assembly workbench (Ondsel solver) and dump
# their solved placements for crates/fc-assembly's parity test.
#
#   /Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd tools/parity/assembly/make_assemblies.py
#
# For every case this writes <case>.FCStd and appends to assemblies.json:
#   parts:   name, shape (box/cylinder dims), grounded, placement at creation
#   joints:  type, refs, Placement1/2 (JCS local to the part), Offset1/2, Distance, Angle,
#            and the referenced element's geometry (to check our JCS inference)
#   start:   placements right before the final solve (chosen per case, not satisfying the joints)
#   solved:  placements after assembly.solve()
import json
import math
import os

import FreeCAD as App
import Part
import JointObject
import UtilsAssembly

OUT = os.path.dirname(os.path.abspath(__file__))
TYPES = JointObject.JointTypes


def plc_json(p):
    q = p.Rotation.Q  # (x, y, z, w)
    return {"pos": [p.Base.x, p.Base.y, p.Base.z], "rot": [q[0], q[1], q[2], q[3]]}


def P(x=0, y=0, z=0, yaw=0, pitch=0, roll=0):
    return App.Placement(App.Vector(x, y, z), App.Rotation(yaw, pitch, roll))


def elem_geom(obj, name):
    """Geometry of a sub-element in the object's local frame (Shape without Placement)."""
    if not name or name.endswith("."):
        return None
    shape = obj.Shape.copy()
    shape.Placement = App.Placement()
    e = shape.getElement(name)
    v = lambda a: [a.x, a.y, a.z]
    if name.startswith("Vertex"):
        return {"type": "vertex", "point": v(e.Point)}
    if name.startswith("Edge"):
        c = e.Curve
        if c.TypeId == "Part::GeomLine":
            # start/end in the curve parameter direction (= curve.Direction)
            return {"type": "line", "start": v(e.valueAt(e.FirstParameter)), "end": v(e.valueAt(e.LastParameter)), "dir": v(c.Direction)}
        if c.TypeId == "Part::GeomCircle":
            q = c.Rotation.Q
            return {"type": "circle", "center": v(c.Location), "axis": v(c.Axis), "radius": c.Radius, "rot": list(q)}
        return {"type": c.TypeId}
    if name.startswith("Face"):
        s = e.Surface
        q = s.Rotation.Q if hasattr(s, "Rotation") and s.Rotation is not None else None
        d = {"type": s.TypeId, "cog": v(e.CenterOfGravity), "rot": list(q) if q else None}
        if s.TypeId == "Part::GeomPlane":
            d.update(type="plane", origin=v(s.Position), normal=v(s.Axis))
        elif s.TypeId == "Part::GeomCylinder":
            d.update(type="cylinder", origin=v(s.Center), axis=v(s.Axis), radius=s.Radius)
        elif s.TypeId == "Part::GeomSphere":
            d.update(type="sphere", origin=v(s.Center), radius=s.Radius)
        return d
    return None


class Case:
    def __init__(self, name, note):
        self.name = name
        self.note = note
        self.doc = App.newDocument(name)
        self.asm = self.doc.addObject("Assembly::AssemblyObject", "Assembly")
        self.jg = self.asm.newObject("Assembly::JointGroup", "Joints")
        self.parts = []
        self.joints = []

    def box(self, name, l, w, h, plc, grounded=False):
        b = self.asm.newObject("Part::Box", name)
        b.Length, b.Width, b.Height = l, w, h
        b.Placement = plc
        self.doc.recompute()
        self.parts.append({"name": b.Name, "shape": {"box": [l, w, h]}, "grounded": grounded, "placement": plc_json(plc)})
        if grounded:
            g = self.jg.newObject("App::FeaturePython", "GroundedJoint")
            JointObject.GroundedJoint(g, b)
        return b

    def cyl(self, name, r, h, plc, grounded=False):
        c = self.asm.newObject("Part::Cylinder", name)
        c.Radius, c.Height = r, h
        c.Placement = plc
        self.doc.recompute()
        self.parts.append({"name": c.Name, "shape": {"cylinder": [r, h]}, "grounded": grounded, "placement": plc_json(plc)})
        if grounded:
            g = self.jg.newObject("App::FeaturePython", "GroundedJoint")
            JointObject.GroundedJoint(g, c)
        return c

    def joint(self, jtype, a, ea, va, b, eb, vb, distance=None, angle=None, offset2=None, distance2=None):
        j = self.jg.newObject("App::FeaturePython", jtype)
        JointObject.Joint(j, TYPES.index(jtype))
        if distance is not None:
            j.Distance = distance
        if distance2 is not None:
            j.Distance2 = distance2
        if angle is not None:
            j.Angle = angle
        if offset2 is not None:
            j.Offset2 = offset2
        refs = [[a, [ea, va]], [b, [eb, vb]]]
        j.Proxy.setJointConnectors(j, refs)
        self.doc.recompute()
        self.joints.append((j, a, ea, va, b, eb, vb))
        return j

    def finish(self, start):
        # Joints were created the FreeCAD way (setJointConnectors pre-solves and solves after
        # every joint). Now put the moving parts at a chosen start pose (no recompute, which
        # would solve) and run one full solve from there.
        for name, d in start.items():
            self.doc.getObject(name).Placement = P(*d)
        start = {p["name"]: plc_json(self.doc.getObject(p["name"]).Placement) for p in self.parts}
        status = self.asm.solve()
        self.doc.recompute()
        solved = {p["name"]: plc_json(self.doc.getObject(p["name"]).Placement) for p in self.parts}
        joints = []
        for (j, a, ea, va, b, eb, vb) in self.joints:
            joints.append({
                "name": j.Name, "type": j.JointType,
                "ref1": [a.Name, ea, va], "ref2": [b.Name, eb, vb],
                "placement1": plc_json(j.Placement1), "placement2": plc_json(j.Placement2),
                "offset1": plc_json(j.Offset1), "offset2": plc_json(j.Offset2),
                "distance": float(j.Distance), "distance2": float(j.Distance2), "angle": float(j.Angle),
                "geom1": elem_geom(a, ea), "vtx1": elem_geom(a, va),
                "geom2": elem_geom(b, eb), "vtx2": elem_geom(b, vb),
            })
        path = os.path.join(OUT, self.name + ".FCStd")
        self.doc.saveAs(path)
        App.closeDocument(self.doc.Name)
        return {"name": self.name, "note": self.note, "status": status, "parts": self.parts,
                "joints": joints, "start": start, "solved": solved}


cases = []

# 1. Hinge: two plates, revolute on the long top edge of A (edge + its end vertex).
c = Case("hinge", "two plates + revolute on an edge; 1 DOF (rotation about the hinge)")
A = c.box("PlateA", 40, 20, 2, P(), grounded=True)
B = c.box("PlateB", 40, 20, 2, P(5, 30, 4, 0, 0, 20))
c.joint("Revolute", A, "Edge12", "Vertex7", B, "Edge10", "Vertex5")
cases.append(c.finish({"PlateB": (1, 21, 1, 0, 0, 40)}))

# 2. Slider: carriage on top of a rail, slider along the rail's top edge.
c = Case("slider", "carriage on a rail, slider on face+vertex; 1 DOF (translation)")
R = c.box("Rail", 100, 10, 10, P(), grounded=True)
K = c.box("Carriage", 10, 20, 5, P(30, -5, 12, 5, 0, 0))
c.joint("Slider", R, "Face6", "Vertex1", K, "Face5", "Vertex2")
cases.append(c.finish({"Carriage": (32, 1, 11, 4, 2, 0)}))

# 3. Four-bar linkage: 3 moving links, 4 revolutes at link corners (vertex refs), planar.
c = Case("fourbar", "4-bar linkage, 4 revolutes, planar; 1 DOF with redundant out-of-plane equations")
G = c.box("Ground", 60, 4, 2, P(), grounded=True)
Cr = c.box("Crank", 20, 4, 2, P(0, 0, 3, 60, 0, 0))
Co = c.box("Coupler", 50, 4, 2, P(10, 17.3, 6, -10, 0, 0))
Ro = c.box("Rocker", 40, 4, 2, P(60, 0, 9, 110, 0, 0))
c.joint("Revolute", G, "Vertex2", "Vertex2", Cr, "Vertex2", "Vertex2")
c.joint("Revolute", Cr, "Vertex6", "Vertex6", Co, "Vertex2", "Vertex2")
c.joint("Revolute", Co, "Vertex6", "Vertex6", Ro, "Vertex6", "Vertex6")
c.joint("Revolute", Ro, "Vertex2", "Vertex2", G, "Vertex6", "Vertex6")
cases.append(c.finish({"Ground": (0, 0, 0, 0, 0, 0), "Crank": (0, 0, 0, 60, 0, 0), "Coupler": (10, 17.32, 0, 27, 0, 0), "Rocker": (60, 0, 0, 100, 0, 0)}))

# 4. Ball joint between two box corners.
c = Case("ball", "ball joint at box corners; 3 DOF")
A = c.box("Base", 20, 20, 20, P(), grounded=True)
B = c.box("Arm", 30, 5, 5, P(22, 22, 22, 30, -20, 10))
c.joint("Ball", A, "Vertex7", "Vertex7", B, "Vertex2", "Vertex2")
cases.append(c.finish({"Arm": (21, 22, 23, 40, -15, 5)}))

# 5. Fixed stack: three boxes stacked with Fixed joints (top face/bottom face with vertices), one with offset.
c = Case("fixed_stack", "three boxes stacked by Fixed joints (second with a 2 mm offset and 30 deg rotation); 0 DOF")
A = c.box("Bottom", 30, 30, 10, P(), grounded=True)
B = c.box("Middle", 20, 20, 10, P(40, 0, 0, 10, 0, 0))
C = c.box("Top", 10, 10, 10, P(80, 0, 0, 0, 10, 0))
c.joint("Fixed", A, "Face6", "Vertex1", B, "Face5", "Vertex2")
c.joint("Fixed", B, "Face6", "Vertex7", C, "Face5", "Vertex8", offset2=App.Placement(App.Vector(0, 0, 2), App.Rotation(30, 0, 0)))
cases.append(c.finish({"Middle": (3, 4, 12, 15, 5, 0), "Top": (5, 18, 21, 20, 5, 10)}))

# 6. Hinged flap held at 30 degrees by an Angle joint.
c = Case("angle", "revolute about a vertical edge + Angle joint 30 deg between side faces; 0 DOF")
A = c.box("Frame", 40, 20, 30, P(), grounded=True)
B = c.box("Door", 20, 2, 30, P(40, 20, 0, 20, 0, 0))
c.joint("Revolute", A, "Edge7", "Vertex8", B, "Edge3", "Vertex4")
c.joint("Angle", A, "Face4", "Face4", B, "Face4", "Face4", angle=30)
cases.append(c.finish({"Door": (40.5, 20.5, 0.3, 20, 0, 0)}))

# 7. Distance: plane-plane 5 mm + point-point 15 mm.
c = Case("distance", "plane-plane Distance 5 (planar, 3 eq) + point-point Distance 15 (1 eq); 2 DOF")
A = c.box("Table", 50, 50, 5, P(), grounded=True)
B = c.box("Block", 10, 10, 10, P(20, 20, 12, 15, 0, 0))
c.joint("Distance", A, "Face6", "Face6", B, "Face5", "Face5", distance=5)
c.joint("Distance", A, "Vertex1", "Vertex1", B, "Vertex2", "Vertex2", distance=15)
cases.append(c.finish({"Block": (12, 8, 11, 20, 2, 0)}))

# 8. Cylindrical: pin in a sleeve (cylinder faces) + Perpendicular helper case.
c = Case("cylindrical", "two cylinders on a common axis via cylindrical faces; 2 DOF")
S = c.cyl("Sleeve", 5, 20, P(), grounded=True)
N = c.cyl("Pin", 5, 40, P(12, 3, 5, 0, 10, 5))
c.joint("Cylindrical", S, "Face1", "Face1", N, "Face1", "Face1")
cases.append(c.finish({"Pin": (0.5, 0.5, -8, 5, 2, 1)}))

with open(os.path.join(OUT, "assemblies.json"), "w") as f:
    json.dump(cases, f, indent=1)
print("wrote", len(cases), "cases")
for k in cases:
    print(k["name"], "status", k["status"])
