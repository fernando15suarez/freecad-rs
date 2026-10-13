# Export the Ondsel model (ASMT) of every parity case at its recorded start pose, as
# FreeCAD 1.1 builds it for a solve, and check that FreeCAD's solve from there gives the
# recorded result.
#
#   /Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd tools/parity/assembly/ondsel/export_start.py
#
# Writes asmt/<case>_start.asmt. FreeCAD 1.1.3 writes the joint type names truncated
# ("dJointE" for FixedJoint, ...); they are repaired here so OndselSolver can read the file.
import json
import os

import FreeCAD as App

HERE = os.path.dirname(os.path.abspath(__file__))
D = os.path.dirname(HERE)
OUT = os.path.join(HERE, "asmt")
os.makedirs(OUT, exist_ok=True)
NAMES = {
    "dJointE": "FixedJoint", "luteJointE": "RevoluteJoint", "arJointE": "PlanarJoint", "eJointE": "AngleJoint",
    "ndricalJointE": "CylindricalJoint", "phJointE": "SphSphJoint", "ricalJointE": "SphericalJoint",
    "slationalJointE": "TranslationalJoint",
}
for c in json.load(open(os.path.join(D, "assemblies.json"))):
    doc = App.openDocument(os.path.join(D, c["name"] + ".FCStd"))
    asm = doc.getObject("Assembly")
    for n, f in c["start"].items():
        doc.getObject(n).Placement = App.Placement(App.Vector(*f["pos"]), App.Rotation(*f["rot"]))
    path = os.path.join(OUT, c["name"] + "_start.asmt")
    asm.exportAsASMT(path)
    lines = open(path).read().split("\n")
    lines = ["\t\t\t" + NAMES[l.strip()] if l.startswith("\t\t\t") and l.strip() in NAMES else l for l in lines]
    open(path, "w").write("\n".join(lines))
    asm.solve()
    worst = 0.0
    for n, f in c["solved"].items():
        p = doc.getObject(n).Placement
        worst = max(worst, max(abs(a - b) for a, b in zip([p.Base.x, p.Base.y, p.Base.z], f["pos"])))
    print(c["name"], "FreeCAD re-solve differs from assemblies.json by", worst)
    App.closeDocument(doc.Name)
