# Structured STEP parity with FreeCAD 1.1 (Import / ImportOCAF2).
#
#   /Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd tools/parity/assembly/step_assembly.py
#
# 1. Writes freecad_asm.step next to this script: App::Part "Top" with a Part::Box plate and two
#    App::Links to App::Part "PinPair" (two links to one Part::Cylinder), the second turned 90°
#    about Z. crates/fc-assembly/tests/tools.rs imports it (`freecad_step_import`).
# 2. If FCRS_STEP is set (paths separated by ':'), imports each of those files (our exports, e.g.
#    from `cargo test -p fc-assembly --test tools`, written to $TMPDIR/fc_assembly_tools/) and
#    prints the assembly tree, solid count, volumes and bounding boxes FreeCAD sees.
import os

import FreeCAD as App
import Import

HERE = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else os.path.join(os.getcwd(), "tools/parity/assembly")


def make_reference():
    doc = App.newDocument("Ref")
    plate = doc.addObject("Part::Box", "Plate")
    plate.Length, plate.Width, plate.Height = 60, 40, 4
    pin = doc.addObject("Part::Cylinder", "Pin")
    pin.Radius, pin.Height = 3, 15
    pair = doc.addObject("App::Part", "PinPair")
    a = doc.addObject("App::Link", "PinA")
    a.LinkedObject = pin
    b = doc.addObject("App::Link", "PinB")
    b.LinkedObject = pin
    b.Placement = App.Placement(App.Vector(12, 0, 0), App.Rotation())
    pair.addObjects([a, b])
    top = doc.addObject("App::Part", "Top")
    p1 = doc.addObject("App::Link", "Pair1")
    p1.LinkedObject = pair
    p1.Placement = App.Placement(App.Vector(10, 10, 4), App.Rotation())
    p2 = doc.addObject("App::Link", "Pair2")
    p2.LinkedObject = pair
    p2.Placement = App.Placement(App.Vector(45, 10, 4), App.Rotation(App.Vector(0, 0, 1), 90))
    top.addObjects([plate, p1, p2])
    doc.recompute()
    Import.export([top], os.path.join(HERE, "freecad_asm.step"))
    App.closeDocument(doc.Name)


def tree(o, depth=0):
    kids = getattr(o, "Group", []) if o.TypeId == "App::Part" else []
    print("  " * depth + f"{o.TypeId} {o.Label!r} at {tuple(round(v, 6) for v in o.Placement.Base)}")
    for k in kids:
        if k.TypeId not in ("App::Origin",):
            tree(k, depth + 1)


def check(path):
    doc = App.newDocument("Check")
    Import.insert(path, doc.Name)
    doc.recompute()
    print("==", path)
    for r in doc.RootObjects:
        tree(r)
    solids = [s for r in doc.RootObjects if hasattr(r, "Shape") for s in r.Shape.Solids]
    print(f"solids: {len(solids)}  volume: {sum(s.Volume for s in solids):.6f}")
    for s in solids:
        bb = s.BoundBox
        print(f"  {s.Volume:10.3f}  [{bb.XMin:.6g}, {bb.YMin:.6g}, {bb.ZMin:.6g}] - [{bb.XMax:.6g}, {bb.YMax:.6g}, {bb.ZMax:.6g}]")
    App.closeDocument(doc.Name)


make_reference()
for p in filter(None, os.environ.get("FCRS_STEP", "").split(":")):
    check(p)
