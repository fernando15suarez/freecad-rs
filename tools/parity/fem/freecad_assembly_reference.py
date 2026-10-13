# FreeCAD FEM reference generator for fc-fem's multi-solid (assembly) analyses.
#
# Run with FreeCAD's headless interpreter (FreeCAD 1.1.x):
#   FEM_PARITY_OUT=tools/parity/fem FEM_PARITY_WORK=/tmp/femref \
#     /Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd tools/parity/fem/freecad_assembly_reference.py
#
# Cases (FreeCAD's own pipeline: Part boxes, BooleanFragments / Part Compound, MaterialSolid per
# solid, ConstraintTie / ConstraintContact, GmshTools, ccxtools):
#   asm_stack    two stacked blocks (aluminium below, steel above) bonded by BooleanFragments,
#                bottom fixed, 10 MPa on top
#   asm_lap_tie  lap joint of two plates (a compound, CoherenceMesh off) bonded by a tie,
#                one end fixed, 2 kN pulling the other end
#   asm_contact  a punch pressed into a block through penalty contact (compound, CoherenceMesh off)
# For every case: <case>.brep (what FreeCAD meshed), <case>.geo, <case>.json (counts, maxima,
# per-solid maxima, reactions) and <case>.inp (FreeCAD's CalculiX input).

import json
import os
import re
import shutil

import FreeCAD
import Part
import ObjectsFem
from BOPTools import SplitFeatures
from femtools import ccxtools
from femmesh.gmshtools import GmshTools

OUT = os.path.abspath(os.environ.get("FEM_PARITY_OUT", "tools/parity/fem"))
WORK = os.path.abspath(os.environ.get("FEM_PARITY_WORK", "/tmp/fc-fem-parity"))
ONLY = [c for c in os.environ.get("FEM_PARITY_CASES", "").split(",") if c]

STEEL = {
    "Name": "Steel-Generic",
    "YoungsModulus": "200000 MPa",
    "PoissonRatio": "0.3",
    "Density": "7900 kg/m^3",
    "ThermalConductivity": "50 W/m/K",
    "ThermalExpansionCoefficient": "0.000012 m/m/K",
    "SpecificHeat": "500 J/kg/K",
}
ALU = {
    "Name": "Aluminum 6061-T6",
    "YoungsModulus": "68900 MPa",
    "PoissonRatio": "0.33",
    "Density": "2700 kg/m^3",
    "ThermalConductivity": "167 W/m/K",
    "ThermalExpansionCoefficient": "0.0000236 m/m/K",
    "SpecificHeat": "896 J/kg/K",
}


def new_doc(name):
    if name in FreeCAD.listDocuments():
        FreeCAD.closeDocument(name)
    return FreeCAD.newDocument(name)


def box(doc, name, size, pos):
    b = doc.addObject("Part::Box", name)
    b.Length, b.Width, b.Height = size
    b.Placement.Base = FreeCAD.Vector(*pos)
    return b


def faces(shape, pred):
    return [f"Face{i}" for i, f in enumerate(shape.Faces, start=1) if pred(f)]


def plane(f, axis, value, tol=1e-6):
    if f.Surface.__class__.__name__ != "Plane":
        return False
    bb = f.BoundBox
    lo = [bb.XMin, bb.YMin, bb.ZMin][axis]
    hi = [bb.XMax, bb.YMax, bb.ZMax][axis]
    return abs(lo - value) < tol and abs(hi - value) < tol


def solid_index(shape, pred):
    for i, s in enumerate(shape.Solids, start=1):
        if pred(s):
            return i
    raise RuntimeError("no such solid")


def setup(doc, geom, max_len, coherence=True):
    analysis = ObjectsFem.makeAnalysis(doc, "Analysis")
    solver = ObjectsFem.makeSolverCalculiXCcxTools(doc, "CalculiXCcxTools")
    solver.WorkingDir = ""
    solver.SplitInputWriter = False
    solver.AnalysisType = "static"
    solver.GeometricalNonlinearity = "linear"
    solver.MatrixSolverType = "default"
    solver.IterationsControlParameterTimeUse = False
    solver.ThermoMechSteadyState = False
    analysis.addObject(solver)
    mesh = ObjectsFem.makeMeshGmsh(doc, "Mesh")
    mesh.Shape = geom
    mesh.CharacteristicLengthMax = f"{max_len} mm"
    mesh.CharacteristicLengthMin = "0 mm"
    mesh.ElementOrder = "2nd"
    mesh.SecondOrderLinear = False
    mesh.CoherenceMesh = coherence
    analysis.addObject(mesh)
    return analysis, solver, mesh


def material(doc, analysis, name, card, refs):
    m = ObjectsFem.makeMaterialSolid(doc, name)
    c = m.Material
    for k, v in card.items():
        c[k] = v
    m.Material = c
    if refs:
        m.References = refs
    analysis.addObject(m)
    return m


def mesh_and_solve(case, doc, analysis, solver, mesh):
    wd = os.path.join(WORK, case)
    if os.path.isdir(wd):
        shutil.rmtree(wd)
    os.makedirs(wd)
    gwd = os.path.join(wd, "gmsh")
    os.makedirs(gwd)
    gt = GmshTools(mesh)
    gt.load_properties()
    gt.update_mesh_data()
    gt.get_tmp_file_paths(gwd, True)
    gt.get_gmsh_command()
    gt.write_gmsh_input_files()
    # single-threaded Gmsh (deterministic), as in freecad_reference.py
    geo = open(gt.temp_file_geo).read()
    geo = re.sub(r"General.NumThreads = \d+;", "General.NumThreads = 1;", geo)
    open(gt.temp_file_geo, "w").write(geo)
    err = gt.run_gmsh_with_geo()
    if err:
        print("GMSH:", err)
    gt.read_and_set_new_mesh()
    doc.recompute()
    shutil.copy(gt.temp_file_geometry, os.path.join(OUT, case + ".brep"))
    shutil.copy(gt.temp_file_geo, os.path.join(OUT, case + ".geo"))
    fea = ccxtools.FemToolsCcx(analysis, solver)
    fea.update_objects()
    fea.setup_working_dir(wd)
    fea.setup_ccx()
    msg = fea.check_prerequisites()
    if msg:
        raise RuntimeError(msg)
    fea.write_inp_file()
    fea.ccx_run()
    fea.load_results()
    with open(os.path.join(OUT, case + ".inp"), "w") as f:
        f.write(trim_inp(open(os.path.join(wd, "Mesh.inp")).read()))
    return wd


def trim_inp(text):
    """Mesh.inp without the node and element data lines (kept small in the repository; the
    mesh itself is checked through counts). fc-fem's parity test trims its own file the same way."""
    out = []
    data = False
    for line in text.splitlines(True):
        if line.startswith("*"):
            data = line.startswith("*Node, NSET=Nall") or line.startswith("*Element, TYPE=")
            out.append(line)
        elif data and line.strip():
            continue
        else:
            out.append(line)
    return "".join(out)


def parse_dat_totals(path):
    out = {}
    if not os.path.exists(path):
        return out
    lines = open(path).read().splitlines()
    for i, l in enumerate(lines):
        if l.strip().startswith("total force (fx,fy,fz) for set"):
            name = l.split("for set")[1].split("and time")[0].strip()
            out[name] = [float(v) for v in lines[i + 2].split()]
    return out


def collect(case, doc, wd, geom, mesh, extra):
    res = [o for o in doc.Objects if o.isDerivedFrom("Fem::FemResultObject")]
    fm = mesh.FemMesh
    r = res[-1]
    out = {
        "case": case,
        "freecad_version": ".".join(FreeCAD.Version()[0:3]),
        "nodes": fm.NodeCount,
        "volume_elements": fm.VolumeCount,
        "reactions": parse_dat_totals(os.path.join(wd, "Mesh.dat")),
    }
    out.update(extra)
    pos = {n: k for k, n in enumerate(r.NodeNumbers)}
    i = max(range(len(r.DisplacementLengths)), key=lambda k: r.DisplacementLengths[k])
    out["max_disp"] = r.DisplacementLengths[i]
    out["max_disp_node"] = r.NodeNumbers[i]
    i = max(range(len(r.vonMises)), key=lambda k: r.vonMises[k])
    out["max_von_mises"] = r.vonMises[i]
    out["max_von_mises_node"] = r.NodeNumbers[i]
    per = []
    for s in geom.Shape.Solids:
        nodes = [n for n in fm.getNodesBySolid(s) if n in pos]
        per.append(
            {
                "nodes": len(nodes),
                "max_disp": max(r.DisplacementLengths[pos[n]] for n in nodes),
                "max_von_mises": max(r.vonMises[pos[n]] for n in nodes),
            }
        )
    out["solids"] = per
    with open(os.path.join(OUT, case + ".json"), "w") as f:
        json.dump(out, f, indent=2, sort_keys=True)
    print(case, json.dumps(out, sort_keys=True))


def case_stack():
    case = "asm_stack"
    doc = new_doc(case)
    low = box(doc, "Lower", (40, 40, 20), (0, 0, 0))
    upp = box(doc, "Upper", (40, 40, 20), (0, 0, 20))
    doc.recompute()
    geom = SplitFeatures.makeBooleanFragments(name="BooleanFragments")
    geom.Objects = [low, upp]
    geom.Mode = "Standard"
    doc.recompute()
    sh = geom.Shape
    analysis, solver, mesh = setup(doc, geom, 4.0)
    s_low = solid_index(sh, lambda s: s.CenterOfMass.z < 20)
    s_upp = solid_index(sh, lambda s: s.CenterOfMass.z > 20)
    material(doc, analysis, "FemMaterial", ALU, [(geom, f"Solid{s_low}")])
    material(doc, analysis, "FemMaterial001", STEEL, [(geom, f"Solid{s_upp}")])
    fixed = faces(sh, lambda f: plane(f, 2, 0.0))
    top = faces(sh, lambda f: plane(f, 2, 40.0))
    cf = ObjectsFem.makeConstraintFixed(doc, "ConstraintFixed")
    cf.References = [(geom, f) for f in fixed]
    analysis.addObject(cf)
    pr = ObjectsFem.makeConstraintPressure(doc, "ConstraintPressure")
    pr.References = [(geom, f) for f in top]
    pr.Pressure = "10 MPa"
    pr.Reversed = False
    analysis.addObject(pr)
    doc.recompute()
    wd = mesh_and_solve(case, doc, analysis, solver, mesh)
    collect(
        case,
        doc,
        wd,
        geom,
        mesh,
        {
            "mesh": {"max": 4.0, "order": 2, "coherence": True},
            "solid_lower": s_low,
            "solid_upper": s_upp,
            "fixed": fixed,
            "pressure_faces": top,
            "pressure": 10.0,
            "shared_faces": len(sh.Faces),
        },
    )


def case_lap_tie():
    case = "asm_lap_tie"
    doc = new_doc(case)
    a = box(doc, "PlateA", (100, 40, 5), (0, 0, 0))
    # B is placed so that no mesh node of its face edges can coincide with one of A's, and
    # 0.05 mm above A: FreeCAD finds the nodes of a face by a geometric search, so on exactly
    # coincident faces both tie surfaces would get both plates' element faces and ccx loops in
    # its MPC decascading. FreeCAD's own tie example has a gap too.
    b = box(doc, "PlateB", (98.3, 31.4, 5), (61.7, 4.3, 5.05))
    doc.recompute()
    geom = doc.addObject("Part::Compound", "Compound")
    geom.Links = [a, b]
    doc.recompute()
    sh = geom.Shape
    analysis, solver, mesh = setup(doc, geom, 2.5, coherence=False)
    material(doc, analysis, "FemMaterial", STEEL, [])
    fixed = faces(sh, lambda f: plane(f, 0, 0.0))
    pulled = faces(sh, lambda f: plane(f, 0, 160.0))
    master = faces(sh, lambda f: plane(f, 2, 5.0))
    slave = faces(sh, lambda f: plane(f, 2, 5.05))
    cf = ObjectsFem.makeConstraintFixed(doc, "ConstraintFixed")
    cf.References = [(geom, f) for f in fixed]
    analysis.addObject(cf)
    fo = ObjectsFem.makeConstraintForce(doc, "ConstraintForce")
    fo.References = [(geom, pulled[0])]
    fo.Force = "2000 N"
    fo.Direction = (geom, [pulled[0]])
    fo.Reversed = False
    analysis.addObject(fo)
    tie = ObjectsFem.makeConstraintTie(doc, "ConstraintTie")
    tie.References = [(geom, slave[0]), (geom, master[0])]
    tie.Tolerance = "0.1 mm"
    analysis.addObject(tie)
    doc.recompute()
    wd = mesh_and_solve(case, doc, analysis, solver, mesh)
    collect(
        case,
        doc,
        wd,
        geom,
        mesh,
        {
            "mesh": {"max": 2.5, "order": 2, "coherence": False},
            "fixed": fixed,
            "force_faces": pulled,
            "force": 2000.0,
            "force_dir": [1, 0, 0],
            "tie_slave": slave,
            "tie_master": master,
            "tie_tolerance": 0.1,
        },
    )


def case_contact():
    case = "asm_contact"
    doc = new_doc(case)
    a = box(doc, "Block", (40, 40, 10), (0, 0, 0))
    # 0.05 mm gap closed by the contact's Adjust (see case_lap_tie for why not touching)
    b = box(doc, "Punch", (20, 20, 10), (10, 10, 10.05))
    doc.recompute()
    geom = doc.addObject("Part::Compound", "Compound")
    geom.Links = [a, b]
    doc.recompute()
    sh = geom.Shape
    analysis, solver, mesh = setup(doc, geom, 2.5, coherence=False)
    material(doc, analysis, "FemMaterial", STEEL, [])
    fixed = faces(sh, lambda f: plane(f, 2, 0.0))
    top = faces(sh, lambda f: plane(f, 2, 20.05))
    master = faces(sh, lambda f: plane(f, 2, 10.0))
    slave = faces(sh, lambda f: plane(f, 2, 10.05))
    cf = ObjectsFem.makeConstraintFixed(doc, "ConstraintFixed")
    cf.References = [(geom, f) for f in fixed]
    analysis.addObject(cf)
    cd = ObjectsFem.makeConstraintDisplacement(doc, "ConstraintDisplacement")
    cd.References = [(geom, top[0])]
    cd.xFree = False
    cd.xDisplacement = "0 mm"
    cd.yFree = False
    cd.yDisplacement = "0 mm"
    analysis.addObject(cd)
    pr = ObjectsFem.makeConstraintPressure(doc, "ConstraintPressure")
    pr.References = [(geom, top[0])]
    pr.Pressure = "50 MPa"
    pr.Reversed = False
    analysis.addObject(pr)
    con = ObjectsFem.makeConstraintContact(doc, "ConstraintContact")
    con.References = [(geom, slave[0]), (geom, master[0])]
    con.Slope = "1000000 GPa/m"
    con.Adjust = "0.1 mm"
    con.Friction = False
    analysis.addObject(con)
    doc.recompute()
    wd = mesh_and_solve(case, doc, analysis, solver, mesh)
    collect(
        case,
        doc,
        wd,
        geom,
        mesh,
        {
            "mesh": {"max": 2.5, "order": 2, "coherence": False},
            "fixed": fixed,
            "top": top,
            "pressure": 50.0,
            "contact_slave": slave,
            "contact_master": master,
            "contact_slope": 1e6,
            "contact_adjust": 0.1,
        },
    )


CASES = {"asm_stack": case_stack, "asm_lap_tie": case_lap_tie, "asm_contact": case_contact}

os.makedirs(OUT, exist_ok=True)
os.makedirs(WORK, exist_ok=True)
for name, fn in CASES.items():
    if ONLY and name not in ONLY:
        continue
    fn()
