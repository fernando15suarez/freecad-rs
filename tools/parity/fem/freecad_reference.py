# FreeCAD FEM reference generator for fc-fem parity.
#
# Run with FreeCAD's headless interpreter (FreeCAD 1.1.x):
#   FEM_PARITY_OUT=tools/parity/fem FEM_PARITY_WORK=/tmp/femref \
#     /Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd tools/parity/fem/freecad_reference.py
#
# For every case this builds the geometry with Part, meshes it with FreeCAD's own Gmsh tools
# (GmshTools, the same .geo writer the GUI uses), writes the CalculiX input with ccxtools,
# runs ccx and stores:
#   <case>.brep  the exact geometry FreeCAD meshed (fc-fem meshes the same file)
#   <case>.json  settings + FreeCAD's results (counts, max displacement, max von Mises, reactions...)
# The ccx working directory (shape2mesh.geo, Mesh.inp, Mesh.frd, Mesh.dat) stays in FEM_PARITY_WORK.
#
# The bracket case needs bracket.step, written by the fc-fem test `write_bracket_step`
# (cargo test -p fc-fem -- --ignored write_bracket_step).

import json
import math
import os
import re
import shutil
import sys

import FreeCAD
import Part
import ObjectsFem
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


def faces_where(shape, pred):
    out = []
    for i, f in enumerate(shape.Faces, start=1):
        if pred(f):
            out.append(f"Face{i}")
    return out


def plane_at(axis, value, tol=1e-6):
    def pred(f):
        if f.Surface.__class__.__name__ != "Plane":
            return False
        bb = f.BoundBox
        lo = [bb.XMin, bb.YMin, bb.ZMin][axis]
        hi = [bb.XMax, bb.YMax, bb.ZMax][axis]
        return abs(lo - value) < tol and abs(hi - value) < tol

    return pred


def is_cylinder(f):
    return f.Surface.__class__.__name__ == "Cylinder"


def edge_along(shape, vec):
    v = FreeCAD.Vector(*vec)
    for i, e in enumerate(shape.Edges, start=1):
        if e.Curve.__class__.__name__ in ("Line", "LineSegment"):
            d = e.Vertexes[-1].Point - e.Vertexes[0].Point
            if d.Length > 0 and abs(abs(d.normalize().dot(v)) - 1) < 1e-9:
                return f"Edge{i}"
    raise RuntimeError("no edge along %s" % (vec,))


def point_force(doc, fo, geom, edge, want):
    """Direction from an edge, flipped with Reversed so DirectionVector == want."""
    fo.Direction = (geom, [edge])
    fo.Reversed = False
    doc.recompute()
    d = fo.DirectionVector
    if d.dot(FreeCAD.Vector(*want)) < 0:
        fo.Reversed = True
        doc.recompute()
    d = fo.DirectionVector
    assert d.dot(FreeCAD.Vector(*want)) > 0.999, d
    return [d.x, d.y, d.z]


def new_doc(name):
    if name in FreeCAD.listDocuments():
        FreeCAD.closeDocument(name)
    return FreeCAD.newDocument(name)


def setup_common(doc, shape, analysis_type, mat, max_len, order="2nd", min_len=0.0):
    geom = doc.addObject("Part::Feature", "Geometry")
    geom.Shape = shape
    doc.recompute()
    analysis = ObjectsFem.makeAnalysis(doc, "Analysis")
    solver = ObjectsFem.makeSolverCalculiXCcxTools(doc, "CalculiXCcxTools")
    solver.WorkingDir = ""
    solver.SplitInputWriter = False
    solver.AnalysisType = analysis_type
    solver.GeometricalNonlinearity = "linear"
    solver.MatrixSolverType = "default"
    solver.IterationsControlParameterTimeUse = False
    if analysis_type == "frequency":
        solver.EigenmodesCount = 6
    if analysis_type == "thermomech":
        solver.ThermoMechSteadyState = True
        solver.IncrementsMaximum = 2000
    else:
        solver.ThermoMechSteadyState = False
    analysis.addObject(solver)
    m = ObjectsFem.makeMaterialSolid(doc, "FemMaterial")
    card = m.Material
    for k, v in mat.items():
        card[k] = v
    m.Material = card
    analysis.addObject(m)
    mesh = ObjectsFem.makeMeshGmsh(doc, "Mesh")
    mesh.Shape = geom
    mesh.CharacteristicLengthMax = f"{max_len} mm"
    mesh.CharacteristicLengthMin = f"{min_len} mm"
    mesh.ElementOrder = order
    mesh.SecondOrderLinear = False
    analysis.addObject(mesh)
    return geom, analysis, solver, mesh


def mesh_and_solve(case, doc, geom, analysis, solver, mesh):
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
    # Gmsh's multi-threaded meshing is not deterministic; pin it to one thread (in the .geo only,
    # FreeCAD's preferences stay untouched) so fc-fem can reproduce the identical mesh.
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
    if solver.AnalysisType != "frequency":
        # FreeCAD's frequency result import crashes freecadcmd 1.1.3 (headless); for modal cases
        # the eigenfrequencies come from Mesh.dat instead.
        fea.load_results()
    return wd


def parse_dat_totals(path):
    """Sum of reaction forces per node set from `*NODE PRINT ... TOTALS=ONLY`."""
    out = {}
    if not os.path.exists(path):
        return out
    lines = open(path).read().splitlines()
    for i, l in enumerate(lines):
        if l.strip().startswith("total force (fx,fy,fz) for set"):
            name = l.split("for set")[1].split("and time")[0].strip()
            vals = [float(v) for v in lines[i + 2].split()]
            out[name] = vals
    return out


def parse_dat_eigen(path):
    freqs = []
    lines = open(path).read().splitlines()
    for i, l in enumerate(lines):
        if "E I G E N V A L U E   O U T P U T" in l:
            j = i + 6
            while j < len(lines) and lines[j].strip():
                p = lines[j].split()
                freqs.append(float(p[3]))
                j += 1
            break
    return freqs


def collect(case, doc, wd, mesh, extra):
    res = [o for o in doc.Objects if o.isDerivedFrom("Fem::FemResultObject")]
    fm = mesh.FemMesh
    out = {
        "case": case,
        "freecad_version": ".".join(FreeCAD.Version()[0:3]),
        "nodes": fm.NodeCount,
        "volume_elements": fm.VolumeCount,
        "reactions": parse_dat_totals(os.path.join(wd, "Mesh.dat")),
    }
    out.update(extra)
    r = res[0] if res else None
    if r is not None and len(r.DisplacementLengths):
        i = max(range(len(r.DisplacementLengths)), key=lambda k: r.DisplacementLengths[k])
        out["max_disp"] = r.DisplacementLengths[i]
        out["max_disp_node"] = r.NodeNumbers[i]
        dv = r.DisplacementVectors
        out["min_disp_z"] = min(v.z for v in dv)
        out["max_disp_xyz"] = [max(abs(v.x) for v in dv), max(abs(v.y) for v in dv), max(abs(v.z) for v in dv)]
    if r is not None and len(r.vonMises):
        i = max(range(len(r.vonMises)), key=lambda k: r.vonMises[k])
        out["max_von_mises"] = r.vonMises[i]
        out["max_von_mises_node"] = r.NodeNumbers[i]
        out["max_principal"] = max(r.PrincipalMax)
        out["min_principal"] = min(r.PrincipalMin)
    if r is not None and hasattr(r, "Temperature") and len(r.Temperature):
        out["max_temperature"] = max(r.Temperature)
        out["min_temperature"] = min(r.Temperature)
    if extra.get("analysis_type") == "frequency":
        out["frequencies"] = parse_dat_eigen(os.path.join(wd, "Mesh.dat"))
    with open(os.path.join(OUT, case + ".json"), "w") as f:
        json.dump(out, f, indent=2, sort_keys=True)
    print(case, json.dumps(out, sort_keys=True))


def case_cantilever(case="cantilever", analysis_type="static", force=True, selfweight=False):
    L, W, H = 1000.0, 100.0, 100.0
    doc = new_doc(case)
    shape = Part.makeBox(L, W, H)
    geom, analysis, solver, mesh = setup_common(doc, shape, analysis_type, STEEL, 25.0)
    fixed = faces_where(shape, plane_at(0, 0.0))
    loaded = faces_where(shape, plane_at(0, L))
    cf = ObjectsFem.makeConstraintFixed(doc, "ConstraintFixed")
    cf.References = [(geom, fixed[0])]
    analysis.addObject(cf)
    extra = {
        "analysis_type": analysis_type,
        "mesh": {"max": 25.0, "min": 0.0, "order": 2},
        "fixed": fixed,
        "dims": [L, W, H],
        "material": STEEL["Name"],
    }
    if force:
        fo = ObjectsFem.makeConstraintForce(doc, "ConstraintForce")
        fo.References = [(geom, loaded[0])]
        fo.Force = "1000 N"
        analysis.addObject(fo)
        d = point_force(doc, fo, geom, edge_along(shape, (0, 0, 1)), (0, 0, -1))
        extra.update({"force_faces": loaded, "force": 1000.0, "force_dir": d})
    if selfweight:
        sw = ObjectsFem.makeConstraintSelfWeight(doc, "ConstraintSelfWeight")
        sw.GravityDirection = FreeCAD.Vector(0, 0, -1)
        analysis.addObject(sw)
        extra.update({"gravity": [0, 0, -1]})
    doc.recompute()
    wd = mesh_and_solve(case, doc, geom, analysis, solver, mesh)
    collect(case, doc, wd, mesh, extra)


def case_plate_hole():
    case = "plate_hole"
    doc = new_doc(case)
    box = Part.makeBox(200.0, 100.0, 10.0)
    cyl = Part.makeCylinder(10.0, 10.0, FreeCAD.Vector(100, 50, 0), FreeCAD.Vector(0, 0, 1))
    shape = box.cut(cyl)
    geom, analysis, solver, mesh = setup_common(doc, shape, "static", STEEL, 5.0)
    fixed = faces_where(shape, plane_at(0, 0.0))
    loaded = faces_where(shape, plane_at(0, 200.0))
    hole = faces_where(shape, is_cylinder)
    reg = ObjectsFem.makeMeshRegion(doc, mesh, 1.5, "MeshRegion")
    reg.References = [(geom, h) for h in hole]
    cf = ObjectsFem.makeConstraintFixed(doc, "ConstraintFixed")
    cf.References = [(geom, fixed[0])]
    analysis.addObject(cf)
    fo = ObjectsFem.makeConstraintForce(doc, "ConstraintForce")
    fo.References = [(geom, loaded[0])]
    fo.Force = "10000 N"
    fo.Direction = (geom, [loaded[0]])  # face normal, pointing out of the solid (+x)
    fo.Reversed = False
    analysis.addObject(fo)
    doc.recompute()
    wd = mesh_and_solve(case, doc, geom, analysis, solver, mesh)
    collect(
        case,
        doc,
        wd,
        mesh,
        {
            "analysis_type": "static",
            "mesh": {"max": 5.0, "min": 0.0, "order": 2, "refine": {"faces": hole, "size": 1.5}},
            "fixed": fixed,
            "force_faces": loaded,
            "force": 10000.0,
            "force_dir": [1, 0, 0],
            "hole_faces": hole,
            "material": STEEL["Name"],
        },
    )


def case_pressure():
    case = "pressure"
    doc = new_doc(case)
    shape = Part.makeBox(200.0, 50.0, 10.0)
    geom, analysis, solver, mesh = setup_common(doc, shape, "static", STEEL, 5.0)
    ends = faces_where(shape, plane_at(0, 0.0)) + faces_where(shape, plane_at(0, 200.0))
    top = faces_where(shape, plane_at(2, 10.0))
    cf = ObjectsFem.makeConstraintFixed(doc, "ConstraintFixed")
    cf.References = [(geom, f) for f in ends]
    analysis.addObject(cf)
    pr = ObjectsFem.makeConstraintPressure(doc, "ConstraintPressure")
    pr.References = [(geom, top[0])]
    pr.Pressure = "0.1 MPa"
    pr.Reversed = False
    analysis.addObject(pr)
    doc.recompute()
    wd = mesh_and_solve(case, doc, geom, analysis, solver, mesh)
    collect(
        case,
        doc,
        wd,
        mesh,
        {
            "analysis_type": "static",
            "mesh": {"max": 5.0, "min": 0.0, "order": 2},
            "fixed": ends,
            "pressure_faces": top,
            "pressure": 0.1,
            "material": STEEL["Name"],
        },
    )


def case_bracket():
    case = "bracket"
    step = os.path.join(OUT, "bracket.step")
    if not os.path.exists(step):
        print("bracket.step missing, skipping bracket case")
        return
    doc = new_doc(case)
    shape = Part.read(step)
    if shape.ShapeType != "Solid" and len(shape.Solids) == 1:
        shape = shape.Solids[0]
    bb = shape.BoundBox
    geom, analysis, solver, mesh = setup_common(doc, shape, "static", STEEL, 4.0)
    # mounting face: the back of the upright (x = xmin); load: the end face of the arm (x = xmax)
    fixed = faces_where(shape, plane_at(0, bb.XMin))
    top = faces_where(shape, plane_at(0, bb.XMax))
    cf = ObjectsFem.makeConstraintFixed(doc, "ConstraintFixed")
    cf.References = [(geom, f) for f in fixed]
    analysis.addObject(cf)
    fo = ObjectsFem.makeConstraintForce(doc, "ConstraintForce")
    fo.References = [(geom, top[0])]
    fo.Force = "500 N"
    analysis.addObject(fo)
    point_force(doc, fo, geom, edge_along(shape, (0, 0, 1)), (0, 0, -1))
    doc.recompute()
    wd = mesh_and_solve(case, doc, geom, analysis, solver, mesh)
    collect(
        case,
        doc,
        wd,
        mesh,
        {
            "analysis_type": "static",
            "mesh": {"max": 4.0, "min": 0.0, "order": 2},
            "fixed": fixed,
            "force_faces": top[:1],
            "force": 500.0,
            "force_dir": [0, 0, -1],
            "material": STEEL["Name"],
            "volume": shape.Volume,
        },
    )


def case_thermal():
    case = "thermal_bar"
    doc = new_doc(case)
    shape = Part.makeBox(100.0, 10.0, 10.0)
    geom, analysis, solver, mesh = setup_common(doc, shape, "thermomech", STEEL, 2.5)
    cold = faces_where(shape, plane_at(0, 0.0))
    hot = faces_where(shape, plane_at(0, 100.0))
    cf = ObjectsFem.makeConstraintFixed(doc, "ConstraintFixed")
    cf.References = [(geom, cold[0])]
    analysis.addObject(cf)
    it = ObjectsFem.makeConstraintInitialTemperature(doc, "ConstraintInitialTemperature")
    it.initialTemperature = 293.0
    analysis.addObject(it)
    t1 = ObjectsFem.makeConstraintTemperature(doc, "ConstraintTemperature")
    t1.References = [(geom, cold[0])]
    t1.Temperature = 293.0
    analysis.addObject(t1)
    t2 = ObjectsFem.makeConstraintTemperature(doc, "ConstraintTemperature001")
    t2.References = [(geom, hot[0])]
    t2.Temperature = 393.0
    analysis.addObject(t2)
    doc.recompute()
    wd = mesh_and_solve(case, doc, geom, analysis, solver, mesh)
    collect(
        case,
        doc,
        wd,
        mesh,
        {
            "analysis_type": "thermomech",
            "mesh": {"max": 2.5, "min": 0.0, "order": 2},
            "fixed": cold,
            "temperatures": {"cold": [cold, 293.0], "hot": [hot, 393.0]},
            "initial_temperature": 293.0,
            "material": STEEL["Name"],
        },
    )


def case_snapshot_static():
    """Small mesh with every static constraint type; Mesh.inp/.frd/.dat are kept as snapshots."""
    case = "snapshot_static"
    doc = new_doc(case)
    shape = Part.makeBox(100.0, 20.0, 20.0)
    geom, analysis, solver, mesh = setup_common(doc, shape, "static", STEEL, 10.0)
    cf = ObjectsFem.makeConstraintFixed(doc, "ConstraintFixed")
    cf.References = [(geom, faces_where(shape, plane_at(0, 0.0))[0])]
    analysis.addObject(cf)
    cd = ObjectsFem.makeConstraintDisplacement(doc, "ConstraintDisplacement")
    cd.References = [(geom, faces_where(shape, plane_at(1, 0.0))[0])]
    cd.yFree = False
    cd.yDisplacement = "0 mm"
    analysis.addObject(cd)
    fo = ObjectsFem.makeConstraintForce(doc, "ConstraintForce")
    fo.References = [(geom, faces_where(shape, plane_at(0, 100.0))[0])]
    fo.Force = "500 N"
    analysis.addObject(fo)
    point_force(doc, fo, geom, edge_along(shape, (0, 0, 1)), (0, 0, -1))
    pr = ObjectsFem.makeConstraintPressure(doc, "ConstraintPressure")
    pr.References = [(geom, faces_where(shape, plane_at(2, 20.0))[0])]
    pr.Pressure = "2 MPa"
    analysis.addObject(pr)
    sw = ObjectsFem.makeConstraintSelfWeight(doc, "ConstraintSelfWeight")
    sw.GravityDirection = FreeCAD.Vector(0, 0, -1)
    analysis.addObject(sw)
    doc.recompute()
    wd = mesh_and_solve(case, doc, geom, analysis, solver, mesh)
    for ext in ("inp", "frd", "dat"):
        shutil.copy(os.path.join(wd, "Mesh." + ext), os.path.join(OUT, case + "." + ext))
    collect(case, doc, wd, mesh, {"analysis_type": "static", "mesh": {"max": 10.0, "min": 0.0, "order": 2}})


def case_snapshot_thermo():
    case = "snapshot_thermo"
    doc = new_doc(case)
    shape = Part.makeBox(50.0, 10.0, 10.0)
    geom, analysis, solver, mesh = setup_common(doc, shape, "thermomech", STEEL, 5.0)
    f1 = faces_where(shape, plane_at(0, 0.0))[0]
    cf = ObjectsFem.makeConstraintFixed(doc, "ConstraintFixed")
    cf.References = [(geom, f1)]
    analysis.addObject(cf)
    it = ObjectsFem.makeConstraintInitialTemperature(doc, "ConstraintInitialTemperature")
    it.initialTemperature = 293.0
    analysis.addObject(it)
    t1 = ObjectsFem.makeConstraintTemperature(doc, "ConstraintTemperature")
    t1.References = [(geom, f1)]
    t1.Temperature = 293.0
    analysis.addObject(t1)
    hf = ObjectsFem.makeConstraintHeatflux(doc, "ConstraintHeatflux")
    hf.References = [(geom, faces_where(shape, plane_at(0, 50.0))[0])]
    hf.ConstraintType = "DFlux"
    hf.DFlux = "5000 W/m^2"
    analysis.addObject(hf)
    hc = ObjectsFem.makeConstraintHeatflux(doc, "ConstraintHeatflux001")
    hc.References = [(geom, faces_where(shape, plane_at(2, 10.0))[0])]
    hc.ConstraintType = "Convection"
    hc.FilmCoef = "10 W/m^2/K"
    hc.AmbientTemp = 293.0
    analysis.addObject(hc)
    doc.recompute()
    wd = mesh_and_solve(case, doc, geom, analysis, solver, mesh)
    for ext in ("inp", "frd", "dat"):
        shutil.copy(os.path.join(wd, "Mesh." + ext), os.path.join(OUT, case + "." + ext))
    collect(case, doc, wd, mesh, {"analysis_type": "thermomech", "mesh": {"max": 5.0, "min": 0.0, "order": 2}})


CASES = {
    "snapshot_static": case_snapshot_static,
    "snapshot_thermo": case_snapshot_thermo,
    "cantilever": lambda: case_cantilever(),
    "selfweight": lambda: case_cantilever("selfweight", force=False, selfweight=True),
    "frequency": lambda: case_cantilever("frequency", analysis_type="frequency", force=False),
    "plate_hole": case_plate_hole,
    "pressure": case_pressure,
    "bracket": case_bracket,
    "thermal_bar": case_thermal,
}

os.makedirs(OUT, exist_ok=True)
os.makedirs(WORK, exist_ok=True)
for name, fn in CASES.items():
    if ONLY and name not in ONLY:
        continue
    fn()
