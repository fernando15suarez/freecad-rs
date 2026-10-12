//! Tetrahedral FEM mesh: Gmsh driving (FreeCAD GmshTools style .geo), `.msh` v4.1 parsing,
//! sub-element (Face/Edge/Vertex) -> node / element-face lookup and boundary extraction.

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::process::Command;

use crate::model::{MeshSettings, Signature, SubKind, SubRef};
use crate::FemError;

/// A volume element in CalculiX node order (node ids, not indices).
#[derive(Debug, Clone, PartialEq)]
pub struct VolumeElement {
    pub id: u32,
    /// 4 (C3D4) or 10 (C3D10) node ids in CalculiX order.
    pub nodes: Vec<u32>,
    /// Gmsh volume entity tag (= FreeCAD `Solid<n>` index).
    pub solid: usize,
}

impl VolumeElement {
    pub fn ccx_type(&self) -> &'static str {
        if self.nodes.len() == 10 {
            "C3D10"
        } else {
            "C3D4"
        }
    }
}

/// CalculiX tetrahedron face definitions (0-based corner indices), faces 1..4:
/// F1 = 1-2-3, F2 = 1-4-2, F3 = 2-4-3, F4 = 3-4-1.
pub const TET_FACES: [[usize; 3]; 4] = [[0, 1, 2], [0, 3, 1], [1, 3, 2], [2, 3, 0]];
/// Midside nodes (0-based, C3D10) of each face, matching the corner order of `TET_FACES`
/// (edge c0-c1, c1-c2, c2-c0).
pub const TET10_FACE_MIDS: [[usize; 3]; 4] = [[4, 5, 6], [7, 8, 4], [8, 9, 5], [9, 7, 6]];

/// A tetrahedral mesh with the Gmsh entity classification kept, so FreeCAD-style references
/// (`Face3`) map straight to nodes and element faces.
#[derive(Debug, Clone, Default)]
pub struct FemMesh {
    /// 1 or 2.
    pub order: u8,
    /// Node ids in ascending order; `coords[i]` belongs to `node_ids[i]`.
    pub node_ids: Vec<u32>,
    pub coords: Vec<[f64; 3]>,
    index: HashMap<u32, usize>,
    pub elements: Vec<VolumeElement>,
    /// Surface triangles per Gmsh surface tag (= FreeCAD face index), node ids in tri3/tri6 order.
    pub surfaces: BTreeMap<usize, Vec<Vec<u32>>>,
    /// Line elements per curve tag (= FreeCAD edge index).
    pub curves: BTreeMap<usize, Vec<Vec<u32>>>,
    /// Mesh node per point tag (= FreeCAD vertex index).
    pub points: BTreeMap<usize, u32>,
}

/// Where a referenced face sits in the volume mesh: (element id, CalculiX face number 1..4).
pub type ElementFace = (u32, u8);

impl FemMesh {
    pub fn node_count(&self) -> usize {
        self.node_ids.len()
    }
    pub fn element_count(&self) -> usize {
        self.elements.len()
    }
    /// Index of a node id into `coords`.
    pub fn node_index(&self, id: u32) -> Option<usize> {
        self.index.get(&id).copied()
    }
    pub fn coord(&self, id: u32) -> [f64; 3] {
        self.coords[self.index[&id]]
    }
    fn rebuild_index(&mut self) {
        self.index = self.node_ids.iter().enumerate().map(|(i, &n)| (n, i)).collect();
    }

    /// Axis-aligned bounds `[xmin, ymin, zmin, xmax, ymax, zmax]`.
    pub fn bounds(&self) -> [f64; 6] {
        let mut b = [f64::MAX, f64::MAX, f64::MAX, f64::MIN, f64::MIN, f64::MIN];
        for p in &self.coords {
            for k in 0..3 {
                b[k] = b[k].min(p[k]);
                b[k + 3] = b[k + 3].max(p[k]);
            }
        }
        b
    }
    fn diagonal(&self) -> f64 {
        let b = self.bounds();
        ((b[3] - b[0]).powi(2) + (b[4] - b[1]).powi(2) + (b[5] - b[2]).powi(2)).sqrt().max(1e-12)
    }

    /// Total volume of the tetrahedra (straight-sided approximation for C3D10).
    pub fn volume(&self) -> f64 {
        self.elements
            .iter()
            .map(|e| {
                let p: Vec<[f64; 3]> = e.nodes[..4].iter().map(|&n| self.coord(n)).collect();
                tet_volume(p[0], p[1], p[2], p[3]).abs()
            })
            .sum()
    }

    /// Number of entities of a kind (faces, edges, ...) present in the mesh.
    pub fn entity_tags(&self, kind: SubKind) -> Vec<usize> {
        match kind {
            SubKind::Face => self.surfaces.keys().copied().collect(),
            SubKind::Edge => self.curves.keys().copied().collect(),
            SubKind::Vertex => self.points.keys().copied().collect(),
            SubKind::Solid => self.elements.iter().map(|e| e.solid).collect::<BTreeSet<_>>().into_iter().collect(),
        }
    }

    /// All mesh nodes on a sub-element (FreeCAD getNodesByFace / ByEdge / ByVertex / BySolid).
    pub fn nodes_on(&self, kind: SubKind, tag: usize) -> BTreeSet<u32> {
        let mut s = BTreeSet::new();
        match kind {
            SubKind::Face => {
                for t in self.surfaces.get(&tag).into_iter().flatten() {
                    s.extend(t.iter().copied());
                }
            }
            SubKind::Edge => {
                for t in self.curves.get(&tag).into_iter().flatten() {
                    s.extend(t.iter().copied());
                }
            }
            SubKind::Vertex => {
                if let Some(&n) = self.points.get(&tag) {
                    s.insert(n);
                }
            }
            SubKind::Solid => {
                for e in self.elements.iter().filter(|e| e.solid == tag) {
                    s.extend(e.nodes.iter().copied());
                }
            }
        }
        s
    }

    /// Map from sorted corner triple to (element id, ccx face number) for every tet face.
    pub fn face_lookup(&self) -> HashMap<[u32; 3], Vec<ElementFace>> {
        let mut m: HashMap<[u32; 3], Vec<ElementFace>> = HashMap::with_capacity(self.elements.len() * 4);
        for e in &self.elements {
            for (fi, f) in TET_FACES.iter().enumerate() {
                let mut k = [e.nodes[f[0]], e.nodes[f[1]], e.nodes[f[2]]];
                k.sort_unstable();
                m.entry(k).or_default().push((e.id, (fi + 1) as u8));
            }
        }
        m
    }

    /// Element faces lying on B-rep face `tag`, sorted by element id (for *DLOAD / *DFLUX).
    pub fn element_faces_on(&self, tag: usize, lookup: &HashMap<[u32; 3], Vec<ElementFace>>) -> Vec<ElementFace> {
        let mut out = Vec::new();
        for t in self.surfaces.get(&tag).into_iter().flatten() {
            let mut k = [t[0], t[1], t[2]];
            k.sort_unstable();
            if let Some(v) = lookup.get(&k) {
                out.extend(v.iter().copied());
            }
        }
        out.sort_unstable();
        out.dedup();
        out
    }

    /// Mesh-derived signature of an entity (center, area/length, plane normal).
    pub fn signature(&self, kind: SubKind, tag: usize) -> Option<Signature> {
        match kind {
            SubKind::Face => {
                let tris = self.surfaces.get(&tag)?;
                let (mut area, mut c, mut n) = (0.0, [0.0; 3], [0.0; 3]);
                for t in tris {
                    for sub in split_triangle(t) {
                        let p = sub.map(|id| self.coord(id));
                        let nn = cross(sub3(p[1], p[0]), sub3(p[2], p[0]));
                        let a = 0.5 * norm(nn);
                        area += a;
                        for k in 0..3 {
                            c[k] += a * (p[0][k] + p[1][k] + p[2][k]) / 3.0;
                            n[k] += 0.5 * nn[k];
                        }
                    }
                }
                if area <= 0.0 {
                    return None;
                }
                let c = c.map(|v| v / area);
                // A planar face has |sum of normals| == area.
                let nl = norm(n);
                let normal = if (nl - area).abs() < 1e-6 * area { Some(n.map(|v| v / nl)) } else { None };
                Some(Signature { center: c, size: area, normal })
            }
            SubKind::Edge => {
                let segs = self.curves.get(&tag)?;
                let (mut len, mut c) = (0.0, [0.0; 3]);
                for s in segs {
                    // line3: [end, end, mid]
                    let pts: Vec<[f64; 3]> = if s.len() == 3 {
                        vec![self.coord(s[0]), self.coord(s[2]), self.coord(s[1])]
                    } else {
                        s.iter().map(|&n| self.coord(n)).collect()
                    };
                    for w in pts.windows(2) {
                        let l = norm(sub3(w[1], w[0]));
                        len += l;
                        for k in 0..3 {
                            c[k] += l * 0.5 * (w[0][k] + w[1][k]);
                        }
                    }
                }
                if len <= 0.0 {
                    return None;
                }
                Some(Signature { center: c.map(|v| v / len), size: len, normal: None })
            }
            SubKind::Vertex => self.points.get(&tag).map(|&n| Signature { center: self.coord(n), size: 0.0, normal: None }),
            SubKind::Solid => None,
        }
    }

    /// Resolve a FreeCAD-style reference to the Gmsh entity tag. With a signature the indexed
    /// entity is verified and, if it does not match (the model was renumbered), the best matching
    /// entity of the same kind is used.
    pub fn resolve(&self, r: &SubRef) -> Result<usize, FemError> {
        let tags = self.entity_tags(r.kind);
        let Some(sig) = &r.signature else {
            if tags.contains(&r.index) {
                return Ok(r.index);
            }
            return Err(FemError::Reference(format!("{} does not exist in the mesh ({} {}s)", r.name(), tags.len(), r.kind.prefix())));
        };
        if r.kind == SubKind::Solid {
            return if tags.contains(&r.index) { Ok(r.index) } else { Err(FemError::Reference(format!("{} not in mesh", r.name()))) };
        }
        let diag = self.diagonal();
        let score = |t: usize| -> f64 {
            let Some(m) = self.signature(r.kind, t) else { return f64::MAX };
            let d = norm(sub3(m.center, sig.center)) / diag;
            let s = if sig.size > 0.0 || m.size > 0.0 { (m.size - sig.size).abs() / m.size.max(sig.size) } else { 0.0 };
            let n = match (m.normal, sig.normal) {
                (Some(a), Some(b)) => 1.0 - dot(a, b).abs() / (norm(a) * norm(b)).max(1e-30),
                (None, None) => 0.0,
                _ => 0.05,
            };
            d + s + n
        };
        if tags.contains(&r.index) && score(r.index) < 0.01 {
            return Ok(r.index);
        }
        let best = tags.iter().map(|&t| (score(t), t)).min_by(|a, b| a.0.total_cmp(&b.0));
        match best {
            Some((s, t)) if s < 0.05 => Ok(t),
            _ => Err(FemError::Reference(format!("{} not found: no {} matches its signature", r.name(), r.kind.prefix()))),
        }
    }

    /// Outer (boundary) faces of the volume mesh as display triangles. Quadratic faces are split
    /// into 4 triangles through their midside nodes so per-node results can be shown.
    pub fn boundary(&self) -> SurfaceMesh {
        let mut count: HashMap<[u32; 3], (u32, usize, usize)> = HashMap::new();
        for (ei, e) in self.elements.iter().enumerate() {
            for (fi, f) in TET_FACES.iter().enumerate() {
                let mut k = [e.nodes[f[0]], e.nodes[f[1]], e.nodes[f[2]]];
                k.sort_unstable();
                count.entry(k).and_modify(|v| v.0 += 1).or_insert((1, ei, fi));
            }
        }
        // B-rep face tag for each boundary triangle (from the classified surface triangles).
        let mut face_of: HashMap<[u32; 3], u32> = HashMap::new();
        for (&tag, tris) in &self.surfaces {
            for t in tris {
                let mut k = [t[0], t[1], t[2]];
                k.sort_unstable();
                face_of.insert(k, tag as u32);
            }
        }
        let mut keys: Vec<_> = count.iter().filter(|(_, v)| v.0 == 1).map(|(k, v)| (*k, v.1, v.2)).collect();
        keys.sort_unstable();
        let mut s = SurfaceMesh::default();
        for (k, ei, fi) in keys {
            let e = &self.elements[ei];
            let c = TET_FACES[fi];
            // ccx face corner order 1-2-3 / 1-4-2 / ... points into the element; flip for outward.
            let corners = [e.nodes[c[0]], e.nodes[c[2]], e.nodes[c[1]]];
            let tag = face_of.get(&k).copied().unwrap_or(0);
            let idx = |n: u32| self.index[&n] as u32;
            if e.nodes.len() == 10 {
                let m = TET10_FACE_MIDS[fi];
                // mids of edges (c0,c1), (c1,c2), (c2,c0); after the flip: (c0,c2)=m[2], (c2,c1)=m[1], (c1,c0)=m[0]
                let (a, b, cc) = (corners[0], corners[1], corners[2]);
                let (ab, bc, ca) = (e.nodes[m[2]], e.nodes[m[1]], e.nodes[m[0]]);
                for t in [[a, ab, ca], [ab, b, bc], [ca, bc, cc], [ab, bc, ca]] {
                    s.triangles.push(t.map(idx));
                    s.tri_face.push(tag);
                }
            } else {
                s.triangles.push(corners.map(idx));
                s.tri_face.push(tag);
            }
        }
        s
    }
}

/// Display surface of a mesh. Indices point into the mesh/result node arrays
/// (`FemMesh::coords`, `Results::displacement`, ...).
#[derive(Debug, Clone, Default)]
pub struct SurfaceMesh {
    pub triangles: Vec<[u32; 3]>,
    /// B-rep face index (FreeCAD `Face<n>`) of each triangle, 0 if unknown.
    pub tri_face: Vec<u32>,
}

/// Split a tri3/tri6 into straight sub-triangles (tri6 order: c0 c1 c2 m01 m12 m20).
fn split_triangle(t: &[u32]) -> Vec<[u32; 3]> {
    if t.len() == 6 {
        vec![[t[0], t[3], t[5]], [t[3], t[1], t[4]], [t[5], t[4], t[2]], [t[3], t[4], t[5]]]
    } else {
        vec![[t[0], t[1], t[2]]]
    }
}

pub(crate) fn sub3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
pub(crate) fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
pub(crate) fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
pub(crate) fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}
pub(crate) fn tet_volume(a: [f64; 3], b: [f64; 3], c: [f64; 3], d: [f64; 3]) -> f64 {
    dot(cross(sub3(b, a), sub3(c, a)), sub3(d, a)) / 6.0
}

/// Signed volume of a tetrahedron (positive when d is on the side of the right-handed abc normal).
pub fn tet_volume_pub(a: [f64; 3], b: [f64; 3], c: [f64; 3], d: [f64; 3]) -> f64 {
    tet_volume(a, b, c, d)
}

/// Triangle area (FreeCAD meshtools.get_triangle_area).
pub(crate) fn tri_area(a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> f64 {
    0.5 * norm(cross(sub3(b, a), sub3(c, a)))
}

// ------------------------------------------------------------------------------------------
// .msh v4.1 (ASCII) parser
// ------------------------------------------------------------------------------------------

/// Gmsh tet10 node order -> CalculiX C3D10 order as FreeCAD writes it. Gmsh numbers the last two
/// midside nodes (2-3), (1-3) while Abaqus/CalculiX use (1-3), (2-3); FreeCAD's UNV import also
/// rotates the first three corners (c, a, b), which we reproduce so element lines are identical.
const GMSH_TET10_TO_CCX: [usize; 10] = [2, 0, 1, 3, 6, 4, 5, 8, 7, 9];
/// Same corner rotation for linear tetrahedra.
const GMSH_TET4_TO_CCX: [usize; 4] = [2, 0, 1, 3];

/// Parse an ASCII `.msh` 4.1 file as written by Gmsh with `Mesh.SaveAll = 1`.
pub fn parse_msh(text: &str) -> Result<FemMesh, FemError> {
    let err = |m: &str| FemError::Mesh(format!("msh parse error: {m}"));
    let mut lines = text.lines().map(str::trim);
    let mut mesh = FemMesh::default();
    let mut nodes: Vec<(u32, [f64; 3])> = Vec::new();
    let mut version_ok = false;
    while let Some(l) = lines.next() {
        match l {
            "$MeshFormat" => {
                let v = lines.next().ok_or_else(|| err("format"))?;
                let mut it = v.split_whitespace();
                let ver = it.next().unwrap_or("");
                let binary = it.next().unwrap_or("0");
                if !ver.starts_with('4') || binary != "0" {
                    return Err(err(&format!("need ASCII msh 4.x, got '{v}'")));
                }
                version_ok = true;
            }
            "$Nodes" => {
                let h: Vec<usize> = parse_nums(lines.next().ok_or_else(|| err("nodes header"))?)?;
                let nblocks = h[0];
                nodes.reserve(h[1]);
                for _ in 0..nblocks {
                    let b: Vec<usize> = parse_nums(lines.next().ok_or_else(|| err("node block"))?)?;
                    let (parametric, n) = (b[2], b[3]);
                    let mut tags = Vec::with_capacity(n);
                    for _ in 0..n {
                        tags.push(lines.next().ok_or_else(|| err("node tag"))?.parse::<u32>().map_err(|_| err("node tag"))?);
                    }
                    for t in tags {
                        let c: Vec<f64> = parse_nums(lines.next().ok_or_else(|| err("node coords"))?)?;
                        let _ = parametric;
                        nodes.push((t, [c[0], c[1], c[2]]));
                    }
                }
            }
            "$Elements" => {
                let h: Vec<usize> = parse_nums(lines.next().ok_or_else(|| err("elements header"))?)?;
                for _ in 0..h[0] {
                    let b: Vec<usize> = parse_nums(lines.next().ok_or_else(|| err("element block"))?)?;
                    let (dim, tag, ty, n) = (b[0], b[1], b[2], b[3]);
                    for _ in 0..n {
                        let v: Vec<u32> = parse_nums(lines.next().ok_or_else(|| err("element"))?)?;
                        let (id, ns) = (v[0], &v[1..]);
                        match (dim, ty) {
                            (0, 15) => {
                                mesh.points.insert(tag, ns[0]);
                            }
                            (1, 1) | (1, 8) => mesh.curves.entry(tag).or_default().push(ns.to_vec()),
                            (2, 2) | (2, 9) => mesh.surfaces.entry(tag).or_default().push(ns.to_vec()),
                            (3, 4) => mesh.elements.push(VolumeElement { id, nodes: GMSH_TET4_TO_CCX.iter().map(|&i| ns[i]).collect(), solid: tag }),
                            (3, 11) => mesh.elements.push(VolumeElement {
                                id,
                                nodes: GMSH_TET10_TO_CCX.iter().map(|&i| ns[i]).collect(),
                                solid: tag,
                            }),
                            (3, _) => return Err(FemError::Mesh(format!("unsupported volume element type {ty} (only tetrahedra)"))),
                            _ => {} // quads etc. are not produced for solids
                        }
                    }
                }
            }
            _ => {}
        }
    }
    if !version_ok {
        return Err(err("missing $MeshFormat"));
    }
    if mesh.elements.is_empty() {
        return Err(FemError::Mesh("the mesh has no volume elements (is the shape a closed solid?)".into()));
    }
    nodes.sort_unstable_by_key(|n| n.0);
    mesh.node_ids = nodes.iter().map(|n| n.0).collect();
    mesh.coords = nodes.iter().map(|n| n.1).collect();
    mesh.rebuild_index();
    mesh.order = if mesh.elements[0].nodes.len() == 10 { 2 } else { 1 };
    Ok(mesh)
}

fn parse_nums<T: std::str::FromStr>(l: &str) -> Result<Vec<T>, FemError> {
    l.split_whitespace()
        .map(|s| s.parse::<T>().map_err(|_| FemError::Mesh(format!("msh parse error: bad number '{s}'"))))
        .collect()
}

// ------------------------------------------------------------------------------------------
// Gmsh driving
// ------------------------------------------------------------------------------------------

/// Write the Gmsh .geo script the way FreeCAD's `GmshTools.write_geo` does. `refs` are the
/// sub-elements referenced by constraints, written as physical groups (documentation only, all
/// elements are saved and classified by entity anyway).
pub fn write_geo(settings: &MeshSettings, brep_file: &str, mesh_file: &str, refs: &[SubRef]) -> String {
    let mut g = String::new();
    let clmax = if settings.max_size > 0.0 { settings.max_size } else { 1e22 };
    g.push_str("// geo file for meshing with Gmsh meshing software created by FreeCAD\n\n");
    g.push_str("// enable multi-core processing\n");
    let _ = writeln!(g, "General.NumThreads = {};\n", settings.threads.max(1));
    g.push_str("// open brep geometry\n");
    let _ = writeln!(g, "Merge \"{brep_file}\";\n");
    let mut groups: BTreeMap<String, (SubKind, usize)> = BTreeMap::new();
    for r in refs {
        groups.insert(r.name(), (r.kind, r.index));
    }
    if !groups.is_empty() {
        g.push_str("// group data\n");
        for (name, (kind, idx)) in &groups {
            let phys = match kind {
                SubKind::Solid => "Volume",
                SubKind::Face => "Surface",
                SubKind::Edge => "Line",
                SubKind::Vertex => "Point",
            };
            let _ = writeln!(g, "Physical {phys}(\"{name}\") = {{{idx}}};");
        }
        g.push('\n');
    }
    g.push_str("// Characteristic Length\n");
    if !settings.refinements.is_empty() {
        g.push_str("// Characteristic Length according CharacteristicLengthMap\n");
        for r in &settings.refinements {
            for s in &r.refs {
                let _ = writeln!(g, "// {}", s.name());
                let pts = match s.kind {
                    SubKind::Vertex => format!("{}", s.index),
                    SubKind::Edge => format!("PointsOf {{ Curve{{{}}}; }}", s.index),
                    SubKind::Face => format!("PointsOf {{ Surface{{{}}}; }}", s.index),
                    SubKind::Solid => format!("PointsOf {{ Volume{{{}}}; }}", s.index),
                };
                let _ = writeln!(g, "Characteristic Length {{ {pts} }} = {};", crate::fmt::py(r.size));
            }
        }
        g.push('\n');
    }
    g.push_str("// no boundary layer settings for this mesh\n");
    g.push_str("// min, max Characteristic Length\n");
    let _ = writeln!(g, "Mesh.CharacteristicLengthMax = {};", crate::fmt::py(clmax));
    let _ = writeln!(g, "Mesh.CharacteristicLengthMin = {};", crate::fmt::py(settings.min_size));
    let _ = writeln!(
        g,
        "Mesh.MeshSizeFromCurvature = {}; // number of elements per 2*pi radians, 0 to deactivate\n",
        settings.size_from_curvature
    );
    g.push_str("// optimize the mesh\n");
    let _ = writeln!(g, "Mesh.Optimize = {};", settings.optimize as u8);
    let _ = writeln!(g, "Mesh.OptimizeNetgen = {};", settings.optimize_netgen as u8);
    g.push_str("// High-order meshes optimization (0=none, 1=optimization, 2=elastic+optimization, 3=elastic, 4=fast curving)\n");
    g.push_str("Mesh.HighOrderOptimize = 0;\n\n");
    g.push_str("// mesh order\n");
    let _ = writeln!(g, "Mesh.ElementOrder = {};", settings.order.clamp(1, 2));
    if settings.order == 2 {
        let _ = writeln!(
            g,
            "Mesh.SecondOrderLinear = {}; // Second order nodes are created by linear interpolation instead by curvilinear",
            settings.second_order_linear as u8
        );
    }
    g.push('\n');
    g.push_str("// mesh algorithm, only a few algorithms are usable with 3D boundary layer generation\n");
    g.push_str("// 2D mesh algorithm (1=MeshAdapt, 2=Automatic, 5=Delaunay, 6=Frontal, 7=BAMG, 8=DelQuad, 9=Packing Parallelograms, 11=Quasi-structured Quad)\n");
    let _ = writeln!(g, "Mesh.Algorithm = {};", settings.algorithm_2d);
    g.push_str("// 3D mesh algorithm (1=Delaunay, 2=New Delaunay, 4=Frontal, 7=MMG3D, 9=R-tree, 10=HTX)\n");
    let _ = writeln!(g, "Mesh.Algorithm3D = {};\n", settings.algorithm_3d);
    g.push_str("// subdivision algorithm\nMesh.SubdivisionAlgorithm = 0;\n\n");
    g.push_str("// incomplete second order elements\nMesh.SecondOrderIncomplete = 0;\n\n");
    g.push_str("// meshing\n");
    let _ = writeln!(
        g,
        "Geometry.Tolerance = {}; // set geometrical tolerance (also used for merging nodes)",
        crate::fmt::py(settings.geometry_tolerance)
    );
    g.push_str("Mesh  3;\nCoherence Mesh; // Remove duplicate vertices\n\n");
    g.push_str("// save\n");
    g.push_str("// Ignore Physical definitions and save all elements;\nMesh.SaveAll = 1;\n");
    g.push_str("Mesh.MshFileVersion = 4.1;\nMesh.Binary = 0;\n");
    let _ = writeln!(g, "Save \"{mesh_file}\";\n");
    g
}

/// Result of a Gmsh run.
#[derive(Debug, Clone)]
pub struct MeshRun {
    pub mesh: FemMesh,
    pub geo_file: PathBuf,
    pub msh_file: PathBuf,
    /// Gmsh's stderr (warnings).
    pub log: String,
}

/// Count (solids, faces, edges) of an ASCII OCCT BREP file from its TShapes section.
pub fn brep_counts(text: &str) -> (usize, usize, usize) {
    let (mut so, mut fa, mut ed) = (0, 0, 0);
    for l in text.lines() {
        match l.trim_end() {
            "So" => so += 1,
            "Fa" => fa += 1,
            "Ed" => ed += 1,
            _ => {}
        }
    }
    (so, fa, ed)
}

/// FreeCAD 1.1's GmshTools writes a physical group for every solid, face and edge of the shape.
pub fn all_entity_groups(brep_text: &str) -> Vec<SubRef> {
    let (so, fa, ed) = brep_counts(brep_text);
    let mut v: Vec<SubRef> = (1..=so).map(SubRef::solid).collect();
    v.extend((1..=fa).map(SubRef::face));
    v.extend((1..=ed).map(SubRef::edge));
    v
}

/// Mesh `brep` (a file in `work_dir`) with Gmsh, FreeCAD style. `refs` are extra physical groups
/// (all solids, faces and edges of the BREP are always written, as FreeCAD does).
pub fn run_gmsh(work_dir: &Path, brep: &Path, settings: &MeshSettings, refs: &[SubRef]) -> Result<MeshRun, FemError> {
    let mut refs: Vec<SubRef> = refs.to_vec();
    refs.extend(all_entity_groups(&std::fs::read_to_string(brep)?));
    let gmsh = crate::tools::gmsh()?;
    let geo_file = work_dir.join("shape2mesh.geo");
    let msh_file = work_dir.join("mesh.msh");
    let rel = |p: &Path| p.file_name().unwrap().to_string_lossy().into_owned();
    let brep_name = if brep.parent() == Some(work_dir) {
        rel(brep)
    } else {
        brep.to_string_lossy().into_owned()
    };
    let geo = write_geo(settings, &brep_name, &rel(&msh_file), &refs);
    std::fs::write(&geo_file, geo)?;
    let _ = std::fs::remove_file(&msh_file);
    let out = Command::new(&gmsh)
        .arg("-")
        .arg(&geo_file)
        .current_dir(work_dir)
        .output()
        .map_err(|e| FemError::ToolFailed { tool: "gmsh", detail: format!("could not start {}: {e}", gmsh.display()) })?;
    let log = String::from_utf8_lossy(&out.stderr).into_owned() + &String::from_utf8_lossy(&out.stdout);
    if !out.status.success() || !msh_file.exists() {
        let errs: Vec<&str> = log.lines().filter(|l| l.contains("Error")).collect();
        return Err(FemError::ToolFailed {
            tool: "gmsh",
            detail: if errs.is_empty() { log.lines().rev().take(10).collect::<Vec<_>>().join("\n") } else { errs.join("\n") },
        });
    }
    let mesh = parse_msh(&std::fs::read_to_string(&msh_file)?)?;
    Ok(MeshRun { mesh, geo_file, msh_file, log })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn subref_parse() {
        assert_eq!(SubRef::parse("Face12"), Some(SubRef::face(12)));
        assert_eq!(SubRef::parse("Vertex1"), Some(SubRef::vertex(1)));
        assert_eq!(SubRef::parse("Face0"), None);
        assert_eq!(SubRef::parse("Shell1"), None);
    }
}
