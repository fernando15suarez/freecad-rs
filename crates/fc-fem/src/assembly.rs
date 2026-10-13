//! Multi-solid (assembly) analyses, the way FreeCAD's FEM workbench analyses several parts:
//!
//! * [`Joining::Fragments`]: the parts are combined with BooleanFragments (OCCT general fuse).
//!   Touching parts then share their common faces, Gmsh meshes the shared face once and the
//!   mesh is conformal: the parts are bonded without any constraint. This is FreeCAD's
//!   documented way to analyse assemblies and multi-material bodies.
//! * [`Joining::Separate`]: the parts are only collected in a compound and meshed separately
//!   (no `Coherence Mesh`, so touching faces keep their own nodes). They are joined by tie
//!   (bonded) or contact interactions, e.g. from [`detect_contacts`].
//!
//! Every part gets its own material (parts with equal materials share one FreeCAD
//! MaterialSolid). [`PartMap`]s record which solids / faces of the analysed shape each part
//! became, for references, per-part results and per-part colours.

use std::collections::{BTreeMap, BTreeSet};

use fc_occt::Shape;
use serde::{Deserialize, Serialize};

use crate::material::Material;
use crate::mesh::{cross, dot, norm, sub3};
use crate::model::{Analysis, InteractionKind, MaterialAssignment, PartMap, ShapeSource, SubRef};
use crate::results::Extreme;
use crate::solve::Solution;
use crate::FemError;

/// One part of an assembly analysis: its posed solid(s) and material.
#[derive(Debug, Clone)]
pub struct AssemblyPart {
    pub name: String,
    pub shape: Shape,
    pub material: Material,
}

impl AssemblyPart {
    pub fn new(name: &str, shape: Shape, material: Material) -> Self {
        AssemblyPart { name: name.to_string(), shape, material }
    }
}

/// How the parts are combined for meshing.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum Joining {
    /// BooleanFragments: one conformal mesh, touching faces shared (bonded). `fuzzy` (mm, 0 =
    /// exact) lets OCCT treat nearly touching faces as touching.
    Fragments { fuzzy: f64 },
    /// A compound of separately meshed parts; join them with ties or contact.
    Separate,
}

impl Analysis {
    /// A static analysis of several parts, each with its own material. The analysed shape is
    /// the parts' BooleanFragments (or plain compound), named "BooleanFragments" ("Compound").
    pub fn from_parts(parts: &[AssemblyPart], joining: Joining) -> Result<Analysis, FemError> {
        if parts.is_empty() {
            return Err(FemError::Geometry("no parts to analyse".into()));
        }
        for p in parts {
            if p.shape.is_null() || p.shape.solid_count() == 0 {
                return Err(FemError::Geometry(format!("part {} has no solid", p.name)));
            }
        }
        let shapes: Vec<Shape> = parts.iter().map(|p| p.shape.clone()).collect();
        let (fuse, fuzzy) = match joining {
            Joining::Fragments { fuzzy } => (true, fuzzy),
            Joining::Separate => (false, 0.0),
        };
        let fr = Shape::fragments(&shapes, fuse, fuzzy).map_err(|e| FemError::Geometry(format!("BooleanFragments failed: {e}")))?;
        // A solid claimed by two parts is an interference (overlapping volume): it goes to the
        // first part only.
        let mut taken = BTreeSet::new();
        let mut maps = Vec::new();
        for (i, p) in parts.iter().enumerate() {
            let solids: Vec<usize> = fr.solids[i].iter().copied().filter(|s| taken.insert(*s)).collect();
            if solids.is_empty() {
                return Err(FemError::Geometry(format!("part {} has no solid of its own in the BooleanFragments", p.name)));
            }
            maps.push(PartMap { name: p.name.clone(), solids, faces: fr.faces[i].clone() });
        }
        let mut a = Analysis::new(ShapeSource::Shape(fr.shape), parts[0].material.clone());
        a.shape_name = if fuse { "BooleanFragments" } else { "Compound" }.into();
        if !fuse {
            a.mesh.coherence = false;
        }
        // one FreeCAD MaterialSolid per distinct material
        let mut groups: Vec<(Material, Vec<usize>)> = Vec::new();
        for (p, m) in parts.iter().zip(&maps) {
            match groups.iter_mut().find(|g| g.0 == p.material) {
                Some(g) => g.1.extend(&m.solids),
                None => groups.push((p.material.clone(), m.solids.clone())),
            }
        }
        if groups.len() > 1 {
            a.materials = groups
                .into_iter()
                .enumerate()
                .map(|(i, (material, mut solids))| {
                    solids.sort_unstable();
                    let name = if i == 0 { "FemMaterial".to_string() } else { format!("FemMaterial{i:03}") };
                    MaterialAssignment { name, material, solids: solids.into_iter().map(SubRef::solid).collect() }
                })
                .collect();
        }
        a.parts = maps;
        Ok(a)
    }

    /// The analysed shape when it is in memory.
    pub fn shape_in_memory(&self) -> Option<&Shape> {
        match &self.shape {
            ShapeSource::Shape(s) => Some(s),
            _ => None,
        }
    }

    /// References (with signatures) to the faces of the analysed shape that came from
    /// `Face<local>` of part `part` (several when BooleanFragments split it).
    pub fn part_face(&self, part: &str, local: usize) -> Result<Vec<SubRef>, FemError> {
        let p = self.part(part).ok_or_else(|| FemError::Reference(format!("no part named {part}")))?;
        let faces = p.faces_of(local);
        if faces.is_empty() {
            return Err(FemError::Reference(format!("{part} has no Face{local}")));
        }
        Ok(faces.into_iter().map(|g| self.face_ref(g)).collect())
    }

    /// `Face<global>` of the analysed shape, with its signature when the shape is in memory.
    pub fn face_ref(&self, global: usize) -> SubRef {
        self.shape_in_memory().and_then(|s| SubRef::face_of(s, global)).unwrap_or_else(|| SubRef::face(global))
    }

    /// Ties for detected contact pairs (slave = the smaller face). Pairs that are already
    /// bonded by a shared face are skipped. Returns the number of ties added.
    pub fn tie_contacts(&mut self, pairs: &[ContactPair], tolerance: f64) -> usize {
        let mut n = 0;
        for p in pairs.iter().filter(|p| !p.shared) {
            let (s, m) = (self.face_ref(p.slave_face), self.face_ref(p.master_face));
            self.add_interaction(InteractionKind::Tie { tolerance, adjust: false }, vec![s], vec![m]);
            n += 1;
        }
        n
    }
}

/// Kind of the touching surfaces of a [`ContactPair`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ContactKind {
    Planar,
    Cylindrical,
}

/// Two faces of different parts that touch (or nearly touch).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ContactPair {
    /// Part indices (into `Analysis::parts`) of the slave and master face.
    pub slave_part: usize,
    pub master_part: usize,
    /// `Face<n>` of the analysed shape: the slave is the smaller face.
    pub slave_face: usize,
    pub master_face: usize,
    pub kind: ContactKind,
    /// Estimated touching area in mm² (part of the slave face within the tolerance).
    pub area: f64,
    /// Largest gap found between the touching parts of the faces (mm).
    pub gap: f64,
    /// The faces are one shared face of a BooleanFragments shape (already bonded).
    pub shared: bool,
}

/// Find touching face pairs between different parts of a multi-solid analysis: coincident or
/// overlapping planar faces with opposite normals, and coaxial cylindrical faces of (nearly)
/// equal radius, within `tolerance` mm. Shared faces of a BooleanFragments shape are reported
/// with `shared = true`.
pub fn detect_contacts(a: &Analysis, tolerance: f64) -> Result<Vec<ContactPair>, FemError> {
    let shape = a.shape_in_memory().ok_or_else(|| FemError::Geometry("contact detection needs the shape in memory".into()))?;
    let tol = tolerance.max(1e-7);
    let infos = shape.faces();
    let mut owner: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (pi, p) in a.parts.iter().enumerate() {
        for &s in &p.solids {
            for f in shape.solid_faces(s).map_err(|e| FemError::Geometry(e.to_string()))? {
                let o = owner.entry(f).or_default();
                if !o.contains(&pi) {
                    o.push(pi);
                }
            }
        }
    }
    let mut out = Vec::new();
    // shared faces (fragments)
    for (&f, o) in &owner {
        if o.len() >= 2 {
            let kind = if infos[f - 1].kind == 1 { ContactKind::Cylindrical } else { ContactKind::Planar };
            out.push(ContactPair { slave_part: o[0], master_part: o[1], slave_face: f, master_face: f, kind, area: infos[f - 1].area, gap: 0.0, shared: true });
        }
    }
    // sample points of every face from a display triangulation
    let diag = {
        let b = shape.bbox_exact();
        ((b[3] - b[0]).powi(2) + (b[4] - b[1]).powi(2) + (b[5] - b[2]).powi(2)).sqrt()
    };
    let md = shape.mesh((diag * 2e-3).max(1e-3), 0.3).map_err(|e| FemError::Geometry(e.to_string()))?;
    let mut tris: BTreeMap<usize, Vec<[[f64; 3]; 3]>> = BTreeMap::new();
    for (t, &face) in md.tri_face.iter().enumerate() {
        let p = |k: usize| {
            let i = md.indices[3 * t + k] as usize;
            [md.positions[3 * i] as f64, md.positions[3 * i + 1] as f64, md.positions[3 * i + 2] as f64]
        };
        tris.entry(face as usize).or_default().push([p(0), p(1), p(2)]);
    }
    let single: Vec<(usize, usize)> = owner.iter().filter(|(_, o)| o.len() == 1).map(|(&f, o)| (f, o[0])).collect();
    for (i, &(fa, pa)) in single.iter().enumerate() {
        for &(fb, pb) in &single[i + 1..] {
            if pa == pb {
                continue;
            }
            let (ia, ib) = (&infos[fa - 1], &infos[fb - 1]);
            if !bbox_near(ia, ib, tol) {
                continue;
            }
            let kind = match (ia.kind, ib.kind) {
                (0, 0) => {
                    let (na, nb) = (unit(ia.normal), unit(ib.normal));
                    if dot(na, nb) > -0.999 || dot(sub3(ib.origin, ia.origin), na).abs() > tol {
                        continue;
                    }
                    ContactKind::Planar
                }
                (1, 1) => {
                    let (da, db) = (unit(ia.normal), unit(ib.normal));
                    if dot(da, db).abs() < 0.999 || (ia.radius - ib.radius).abs() > tol {
                        continue;
                    }
                    // axis distance
                    let d = sub3(ib.origin, ia.origin);
                    if norm(cross(d, da)) > tol {
                        continue;
                    }
                    ContactKind::Cylindrical
                }
                _ => continue,
            };
            // sample the smaller face, project onto the larger
            let (s, m, ps, pm) = if ia.area <= ib.area { (fa, fb, pa, pb) } else { (fb, fa, pb, pa) };
            let Some(st) = tris.get(&s) else { continue };
            let (area, gap) = overlap(shape, st, s, m, tol);
            if area > 1e-6 * infos[s - 1].area.max(1e-12) && area > 0.0 {
                out.push(ContactPair { slave_part: ps, master_part: pm, slave_face: s, master_face: m, kind, area, gap, shared: false });
            }
        }
    }
    Ok(out)
}

fn unit(v: [f64; 3]) -> [f64; 3] {
    let l = norm(v).max(1e-300);
    v.map(|x| x / l)
}

fn bbox_near(a: &fc_occt::FaceInfo, b: &fc_occt::FaceInfo, tol: f64) -> bool {
    (0..3).all(|k| a.bbox_min[k] <= b.bbox_max[k] + tol && b.bbox_min[k] <= a.bbox_max[k] + tol)
}

/// Area of the triangles `tris` (subdivided for sampling) whose centres lie within `tol` of
/// face `master` where its outward normal opposes face `slave`'s, and the largest such gap.
fn overlap(shape: &Shape, tris: &[[[f64; 3]; 3]], slave: usize, master: usize, tol: f64) -> (f64, f64) {
    let n = tris.len().max(1);
    // about 256 samples per face
    let mut level = 0;
    while n * 4usize.pow(level) < 256 && level < 4 {
        level += 1;
    }
    let mut area = 0.0;
    let mut gap: f64 = 0.0;
    for t in tris {
        let tn = cross(sub3(t[1], t[0]), sub3(t[2], t[0]));
        if norm(tn) < 1e-300 {
            continue;
        }
        for sub in subdivide(*t, level) {
            let a = 0.5 * norm(cross(sub3(sub[1], sub[0]), sub3(sub[2], sub[0])));
            let c = [0, 1, 2].map(|k| (sub[0][k] + sub[1][k] + sub[2][k]) / 3.0);
            // the sample on the exact slave surface (the triangulation deviates from curved faces)
            let Ok((c, ns)) = shape.face_point_normal(slave, c) else { continue };
            let Ok((q, nm)) = shape.face_point_normal(master, c) else { continue };
            let d = norm(sub3(q, c));
            if d > tol {
                continue;
            }
            // opposing surfaces: the outward normals of the two faces point against each other
            if dot(nm, ns) > -0.5 {
                continue;
            }
            area += a;
            gap = gap.max(d);
        }
    }
    (area, gap)
}

fn subdivide(t: [[f64; 3]; 3], level: u32) -> Vec<[[f64; 3]; 3]> {
    let mut v = vec![t];
    for _ in 0..level {
        let mut nv = Vec::with_capacity(v.len() * 4);
        for [a, b, c] in v {
            let m = |p: [f64; 3], q: [f64; 3]| [0.5 * (p[0] + q[0]), 0.5 * (p[1] + q[1]), 0.5 * (p[2] + q[2])];
            let (ab, bc, ca) = (m(a, b), m(b, c), m(c, a));
            nv.extend([[a, ab, ca], [ab, b, bc], [ca, bc, c], [ab, bc, ca]]);
        }
        v = nv;
    }
    v
}

/// Results of one part of a multi-solid analysis.
#[derive(Debug, Clone)]
pub struct PartResult {
    pub name: String,
    pub solids: Vec<usize>,
    pub node_count: usize,
    pub element_count: usize,
    /// Mesh volume of the part's elements (mm³).
    pub volume: f64,
    pub max_displacement: Option<Extreme>,
    pub max_von_mises: Option<Extreme>,
    /// Sum of the reaction totals of the constraints whose references all lie on this part.
    pub reaction: [f64; 3],
}

/// Per-part maxima and reactions of a solved multi-solid analysis. Nodal stresses on faces
/// shared by two parts are CalculiX's averages over all elements at the node, as FreeCAD
/// shows them.
pub fn part_results(a: &Analysis, sol: &Solution) -> Vec<PartResult> {
    let mesh = &sol.mesh;
    let r = &sol.results;
    let vm = &r.von_mises;
    let dm: Vec<f64> = r.displacement.iter().map(|d| norm(*d)).collect();
    // reactions: constraint -> part when all its face references belong to one part
    let shape = a.shape_in_memory();
    let face_part = |f: usize| -> Option<usize> {
        if let Some(s) = shape {
            a.parts.iter().position(|p| p.solids.iter().any(|&so| s.solid_faces(so).map(|v| v.contains(&f)).unwrap_or(false)))
        } else {
            a.parts.iter().position(|p| p.faces.iter().any(|x| x.1 == f))
        }
    };
    let mut reactions = vec![[0.0; 3]; a.parts.len()];
    for c in &a.constraints {
        let Some(rf) = r.reactions.get(&c.name.to_uppercase()) else { continue };
        let parts: BTreeSet<Option<usize>> = c
            .refs
            .iter()
            .map(|re| match re.kind {
                crate::model::SubKind::Face => mesh.resolve(re).ok().and_then(face_part),
                _ => None,
            })
            .collect();
        if parts.len() == 1 {
            if let Some(Some(p)) = parts.into_iter().next() {
                for k in 0..3 {
                    reactions[p][k] += rf[k];
                }
            }
        }
    }
    a.parts
        .iter()
        .enumerate()
        .map(|(pi, p)| {
            let solids: BTreeSet<usize> = p.solids.iter().copied().collect();
            let mut nodes = BTreeSet::new();
            let mut elements = 0;
            let mut volume = 0.0;
            for e in mesh.elements.iter().filter(|e| solids.contains(&e.solid)) {
                elements += 1;
                nodes.extend(e.nodes.iter().copied());
                let c: Vec<[f64; 3]> = e.nodes[..4].iter().map(|&n| mesh.coord(n)).collect();
                volume += crate::mesh::tet_volume(c[0], c[1], c[2], c[3]).abs();
            }
            let idx: Vec<usize> = nodes.iter().filter_map(|&n| mesh.node_index(n)).collect();
            let ext = |v: &[f64]| -> Option<Extreme> {
                let &i = idx.iter().max_by(|&&x, &&y| v[x].total_cmp(&v[y]))?;
                Some(Extreme { value: v[i], node: r.node_ids[i], index: i, position: r.coords[i] })
            };
            PartResult {
                name: p.name.clone(),
                solids: p.solids.clone(),
                node_count: idx.len(),
                element_count: elements,
                volume,
                max_displacement: if dm.is_empty() { None } else { ext(&dm) },
                max_von_mises: if vm.is_empty() { None } else { ext(vm) },
                reaction: reactions[pi],
            }
        })
        .collect()
}

/// Per-triangle part index (into `Analysis::parts`) of a display surface, `u32::MAX` where no
/// part owns the solid.
pub fn triangle_parts(a: &Analysis, surface: &crate::mesh::SurfaceMesh) -> Vec<u32> {
    let mut map: BTreeMap<u32, u32> = BTreeMap::new();
    for (pi, p) in a.parts.iter().enumerate() {
        for &s in &p.solids {
            map.insert(s as u32, pi as u32);
        }
    }
    surface.tri_solid.iter().map(|s| map.get(s).copied().unwrap_or(u32::MAX)).collect()
}

// ------------------------------------------------------------------------------------------
// fc-assembly glue
// ------------------------------------------------------------------------------------------

/// The material of an assembly part: its BOM material name if it is a known card, else
/// Steel-Generic.
pub fn part_material(meta: &fc_assembly::PartMeta) -> Material {
    Material::by_name(&meta.material).unwrap_or_else(Material::steel_generic)
}

/// The visible part instances of assembly `asm` (sub-assemblies flattened) as analysis parts,
/// posed in the assembly's coordinates and named by their instance path (`"Sub.Bolt"`).
/// `material` picks each part's material (see [`part_material`]).
pub fn assembly_parts(
    ws: &fc_assembly::Workspace,
    asm: fc_assembly::AssemblyId,
    material: &dyn Fn(&fc_assembly::LeafInstance, &fc_assembly::PartEntry) -> Material,
) -> Result<Vec<AssemblyPart>, FemError> {
    let mut out = Vec::new();
    for leaf in ws.leaves(asm).into_iter().filter(|l| l.visible) {
        let Some(entry) = ws.part(leaf.part) else { continue };
        let Some(shape) = ws.leaf_shape(&leaf) else { continue };
        if shape.solid_count() == 0 {
            continue;
        }
        out.push(AssemblyPart { name: leaf.name.clone(), material: material(&leaf, entry), shape });
    }
    if out.is_empty() {
        return Err(FemError::Geometry("the assembly has no visible part with a solid".into()));
    }
    Ok(out)
}

/// A multi-solid analysis of an fc-assembly assembly (see [`assembly_parts`] and
/// [`Analysis::from_parts`]); materials come from the parts' BOM data.
pub fn analysis_from_assembly(ws: &fc_assembly::Workspace, asm: fc_assembly::AssemblyId, joining: Joining) -> Result<Analysis, FemError> {
    let parts = assembly_parts(ws, asm, &|_, e| part_material(&e.meta))?;
    Analysis::from_parts(&parts, joining)
}
