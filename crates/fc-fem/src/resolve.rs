//! Turning constraint references into mesh data: node sets, nodal loads (FreeCAD meshtools
//! `get_force_obj_*_nodeload_table`) and element faces (`get_pressure_obj_faces`).

use std::collections::{BTreeMap, BTreeSet, HashMap};

use crate::mesh::{cross, dot, norm, sub3, tri_area, ElementFace, FemMesh};
use crate::model::{Analysis, ConstraintKind, ForceDirection, SubKind, SubRef};
use crate::FemError;

/// Mesh data of one constraint.
#[derive(Debug, Clone, PartialEq)]
pub enum ConstraintData {
    /// Node set (fixed, displacement, temperature, heat source).
    Nodes(BTreeSet<u32>),
    /// Nodal force vectors per referenced sub-element: (label, node -> [fx, fy, fz]).
    /// `uniform` is the single load direction when there is one (FreeCAD writes only its
    /// non-zero components).
    NodeLoads { tables: Vec<(String, BTreeMap<u32, f64>)>, direction: Option<[f64; 3]>, vectors: Vec<BTreeMap<u32, [f64; 3]>> },
    /// Element faces per referenced face: (label "Face3", faces).
    ElementFaces(Vec<(String, Vec<ElementFace>)>),
    /// Constraint without mesh data (self weight, initial temperature).
    None,
}

/// Per-constraint mesh data, in the order of `Analysis::constraints`.
#[derive(Debug, Clone)]
pub struct Resolved {
    pub data: Vec<ConstraintData>,
    /// Elements per material assignment (empty = all elements).
    pub material_elements: Vec<Vec<u32>>,
    /// Element faces of every interaction (in the order of `Analysis::interactions`).
    pub interactions: Vec<InteractionFaces>,
}

/// Element faces of the slave and master side of a tie / contact.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct InteractionFaces {
    /// (label "Face3", element faces) per referenced slave face.
    pub slave: Vec<(String, Vec<ElementFace>)>,
    pub master: Vec<(String, Vec<ElementFace>)>,
}

/// Exact B-rep face areas by 1-based face index, when the shape is known (FreeCAD divides the
/// force by the B-rep area, then multiplies by the mesh node areas).
pub type FaceAreas = HashMap<usize, f64>;

/// Outward unit normal of a surface triangle (oriented using the tetrahedron it belongs to).
fn outward_normal(mesh: &FemMesh, tri: &[u32], lookup: &HashMap<[u32; 3], Vec<ElementFace>>, elems: &HashMap<u32, usize>) -> [f64; 3] {
    let p = [mesh.coord(tri[0]), mesh.coord(tri[1]), mesh.coord(tri[2])];
    let mut n = cross(sub3(p[1], p[0]), sub3(p[2], p[0]));
    let l = norm(n).max(1e-300);
    n = n.map(|v| v / l);
    let mut k = [tri[0], tri[1], tri[2]];
    k.sort_unstable();
    if let Some(&(eid, _)) = lookup.get(&k).and_then(|v| v.first()) {
        let e = &mesh.elements[elems[&eid]];
        if let Some(&other) = e.nodes[..4].iter().find(|n| !k.contains(n)) {
            if dot(sub3(mesh.coord(other), p[0]), n) > 0.0 {
                n = n.map(|v| -v);
            }
        }
    }
    n
}

/// (node, area share) entries of a surface triangle, FreeCAD get_ref_facenodes_areas.
fn face_node_areas(mesh: &FemMesh, t: &[u32]) -> (f64, Vec<(u32, f64)>) {
    let c = |i: usize| mesh.coord(t[i]);
    if t.len() == 6 {
        let a = tri_area(c(0), c(3), c(5)) + tri_area(c(1), c(4), c(3)) + tri_area(c(2), c(5), c(4)) + tri_area(c(3), c(4), c(5));
        (a, vec![(t[0], 0.0), (t[1], 0.0), (t[2], 0.0), (t[3], a / 3.0), (t[4], a / 3.0), (t[5], a / 3.0)])
    } else {
        let a = tri_area(c(0), c(1), c(2));
        (a, t.iter().map(|&n| (n, a / 3.0)).collect())
    }
}

/// Resolve every constraint of `analysis` on `mesh`.
pub fn resolve(analysis: &Analysis, mesh: &FemMesh, face_areas: Option<&FaceAreas>) -> Result<Resolved, FemError> {
    let lookup = mesh.face_lookup();
    let elems: HashMap<u32, usize> = mesh.elements.iter().enumerate().map(|(i, e)| (e.id, i)).collect();
    let mut data = Vec::new();
    for c in &analysis.constraints {
        let tags: Vec<(SubRef, usize)> = c.refs.iter().map(|r| mesh.resolve(r).map(|t| (r.clone(), t))).collect::<Result<_, _>>()?;
        let need_refs = !matches!(c.kind, ConstraintKind::SelfWeight { .. } | ConstraintKind::InitialTemperature { .. });
        if need_refs && tags.is_empty() {
            return Err(FemError::Reference(format!("{} has no references", c.name)));
        }
        let d = match &c.kind {
            ConstraintKind::Fixed | ConstraintKind::Displacement { .. } | ConstraintKind::Temperature { .. } | ConstraintKind::HeatSource { .. } => {
                let mut s = BTreeSet::new();
                for (r, t) in &tags {
                    s.extend(mesh.nodes_on(r.kind, *t));
                }
                ConstraintData::Nodes(s)
            }
            ConstraintKind::Force { force, direction } => resolve_force(c.name.as_str(), &tags, *force, direction, mesh, face_areas, &lookup, &elems)?,
            ConstraintKind::Pressure { .. } | ConstraintKind::HeatFlux(_) => {
                let mut v = Vec::new();
                for (r, t) in &tags {
                    if r.kind != SubKind::Face {
                        return Err(FemError::Reference(format!("{}: {} is not a face", c.name, r.name())));
                    }
                    v.push((format!("Face{t}"), mesh.element_faces_on(*t, &lookup)));
                }
                ConstraintData::ElementFaces(v)
            }
            ConstraintKind::SelfWeight { .. } | ConstraintKind::InitialTemperature { .. } => ConstraintData::None,
        };
        data.push(d);
    }
    // material element sets
    let mut material_elements = Vec::new();
    if analysis.materials.len() > 1 {
        for m in &analysis.materials {
            if m.solids.is_empty() {
                return Err(FemError::Model(format!("material {} needs solid references when there are several materials", m.name)));
            }
            let solids: BTreeSet<usize> = m.solids.iter().map(|r| r.index).collect();
            material_elements.push(mesh.elements.iter().filter(|e| solids.contains(&e.solid)).map(|e| e.id).collect());
        }
    } else {
        material_elements.push(Vec::new());
    }
    let mut interactions = Vec::new();
    for it in &analysis.interactions {
        let side = |refs: &[SubRef], what: &str| -> Result<Vec<(String, Vec<ElementFace>)>, FemError> {
            if refs.is_empty() {
                return Err(FemError::Reference(format!("{} has no {what} faces", it.name)));
            }
            let mut v = Vec::new();
            for r in refs {
                if r.kind != SubKind::Face {
                    return Err(FemError::Reference(format!("{}: {} is not a face", it.name, r.name())));
                }
                let t = mesh.resolve(r)?;
                v.push((format!("Face{t}"), mesh.element_faces_on(t, &lookup)));
            }
            Ok(v)
        };
        interactions.push(InteractionFaces { slave: side(&it.slave, "slave")?, master: side(&it.master, "master")? });
    }
    Ok(Resolved { data, material_elements, interactions })
}

#[allow(clippy::too_many_arguments)]
fn resolve_force(
    name: &str,
    tags: &[(SubRef, usize)],
    force: f64,
    direction: &ForceDirection,
    mesh: &FemMesh,
    face_areas: Option<&FaceAreas>,
    lookup: &HashMap<[u32; 3], Vec<ElementFace>>,
    elems: &HashMap<u32, usize>,
) -> Result<ConstraintData, FemError> {
    let kinds: BTreeSet<SubKind> = tags.iter().map(|(r, _)| r.kind).collect();
    if kinds.len() != 1 {
        return Err(FemError::Reference(format!("{name}: a force must reference only faces, only edges or only vertices")));
    }
    let kind = tags[0].0.kind;
    let uniform = match direction {
        ForceDirection::Vector(d) => {
            let l = norm(*d);
            if l == 0.0 {
                return Err(FemError::Model(format!("{name}: zero force direction")));
            }
            Some(d.map(|v| v / l))
        }
        ForceDirection::Normal { reversed } => {
            if kind != SubKind::Face {
                return Err(FemError::Model(format!("{name}: a normal force needs faces")));
            }
            // planar faces: one direction like FreeCAD's DirectionVector
            let sign = if *reversed { -1.0 } else { 1.0 };
            let mut dir: Option<[f64; 3]> = None;
            let mut planar = true;
            for (_, t) in tags {
                match mesh.signature(SubKind::Face, *t).and_then(|s| s.normal) {
                    Some(_) => {
                        let tri = &mesh.surfaces[t][0];
                        let n = outward_normal(mesh, tri, lookup, elems).map(|v| sign * v);
                        match dir {
                            None => dir = Some(n),
                            Some(d0) if dot(d0, n) > 1.0 - 1e-9 => {}
                            _ => planar = false,
                        }
                    }
                    None => planar = false,
                }
            }
            if planar {
                dir.map(|d| d.map(|v| if v.abs() < 1e-12 { 0.0 } else { v }))
            } else {
                None
            }
        }
    };
    let mut tables = Vec::new();
    let mut vectors = Vec::new();
    match kind {
        SubKind::Face => {
            let total_area: f64 = tags
                .iter()
                .map(|(_, t)| {
                    face_areas
                        .and_then(|m| m.get(t).copied())
                        .unwrap_or_else(|| mesh.signature(SubKind::Face, *t).map(|s| s.size).unwrap_or(0.0))
                })
                .sum();
            if total_area <= 0.0 {
                return Err(FemError::Model(format!("{name}: referenced faces have no area")));
            }
            let fpa = force / total_area;
            for (_, t) in tags {
                let mut table: BTreeMap<u32, f64> = BTreeMap::new();
                let mut vecs: BTreeMap<u32, [f64; 3]> = BTreeMap::new();
                for tri in &mesh.surfaces[t] {
                    let (_, shares) = face_node_areas(mesh, tri);
                    let n = if uniform.is_none() {
                        let s = if matches!(direction, ForceDirection::Normal { reversed: true }) { -1.0 } else { 1.0 };
                        Some(outward_normal(mesh, tri, lookup, elems).map(|v| s * v))
                    } else {
                        None
                    };
                    for (node, a) in shares {
                        *table.entry(node).or_insert(0.0) += a * fpa;
                        if let Some(n) = n {
                            let e = vecs.entry(node).or_insert([0.0; 3]);
                            for k in 0..3 {
                                e[k] += a * fpa * n[k];
                            }
                        }
                    }
                }
                tables.push((format!("Face{t}"), table));
                vectors.push(vecs);
            }
        }
        SubKind::Edge => {
            let total_len: f64 = tags.iter().map(|(_, t)| mesh.signature(SubKind::Edge, *t).map(|s| s.size).unwrap_or(0.0)).sum();
            if total_len <= 0.0 {
                return Err(FemError::Model(format!("{name}: referenced edges have no length")));
            }
            let fpl = force / total_len;
            for (_, t) in tags {
                let mut table: BTreeMap<u32, f64> = BTreeMap::new();
                for seg in &mesh.curves[t] {
                    if seg.len() == 3 {
                        let l = norm(sub3(mesh.coord(seg[2]), mesh.coord(seg[0]))) + norm(sub3(mesh.coord(seg[1]), mesh.coord(seg[2])));
                        *table.entry(seg[0]).or_insert(0.0) += l / 6.0 * fpl;
                        *table.entry(seg[1]).or_insert(0.0) += l / 6.0 * fpl;
                        *table.entry(seg[2]).or_insert(0.0) += l * 2.0 / 3.0 * fpl;
                    } else {
                        let l = norm(sub3(mesh.coord(seg[1]), mesh.coord(seg[0])));
                        *table.entry(seg[0]).or_insert(0.0) += l / 2.0 * fpl;
                        *table.entry(seg[1]).or_insert(0.0) += l / 2.0 * fpl;
                    }
                }
                tables.push((format!("Edge{t}"), table));
                vectors.push(BTreeMap::new());
            }
        }
        SubKind::Vertex => {
            let per = force / tags.len() as f64;
            for (_, t) in tags {
                let node = *mesh.points.get(t).ok_or_else(|| FemError::Reference(format!("{name}: Vertex{t} has no mesh node")))?;
                tables.push((format!("Vertex{t}"), BTreeMap::from([(node, per)])));
                vectors.push(BTreeMap::new());
            }
        }
        SubKind::Solid => return Err(FemError::Model(format!("{name}: forces on solids are not supported"))),
    }
    Ok(ConstraintData::NodeLoads { tables, direction: uniform, vectors })
}
