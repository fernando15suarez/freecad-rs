//! Import of FreeCAD 1.1 Assembly workbench documents (`.FCStd`).
//!
//! Reads `Document.xml` and converts every `Assembly::AssemblyObject` into an
//! [`Assembly`]: its parts become instances (with their `Placement`), `GroundedJoint`s
//! ground them, and each joint (`JointType`, `Placement1/2`, `Distance`, `Distance2`,
//! `Angle`, limits, `Suppressed`) becomes a [`Joint`] on detached frames
//! ([`JointRef::Frame`] = FreeCAD's `Placement1/2`, which already include `Offset1/2`).
//!
//! Part geometry is not imported (each part object becomes an empty fc-part document
//! named after it); the Distance joint variant is guessed from the element names
//! (Vertex/Edge/Face) since the faces' surface types are not available. Both are listed in
//! the returned notes.

use crate::frame::Frame;
use crate::joint::{DistanceMode, Joint, JointKind, JointRef, Limits, Sense};
use crate::model::{Assembly, InstanceId, Workspace};
use std::collections::HashMap;
use std::io::Read;
use std::path::Path;

#[derive(Debug, thiserror::Error)]
pub enum FcstdError {
    #[error("io: {0}")]
    Io(#[from] std::io::Error),
    #[error("not a FreeCAD document: {0}")]
    Format(String),
    #[error("Document.xml: {0}")]
    Xml(String),
    #[error("the document has no Assembly")]
    NoAssembly,
}

pub fn import(path: impl AsRef<Path>) -> Result<(Workspace, Vec<String>), FcstdError> {
    let bytes = std::fs::read(path)?;
    import_bytes(&bytes)
}

pub fn import_bytes(bytes: &[u8]) -> Result<(Workspace, Vec<String>), FcstdError> {
    let mut zip = zip::ZipArchive::new(std::io::Cursor::new(bytes)).map_err(|e| FcstdError::Format(e.to_string()))?;
    let mut f = zip.by_name("Document.xml").map_err(|e| FcstdError::Format(e.to_string()))?;
    let mut text = String::new();
    f.read_to_string(&mut text)?;
    import_document_xml(&text)
}

struct Obj<'a> {
    ty: String,
    props: HashMap<String, roxmltree::Node<'a, 'a>>,
}

fn prop_float(o: &Obj, name: &str) -> Option<f64> {
    let p = o.props.get(name)?;
    let n = p.children().find(|c| c.has_tag_name("Float"))?;
    n.attribute("value")?.parse().ok()
}
fn prop_bool(o: &Obj, name: &str) -> Option<bool> {
    let p = o.props.get(name)?;
    let n = p.children().find(|c| c.has_tag_name("Bool"))?;
    Some(n.attribute("value")? == "true")
}
fn prop_placement(o: &Obj, name: &str) -> Option<Frame> {
    let p = o.props.get(name)?;
    let n = p.children().find(|c| c.has_tag_name("PropertyPlacement"))?;
    let g = |a: &str| n.attribute(a).and_then(|v| v.parse::<f64>().ok()).unwrap_or(0.0);
    Some(Frame::new([g("Px"), g("Py"), g("Pz")], [g("Q0"), g("Q1"), g("Q2"), g("Q3")]))
}
fn prop_links(o: &Obj, name: &str) -> Vec<String> {
    let Some(p) = o.props.get(name) else { return vec![] };
    p.descendants().filter(|c| c.has_tag_name("Link")).filter_map(|c| c.attribute("value").map(str::to_string)).collect()
}
/// (object, subs) of an XLink property.
fn prop_xlink(o: &Obj, name: &str) -> Option<(String, Vec<String>)> {
    let p = o.props.get(name)?;
    let x = p.children().find(|c| c.has_tag_name("XLink"))?;
    let subs = x.children().filter(|c| c.has_tag_name("Sub")).filter_map(|c| c.attribute("value").map(str::to_string)).collect();
    Some((x.attribute("name")?.to_string(), subs))
}
fn prop_enum(o: &Obj, name: &str) -> Option<String> {
    let p = o.props.get(name)?;
    let i: usize = p.children().find(|c| c.has_tag_name("Integer"))?.attribute("value")?.parse().ok()?;
    let list = p.descendants().find(|c| c.has_tag_name("CustomEnumList"))?;
    list.children().filter(|c| c.has_tag_name("Enum")).nth(i)?.attribute("value").map(str::to_string)
}
fn prop_string(o: &Obj, name: &str) -> Option<String> {
    let p = o.props.get(name)?;
    p.children().find(|c| c.has_tag_name("String"))?.attribute("value").map(str::to_string)
}

fn last_element(subs: &[String], k: usize) -> String {
    subs.get(k).map(|s| s.rsplit('.').next().unwrap_or("").to_string()).unwrap_or_default()
}

fn guess_distance_mode(e1: &str, e2: &str) -> DistanceMode {
    let k = |e: &str| {
        if e.starts_with("Vertex") {
            0
        } else if e.starts_with("Edge") {
            1
        } else {
            2
        }
    };
    match (k(e1), k(e2)) {
        (0, 0) => DistanceMode::PointPoint,
        (2, 2) => DistanceMode::PlanePlane,
        (2, 0) | (0, 2) => DistanceMode::PointPlane,
        (1, 1) => DistanceMode::LineLine,
        (1, 0) | (0, 1) => DistanceMode::PointLine,
        _ => DistanceMode::LinePlane,
    }
}

/// The PartDesign Body an assembly member stands for (FreeCAD object name).
fn member_body(objects: &HashMap<String, Obj>, m: &str) -> Option<String> {
    let o = objects.get(m)?;
    match o.ty.as_str() {
        "PartDesign::Body" => Some(m.to_string()),
        "App::Link" => {
            let t = prop_xlink(o, "LinkedObject").map(|x| x.0).or_else(|| prop_links(o, "LinkedObject").into_iter().next())?;
            if t == m {
                return None;
            }
            member_body(objects, &t)
        }
        "App::Part" => prop_links(o, "Group").into_iter().find(|c| objects.get(c).is_some_and(|x| x.ty == "PartDesign::Body")),
        _ => None,
    }
}

pub fn import_document_xml(text: &str) -> Result<(Workspace, Vec<String>), FcstdError> {
    let doc = roxmltree::Document::parse(text).map_err(|e| FcstdError::Xml(e.to_string()))?;
    let root = doc.root_element();
    let mut types: HashMap<String, String> = HashMap::new();
    if let Some(objs) = root.children().find(|c| c.has_tag_name("Objects")) {
        for o in objs.children().filter(|c| c.has_tag_name("Object")) {
            if let (Some(n), Some(t)) = (o.attribute("name"), o.attribute("type")) {
                types.insert(n.to_string(), t.to_string());
            }
        }
    }
    let mut objects: HashMap<String, Obj> = HashMap::new();
    if let Some(data) = root.children().find(|c| c.has_tag_name("ObjectData")) {
        for o in data.children().filter(|c| c.has_tag_name("Object")) {
            let Some(name) = o.attribute("name") else { continue };
            let mut props = HashMap::new();
            if let Some(ps) = o.children().find(|c| c.has_tag_name("Properties")) {
                for p in ps.children().filter(|c| c.has_tag_name("Property")) {
                    if let Some(pn) = p.attribute("name") {
                        props.insert(pn.to_string(), p);
                    }
                }
            }
            let ty = types.get(name).cloned().unwrap_or_default();
            objects.insert(name.to_string(), Obj { ty, props });
        }
    }
    let mut asm_names: Vec<&String> = objects.iter().filter(|(_, o)| o.ty == "Assembly::AssemblyObject").map(|(n, _)| n).collect();
    asm_names.sort();
    if asm_names.is_empty() {
        return Err(FcstdError::NoAssembly);
    }
    let mut ws = Workspace::new();
    let mut notes = vec![];
    let is_joint = |o: &Obj| o.props.contains_key("JointType") || o.props.contains_key("ObjectToGround");
    for an in asm_names {
        let ao = &objects[an];
        let label = prop_string(ao, "Label").unwrap_or_else(|| an.clone());
        let aid = ws.add_assembly(&label);
        let mut asm = Assembly::new(&label);
        let mut body_parts: HashMap<String, crate::model::PartId> = HashMap::new();
        asm.id = aid;
        let group = prop_links(ao, "Group");
        let mut members: Vec<String> = vec![];
        let mut joints: Vec<String> = vec![];
        for m in &group {
            let Some(o) = objects.get(m) else { continue };
            if o.ty == "Assembly::JointGroup" {
                joints.extend(prop_links(o, "Group"));
            } else if is_joint(o) {
                joints.push(m.clone());
            } else if o.props.contains_key("Placement") && !o.ty.starts_with("App::Origin") && !o.ty.starts_with("Assembly::") {
                members.push(m.clone());
            }
        }
        joints.dedup();
        let mut inst: HashMap<String, InstanceId> = HashMap::new();
        for m in &members {
            let o = &objects[m];
            let name = prop_string(o, "Label").unwrap_or_else(|| m.clone());
            let pl = prop_placement(o, "Placement").unwrap_or_default();
            // The PartDesign Body behind the member: itself, a link's target, or the Body inside an App::Part.
            let body = member_body(&objects, m);
            let pid = match body.as_ref().and_then(|b| body_parts.get(b).copied()) {
                Some(pid) => pid,
                None => {
                    let doc = body.as_ref().and_then(|b| match fc_part::fcstd::import_body_xml(text, Some(b)) {
                        Ok((d, rep)) => {
                            for n in rep.notes.iter().filter(|n| !matches!(n.kind, fc_part::fcstd::NoteKind::Info)) {
                                notes.push(format!("{b}/{}: {}", n.object, n.message));
                            }
                            Some(d)
                        }
                        Err(e) => {
                            notes.push(format!("{b}: body not imported ({e})"));
                            None
                        }
                    });
                    let has_geometry = doc.is_some();
                    let pid = ws.add_part(&name, doc.unwrap_or_default());
                    match &body {
                        Some(b) if has_geometry => {
                            body_parts.insert(b.clone(), pid);
                        }
                        _ => notes.push(format!("{m}: {} imported as a placeholder part (no geometry)", o.ty)),
                    }
                    pid
                }
            };
            inst.insert(m.clone(), asm.add_part_instance(&name, pid, pl));
        }
        let mut seen = std::collections::HashSet::new();
        for jn in &joints {
            if !seen.insert(jn.clone()) {
                continue;
            }
            let Some(o) = objects.get(jn) else { continue };
            if o.props.contains_key("ObjectToGround") {
                for g in prop_links(o, "ObjectToGround") {
                    if let Some(&i) = inst.get(&g) {
                        asm.set_grounded(i, true);
                    }
                }
                continue;
            }
            let ty = prop_enum(o, "JointType").unwrap_or_default();
            let d = prop_float(o, "Distance").unwrap_or(0.0);
            let d2 = prop_float(o, "Distance2").unwrap_or(0.0);
            let (Some((o1, s1)), Some((o2, s2))) = (prop_xlink(o, "Reference1"), prop_xlink(o, "Reference2")) else {
                notes.push(format!("{jn}: missing references, skipped"));
                continue;
            };
            // The moving part is the first path component that is an assembly member.
            let part_of = |obj: &str, subs: &[String]| -> Option<InstanceId> {
                if let Some(&i) = inst.get(obj) {
                    return Some(i);
                }
                let first = subs.first()?.split('.').next()?;
                inst.get(first).copied()
            };
            let (Some(ia), Some(ib)) = (part_of(&o1, &s1), part_of(&o2, &s2)) else {
                notes.push(format!("{jn}: references outside the assembly, skipped"));
                continue;
            };
            let kind = match ty.as_str() {
                "Fixed" => JointKind::Fastened,
                "Revolute" => JointKind::Revolute,
                "Cylindrical" => JointKind::Cylindrical,
                "Slider" => JointKind::Slider,
                "Ball" => JointKind::Ball,
                "Parallel" => JointKind::Parallel,
                "Perpendicular" => JointKind::Perpendicular,
                "Angle" => JointKind::Angle { angle: prop_float(o, "Angle").unwrap_or(0.0) },
                "Distance" => {
                    let mode = guess_distance_mode(&last_element(&s1, 0), &last_element(&s2, 0));
                    notes.push(format!("{jn}: Distance variant {mode:?} guessed from the element names"));
                    JointKind::Distance { distance: d, mode }
                }
                "RackPinion" => JointKind::RackPinion { pitch_radius: d },
                "Screw" => JointKind::Screw { pitch: d },
                "Gears" => JointKind::Gears { radius1: d, radius2: d2 },
                "Belt" => JointKind::Belt { radius1: d, radius2: d2 },
                other => {
                    notes.push(format!("{jn}: unknown joint type {other:?}, skipped"));
                    continue;
                }
            };
            let p1 = prop_placement(o, "Placement1").unwrap_or_default();
            let p2 = prop_placement(o, "Placement2").unwrap_or_default();
            let mut j = Joint::new(kind, JointRef::frame(ia, p1), JointRef::frame(ib, p2));
            j.name = prop_string(o, "Label").unwrap_or_else(|| jn.clone());
            j.sense = Sense::Auto;
            j.suppressed = prop_bool(o, "Suppressed").unwrap_or(false);
            let en = |n: &str| prop_bool(o, n).unwrap_or(false);
            j.limits = Limits {
                length_min: en("EnableLengthMin").then(|| prop_float(o, "LengthMin").unwrap_or(0.0)),
                length_max: en("EnableLengthMax").then(|| prop_float(o, "LengthMax").unwrap_or(0.0)),
                angle_min: en("EnableAngleMin").then(|| prop_float(o, "AngleMin").unwrap_or(0.0)),
                angle_max: en("EnableAngleMax").then(|| prop_float(o, "AngleMax").unwrap_or(0.0)),
            };
            asm.add_joint(j);
        }
        *ws.assembly_mut(aid).unwrap() = asm;
    }
    Ok((ws, notes))
}
