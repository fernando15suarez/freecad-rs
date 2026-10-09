//! FreeCAD `.FCStd` import: reads the zip's `Document.xml` and converts the first
//! PartDesign Body into a [`Document`].
//!
//! What is imported (in the Body's tree order):
//! * `Sketcher::SketchObject`: line segments, circles, arcs (also clockwise ones), points,
//!   construction flags (old `<Construction>` element and the newer
//!   SketchGeometryExtension mode flags), constraints with FreeCAD's element conventions
//!   (see [`sketch`]), names, driving/reference state. External geometry is skipped.
//! * Attachment: origin planes (with `MapReversed` and `AttachmentOffset`) and planar
//!   faces of earlier features (`Pad.Face6`). FreeCAD's sketch frame is taken from the
//!   attachment (planes) or the stored `Placement` (faces, anything else); when it differs
//!   from the frame fc-part computes for the same support, the sketch is re-expressed in
//!   fc-part's frame (exactly for quarter-turn rotations, see [`ImportReport`] notes).
//! * Pad/Pocket, Revolution/Groove, Hole, Additive/SubtractiveLoft, Additive/SubtractivePipe.
//! * Expressions (`ExpressionEngine`) on feature properties and sketch constraints, and
//!   Spreadsheet cells as document parameters (aliases keep their name).
//!
//! Everything skipped or approximated is listed in the [`ImportReport`].

mod exprs;
mod sketch;
mod xml;

use crate::placement::*;
use crate::{
    AxisRef, Document, ExtentType, FeatureId, FeatureKind, Extrude, Hole, HoleCut, Loft, Pipe, Revolve, SideType,
    SketchFeature, Support,
};
use fc_sketch::Vec2;
use serde::{Deserialize, Serialize};
use sketch::{ImportedSketch, ValueMap};
use std::collections::{BTreeMap, HashMap};
use std::f64::consts::PI;
use std::io::Read;
use std::path::Path;
use xml::{FcObj, FcPlacement};

#[derive(Debug, thiserror::Error)]
pub enum ImportError {
    #[error("io: {0}")]
    Io(#[from] std::io::Error),
    #[error("not a FreeCAD document: {0}")]
    Format(String),
    #[error("Document.xml: {0}")]
    Xml(String),
    #[error("the document has no PartDesign Body")]
    NoBody,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum NoteKind {
    /// Not imported at all.
    Skipped,
    /// Imported, but not exactly as FreeCAD would model it.
    Approximated,
    Info,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ImportNote {
    /// FreeCAD object name.
    pub object: String,
    pub kind: NoteKind,
    pub message: String,
}

/// How an imported sketch relates to the FreeCAD original.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct SketchMap {
    /// FreeCAD geometry index -> our GeoId (None = skipped).
    pub geometry: Vec<Option<usize>>,
    /// FreeCAD constraint index -> our constraint indices (empty = skipped).
    pub constraints: Vec<Vec<usize>>,
    /// In-plane rotation (degrees) and offset (mm) from FreeCAD's sketch frame to ours.
    pub frame_rotation_deg: f64,
    pub frame_offset: [f64; 2],
}

impl SketchMap {
    /// FreeCAD constraint indices for some of our constraint indices (sorted, deduplicated).
    pub fn to_freecad(&self, ours: &[usize]) -> Vec<usize> {
        let mut v: Vec<usize> = self
            .constraints
            .iter()
            .enumerate()
            .filter(|(_, o)| o.iter().any(|i| ours.contains(i)))
            .map(|(f, _)| f)
            .collect();
        v.sort_unstable();
        v.dedup();
        v
    }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ImportReport {
    /// FreeCAD version that wrote the file.
    pub program_version: String,
    /// Name of the imported Body.
    pub body: String,
    /// Other bodies in the file (not imported).
    pub other_bodies: Vec<String>,
    pub notes: Vec<ImportNote>,
    /// Per imported sketch (FreeCAD name).
    pub sketches: BTreeMap<String, SketchMap>,
}

impl ImportReport {
    pub fn skipped(&self) -> impl Iterator<Item = &ImportNote> {
        self.notes.iter().filter(|n| n.kind == NoteKind::Skipped)
    }
    pub fn approximated(&self) -> impl Iterator<Item = &ImportNote> {
        self.notes.iter().filter(|n| n.kind == NoteKind::Approximated)
    }
}

#[derive(Default)]
pub(crate) struct Notes(Vec<ImportNote>);

impl Notes {
    pub fn push(&mut self, object: &str, kind: NoteKind, message: String) {
        self.0.push(ImportNote { object: object.to_string(), kind, message });
    }
}

/// Import the first PartDesign Body of a `.FCStd` file.
pub fn import(path: impl AsRef<Path>) -> Result<Document, ImportError> {
    import_with_report(path).map(|(d, _)| d)
}

/// Like [`import`], also returning what was skipped or approximated.
pub fn import_with_report(path: impl AsRef<Path>) -> Result<(Document, ImportReport), ImportError> {
    let bytes = std::fs::read(path)?;
    import_bytes(&bytes)
}

/// Import from the bytes of a `.FCStd` (zip) file.
pub fn import_bytes(bytes: &[u8]) -> Result<(Document, ImportReport), ImportError> {
    let mut zip = zip::ZipArchive::new(std::io::Cursor::new(bytes)).map_err(|e| ImportError::Format(e.to_string()))?;
    let mut f = zip.by_name("Document.xml").map_err(|e| ImportError::Format(format!("Document.xml: {e}")))?;
    let mut xml = String::new();
    f.read_to_string(&mut xml)?;
    import_document_xml(&xml)
}

/// Import from the text of a FreeCAD `Document.xml`.
pub fn import_document_xml(text: &str) -> Result<(Document, ImportReport), ImportError> {
    let opts = roxmltree::ParsingOptions { allow_dtd: true, ..Default::default() };
    let x = roxmltree::Document::parse_with_options(text, opts).map_err(|e| ImportError::Xml(e.to_string()))?;
    let root = x.root_element();
    if root.tag_name().name() != "Document" {
        return Err(ImportError::Xml("root element is not <Document>".into()));
    }
    let (order, objs) = xml::read_objects(root);
    let bodies: Vec<&String> = order.iter().filter(|n| objs[*n].ty == "PartDesign::Body").collect();
    let body_name = bodies.first().ok_or(ImportError::NoBody)?.to_string();
    let mut imp = Importer {
        objs: &objs,
        doc: Document::new(),
        notes: Notes::default(),
        ids: HashMap::new(),
        sketches: HashMap::new(),
        frames: HashMap::new(),
        offset_maps: HashMap::new(),
        value_maps: HashMap::new(),
    };
    let mut report = ImportReport {
        program_version: root.attribute("ProgramVersion").unwrap_or("").to_string(),
        body: body_name.clone(),
        other_bodies: bodies.iter().skip(1).map(|s| s.to_string()).collect(),
        ..Default::default()
    };
    if !report.other_bodies.is_empty() {
        imp.notes.push(&body_name, NoteKind::Skipped, format!("only the first Body is imported; skipped {:?}", report.other_bodies));
    }
    let body = &objs[&body_name];
    if body.link("BaseFeature").is_some() {
        imp.notes.push(&body_name, NoteKind::Skipped, "the Body's BaseFeature is not imported".into());
    }
    for name in body.link_list("Group") {
        let Some(o) = objs.get(&name) else { continue };
        imp.feature(o);
    }
    if let Some(tip) = body.link("Tip") {
        if let Some(id) = imp.ids.get(&tip) {
            if imp.doc.features.last().map(|f| f.id) != Some(*id) {
                imp.doc.tip = Some(*id);
            }
        }
    }
    imp.expressions(&order, body);
    imp.doc.states.clear();
    let Importer { doc, notes, sketches, frames, .. } = imp;
    for (name, s) in sketches {
        let (theta, t) = frames.get(&name).copied().unwrap_or((0.0, Vec2::ZERO));
        report.sketches.insert(
            name,
            SketchMap { geometry: s.geo_map, constraints: s.con_map, frame_rotation_deg: theta.to_degrees(), frame_offset: [t.x, t.y] },
        );
    }
    report.notes = notes.0;
    Ok((doc, report))
}

const PAD_TYPES: &[&str] = &["Length", "UpToLast", "UpToFirst", "UpToFace", "TwoLengths", "UpToShape"];
const POCKET_TYPES: &[&str] = &["Length", "ThroughAll", "UpToFirst", "UpToFace", "TwoLengths", "UpToShape"];
const REVOLUTION_TYPES: &[&str] = &["Angle", "UpToLast", "UpToFirst", "UpToFace", "TwoAngles"];
const GROOVE_TYPES: &[&str] = &["Angle", "ThroughAll", "UpToFirst", "UpToFace", "TwoAngles"];
const SIDE_TYPES: &[&str] = &["One side", "Two sides", "Symmetric"];
const HOLE_CUT_TYPES: &[&str] = &["None", "Counterbore", "Countersink", "Counterdrill"];
const PIPE_MODES: &[&str] = &["Standard", "Fixed", "Frenet", "Auxiliary", "Binormal"];
const PIPE_TRANSITIONS: &[&str] = &["Transformed", "Right corner", "Round corner"];
/// Attacher map modes used here.
const MM_DEACTIVATED: i64 = 0;
const MM_OBJECT_XY: i64 = 2;
const MM_FLAT_FACE: i64 = 5;

struct Importer<'a> {
    objs: &'a HashMap<String, FcObj<'a>>,
    doc: Document,
    notes: Notes,
    /// FreeCAD name -> our feature id.
    ids: HashMap<String, FeatureId>,
    sketches: HashMap<String, ImportedSketch>,
    /// Sketch -> FreeCAD-to-ours frame change (rotation, offset).
    frames: HashMap<String, (f64, Vec2)>,
    /// Sketch -> how FreeCAD's AttachmentOffset.Base.z maps to our offset.
    offset_maps: HashMap<String, ValueMap>,
    /// Sketch -> per FreeCAD constraint: how its value maps to ours after the frame change.
    value_maps: HashMap<String, Vec<ValueMap>>,
}

fn approx_eq3(a: V3, b: V3, tol: f64) -> bool {
    norm(sub(a, b)) < tol
}

impl<'a> Importer<'a> {
    fn note(&mut self, obj: &str, kind: NoteKind, msg: impl Into<String>) {
        self.notes.push(obj, kind, msg.into());
    }

    fn add(&mut self, o: &FcObj, kind: FeatureKind) -> FeatureId {
        let id = self.doc.add(kind);
        let label = o.string("Label").unwrap_or_else(|| o.name.clone());
        let f = self.doc.get_mut(id).expect("just added");
        f.name = o.name.clone();
        f.label = label;
        f.suppressed = o.bool("Suppressed").unwrap_or(false);
        self.ids.insert(o.name.clone(), id);
        id
    }

    /// The imported sketch a feature property links to.
    fn sketch_link(&mut self, o: &FcObj, prop: &str) -> Option<FeatureId> {
        let target = o.link_sub(prop).map(|(n, _)| n).or_else(|| o.link(prop));
        let Some(t) = target else {
            self.note(&o.name, NoteKind::Skipped, format!("{prop} is empty; feature skipped"));
            return None;
        };
        if !self.sketches.contains_key(&t) {
            let why = match self.objs.get(&t) {
                Some(x) if x.ty == "Sketcher::SketchObject" => "was not imported".to_string(),
                Some(x) => format!("is a {} (only sketches are supported)", x.ty),
                None => "does not exist".to_string(),
            };
            self.note(&o.name, NoteKind::Skipped, format!("{prop} {t} {why}; feature skipped"));
            return None;
        }
        self.ids.get(&t).copied()
    }

    fn feature(&mut self, o: &'a FcObj<'a>) {
        match o.ty.as_str() {
            "Sketcher::SketchObject" => self.sketch(o),
            "PartDesign::Pad" | "PartDesign::Pocket" => self.extrude(o),
            "PartDesign::Revolution" | "PartDesign::Groove" => self.revolve(o),
            "PartDesign::Hole" => self.hole(o),
            "PartDesign::AdditiveLoft" | "PartDesign::SubtractiveLoft" => self.loft(o),
            "PartDesign::AdditivePipe" | "PartDesign::SubtractivePipe" => self.pipe(o),
            other => self.note(&o.name, NoteKind::Skipped, format!("{other} is not supported")),
        }
    }

    // ----------------------------------------------------------------------------- sketches

    fn sketch(&mut self, o: &'a FcObj<'a>) {
        let mut imported = sketch::import_sketch(o, &mut self.notes);
        let stored = o.placement("Placement").unwrap_or(FcPlacement::IDENTITY);
        let mut support = o.link_sub_list("AttachmentSupport");
        if support.is_empty() {
            support = o.link_sub_list("Support");
        }
        let map_mode = o.int("MapMode").unwrap_or(MM_DEACTIVATED);
        let map_reversed = o.bool("MapReversed").unwrap_or(false);
        let att_off = o.placement("AttachmentOffset").unwrap_or(FcPlacement::IDENTITY);

        let single = (support.len() == 1).then(|| support[0].clone());
        let plane_role = single.as_ref().and_then(|(obj, _)| {
            let x = self.objs.get(obj)?;
            if x.ty != "App::Plane" {
                return None;
            }
            let role = x.string("Role").unwrap_or_else(|| obj.clone());
            ["XY_Plane", "XZ_Plane", "YZ_Plane"].into_iter().find(|r| role.starts_with(r)).map(|r| (r, x))
        });
        let face_support = single.as_ref().and_then(|(obj, sub)| {
            (sub.starts_with("Face") && self.ids.contains_key(obj) && map_mode == MM_FLAT_FACE)
                .then(|| (obj.clone(), sub[4..].parse::<usize>().unwrap_or(0)))
        });

        // FreeCAD's sketch placement.
        let fc_pl = match (&plane_role, &face_support) {
            (Some((role, plane)), _) if map_mode == MM_FLAT_FACE || map_mode == MM_OBJECT_XY => {
                let plane_pl = plane.placement("Placement").unwrap_or_else(|| default_plane(role));
                let rev = if map_reversed { FcPlacement::rotation([0.0, 1.0, 0.0], PI) } else { FcPlacement::IDENTITY };
                let computed = plane_pl.mul(&rev).mul(&att_off);
                if !same_frame(&computed, &stored) {
                    self.note(&o.name, NoteKind::Info, "stored Placement differs from the attachment (file not recomputed?); using the attachment");
                }
                computed
            }
            (_, Some(_)) => stored,
            _ => {
                if map_mode != MM_DEACTIVATED || !support.is_empty() {
                    self.note(
                        &o.name,
                        NoteKind::Approximated,
                        format!("attachment {:?} (map mode {map_mode}) is not supported; using the sketch's stored placement", support),
                    );
                }
                stored
            }
        };

        let mut sf = SketchFeature { sketch: imported.sketch.clone(), support: Support::XY, offset: 0.0, reversed: false };
        let mut face_done = false;
        if let Some((fobj, fidx)) = &face_support {
            match self.face_support(o, fobj, *fidx, &fc_pl, &att_off, map_reversed) {
                Some(fr) => {
                    sf.support = Support::Face(fr);
                    face_done = true;
                }
                None => self.note(&o.name, NoteKind::Approximated, format!("could not match {fobj}.Face{fidx}; sketch placed on an origin plane at FreeCAD's placement")),
            }
        }
        let id = self.add(o, FeatureKind::Sketch(sf.clone()));
        // Base frame of our support (offset 0, not reversed).
        let base = if face_done {
            self.doc.recompute();
            match self.doc.state(id).and_then(|s| s.placement) {
                Some(p) => p,
                None => {
                    let err = self.doc.state(id).and_then(|s| s.error.clone()).map(|e| e.to_string()).unwrap_or_default();
                    self.note(&o.name, NoteKind::Approximated, format!("support face could not be resolved ({err})"));
                    Placement::XY
                }
            }
        } else {
            let n = fc_pl.normal();
            let cands = [(Support::XY, Placement::XY), (Support::XZ, Placement::XZ), (Support::YZ, Placement::YZ)];
            let (s, p) = cands.into_iter().max_by(|a, b| dot(a.1.normal(), n).abs().total_cmp(&dot(b.1.normal(), n).abs())).unwrap();
            if dot(p.normal(), n).abs() < 1.0 - 1e-9 {
                self.note(&o.name, NoteKind::Approximated, "sketch plane is not parallel to an origin plane; frame-dependent constraints are approximated");
            }
            sf.support = s;
            p
        };
        let nb = base.normal();
        let n_fc = fc_pl.normal();
        sf.reversed = dot(n_fc, nb) < 0.0;
        // Like FreeCAD, the offset is measured along the (possibly reversed) attached normal.
        let attached = if sf.reversed { base.reversed() } else { base };
        sf.offset = dot(sub(fc_pl.origin, attached.origin), attached.normal());
        let ours = attached.offset(sf.offset);
        // FreeCAD's AttachmentOffset.Base.z vs our offset (for expressions).
        self.offset_maps.insert(o.name.clone(), ValueMap { scale: 1.0, add: sf.offset - att_off.origin[2] });
        if !att_off.is_identity_rotation() || att_off.origin[0].abs() > 1e-12 || att_off.origin[1].abs() > 1e-12 {
            self.note(&o.name, NoteKind::Info, "AttachmentOffset has an in-plane part; it is folded into the sketch geometry");
        }

        // Re-express the geometry in our frame.
        if dot(n_fc, ours.normal()) < 1.0 - 1e-7 {
            self.note(&o.name, NoteKind::Approximated, "sketch normal differs from FreeCAD's");
        }
        let (xf, yf) = (fc_pl.xdir(), fc_pl.ydir());
        let theta = dot(xf, ours.ydir).atan2(dot(xf, ours.xdir));
        let d = sub(fc_pl.origin, ours.origin);
        let t = Vec2::new(dot(d, ours.xdir), dot(d, ours.ydir));
        let _ = yf;
        let mut vmaps = vec![ValueMap::ID; sf.sketch.constraints.len()];
        if theta.abs() > 1e-9 || t.length() > 1e-9 {
            vmaps = sketch::transform_sketch(&mut sf.sketch, theta, t, &o.name, &mut self.notes);
            self.note(
                &o.name,
                NoteKind::Info,
                format!("FreeCAD's sketch frame is rotated {:.1} deg / offset ({:.4}, {:.4}) from ours; geometry re-expressed", theta.to_degrees(), t.x, t.y),
            );
        }
        self.frames.insert(o.name.clone(), (theta, t));
        imported.sketch = sf.sketch.clone();
        if let Some(f) = self.doc.get_mut(id) {
            f.kind = FeatureKind::Sketch(sf);
        }
        // Value maps by FreeCAD constraint index (first of our constraints).
        let by_fc: Vec<ValueMap> = imported.con_map.iter().map(|v| v.first().map(|&i| vmaps[i]).unwrap_or(ValueMap::ID)).collect();
        self.value_maps.insert(o.name.clone(), by_fc);
        self.sketches.insert(o.name.clone(), imported);
    }

    /// FaceRef on our solid of `fobj` matching FreeCAD's attached face.
    fn face_support(&mut self, o: &FcObj, fobj: &str, fidx: usize, fc_pl: &FcPlacement, att_off: &FcPlacement, map_reversed: bool) -> Option<crate::FaceRef> {
        if !att_off.is_identity_rotation() {
            self.note(&o.name, NoteKind::Approximated, "rotated AttachmentOffset on a face attachment is not supported");
        }
        let fid = *self.ids.get(fobj)?;
        self.doc.recompute();
        let shape = self.doc.state(fid)?.shape.clone()?;
        // Attached (pre-offset) frame: origin = placement origin minus the rotated offset.
        let o_att = sub(fc_pl.origin, fc_pl.rotate(att_off.origin));
        let n_att = fc_pl.normal();
        let n_face = if map_reversed { scale(n_att, -1.0) } else { n_att };
        let faces = shape.faces();
        let cands: Vec<usize> = faces
            .iter()
            .enumerate()
            .filter(|(_, f)| f.kind == 0 && dot(normalize(f.normal), n_face) > 1.0 - 1e-6 && dot(sub(f.center, o_att), n_face).abs() < 1e-5)
            .map(|(i, _)| i + 1)
            .collect();
        let idx = if cands.contains(&fidx) {
            fidx
        } else {
            *cands.iter().max_by(|a, b| faces[**a - 1].area.total_cmp(&faces[**b - 1].area))?
        };
        if idx != fidx {
            self.note(&o.name, NoteKind::Info, format!("FreeCAD's {fobj}.Face{fidx} is our Face{idx}"));
        }
        crate::face_ref(fid, &shape, idx).ok()
    }

    // ----------------------------------------------------------------------------- features

    fn extrude(&mut self, o: &FcObj) {
        let pad = o.ty == "PartDesign::Pad";
        let Some(profile) = self.sketch_link(o, "Profile") else { return };
        let mut e = Extrude::new(profile, o.float("Length").unwrap_or(10.0));
        e.length2 = o.float("Length2").unwrap_or(0.0);
        e.reversed = o.bool("Reversed").unwrap_or(false);
        let types = if pad { PAD_TYPES } else { POCKET_TYPES };
        let side = o.enumeration("SideType", SIDE_TYPES).and_then(|(_, s)| s);
        let (_, ty) = o.enumeration("Type", types).unwrap_or((0, Some("Length".into())));
        let ty = ty.unwrap_or_default();
        let (_, ty2) = o.enumeration("Type2", types).unwrap_or((0, Some("Length".into())));
        let ty2 = ty2.unwrap_or_default();
        e.side = match side.as_deref() {
            Some("Two sides") => SideType::TwoSides,
            Some("Symmetric") => SideType::Symmetric,
            _ if o.bool("Midplane").unwrap_or(false) => SideType::Symmetric,
            _ if ty == "TwoLengths" => SideType::TwoSides,
            _ => SideType::OneSide,
        };
        let mut kind_of = |t: &str, which: &str| -> ExtentType {
            match t {
                "Length" | "TwoLengths" | "" => ExtentType::Length,
                "ThroughAll" => ExtentType::ThroughAll,
                "UpToFirst" => ExtentType::UpToFirst,
                "UpToLast" => {
                    self.notes.push(&o.name, NoteKind::Approximated, format!("{which} UpToLast imported as ThroughAll"));
                    ExtentType::ThroughAll
                }
                other => {
                    self.notes.push(&o.name, NoteKind::Approximated, format!("{which} {other} is not supported; using the stored length"));
                    ExtentType::Length
                }
            }
        };
        e.kind = kind_of(&ty, "Type");
        e.kind2 = if e.side == SideType::TwoSides && o.has("Type2") { kind_of(&ty2, "Type2") } else { ExtentType::Length };
        for (p, what) in [("TaperAngle", "taper"), ("TaperAngle2", "taper"), ("Offset", "offset"), ("Offset2", "offset")] {
            if o.float(p).unwrap_or(0.0).abs() > 1e-12 {
                self.note(&o.name, NoteKind::Approximated, format!("{p} ({what}) is ignored"));
            }
        }
        if o.bool("UseCustomVector").unwrap_or(false) {
            self.note(&o.name, NoteKind::Approximated, "custom direction is ignored (extruding along the sketch normal)");
        }
        let kind = if pad { FeatureKind::Pad(e) } else { FeatureKind::Pocket(e) };
        self.add(o, kind);
    }

    fn revolve(&mut self, o: &FcObj) {
        let additive = o.ty == "PartDesign::Revolution";
        let Some(profile) = self.sketch_link(o, "Profile") else { return };
        let psk = self.doc.get(profile).map(|f| f.name.clone()).unwrap_or_default();
        let mut r = Revolve::new(profile);
        r.angle = o.float("Angle").unwrap_or(360.0);
        r.angle2 = o.float("Angle2").unwrap_or(0.0);
        r.reversed = o.bool("Reversed").unwrap_or(false);
        let types = if additive { REVOLUTION_TYPES } else { GROOVE_TYPES };
        let (_, ty) = o.enumeration("Type", types).unwrap_or((0, None));
        let ty = ty.unwrap_or_default();
        let side = o.enumeration("SideType", SIDE_TYPES).and_then(|(_, s)| s);
        r.side = match side.as_deref() {
            Some("Two sides") => SideType::TwoSides,
            Some("Symmetric") => SideType::Symmetric,
            _ if o.bool("Midplane").unwrap_or(false) => SideType::Symmetric,
            _ if ty == "TwoAngles" => SideType::TwoSides,
            _ => SideType::OneSide,
        };
        match ty.as_str() {
            "Angle" | "TwoAngles" | "" => {}
            "ThroughAll" if !additive => r.angle = 360.0,
            other => self.note(&o.name, NoteKind::Approximated, format!("Type {other} is not supported; using Angle")),
        }
        // Axis.
        let (aobj, asubs) = o.link_sub("ReferenceAxis").unwrap_or_default();
        let sub = asubs.first().cloned().unwrap_or_default();
        let mut flip = false;
        r.axis = if aobj == psk || (aobj.is_empty() && sub.is_empty()) {
            let (theta, t) = self.frames.get(&psk).copied().unwrap_or((0.0, Vec2::ZERO));
            let q = theta / (PI / 2.0);
            let k = ((q.round() - q).abs() < 1e-9).then(|| (q.round() as i64).rem_euclid(4));
            let sk = self.sketches.get(&psk);
            let mut sketch_axis = |vertical: bool| -> AxisRef {
                if t.length() > 1e-9 || k.is_none() {
                    self.notes.push(&o.name, NoteKind::Approximated, "sketch axis of a re-expressed sketch frame is approximated".into());
                }
                // FreeCAD's sketch V (H) axis in our frame.
                let k = k.unwrap_or(0);
                let (v, f) = match (vertical, k) {
                    (true, 0) => (true, false),
                    (true, 1) => (false, true),
                    (true, 2) => (true, true),
                    (true, _) => (false, false),
                    (false, 0) => (false, false),
                    (false, 1) => (true, false),
                    (false, 2) => (false, true),
                    (false, _) => (true, true),
                };
                flip = f;
                if v { AxisRef::SketchV } else { AxisRef::SketchH }
            };
            match sub.as_str() {
                "V_Axis" | "" => sketch_axis(true),
                "H_Axis" => sketch_axis(false),
                s if s.starts_with("Edge") || s.starts_with("Axis") => {
                    let n: usize = s.trim_start_matches(|c: char| c.is_alphabetic()).parse().unwrap_or(0);
                    let fc_geo = sk.and_then(|sk| if s.starts_with("Edge") { sk.edges.get(n.wrapping_sub(1)) } else { sk.axes.get(n) }).copied();
                    match fc_geo.and_then(|g| sk.and_then(|sk| sk.geo_map.get(g).copied().flatten())) {
                        Some(g) => AxisRef::SketchLine(g),
                        None => {
                            self.note(&o.name, NoteKind::Skipped, format!("axis {psk}.{s} not found; feature skipped"));
                            return;
                        }
                    }
                }
                other => {
                    self.note(&o.name, NoteKind::Skipped, format!("axis {psk}.{other} is not supported; feature skipped"));
                    return;
                }
            }
        } else {
            let role = self.objs.get(&aobj).and_then(|x| x.string("Role")).unwrap_or_else(|| aobj.clone());
            if role.starts_with("X_Axis") {
                AxisRef::X
            } else if role.starts_with("Y_Axis") {
                AxisRef::Y
            } else if role.starts_with("Z_Axis") {
                AxisRef::Z
            } else {
                self.note(&o.name, NoteKind::Skipped, format!("axis {aobj}.{sub} is not supported; feature skipped"));
                return;
            }
        };
        if flip {
            r.reversed = !r.reversed;
        }
        let kind = if additive { FeatureKind::Revolution(r) } else { FeatureKind::Groove(r) };
        self.add(o, kind);
    }

    fn hole(&mut self, o: &FcObj) {
        let Some(profile) = self.sketch_link(o, "Profile") else { return };
        let mut h = Hole::new(profile, o.float("Diameter").unwrap_or(6.0), o.float("Depth").unwrap_or(25.0));
        h.through_all = o.enumeration("DepthType", &["Dimension", "ThroughAll"]).and_then(|(_, s)| s).as_deref() == Some("ThroughAll");
        let angled = o.enumeration("DrillPoint", &["Flat", "Angled"]).and_then(|(_, s)| s).as_deref() != Some("Flat");
        h.drill_point_angle = if angled { o.float("DrillPointAngle").unwrap_or(118.0) } else { 0.0 };
        let (ci, cut) = o.enumeration("HoleCutType", HOLE_CUT_TYPES).unwrap_or((0, None));
        h.cut = match (ci, cut.as_deref()) {
            (_, Some("None")) | (0, _) => HoleCut::None,
            (_, Some("Counterbore")) | (1, None) => HoleCut::Counterbore,
            (_, Some("Countersink")) | (2, None) => HoleCut::Countersink,
            (_, other) => {
                self.note(&o.name, NoteKind::Approximated, format!("hole cut {:?} imported as a counterbore of HoleCutDiameter/HoleCutDepth", other.unwrap_or("?")));
                HoleCut::Counterbore
            }
        };
        h.cut_diameter = o.float("HoleCutDiameter").unwrap_or(h.cut_diameter);
        h.cut_depth = o.float("HoleCutDepth").unwrap_or(h.cut_depth);
        h.countersink_angle = o.float("HoleCutCountersinkAngle").unwrap_or(90.0);
        if o.bool("Threaded").unwrap_or(false) {
            self.note(&o.name, NoteKind::Approximated, "thread is not modelled (plain hole of Diameter)");
        }
        if o.bool("Tapered").unwrap_or(false) {
            self.note(&o.name, NoteKind::Approximated, "tapered hole imported as straight");
        }
        if o.bool("DrillForDepth").unwrap_or(false) && angled {
            self.note(&o.name, NoteKind::Approximated, "DrillForDepth is ignored (depth is measured to the shoulder)");
        }
        if o.bool("Reversed").unwrap_or(false) {
            self.note(&o.name, NoteKind::Approximated, "Reversed is ignored");
        }
        self.add(o, FeatureKind::Hole(h));
    }

    fn loft(&mut self, o: &FcObj) {
        let additive = o.ty == "PartDesign::AdditiveLoft";
        let Some(profile) = self.sketch_link(o, "Profile") else { return };
        let mut sections = Vec::new();
        for (s, _) in o.link_sub_list("Sections") {
            match (self.sketches.contains_key(&s), self.ids.get(&s)) {
                (true, Some(id)) => sections.push(*id),
                _ => {
                    self.note(&o.name, NoteKind::Skipped, format!("section {s} is not an imported sketch; feature skipped"));
                    return;
                }
            }
        }
        if o.bool("Closed").unwrap_or(false) {
            self.note(&o.name, NoteKind::Approximated, "Closed loft imported as open");
        }
        let l = Loft { profile, sections, ruled: o.bool("Ruled").unwrap_or(false) };
        self.add(o, if additive { FeatureKind::AdditiveLoft(l) } else { FeatureKind::SubtractiveLoft(l) });
    }

    fn pipe(&mut self, o: &FcObj) {
        let additive = o.ty == "PartDesign::AdditivePipe";
        let Some(profile) = self.sketch_link(o, "Profile") else { return };
        let Some(spine) = self.sketch_link(o, "Spine") else { return };
        if let Some((sname, subs)) = o.link_sub("Spine") {
            if let Some(sk) = self.sketches.get(&sname) {
                if !subs.is_empty() && subs.len() != sk.edges.len() {
                    self.notes.push(&o.name, NoteKind::Approximated, format!("spine uses {} of {} edges; all edges are used", subs.len(), sk.edges.len()));
                }
            }
        }
        if !o.link_sub_list("Sections").is_empty() {
            self.note(&o.name, NoteKind::Approximated, "multi-section pipe: intermediate sections are ignored");
        }
        let mode = o.enumeration("Mode", PIPE_MODES).and_then(|(_, s)| s).unwrap_or_default();
        let frenet = match mode.as_str() {
            "Standard" | "" => false,
            "Frenet" => true,
            other => {
                self.note(&o.name, NoteKind::Approximated, format!("orientation mode {other} imported as Standard"));
                false
            }
        };
        let transition = o.enumeration("Transition", PIPE_TRANSITIONS).and_then(|(_, s)| s).unwrap_or_default();
        let transition = match transition.as_str() {
            "Right corner" => crate::Transition::RightCorner,
            "Round corner" => crate::Transition::RoundCorner,
            _ => crate::Transition::Transformed,
        };
        let p = Pipe { profile, spine, frenet, transition };
        self.add(o, if additive { FeatureKind::AdditivePipe(p) } else { FeatureKind::SubtractivePipe(p) });
    }

    // ----------------------------------------------------------------------------- expressions

    fn expressions(&mut self, order: &[String], _body: &FcObj) {
        // Spreadsheet cells -> parameters.
        let mut sheets: HashMap<String, HashMap<String, String>> = HashMap::new();
        let mut cells: Vec<(String, String, String)> = Vec::new(); // (sheet, param, content)
        for name in order {
            let o = &self.objs[name];
            if o.ty != "Spreadsheet::Sheet" {
                continue;
            }
            let mut map = HashMap::new();
            if let Some(c) = o.prop("cells").and_then(|p| xml::elements(p).find(|c| c.tag_name().name() == "Cells")) {
                for cell in xml::elements(c).filter(|n| n.tag_name().name() == "Cell") {
                    let (Some(addr), Some(content)) = (cell.attribute("address"), cell.attribute("content")) else { continue };
                    if content.starts_with('\'') || content.is_empty() {
                        continue;
                    }
                    let param = cell.attribute("alias").map(str::to_string).unwrap_or_else(|| format!("{name}_{addr}"));
                    map.insert(addr.to_string(), param.clone());
                    if let Some(a) = cell.attribute("alias") {
                        map.insert(a.to_string(), param.clone());
                    }
                    cells.push((name.clone(), param, content.to_string()));
                }
            }
            sheets.insert(name.clone(), map);
        }
        let labels: HashMap<String, String> =
            self.objs.values().filter_map(|o| o.string("Label").map(|l| (l, o.name.clone()))).collect();
        let con_maps: HashMap<String, Vec<Vec<usize>>> = self.sketches.iter().map(|(k, v)| (k.clone(), v.con_map.clone())).collect();
        let feats: std::collections::HashSet<String> = self.ids.keys().cloned().collect();
        let res = Res { labels, sheets: &sheets, con_maps: &con_maps, feats: &feats };
        for (sheet, param, content) in &cells {
            let v = exprs::rewrite(content, sheet, &res);
            if self.doc.parameters.contains_key(param) {
                self.note(sheet, NoteKind::Approximated, format!("parameter {param} defined twice; keeping the first"));
                continue;
            }
            self.doc.parameters.insert(param.clone(), v);
        }
        // Bindings.
        let mut owners: Vec<String> = self.doc.features.iter().map(|f| f.name.clone()).collect();
        owners.dedup();
        for owner in owners {
            let o = &self.objs[&owner];
            for (path, src) in o.expressions() {
                let e = exprs::rewrite(&src, &owner, &res);
                let p = path.trim_start_matches('.');
                let (target, vm) = if let Some(rest) = p.strip_prefix("Constraints") {
                    let fc = if let Some(ix) = rest.strip_prefix('[').and_then(|r| r.strip_suffix(']')) {
                        ix.trim().parse::<usize>().ok()
                    } else {
                        let cname = rest.trim_start_matches('.');
                        self.constraint_by_name(&owner, cname)
                    };
                    let Some(fc) = fc else {
                        self.note(&owner, NoteKind::Skipped, format!("expression on {path}: constraint not found"));
                        continue;
                    };
                    let Some(ours) = self.sketches.get(&owner).and_then(|s| s.our_con(fc)) else {
                        self.note(&owner, NoteKind::Skipped, format!("expression on {path}: constraint was not imported"));
                        continue;
                    };
                    let vm = self.value_maps.get(&owner).and_then(|v| v.get(fc).copied()).unwrap_or(ValueMap::ID);
                    (format!("{owner}.Constraints[{ours}]"), vm)
                } else if p == "AttachmentOffset.Base.z" || p == "AttachmentOffset.z" {
                    (format!("{owner}.AttachmentOffset"), self.offset_maps.get(&owner).copied().unwrap_or(ValueMap::ID))
                } else {
                    let prop = match p {
                        "HoleCutDiameter" => "CutDiameter",
                        "HoleCutDepth" => "CutDepth",
                        "HoleCutCountersinkAngle" => "CountersinkAngle",
                        other => other,
                    };
                    (format!("{owner}.{prop}"), ValueMap::ID)
                };
                let Some(key) = self.doc.canonical_name(&target) else {
                    self.note(&owner, NoteKind::Skipped, format!("expression on {path} has no fc-part counterpart"));
                    continue;
                };
                let e = if vm.is_identity() {
                    e
                } else if vm.scale == 1.0 {
                    format!("({e}) + {}", vm.add)
                } else {
                    format!("({e}) * {} + {}", vm.scale, vm.add)
                };
                if crate::expr::parse(&e).is_err() {
                    self.note(&owner, NoteKind::Skipped, format!("expression {path} = {src:?} could not be parsed (as {e:?})"));
                    continue;
                }
                self.doc.expressions.insert(key, e);
            }
        }
        // Drop what does not evaluate (keeps the stored values).
        let (_, errs) = self.doc.evaluate_expressions();
        for (k, err) in errs {
            if let Some(p) = k.strip_prefix("param:") {
                let v = self.doc.parameters.remove(p);
                self.note(p, NoteKind::Skipped, format!("parameter {p} = {:?}: {err}", v.unwrap_or_default()));
            } else {
                let v = self.doc.expressions.remove(&k);
                self.note(&k, NoteKind::Skipped, format!("expression {k} = {:?}: {err}", v.unwrap_or_default()));
            }
        }
    }

    fn constraint_by_name(&self, sketch: &str, cname: &str) -> Option<usize> {
        let o = self.objs.get(sketch)?;
        let list = o.prop("Constraints").and_then(|p| xml::elements(p).find(|c| c.tag_name().name() == "ConstraintList"))?;
        xml::elements(list).filter(|n| n.tag_name().name() == "Constrain").position(|c| c.attribute("Name") == Some(cname))
    }
}

struct Res<'b> {
    labels: HashMap<String, String>,
    sheets: &'b HashMap<String, HashMap<String, String>>,
    con_maps: &'b HashMap<String, Vec<Vec<usize>>>,
    feats: &'b std::collections::HashSet<String>,
}

impl exprs::Resolver for Res<'_> {
    fn by_label(&self, label: &str) -> Option<String> {
        self.labels.get(label).cloned()
    }
    fn path(&self, segs: &[String], owner: &str) -> Option<String> {
        let (obj, rest): (&str, &[String]) = if segs[0].is_empty() {
            (owner, &segs[1..])
        } else if segs[0].starts_with("Constraints") && self.con_maps.contains_key(owner) {
            (owner, segs)
        } else {
            (segs[0].as_str(), &segs[1..])
        };
        // FreeCAD also accepts a label that is a valid identifier in place of the name.
        let by_label;
        let obj = if !self.sheets.contains_key(obj) && !self.feats.contains(obj) && !rest.is_empty() {
            by_label = self.labels.get(obj).cloned().unwrap_or_else(|| obj.to_string());
            by_label.as_str()
        } else {
            obj
        };
        if let Some(sheet) = self.sheets.get(obj) {
            return rest.first().and_then(|c| sheet.get(c)).cloned();
        }
        if rest.is_empty() {
            // Bare name inside a spreadsheet cell: alias or cell of the same sheet.
            return self.sheets.get(owner).and_then(|s| s.get(obj)).cloned();
        }
        if !self.feats.contains(obj) {
            return None;
        }
        if let Some(cm) = self.con_maps.get(obj) {
            if let Some(ix) = rest[0].strip_prefix("Constraints[").and_then(|r| r.strip_suffix(']')) {
                let ours = ix.trim().parse::<usize>().ok().and_then(|i| cm.get(i)).and_then(|v| v.first().copied());
                return ours.map(|o| format!("{obj}.Constraints[{o}]"));
            }
        }
        let prop: Vec<String> = rest
            .iter()
            .map(|p| match p.as_str() {
                "HoleCutDiameter" => "CutDiameter".to_string(),
                "HoleCutDepth" => "CutDepth".to_string(),
                "HoleCutCountersinkAngle" => "CountersinkAngle".to_string(),
                o => o.to_string(),
            })
            .collect();
        Some(format!("{obj}.{}", prop.join(".")))
    }
}

fn default_plane(role: &str) -> FcPlacement {
    match role {
        "XZ_Plane" => FcPlacement::rotation([1.0, 0.0, 0.0], PI / 2.0),
        "YZ_Plane" => {
            // x -> Y, y -> Z: rotation of 120 deg about (1,1,1).
            let k = 1.0 / 3f64.sqrt();
            FcPlacement::rotation([k, k, k], 2.0 * PI / 3.0)
        }
        _ => FcPlacement::IDENTITY,
    }
}

fn same_frame(a: &FcPlacement, b: &FcPlacement) -> bool {
    approx_eq3(a.origin, b.origin, 1e-6) && approx_eq3(a.xdir(), b.xdir(), 1e-9) && approx_eq3(a.ydir(), b.ydir(), 1e-9)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line(x0: f64, y0: f64, x1: f64, y1: f64, construction: bool) -> String {
        format!(
            r#"<Geometry type="Part::GeomLineSegment"><LineSegment StartX="{x0}" StartY="{y0}" StartZ="0" EndX="{x1}" EndY="{y1}" EndZ="0"/><Construction value="{}"/></Geometry>"#,
            construction as u8
        )
    }

    /// A hand-written pre-0.20 style document: `Support` instead of `AttachmentSupport`,
    /// no geometry extensions, legacy First/Second constraint attributes, `Midplane`.
    #[test]
    fn old_style_document() {
        let geo = [
            line(0.0, 0.0, 10.0, 0.0, false),
            line(10.0, 0.0, 10.0, 5.0, false),
            line(10.0, 5.0, 0.0, 5.0, false),
            line(0.0, 5.0, 0.0, 0.0, false),
            line(0.0, 0.0, 3.0, 3.0, true),
        ]
        .join("");
        let c = |ty: i32, f: i32, fp: i32, s: i32, sp: i32, v: f64| {
            format!(r#"<Constrain Name="" Type="{ty}" Value="{v}" First="{f}" FirstPos="{fp}" Second="{s}" SecondPos="{sp}" Third="-2000" ThirdPos="0" LabelDistance="10" LabelPosition="0" IsDriving="1"/>"#)
        };
        let cons = [
            c(1, 0, 2, 1, 1, 0.0),
            c(1, 1, 2, 2, 1, 0.0),
            c(1, 2, 2, 3, 1, 0.0),
            c(1, 3, 2, 0, 1, 0.0),
            c(2, 0, 0, -2000, 0, 0.0),
            c(2, 2, 0, -2000, 0, 0.0),
            c(3, 1, 0, -2000, 0, 0.0),
            c(3, 3, 0, -2000, 0, 0.0),
            c(7, 0, 0, -2000, 0, 10.0),
            c(8, 1, 0, -2000, 0, 5.0),
            c(1, 0, 1, -1, 1, 0.0),
        ]
        .join("");
        let xml = format!(
            r#"<?xml version='1.0' encoding='utf-8'?>
<Document SchemaVersion="4" ProgramVersion="0.19R24291" FileVersion="1">
<Objects Count="4">
  <Object type="PartDesign::Body" name="Body"/>
  <Object type="App::Plane" name="XY_Plane"/>
  <Object type="Sketcher::SketchObject" name="Sketch"/>
  <Object type="PartDesign::Pad" name="Pad"/>
</Objects>
<ObjectData Count="4">
  <Object name="Body"><Properties Count="2">
    <Property name="Group" type="App::PropertyLinkList"><LinkList count="2"><Link value="Sketch"/><Link value="Pad"/></LinkList></Property>
    <Property name="Tip" type="App::PropertyLink"><Link value="Pad"/></Property>
  </Properties></Object>
  <Object name="XY_Plane"><Properties Count="1">
    <Property name="Role" type="App::PropertyString"><String value="XY_Plane"/></Property>
  </Properties></Object>
  <Object name="Sketch"><Properties Count="4">
    <Property name="Support" type="App::PropertyLinkSubList"><LinkSubList count="1"><Link obj="XY_Plane" sub=""/></LinkSubList></Property>
    <Property name="MapMode" type="App::PropertyEnumeration"><Integer value="5"/></Property>
    <Property name="Geometry" type="Part::PropertyGeometryList"><GeometryList count="5">{geo}</GeometryList></Property>
    <Property name="Constraints" type="Sketcher::PropertyConstraintList"><ConstraintList count="11">{cons}</ConstraintList></Property>
  </Properties></Object>
  <Object name="Pad"><Properties Count="4">
    <Property name="Profile" type="App::PropertyLinkSub"><LinkSub value="Sketch" count="0"></LinkSub></Property>
    <Property name="Length" type="App::PropertyLength"><Float value="4"/></Property>
    <Property name="Midplane" type="App::PropertyBool"><Bool value="true"/></Property>
    <Property name="Type" type="App::PropertyEnumeration"><Integer value="0"/></Property>
  </Properties></Object>
</ObjectData>
</Document>"#
        );
        let (mut doc, rep) = import_document_xml(&xml).unwrap();
        assert!(rep.notes.iter().all(|n| n.kind == NoteKind::Info), "{:?}", rep.notes);
        let FeatureKind::Sketch(sf) = &doc.by_name("Sketch").unwrap().kind else { panic!() };
        assert_eq!(sf.sketch.geometry.len(), 5);
        assert!(sf.sketch.geometry[4].construction && !sf.sketch.geometry[0].construction);
        assert_eq!(sf.sketch.constraints.len(), 11);
        assert_eq!(sf.sketch.diagnose().dof, 4, "construction line is free");
        let FeatureKind::Pad(e) = &doc.by_name("Pad").unwrap().kind else { panic!() };
        assert_eq!(e.side, SideType::Symmetric);
        doc.recompute();
        let s = doc.result().unwrap();
        assert!((s.volume() - 200.0).abs() < 1e-9);
        let b = s.bbox();
        assert!((b[2] + 2.0).abs() < 1e-6 && (b[5] - 2.0).abs() < 1e-6, "{b:?}");
    }

    #[test]
    fn default_planes_match_freecad() {
        let xz = default_plane("XZ_Plane");
        assert!(approx_eq3(xz.xdir(), [1.0, 0.0, 0.0], 1e-12));
        assert!(approx_eq3(xz.ydir(), [0.0, 0.0, 1.0], 1e-12));
        let yz = default_plane("YZ_Plane");
        assert!(approx_eq3(yz.xdir(), [0.0, 1.0, 0.0], 1e-12));
        assert!(approx_eq3(yz.ydir(), [0.0, 0.0, 1.0], 1e-12));
    }
}
