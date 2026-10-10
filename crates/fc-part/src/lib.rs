//! Part Design document model: a Body holding sketches and the features that turn them
//! into a single solid, recomputed in tree order (FreeCAD's PartDesign::Body semantics).

pub mod expr;
pub mod fcstd;
pub mod measure;
pub mod params;
pub mod placement;
pub mod profile;

use fc_occt::Shape;
use fc_sketch::{SolveReport, SolveStatus, Sketch};
pub use placement::{Placement, V3};
use placement::*;
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

#[derive(Debug, Clone, thiserror::Error)]
pub enum PartError {
    #[error("kernel: {0}")]
    Kernel(String),
    #[error("profile: {0}")]
    Profile(String),
    #[error("sketch is over-constrained (conflicting constraints {0:?})")]
    SketchConflict(Vec<usize>),
    #[error("sketch has redundant constraints {0:?}")]
    SketchRedundant(Vec<usize>),
    #[error("sketch failed to solve")]
    SketchFailed,
    #[error("missing input: {0}")]
    Missing(String),
    #[error("{0}")]
    Invalid(String),
}
impl From<fc_occt::OcctError> for PartError {
    fn from(e: fc_occt::OcctError) -> Self {
        PartError::Kernel(e.0)
    }
}

/// Stable feature identifier (survives reordering/deletion of other features).
pub type FeatureId = u32;

/// Reference to a face of a feature's resulting solid. `index` is FreeCAD's 1-based
/// `Face<n>`; the signature lets recompute re-find the face when numbering shifts
/// (a light answer to FreeCAD's topological naming problem).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FaceRef {
    pub feature: FeatureId,
    pub index: usize,
    pub normal: V3,
    pub center: V3,
    pub area: f64,
}

/// What a sketch is attached to.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Support {
    XY,
    XZ,
    YZ,
    /// FlatFace attachment to a planar face of a feature's solid.
    Face(FaceRef),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum SideType {
    #[default]
    OneSide,
    TwoSides,
    Symmetric,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum ExtentType {
    #[default]
    Length,
    ThroughAll,
    /// Up to the first face of the base solid hit along the direction.
    UpToFirst,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Extrude {
    pub profile: FeatureId,
    pub side: SideType,
    pub kind: ExtentType,
    pub length: f64,
    /// Second side's type/length for `SideType::TwoSides`.
    pub kind2: ExtentType,
    pub length2: f64,
    pub reversed: bool,
}

impl Extrude {
    pub fn new(profile: FeatureId, length: f64) -> Self {
        Extrude { profile, side: SideType::OneSide, kind: ExtentType::Length, length, kind2: ExtentType::Length, length2: 0.0, reversed: false }
    }
}

/// Revolution axis choice (FreeCAD's ReferenceAxis).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum AxisRef {
    /// Sketch vertical axis (FreeCAD default "V_Axis").
    SketchV,
    SketchH,
    /// A (usually construction) line of the profile sketch, by GeoId.
    SketchLine(usize),
    X,
    Y,
    Z,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Revolve {
    pub profile: FeatureId,
    pub axis: AxisRef,
    /// Degrees, like the FreeCAD property.
    pub angle: f64,
    pub side: SideType,
    pub angle2: f64,
    pub reversed: bool,
}

impl Revolve {
    pub fn new(profile: FeatureId) -> Self {
        Revolve { profile, axis: AxisRef::SketchV, angle: 360.0, side: SideType::OneSide, angle2: 0.0, reversed: false }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Loft {
    pub profile: FeatureId,
    pub sections: Vec<FeatureId>,
    pub ruled: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Pipe {
    pub profile: FeatureId,
    pub spine: FeatureId,
    pub frenet: bool,
    /// How sharp spine corners are swept (FreeCAD's Transition; default Transformed).
    #[serde(default)]
    pub transition: Transition,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum Transition {
    #[default]
    Transformed,
    RightCorner,
    RoundCorner,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum HoleCut {
    #[default]
    None,
    Counterbore,
    Countersink,
}

/// PartDesign Hole: one hole per circle center (and per point) of the sketch.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Hole {
    pub profile: FeatureId,
    pub diameter: f64,
    pub depth: f64,
    pub through_all: bool,
    /// Drill point angle in degrees (118 by default); 0 = flat bottom.
    pub drill_point_angle: f64,
    pub cut: HoleCut,
    pub cut_diameter: f64,
    pub cut_depth: f64,
    /// Countersink included angle in degrees.
    pub countersink_angle: f64,
}

impl Hole {
    pub fn new(profile: FeatureId, diameter: f64, depth: f64) -> Self {
        Hole { profile, diameter, depth, through_all: false, drill_point_angle: 118.0, cut: HoleCut::None, cut_diameter: diameter * 1.8, cut_depth: diameter * 0.6, countersink_angle: 90.0 }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SketchFeature {
    pub sketch: Sketch,
    pub support: Support,
    /// FreeCAD's AttachmentOffset, restricted to an offset along the normal and a flip.
    pub offset: f64,
    pub reversed: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum FeatureKind {
    Sketch(SketchFeature),
    Pad(Extrude),
    Pocket(Extrude),
    Revolution(Revolve),
    Groove(Revolve),
    AdditiveLoft(Loft),
    SubtractiveLoft(Loft),
    AdditivePipe(Pipe),
    SubtractivePipe(Pipe),
    Hole(Hole),
}

impl FeatureKind {
    pub fn type_name(&self) -> &'static str {
        match self {
            FeatureKind::Sketch(_) => "Sketch",
            FeatureKind::Pad(_) => "Pad",
            FeatureKind::Pocket(_) => "Pocket",
            FeatureKind::Revolution(_) => "Revolution",
            FeatureKind::Groove(_) => "Groove",
            FeatureKind::AdditiveLoft(_) => "AdditiveLoft",
            FeatureKind::SubtractiveLoft(_) => "SubtractiveLoft",
            FeatureKind::AdditivePipe(_) => "AdditivePipe",
            FeatureKind::SubtractivePipe(_) => "SubtractivePipe",
            FeatureKind::Hole(_) => "Hole",
        }
    }
    /// Sketches this feature consumes (hidden under it in the tree, like FreeCAD).
    pub fn inputs(&self) -> Vec<FeatureId> {
        match self {
            FeatureKind::Sketch(s) => match &s.support {
                Support::Face(f) => vec![f.feature],
                _ => vec![],
            },
            FeatureKind::Pad(e) | FeatureKind::Pocket(e) => vec![e.profile],
            FeatureKind::Revolution(r) | FeatureKind::Groove(r) => vec![r.profile],
            FeatureKind::AdditiveLoft(l) | FeatureKind::SubtractiveLoft(l) => {
                std::iter::once(l.profile).chain(l.sections.iter().copied()).collect()
            }
            FeatureKind::AdditivePipe(p) | FeatureKind::SubtractivePipe(p) => vec![p.profile, p.spine],
            FeatureKind::Hole(h) => vec![h.profile],
        }
    }
    pub fn is_solid_feature(&self) -> bool {
        !matches!(self, FeatureKind::Sketch(_))
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Feature {
    pub id: FeatureId,
    pub name: String,
    pub label: String,
    pub kind: FeatureKind,
    #[serde(default)]
    pub suppressed: bool,
}

/// Recompute output for one feature.
#[derive(Debug, Default)]
pub struct FeatureState {
    /// Body solid after this feature (solid features), or the support face plane etc.
    pub shape: Option<Shape>,
    /// The tool solid that was added/removed (shown transparent while editing, like FreeCAD's AddSubShape).
    pub tool: Option<Shape>,
    /// Resolved sketch plane (sketches only).
    pub placement: Option<Placement>,
    pub solve: Option<SolveReport>,
    pub error: Option<PartError>,
    /// Non-fatal problem (e.g. redundant sketch constraints), shown like FreeCAD's warning.
    pub warning: Option<String>,
}

/// A PartDesign Body. Features are kept in tree order; the tip is the last solid feature
/// unless set explicitly.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Document {
    pub features: Vec<Feature>,
    pub next_id: FeatureId,
    /// Feature after which modelling stops (FreeCAD's Tip). None = last feature.
    pub tip: Option<FeatureId>,
    /// Named parameters (name -> expression), like spreadsheet aliases.
    #[serde(default)]
    pub parameters: std::collections::BTreeMap<String, String>,
    /// Bound expressions: canonical target (`Pad.Length`, `Sketch.Constraints[3]`) -> expression.
    #[serde(default)]
    pub expressions: std::collections::BTreeMap<String, String>,
    /// Named measurements another program reads (see [`measure`]).
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub measurements: std::collections::BTreeMap<String, measure::MeasureSpec>,
    /// Parameters another program drives: name -> where the value comes from (e.g. "Calc Canvas: Bracket").
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub driven: std::collections::BTreeMap<String, String>,
    #[serde(skip)]
    pub expression_errors: std::collections::BTreeMap<String, expr::ExprError>,
    #[serde(skip)]
    pub states: Vec<FeatureState>,
}

impl Clone for FeatureState {
    fn clone(&self) -> Self {
        FeatureState { shape: self.shape.clone(), tool: self.tool.clone(), placement: self.placement, solve: self.solve.clone(), error: self.error.clone(), warning: self.warning.clone() }
    }
}

const THROUGH_ALL_MIN: f64 = 1000.0;

impl Document {
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds a feature with a FreeCAD-style unique name ("Pad", "Pad001", ...).
    pub fn add(&mut self, kind: FeatureKind) -> FeatureId {
        let base = kind.type_name();
        let mut name = base.to_string();
        let mut n = 0;
        while self.features.iter().any(|f| f.name == name) {
            n += 1;
            name = format!("{base}{n:03}");
        }
        let id = self.next_id;
        self.next_id += 1;
        self.features.push(Feature { id, label: name.clone(), name, kind, suppressed: false });
        id
    }

    pub fn add_sketch(&mut self, support: Support, sketch: Sketch) -> FeatureId {
        self.add(FeatureKind::Sketch(SketchFeature { sketch, support, offset: 0.0, reversed: false }))
    }

    pub fn index_of(&self, id: FeatureId) -> Option<usize> {
        self.features.iter().position(|f| f.id == id)
    }
    pub fn get(&self, id: FeatureId) -> Option<&Feature> {
        self.features.iter().find(|f| f.id == id)
    }
    pub fn get_mut(&mut self, id: FeatureId) -> Option<&mut Feature> {
        self.features.iter_mut().find(|f| f.id == id)
    }
    pub fn by_name(&self, name: &str) -> Option<&Feature> {
        self.features.iter().find(|f| f.name == name)
    }
    pub fn sketch_mut(&mut self, id: FeatureId) -> Option<&mut Sketch> {
        match &mut self.get_mut(id)?.kind {
            FeatureKind::Sketch(s) => Some(&mut s.sketch),
            _ => None,
        }
    }
    pub fn state(&self, id: FeatureId) -> Option<&FeatureState> {
        self.states.get(self.index_of(id)?)
    }

    /// Features that use `id` as an input.
    pub fn dependents(&self, id: FeatureId) -> Vec<FeatureId> {
        self.features.iter().filter(|f| f.kind.inputs().contains(&id)).map(|f| f.id).collect()
    }

    /// Removes a feature. Fails if other features depend on it.
    pub fn remove(&mut self, id: FeatureId) -> Result<(), PartError> {
        let deps = self.dependents(id);
        if !deps.is_empty() {
            let names: Vec<_> = deps.iter().filter_map(|d| self.get(*d)).map(|f| f.name.clone()).collect();
            return Err(PartError::Invalid(format!("used by {}", names.join(", "))));
        }
        if let Some(i) = self.index_of(id) {
            self.features.remove(i);
            if i < self.states.len() {
                self.states.remove(i);
            }
        }
        if self.tip == Some(id) {
            self.tip = None;
        }
        Ok(())
    }

    fn tip_index(&self) -> usize {
        match self.tip.and_then(|t| self.index_of(t)) {
            Some(i) => i,
            None => self.features.len().saturating_sub(1),
        }
    }

    /// The Body's final solid (shape after the tip).
    pub fn result(&self) -> Option<&Shape> {
        let tip = self.tip_index();
        self.states.iter().take(tip + 1).rev().find_map(|s| s.shape.as_ref().filter(|_| s.error.is_none()))
            .or_else(|| self.states.iter().take(tip + 1).rev().find_map(|s| s.shape.as_ref()))
    }

    /// The solid feature output just before index `i` (the BaseFeature).
    fn base_before(&self, states: &[FeatureState], i: usize) -> Option<Shape> {
        for j in (0..i).rev() {
            if self.features[j].kind.is_solid_feature() && !self.features[j].suppressed {
                if let Some(s) = &states[j].shape {
                    return Some(s.clone());
                }
            }
        }
        None
    }

    /// Recompute every feature in order. Errors are stored per feature; a failed solid
    /// feature passes its base shape through so later features still see a solid.
    pub fn recompute(&mut self) {
        self.expression_errors = self.apply_expressions();
        let mut states: Vec<FeatureState> = Vec::with_capacity(self.features.len());
        for i in 0..self.features.len() {
            let mut st = FeatureState::default();
            if self.features[i].suppressed {
                st.shape = self.base_before(&states, i).filter(|_| self.features[i].kind.is_solid_feature());
                states.push(st);
                continue;
            }
            let res = self.compute_one(i, &states, &mut st);
            if let Err(e) = res {
                st.error = Some(e);
                if self.features[i].kind.is_solid_feature() {
                    st.shape = self.base_before(&states, i);
                }
            }
            states.push(st);
        }
        self.states = states;
    }

    fn sketch_input<'a>(&'a self, states: &'a [FeatureState], id: FeatureId) -> Result<(&'a Sketch, Placement), PartError> {
        let i = self.index_of(id).ok_or_else(|| PartError::Missing(format!("feature #{id}")))?;
        let f = &self.features[i];
        let FeatureKind::Sketch(sf) = &f.kind else {
            return Err(PartError::Missing(format!("{} is not a sketch", f.name)));
        };
        let st = states.get(i).ok_or_else(|| PartError::Invalid(format!("{} comes after its user in the tree", f.name)))?;
        if let Some(e) = &st.error {
            return Err(PartError::Invalid(format!("{} has an error: {e}", f.name)));
        }
        Ok((&sf.sketch, st.placement.unwrap_or_default()))
    }

    fn resolve_support(&self, states: &[FeatureState], sf: &SketchFeature) -> Result<Placement, PartError> {
        let base = match &sf.support {
            Support::XY => Placement::XY,
            Support::XZ => Placement::XZ,
            Support::YZ => Placement::YZ,
            Support::Face(fr) => {
                let i = self.index_of(fr.feature).ok_or_else(|| PartError::Missing("support feature".into()))?;
                let shape = states.get(i).and_then(|s| s.shape.as_ref()).ok_or_else(|| PartError::Missing("support shape".into()))?;
                let face = find_face(shape, fr).ok_or_else(|| PartError::Missing(format!("Face{} of support", fr.index)))?;
                if face.kind != 0 {
                    return Err(PartError::Invalid("support face is not planar".into()));
                }
                // FlatFace attachment. OCCT's plane origin is arbitrary (booleans/refine move it),
                // so use the world origin projected onto the face and an X axis aligned with the
                // nearest world axis: sketch coordinates then stay stable across recomputes.
                let n = normalize(face.normal);
                let origin = scale(n, dot(face.origin, n));
                Placement::from_normal(origin, n, flat_face_xhint(n))
            }
        };
        // FreeCAD applies AttachmentOffset in the (possibly reversed) attached frame.
        let p = if sf.reversed { base.reversed() } else { base };
        Ok(p.offset(sf.offset))
    }

    fn compute_one(&mut self, i: usize, states: &[FeatureState], st: &mut FeatureState) -> Result<(), PartError> {
        let kind = self.features[i].kind.clone();
        match kind {
            FeatureKind::Sketch(sf) => {
                let pl = self.resolve_support(states, &sf)?;
                st.placement = Some(pl);
                let FeatureKind::Sketch(sfm) = &mut self.features[i].kind else { unreachable!() };
                let rep = sfm.sketch.solve();
                st.solve = Some(rep.clone());
                match rep.status {
                    SolveStatus::Conflicting | SolveStatus::Failed => {
                        return Err(if rep.conflicting.is_empty() { PartError::SketchFailed } else { PartError::SketchConflict(rep.conflicting) });
                    }
                    // FreeCAD flags sketches with redundant constraints but features still use them.
                    SolveStatus::Redundant => st.warning = Some(PartError::SketchRedundant(rep.redundant).to_string()),
                    SolveStatus::Solved => {}
                }
                Ok(())
            }
            FeatureKind::Pad(e) | FeatureKind::Pocket(e) => {
                let additive = matches!(self.features[i].kind, FeatureKind::Pad(_));
                let base = self.base_before(states, i);
                let (sk, pl) = self.sketch_input(states, e.profile)?;
                let face = profile::sketch_faces(sk, &pl)?;
                let tool = extrude_tool(&face, &pl, &e, additive, base.as_ref())?;
                st.shape = Some(combine(base.as_ref(), &tool, additive)?);
                st.tool = Some(tool);
                Ok(())
            }
            FeatureKind::Revolution(r) | FeatureKind::Groove(r) => {
                let additive = matches!(self.features[i].kind, FeatureKind::Revolution(_));
                let base = self.base_before(states, i);
                let (sk, pl) = self.sketch_input(states, r.profile)?;
                let face = profile::sketch_faces(sk, &pl)?;
                let (o, d) = revolve_axis(sk, &pl, &r.axis)?;
                let tool = revolve_tool(&face, o, d, &r)?;
                st.shape = Some(combine(base.as_ref(), &tool, additive)?);
                st.tool = Some(tool);
                Ok(())
            }
            FeatureKind::AdditiveLoft(l) | FeatureKind::SubtractiveLoft(l) => {
                let additive = matches!(self.features[i].kind, FeatureKind::AdditiveLoft(_));
                let base = self.base_before(states, i);
                let mut secs = fc_occt::Shape::default();
                for id in std::iter::once(l.profile).chain(l.sections.iter().copied()) {
                    let (sk, pl) = self.sketch_input(states, id)?;
                    let w = profile::sketch_wires(sk, &pl)?;
                    secs = secs.compound_with(&w)?;
                }
                let tool = Shape::loft(&secs, true, l.ruled)?;
                st.shape = Some(combine(base.as_ref(), &tool, additive)?);
                st.tool = Some(tool);
                Ok(())
            }
            FeatureKind::AdditivePipe(p) | FeatureKind::SubtractivePipe(p) => {
                let additive = matches!(self.features[i].kind, FeatureKind::AdditivePipe(_));
                let base = self.base_before(states, i);
                let (sk, pl) = self.sketch_input(states, p.profile)?;
                let prof = profile::sketch_wires(sk, &pl)?;
                let (spk, spl) = self.sketch_input(states, p.spine)?;
                let spine = profile::sketch_path(spk, &spl)?;
                let tool = Shape::sweep(&prof, &spine, p.frenet, p.transition as i32)?;
                st.shape = Some(combine(base.as_ref(), &tool, additive)?);
                st.tool = Some(tool);
                Ok(())
            }
            FeatureKind::Hole(h) => {
                let base = self.base_before(states, i).ok_or_else(|| PartError::Missing("Hole needs a base solid".into()))?;
                let (sk, pl) = self.sketch_input(states, h.profile)?;
                let tool = hole_tool(sk, &pl, &h, &base)?;
                st.shape = Some(combine(Some(&base), &tool, false)?);
                st.tool = Some(tool);
                Ok(())
            }
        }
    }

    pub fn to_json(&self) -> String {
        serde_json::to_string_pretty(self).expect("document serializes")
    }
    pub fn from_json(s: &str) -> Result<Document, serde_json::Error> {
        let mut d: Document = serde_json::from_str(s)?;
        d.next_id = d.next_id.max(d.features.iter().map(|f| f.id + 1).max().unwrap_or(0));
        Ok(d)
    }
}

/// Sketch X direction for a face attachment, following FreeCAD's legacy FlatFace rule:
/// normal closest to ±Z -> +X; +Y -> -X; -Y -> +X; +X -> +Y; -X -> -Y.
pub fn flat_face_xhint(n: V3) -> V3 {
    let (ax, ay, az) = (n[0].abs(), n[1].abs(), n[2].abs());
    if az >= ax && az >= ay {
        [1.0, 0.0, 0.0]
    } else if ay >= ax {
        if n[1] > 0.0 { [-1.0, 0.0, 0.0] } else { [1.0, 0.0, 0.0] }
    } else if n[0] > 0.0 {
        [0.0, 1.0, 0.0]
    } else {
        [0.0, -1.0, 0.0]
    }
}

/// Find a referenced face: same index if its signature still matches, else the best match.
pub fn find_face(shape: &Shape, fr: &FaceRef) -> Option<fc_occt::FaceInfo> {
    let faces = shape.faces();
    let score = |f: &fc_occt::FaceInfo| -> f64 {
        let dn = 1.0 - dot(normalize(f.normal), normalize(fr.normal));
        let dc = norm(sub(f.center, fr.center));
        let da = (f.area - fr.area).abs() / fr.area.max(1e-9);
        dn * 100.0 + dc + da * 10.0 + if f.kind != 0 { 1e6 } else { 0.0 }
    };
    if let Some(f) = faces.get(fr.index.wrapping_sub(1)) {
        if score(f) < 1e-6 {
            return Some(f.clone());
        }
    }
    faces.into_iter().filter(|f| f.kind == 0).min_by(|a, b| score(a).total_cmp(&score(b)))
}

/// Build a FaceRef for the current shape's Face<index>.
pub fn face_ref(feature: FeatureId, shape: &Shape, index: usize) -> Result<FaceRef, PartError> {
    let f = shape.face_info(index)?;
    Ok(FaceRef { feature, index, normal: f.normal, center: f.center, area: f.area })
}

fn combine(base: Option<&Shape>, tool: &Shape, additive: bool) -> Result<Shape, PartError> {
    let out = match (base, additive) {
        (None, true) => tool.clone(),
        (None, false) => return Err(PartError::Missing("cannot subtract from an empty body".into())),
        (Some(b), true) => b.fuse(tool)?,
        (Some(b), false) => b.cut(tool)?,
    };
    let out = out.refine().unwrap_or(out);
    // FreeCAD 1.1 bodies allow several solids (AllowCompound is on by default).
    match out.solid_count() {
        0 => Err(PartError::Invalid("result has no solid".into())),
        _ => Ok(out),
    }
}

fn through_all_length(base: Option<&Shape>, face: &Shape) -> f64 {
    let mut b = face.bbox();
    if let Some(s) = base {
        let c = s.bbox();
        for k in 0..3 {
            b[k] = b[k].min(c[k]);
            b[k + 3] = b[k + 3].max(c[k + 3]);
        }
    }
    let d = norm([b[3] - b[0], b[4] - b[1], b[5] - b[2]]);
    (d * 2.0).max(THROUGH_ALL_MIN)
}

/// Length of extrusion along `dir` from the profile up to the first face of `base`.
fn up_to_first(base: Option<&Shape>, face: &Shape, dir: V3) -> Result<f64, PartError> {
    let base = base.ok_or_else(|| PartError::Missing("UpToFirst needs a base solid".into()))?;
    // March the profile prism outward and find the shortest length whose prism touches the base.
    let far = through_all_length(Some(base), face);
    let full = face.prism(scale(dir, far))?;
    let hit = full.common(base)?;
    if hit.is_null() || hit.volume() < 1e-9 {
        return Err(PartError::Invalid("no face found in the extrusion direction".into()));
    }
    let o = face.bbox();
    let start = dot([o[0], o[1], o[2]], dir).min(dot([o[3], o[4], o[5]], dir));
    let hb = hit.bbox();
    let corners = [[hb[0], hb[1], hb[2]], [hb[3], hb[4], hb[5]], [hb[0], hb[4], hb[5]], [hb[3], hb[1], hb[2]], [hb[0], hb[1], hb[5]], [hb[3], hb[4], hb[2]], [hb[0], hb[4], hb[2]], [hb[3], hb[1], hb[5]]];
    let first = corners.iter().map(|c| dot(*c, dir)).fold(f64::INFINITY, f64::min);
    Ok((first - start).max(1e-6))
}

fn extrude_tool(face: &Shape, pl: &Placement, e: &Extrude, additive: bool, base: Option<&Shape>) -> Result<Shape, PartError> {
    // Pad goes along the sketch normal, Pocket against it (into the material).
    let mut dir = pl.normal();
    if !additive {
        dir = scale(dir, -1.0);
    }
    if e.reversed {
        dir = scale(dir, -1.0);
    }
    let len_of = |k: ExtentType, l: f64, d: V3| -> Result<f64, PartError> {
        match k {
            ExtentType::Length => Ok(l),
            ExtentType::ThroughAll => Ok(through_all_length(base, face)),
            ExtentType::UpToFirst => up_to_first(base, face, d),
        }
    };
    let (fwd, back) = match e.side {
        SideType::OneSide => (len_of(e.kind, e.length, dir)?, 0.0),
        SideType::Symmetric => {
            let l = len_of(e.kind, e.length, dir)?;
            if e.kind == ExtentType::ThroughAll { (l, l) } else { (l / 2.0, l / 2.0) }
        }
        SideType::TwoSides => (len_of(e.kind, e.length, dir)?, len_of(e.kind2, e.length2, scale(dir, -1.0))?),
    };
    if fwd + back < 1e-7 {
        return Err(PartError::Invalid("extrusion length is zero".into()));
    }
    if fwd < -1e-9 || back < -1e-9 {
        return Err(PartError::Invalid("negative extrusion length".into()));
    }
    let start = if back > 0.0 { face.translated(scale(dir, -back))? } else { face.clone() };
    Ok(start.prism(scale(dir, fwd + back))?)
}

fn revolve_axis(sk: &Sketch, pl: &Placement, axis: &AxisRef) -> Result<(V3, V3), PartError> {
    Ok(match axis {
        AxisRef::SketchV => (pl.origin, pl.ydir),
        AxisRef::SketchH => (pl.origin, pl.xdir),
        AxisRef::SketchLine(g) => match sk.geometry.get(*g).map(|g| &g.curve) {
            Some(fc_sketch::Curve::Line { a, b }) => {
                let pa = pl.to_world(a.x, a.y);
                let pb = pl.to_world(b.x, b.y);
                (pa, normalize(sub(pb, pa)))
            }
            _ => return Err(PartError::Invalid(format!("axis geometry {g} is not a line"))),
        },
        AxisRef::X => ([0.0; 3], [1.0, 0.0, 0.0]),
        AxisRef::Y => ([0.0; 3], [0.0, 1.0, 0.0]),
        AxisRef::Z => ([0.0; 3], [0.0, 0.0, 1.0]),
    })
}

fn revolve_tool(face: &Shape, o: V3, d: V3, r: &Revolve) -> Result<Shape, PartError> {
    let dir = if r.reversed { scale(d, -1.0) } else { d };
    let a1 = r.angle.to_radians();
    let (start, total) = match r.side {
        SideType::OneSide => (0.0, a1),
        SideType::Symmetric => (-a1 / 2.0, a1),
        SideType::TwoSides => (-r.angle2.to_radians(), a1 + r.angle2.to_radians()),
    };
    if total.abs() < 1e-9 {
        return Err(PartError::Invalid("revolution angle is zero".into()));
    }
    let total = total.min(2.0 * PI);
    let f = if start.abs() > 1e-12 && total < 2.0 * PI - 1e-9 { face.rotated(o, dir, start)? } else { face.clone() };
    Ok(f.revolve(o, dir, total)?)
}

fn hole_tool(sk: &Sketch, pl: &Placement, h: &Hole, base: &Shape) -> Result<Shape, PartError> {
    use fc_sketch::Curve;
    let centers: Vec<(f64, f64)> = sk
        .geometry
        .iter()
        .filter(|g| !g.construction)
        .filter_map(|g| match &g.curve {
            Curve::Circle { c, .. } => Some((c.x, c.y)),
            Curve::Arc { c, .. } => Some((c.x, c.y)),
            Curve::Point { p } => Some((p.x, p.y)),
            _ => None,
        })
        .collect();
    if centers.is_empty() {
        return Err(PartError::Profile("hole sketch has no circles or points".into()));
    }
    let n = pl.normal();
    let down = scale(n, -1.0);
    let depth = if h.through_all { through_all_length(Some(base), base) } else { h.depth };
    let mut tool = Shape::default();
    for (u, v) in centers {
        let top = pl.to_world(u, v);
        // Hole body: profile in the plane containing the axis, revolved 360°.
        // Local 2D profile (radius r along xdir, depth along -n).
        let r = h.diameter / 2.0;
        let mut pts: Vec<(f64, f64)> = vec![(0.0, 0.0)];
        match h.cut {
            HoleCut::None => pts.push((r, 0.0)),
            HoleCut::Counterbore => {
                pts.push((h.cut_diameter / 2.0, 0.0));
                pts.push((h.cut_diameter / 2.0, h.cut_depth));
                pts.push((r, h.cut_depth));
            }
            HoleCut::Countersink => {
                let rc = h.cut_diameter / 2.0;
                let half = (h.countersink_angle / 2.0).to_radians();
                pts.push((rc, 0.0));
                pts.push((r, (rc - r) / half.tan()));
            }
        }
        pts.push((r, depth));
        if h.drill_point_angle > 0.0 && !h.through_all {
            let half = (h.drill_point_angle / 2.0).to_radians();
            pts.push((0.0, depth + r / half.tan()));
        } else {
            pts.push((0.0, depth));
        }
        // Start the profile slightly above the face so the cut is clean; a countersink's rim
        // moves up along its cone so the cone keeps its angle.
        let lift = 1e-3 * h.diameter.max(1.0);
        let cone_slope = if h.cut == HoleCut::Countersink { (h.countersink_angle / 2.0).to_radians().tan() } else { 0.0 };
        let w = |(x, z): (f64, f64)| {
            let (x, z) = if z == 0.0 { (if x > 0.0 { x + lift * cone_slope } else { x }, -lift) } else { (x, z) };
            add(top, add(scale(pl.xdir, x), scale(down, z)))
        };
        let mut b = fc_occt::ProfileBuilder::new();
        b.begin_face();
        for k in 0..pts.len() {
            let a = w(pts[k]);
            let c = w(pts[(k + 1) % pts.len()]);
            if norm(sub(a, c)) > 1e-9 {
                b.line(a, c)?;
            }
        }
        b.end_wire()?;
        let prof = b.build_faces()?;
        let solid = prof.revolve(top, n, 2.0 * PI)?;
        tool = if tool.is_null() || tool.face_count() == 0 { solid } else { tool.fuse(&solid)? };
    }
    Ok(tool)
}

#[cfg(test)]
mod tests;
