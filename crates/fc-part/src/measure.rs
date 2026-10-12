//! Named measurements and externally driven parameters: what another program (Calc Canvas)
//! reads from and writes into a document.
//!
//! - A **measurement** is a named query on the recomputed body (`Document::measurements`):
//!   volume, surface area, bounding-box size, any readable value (a parameter, a named or
//!   reference sketch dimension, a feature property such as `Pad.Length`), or a picked face's
//!   area, diameter or the distance between two picked faces.
//! - A **driven parameter** (`Document::driven`) is a named parameter whose value another
//!   program sets; the app shows it locked, with where it comes from.
//!
//! Lengths are mm, areas mm², volumes mm³, angles degrees.

use crate::params::constraint_value;
use crate::placement::*;
use crate::{Document, FaceRef, FeatureKind};
use fc_occt::{FaceInfo, Shape};
use fc_sketch::ConstraintKind as K;
use serde::{Deserialize, Serialize};

/// What a measurement reads.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum MeasureSpec {
    /// Volume of the body.
    Volume,
    /// Surface area of the body.
    Area,
    /// Bounding-box size along X, Y or Z.
    BBoxX,
    BBoxY,
    BBoxZ,
    /// A value an expression can read: `width`, `Sketch.Constraints[3]`, `Sketch.height`
    /// (named or reference dimensions included), `Pad.Length`.
    Value { key: String },
    /// Area of one picked face.
    FaceArea { face: FaceRef },
    /// Diameter of a picked cylindrical or spherical face.
    FaceDiameter { face: FaceRef },
    /// Distance between two picked faces: perpendicular distance for parallel planes,
    /// otherwise the distance between their centres.
    FaceDistance { a: FaceRef, b: FaceRef },
}

impl MeasureSpec {
    pub fn label(&self) -> String {
        match self {
            MeasureSpec::Volume => "Volume".into(),
            MeasureSpec::Area => "Surface area".into(),
            MeasureSpec::BBoxX => "Bounding box X".into(),
            MeasureSpec::BBoxY => "Bounding box Y".into(),
            MeasureSpec::BBoxZ => "Bounding box Z".into(),
            MeasureSpec::Value { key } => format!("Value of {key}"),
            MeasureSpec::FaceArea { face } => format!("Area of Face{}", face.index),
            MeasureSpec::FaceDiameter { face } => format!("Diameter of Face{}", face.index),
            MeasureSpec::FaceDistance { a, b } => format!("Distance Face{} to Face{}", a.index, b.index),
        }
    }
}

/// The physical dimension of a value.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Dim {
    Length,
    Area,
    Volume,
    Angle,
    Number,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Measured {
    pub value: f64,
    pub dim: Dim,
}

#[derive(Clone, Debug, PartialEq)]
pub enum MeasureError {
    /// No measurement has this name.
    Unknown(String),
    /// The picked geometry can't be found in the recomputed body.
    Lost(String),
    /// The body or value isn't available (recompute error, unknown key, ...).
    Failed(String),
}

impl std::fmt::Display for MeasureError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MeasureError::Unknown(s) | MeasureError::Lost(s) | MeasureError::Failed(s) => f.write_str(s),
        }
    }
}

fn valid_name(name: &str) -> bool {
    !name.is_empty() && name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') && !name.chars().next().unwrap().is_ascii_digit()
}

/// Is `f` plausibly the same face as the picked `fr`? Same surface type, and for planes the same normal.
fn same_kind(f: &FaceInfo, fr: &FaceRef, kind: i32) -> bool {
    f.kind == kind && (kind != 0 || dot(normalize(f.normal), normalize(fr.normal)) > 1.0 - 1e-6)
}

/// Re-find a picked face on a recomputed body. Parameters move faces, so the signature
/// (centre, area) is only a tie-break: the face keeps its index while the topology doesn't
/// change; otherwise the nearest face of the same type (and normal, for planes) wins.
/// None (lost) when no face of that type and orientation is left.
pub fn locate_face(shape: &Shape, fr: &FaceRef, kind: i32) -> Option<FaceInfo> {
    let faces = shape.faces();
    if let Some(f) = faces.get(fr.index.wrapping_sub(1)) {
        if same_kind(f, fr, kind) {
            return Some(f.clone());
        }
    }
    faces.into_iter().filter(|f| same_kind(f, fr, kind)).min_by(|a, b| norm(sub(a.center, fr.center)).total_cmp(&norm(sub(b.center, fr.center))))
}

impl Document {
    /// Add or replace a named measurement. Names follow parameter rules and can't clash with a parameter.
    pub fn set_measurement(&mut self, name: &str, spec: MeasureSpec) -> Result<(), String> {
        if !valid_name(name) {
            return Err(format!("\"{name}\" is not a valid name (letters, digits and _, not starting with a digit)"));
        }
        if self.parameters.contains_key(name) {
            return Err(format!("{name} is already a parameter"));
        }
        self.measurements.insert(name.to_string(), spec);
        Ok(())
    }

    pub fn remove_measurement(&mut self, name: &str) -> bool {
        self.measurements.remove(name).is_some()
    }

    /// Rename a measurement (the new name must be free).
    pub fn rename_measurement(&mut self, from: &str, to: &str) -> Result<(), String> {
        let spec = self.measurements.get(from).cloned().ok_or_else(|| format!("no measurement {from}"))?;
        if from == to {
            return Ok(());
        }
        if self.measurements.contains_key(to) {
            return Err(format!("{to} is already a measurement"));
        }
        self.set_measurement(to, spec)?;
        self.measurements.remove(from);
        Ok(())
    }

    /// The value of a named measurement on the recomputed document.
    pub fn measure(&self, name: &str) -> Result<Measured, MeasureError> {
        let spec = self.measurements.get(name).ok_or_else(|| MeasureError::Unknown(format!("no measurement named {name}")))?;
        self.measure_spec(spec)
    }

    /// Every measurement, by name.
    pub fn measure_all(&self) -> Vec<(String, Result<Measured, MeasureError>)> {
        self.measurements.keys().map(|n| (n.clone(), self.measure(n))).collect()
    }

    /// The dimension a measurement has (known without recomputing).
    pub fn measurement_dim(&self, spec: &MeasureSpec) -> Dim {
        match spec {
            MeasureSpec::Volume => Dim::Volume,
            MeasureSpec::Area | MeasureSpec::FaceArea { .. } => Dim::Area,
            MeasureSpec::BBoxX | MeasureSpec::BBoxY | MeasureSpec::BBoxZ | MeasureSpec::FaceDiameter { .. } | MeasureSpec::FaceDistance { .. } => Dim::Length,
            MeasureSpec::Value { key } => self.canonical_name(key).map(|k| self.key_dim(&k)).unwrap_or(Dim::Number),
        }
    }

    pub fn measure_spec(&self, spec: &MeasureSpec) -> Result<Measured, MeasureError> {
        let dim = self.measurement_dim(spec);
        let ok = |value: f64| Ok(Measured { value, dim });
        if let MeasureSpec::Value { key } = spec {
            let k = self.canonical_name(key).ok_or_else(|| MeasureError::Lost(format!("{key} no longer exists in the model")))?;
            return self.read_value(&k).map(ok).unwrap_or_else(|| Err(MeasureError::Failed(format!("{key} has no value"))));
        }
        let shape = self.result().ok_or_else(|| MeasureError::Failed("the body has no solid".into()))?;
        if let Some((name, err)) = self.features.iter().zip(&self.states).find_map(|(f, s)| s.error.as_ref().map(|e| (&f.name, e))) {
            return Err(MeasureError::Failed(format!("{name} failed to recompute: {err}")));
        }
        let lost = |fr: &FaceRef| MeasureError::Lost(format!("Face{} is no longer in the model", fr.index));
        match spec {
            MeasureSpec::Volume => ok(shape.volume()),
            MeasureSpec::Area => ok(shape.area()),
            MeasureSpec::BBoxX | MeasureSpec::BBoxY | MeasureSpec::BBoxZ => {
                let b = shape.bbox_exact();
                let k = match spec {
                    MeasureSpec::BBoxX => 0,
                    MeasureSpec::BBoxY => 1,
                    _ => 2,
                };
                ok(b[k + 3] - b[k])
            }
            MeasureSpec::FaceArea { face } => {
                let kind = shape.faces().get(face.index.wrapping_sub(1)).map(|f| f.kind).unwrap_or(0);
                let f = locate_face(shape, face, kind).or_else(|| locate_face(shape, face, 0)).ok_or_else(|| lost(face))?;
                ok(f.area)
            }
            MeasureSpec::FaceDiameter { face } => {
                let f = [1, 3].iter().find_map(|&k| locate_face(shape, face, k)).ok_or_else(|| lost(face))?;
                ok(2.0 * f.radius)
            }
            MeasureSpec::FaceDistance { a, b } => {
                let fa = locate_face(shape, a, 0).ok_or_else(|| lost(a))?;
                let fb = locate_face(shape, b, 0).ok_or_else(|| lost(b))?;
                let (na, nb) = (normalize(fa.normal), normalize(fb.normal));
                if dot(na, nb).abs() > 1.0 - 1e-6 {
                    ok(dot(sub(fb.origin, fa.origin), na).abs())
                } else {
                    ok(norm(sub(fb.center, fa.center)))
                }
            }
            MeasureSpec::Value { .. } => unreachable!(),
        }
    }

    /// The dimension of a canonical key (`param:x`, `Pad.Length`, `Sketch.Constraints[2]`).
    pub fn key_dim(&self, key: &str) -> Dim {
        if let Some(p) = key.strip_prefix("param:") {
            return self.parameter_dim(p);
        }
        let Some((head, tail)) = key.split_once('.') else { return Dim::Number };
        let Some(f) = self.features.iter().find(|f| f.name == head) else { return Dim::Number };
        if let Some(ix) = tail.strip_prefix("Constraints[").and_then(|t| t.strip_suffix(']')) {
            let FeatureKind::Sketch(s) = &f.kind else { return Dim::Number };
            return match ix.parse::<usize>().ok().and_then(|i| s.sketch.constraints.get(i)).map(|c| &c.kind) {
                Some(K::Angle(..) | K::AngleBetween(..)) => Dim::Angle,
                Some(k) if constraint_value(k).is_some() => Dim::Length,
                _ => Dim::Number,
            };
        }
        match tail {
            "Angle" | "Angle2" | "DrillPointAngle" | "CountersinkAngle" | "RotationX" | "RotationY" | "RotationZ" => Dim::Angle,
            _ => Dim::Length,
        }
    }

    /// A named parameter's dimension: from a unit in its own expression (`20 mm`, `30 deg`),
    /// else from what it drives (a length property or dimension makes it a length), else a number.
    pub fn parameter_dim(&self, name: &str) -> Dim {
        let Some(src) = self.parameters.get(name) else { return Dim::Number };
        if let Some(d) = unit_dim(src) {
            return d;
        }
        let me = format!("param:{name}");
        let mut seen = vec![me.clone()];
        let mut frontier = vec![me];
        while let Some(k) = frontier.pop() {
            for (target, e) in self.expressions.iter().map(|(t, e)| (t.clone(), e)).chain(self.parameters.iter().map(|(p, e)| (format!("param:{p}"), e))) {
                if seen.contains(&target) {
                    continue;
                }
                let Ok(parsed) = crate::expr::parse(e) else { continue };
                let mut refs = Vec::new();
                parsed.refs(&mut refs);
                if !refs.iter().any(|r| self.canonical_name(r).as_deref() == Some(k.as_str())) {
                    continue;
                }
                if !target.starts_with("param:") {
                    // A bare reference passes the dimension through; anything else (a product, a ratio) doesn't say.
                    if is_bare_ref(e) || is_linear_in_refs(e) {
                        return self.key_dim(&target);
                    }
                    continue;
                }
                seen.push(target.clone());
                if is_bare_ref(e) || is_linear_in_refs(e) {
                    frontier.push(target);
                }
            }
        }
        Dim::Number
    }

    /// Set named parameters from another program, marking them driven by `source`.
    /// Each value is an expression (`"42 mm"`). Parameters that don't exist are created.
    pub fn set_driven_parameters(&mut self, values: &[(String, String)], source: &str) -> Result<(), crate::expr::ExprError> {
        for (name, value) in values {
            self.set_parameter(name, value)?;
            self.driven.insert(name.clone(), source.to_string());
        }
        Ok(())
    }

    /// Stop another program driving a parameter (its last value stays).
    pub fn release_driven(&mut self, name: &str) -> bool {
        self.driven.remove(name).is_some()
    }
}

fn is_bare_ref(e: &str) -> bool {
    matches!(crate::expr::parse(e), Ok(crate::expr::Expr::Ref(_)))
}

/// `a + 3 mm`, `width / 2 + 3`, `-x`: sums and scalings of references keep their dimension.
fn is_linear_in_refs(e: &str) -> bool {
    use crate::expr::Expr;
    fn lin(e: &Expr) -> bool {
        match e {
            Expr::Ref(_) => true,
            Expr::Neg(a) => lin(a),
            Expr::Bin('+' | '-', a, b) => lin(a) || lin(b),
            Expr::Bin('*', a, b) => (lin(a) && matches!(**b, Expr::Num(_))) || (lin(b) && matches!(**a, Expr::Num(_))),
            Expr::Bin('/', a, b) => lin(a) && matches!(**b, Expr::Num(_)),
            _ => false,
        }
    }
    crate::expr::parse(e).map(|x| lin(&x)).unwrap_or(false)
}

/// The dimension a unit written in an expression gives it.
fn unit_dim(src: &str) -> Option<Dim> {
    let mut word = String::new();
    let mut found = None;
    let flush = |w: &mut String, found: &mut Option<Dim>| {
        let d = match w.as_str() {
            "mm" | "cm" | "m" | "in" | "ft" => Some(Dim::Length),
            "deg" | "rad" => Some(Dim::Angle),
            _ => None,
        };
        if found.is_none() {
            *found = d;
        }
        w.clear();
    };
    for c in src.chars() {
        if c == '°' {
            flush(&mut word, &mut found);
            found.get_or_insert(Dim::Angle);
        } else if c.is_ascii_alphabetic() || c == '_' || (c.is_ascii_digit() && !word.is_empty()) {
            word.push(c);
        } else {
            flush(&mut word, &mut found);
        }
    }
    flush(&mut word, &mut found);
    found
}
