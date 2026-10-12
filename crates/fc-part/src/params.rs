//! Wiring expressions into the document: named parameters, named constraints and
//! feature properties as readable/bindable values.

use crate::expr::{self, Expr, ExprError};
use crate::{Document, FeatureKind};
use fc_sketch::ConstraintKind as K;
use std::collections::BTreeMap;

/// Mutable access to a constraint's dimension value. Returns (value, is_angle_in_radians).
pub fn constraint_value_mut(k: &mut K) -> Option<(&mut f64, bool)> {
    match k {
        K::Distance(_, _, v)
        | K::DistanceX(_, _, v)
        | K::DistanceY(_, _, v)
        | K::PointLineDistance(_, _, v) => Some((v, false)),
        K::DistanceLine(_, v) | K::Radius(_, v) | K::Diameter(_, v) | K::LockX(_, v) | K::LockY(_, v) => Some((v, false)),
        K::Angle(_, v) | K::AngleBetween(_, _, v) => Some((v, true)),
        _ => None,
    }
}

pub fn constraint_value(k: &K) -> Option<(f64, bool)> {
    let mut c = k.clone();
    constraint_value_mut(&mut c).map(|(v, a)| (*v, a))
}

/// Names of a feature's numeric properties (FreeCAD property names).
pub fn feature_props(k: &FeatureKind) -> &'static [&'static str] {
    match k {
        FeatureKind::Sketch(_) => &["AttachmentOffset"],
        FeatureKind::Pad(_) | FeatureKind::Pocket(_) => &["Length", "Length2"],
        FeatureKind::Revolution(_) | FeatureKind::Groove(_) => &["Angle", "Angle2"],
        FeatureKind::Hole(_) => &["Diameter", "Depth", "DrillPointAngle", "CutDiameter", "CutDepth", "CountersinkAngle"],
        FeatureKind::MateConnector(_) => &["OffsetX", "OffsetY", "OffsetZ", "RotationX", "RotationY", "RotationZ"],
        _ => &[],
    }
}

fn prop_mut<'a>(k: &'a mut FeatureKind, prop: &str) -> Option<&'a mut f64> {
    Some(match (k, prop) {
        (FeatureKind::Sketch(s), "AttachmentOffset") => &mut s.offset,
        (FeatureKind::Pad(e) | FeatureKind::Pocket(e), "Length") => &mut e.length,
        (FeatureKind::Pad(e) | FeatureKind::Pocket(e), "Length2") => &mut e.length2,
        (FeatureKind::Revolution(r) | FeatureKind::Groove(r), "Angle") => &mut r.angle,
        (FeatureKind::Revolution(r) | FeatureKind::Groove(r), "Angle2") => &mut r.angle2,
        (FeatureKind::Hole(h), "Diameter") => &mut h.diameter,
        (FeatureKind::Hole(h), "Depth") => &mut h.depth,
        (FeatureKind::Hole(h), "DrillPointAngle") => &mut h.drill_point_angle,
        (FeatureKind::Hole(h), "CutDiameter") => &mut h.cut_diameter,
        (FeatureKind::Hole(h), "CutDepth") => &mut h.cut_depth,
        (FeatureKind::Hole(h), "CountersinkAngle") => &mut h.countersink_angle,
        (FeatureKind::MateConnector(m), "OffsetX") => &mut m.offset[0],
        (FeatureKind::MateConnector(m), "OffsetY") => &mut m.offset[1],
        (FeatureKind::MateConnector(m), "OffsetZ") => &mut m.offset[2],
        (FeatureKind::MateConnector(m), "RotationX") => &mut m.rotation[0],
        (FeatureKind::MateConnector(m), "RotationY") => &mut m.rotation[1],
        (FeatureKind::MateConnector(m), "RotationZ") => &mut m.rotation[2],
        _ => return None,
    })
}

const PARAM: &str = "param:";

impl Document {
    /// Canonical key for a name used in an expression or as a binding target:
    /// `param:<name>`, `<Feature>.<Prop>` or `<Sketch>.Constraints[<i>]`.
    pub fn canonical_name(&self, name: &str) -> Option<String> {
        if let Some(p) = name.strip_prefix(PARAM) {
            return self.parameters.contains_key(p).then(|| name.to_string());
        }
        if self.parameters.contains_key(name) {
            return Some(format!("{PARAM}{name}"));
        }
        let (head, tail) = match name.find(['.', '[']) {
            Some(i) => (&name[..i], name[i..].trim_start_matches('.')),
            None => {
                // Bare name: a uniquely named constraint anywhere in the body.
                let mut hits = Vec::new();
                for f in &self.features {
                    if let FeatureKind::Sketch(s) = &f.kind {
                        for (i, c) in s.sketch.constraints.iter().enumerate() {
                            if c.name.as_deref() == Some(name) {
                                hits.push(format!("{}.Constraints[{i}]", f.name));
                            }
                        }
                    }
                }
                return if hits.len() == 1 { hits.pop() } else { None };
            }
        };
        let f = self.features.iter().find(|f| f.name == head || f.label == head)?;
        if let FeatureKind::Sketch(s) = &f.kind {
            let t = tail.strip_prefix("Constraints").map(|t| t.trim_start_matches('.')).unwrap_or(tail);
            if let Some(ix) = t.strip_prefix('[').and_then(|t| t.strip_suffix(']')) {
                let i: usize = ix.trim().parse().ok()?;
                return (i < s.sketch.constraints.len() && constraint_value(&s.sketch.constraints[i].kind).is_some())
                    .then(|| format!("{}.Constraints[{i}]", f.name));
            }
            if let Some(i) = s.sketch.constraints.iter().position(|c| c.name.as_deref() == Some(t)) {
                return constraint_value(&s.sketch.constraints[i].kind).is_some().then(|| format!("{}.Constraints[{i}]", f.name));
            }
        }
        feature_props(&f.kind).contains(&tail).then(|| format!("{}.{tail}", f.name))
    }

    /// Current value of a canonical key (angles in degrees).
    pub fn read_value(&self, key: &str) -> Option<f64> {
        if let Some(p) = key.strip_prefix(PARAM) {
            let (vals, _) = self.evaluate_expressions();
            return vals.get(&format!("{PARAM}{p}")).copied();
        }
        let (head, tail) = key.split_once('.')?;
        let f = self.features.iter().find(|f| f.name == head)?;
        if let Some(ix) = tail.strip_prefix("Constraints[").and_then(|t| t.strip_suffix(']')) {
            let FeatureKind::Sketch(s) = &f.kind else { return None };
            let (v, ang) = constraint_value(&s.sketch.constraints.get(ix.parse::<usize>().ok()?)?.kind)?;
            return Some(if ang { v.to_degrees() } else { v });
        }
        let mut k = f.kind.clone();
        prop_mut(&mut k, tail).map(|v| *v)
    }

    /// Set a constraint/property value by canonical key (angles in degrees). Does not touch bindings.
    pub fn write_value(&mut self, key: &str, v: f64) -> bool {
        let Some((head, tail)) = key.split_once('.') else { return false };
        let Some(f) = self.features.iter_mut().find(|f| f.name == head) else { return false };
        if let Some(ix) = tail.strip_prefix("Constraints[").and_then(|t| t.strip_suffix(']')) {
            let FeatureKind::Sketch(s) = &mut f.kind else { return false };
            let Some(c) = ix.parse::<usize>().ok().and_then(|i| s.sketch.constraints.get_mut(i)) else { return false };
            if let Some((slot, ang)) = constraint_value_mut(&mut c.kind) {
                *slot = if ang { v.to_radians() } else { v };
                return true;
            }
            return false;
        }
        match prop_mut(&mut f.kind, tail) {
            Some(slot) => {
                *slot = v;
                true
            }
            None => false,
        }
    }

    /// Define or change a named parameter (`value` is an expression, e.g. "20 mm" or "width * 2").
    pub fn set_parameter(&mut self, name: &str, value: &str) -> Result<(), ExprError> {
        if name.is_empty() || !name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') || name.chars().next().unwrap().is_ascii_digit() {
            return Err(ExprError(format!("'{name}' is not a valid name")));
        }
        expr::parse(value)?;
        self.parameters.insert(name.to_string(), value.to_string());
        self.check_expressions()
    }

    /// Bind a property or constraint to an expression (FreeCAD's setExpression). An empty
    /// expression removes the binding.
    pub fn set_expression(&mut self, target: &str, src: &str) -> Result<(), ExprError> {
        let key = self.canonical_name(target).ok_or_else(|| ExprError(format!("unknown target '{target}'")))?;
        if key.starts_with(PARAM) {
            return self.set_parameter(&key[PARAM.len()..], src);
        }
        if src.trim().is_empty() {
            self.expressions.remove(&key);
            return Ok(());
        }
        expr::parse(src)?;
        let old = self.expressions.insert(key.clone(), src.to_string());
        if let Err(e) = self.check_expressions() {
            match old {
                Some(o) => self.expressions.insert(key, o),
                None => self.expressions.remove(&key),
            };
            return Err(e);
        }
        Ok(())
    }

    /// Give a sketch constraint a name so expressions can refer to it.
    pub fn name_constraint(&mut self, sketch: crate::FeatureId, index: usize, name: &str) -> Result<(), ExprError> {
        if !name.is_empty() && (!name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') || name.chars().next().unwrap().is_ascii_digit()) {
            return Err(ExprError(format!("'{name}' is not a valid name")));
        }
        let sk = self.sketch_mut(sketch).ok_or_else(|| ExprError("not a sketch".into()))?;
        if !name.is_empty() && sk.constraints.iter().enumerate().any(|(i, c)| i != index && c.name.as_deref() == Some(name)) {
            return Err(ExprError(format!("this sketch already has a constraint named '{name}'")));
        }
        let c = sk.constraints.get_mut(index).ok_or_else(|| ExprError("no such constraint".into()))?;
        c.name = (!name.is_empty()).then(|| name.to_string());
        Ok(())
    }

    fn parsed(&self) -> BTreeMap<String, Expr> {
        let mut m = BTreeMap::new();
        for (k, v) in &self.parameters {
            if let Ok(e) = expr::parse(v) {
                m.insert(format!("{PARAM}{k}"), e);
            }
        }
        for (k, v) in &self.expressions {
            if let Ok(e) = expr::parse(v) {
                m.insert(k.clone(), e);
            }
        }
        m
    }

    /// Evaluate all parameters and bindings without changing the document.
    pub fn evaluate_expressions(&self) -> (BTreeMap<String, f64>, BTreeMap<String, ExprError>) {
        let m = self.parsed();
        let canon = |n: &str| self.canonical_name(n);
        let mut ext = |k: &str| -> Option<f64> {
            if k.starts_with(PARAM) {
                return None;
            }
            let (head, tail) = k.split_once('.')?;
            let f = self.features.iter().find(|f| f.name == head)?;
            if let Some(ix) = tail.strip_prefix("Constraints[").and_then(|t| t.strip_suffix(']')) {
                let FeatureKind::Sketch(s) = &f.kind else { return None };
                let (v, ang) = constraint_value(&s.sketch.constraints.get(ix.parse::<usize>().ok()?)?.kind)?;
                return Some(if ang { v.to_degrees() } else { v });
            }
            let mut kk = f.kind.clone();
            prop_mut(&mut kk, tail).map(|v| *v)
        };
        expr::eval_all(&m, &mut ext, &canon)
    }

    fn check_expressions(&self) -> Result<(), ExprError> {
        let (_, errs) = self.evaluate_expressions();
        match errs.into_iter().next() {
            Some((k, e)) => Err(ExprError(format!("{k}: {e}"))),
            None => Ok(()),
        }
    }

    /// Push expression results into constraints/properties. Called by `recompute`.
    pub fn apply_expressions(&mut self) -> BTreeMap<String, ExprError> {
        let (vals, errs) = self.evaluate_expressions();
        for (k, v) in vals {
            if !k.starts_with(PARAM) && self.expressions.contains_key(&k) {
                self.write_value(&k, v);
            }
        }
        errs
    }

    /// Every name an expression can use right now, with its current value (for autocomplete).
    pub fn available_names(&self) -> Vec<(String, f64)> {
        let mut out = Vec::new();
        let (vals, _) = self.evaluate_expressions();
        for k in self.parameters.keys() {
            if let Some(v) = vals.get(&format!("{PARAM}{k}")) {
                out.push((k.clone(), *v));
            }
        }
        for f in &self.features {
            if let FeatureKind::Sketch(s) = &f.kind {
                for (i, c) in s.sketch.constraints.iter().enumerate() {
                    if let (Some(n), Some(v)) = (&c.name, self.read_value(&format!("{}.Constraints[{i}]", f.name))) {
                        out.push((format!("{}.{n}", f.name), v));
                    }
                }
            }
            for p in feature_props(&f.kind) {
                if let Some(v) = self.read_value(&format!("{}.{p}", f.name)) {
                    out.push((format!("{}.{p}", f.name), v));
                }
            }
        }
        out
    }
}

/// What to do with expressions that use a name being deleted.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OnUsers {
    /// Refuse and report the users.
    Refuse,
    /// Replace each user's formula by its current value (users become plain numbers).
    Freeze,
}

impl Document {
    /// Canonical keys of parameters/bindings whose expressions reference `key`.
    pub fn users_of(&self, key: &str) -> Vec<String> {
        let mut out = Vec::new();
        let uses = |src: &str| -> bool {
            let Ok(e) = expr::parse(src) else { return false };
            let mut refs = Vec::new();
            e.refs(&mut refs);
            refs.iter().any(|r| self.canonical_name(r).as_deref() == Some(key))
        };
        for (k, v) in &self.parameters {
            if uses(v) {
                out.push(format!("{PARAM}{k}"));
            }
        }
        for (k, v) in &self.expressions {
            if uses(v) {
                out.push(k.clone());
            }
        }
        out
    }

    /// Human-readable name for a canonical key (`width`, `Sketch.width`, `Pad.Length`).
    pub fn display_name(&self, key: &str) -> String {
        if let Some(p) = key.strip_prefix(PARAM) {
            return p.to_string();
        }
        if let Some((head, tail)) = key.split_once('.') {
            if let Some(ix) = tail.strip_prefix("Constraints[").and_then(|t| t.strip_suffix(']')) {
                if let Some(FeatureKind::Sketch(s)) = self.features.iter().find(|f| f.name == head).map(|f| &f.kind) {
                    if let Some(n) = ix.parse::<usize>().ok().and_then(|i| s.sketch.constraints.get(i)).and_then(|c| c.name.clone()) {
                        return format!("{head}.{n}");
                    }
                }
            }
        }
        key.to_string()
    }

    /// Make each user of `key` a plain value: parameters get their current number,
    /// bound properties/constraints lose their formula and keep their current value.
    pub fn freeze_users(&mut self, key: &str) {
        let users = self.users_of(key);
        if users.is_empty() {
            return;
        }
        let (vals, _) = self.evaluate_expressions();
        for u in users {
            if let Some(p) = u.strip_prefix(PARAM) {
                if let Some(v) = vals.get(&u) {
                    self.parameters.insert(p.to_string(), format!("{v}"));
                }
            } else {
                if let Some(v) = vals.get(&u) {
                    self.write_value(&u, *v);
                }
                self.expressions.remove(&u);
            }
        }
    }

    fn check_users(&mut self, key: &str, policy: OnUsers) -> Result<(), Vec<String>> {
        let users = self.users_of(key);
        if users.is_empty() {
            return Ok(());
        }
        match policy {
            OnUsers::Refuse => Err(users.iter().map(|u| self.display_name(u)).collect()),
            OnUsers::Freeze => {
                self.freeze_users(key);
                Ok(())
            }
        }
    }

    /// Delete a named parameter. With `OnUsers::Refuse`, fails with the names of the formulas
    /// that use it.
    pub fn remove_parameter(&mut self, name: &str, policy: OnUsers) -> Result<(), Vec<String>> {
        let key = format!("{PARAM}{name}");
        self.check_users(&key, policy)?;
        self.parameters.remove(name);
        Ok(())
    }

    /// Remove a constraint's name (the dimension itself stays).
    pub fn unname_constraint(&mut self, sketch: crate::FeatureId, index: usize, policy: OnUsers) -> Result<(), Vec<String>> {
        let Some(f) = self.get(sketch) else { return Ok(()) };
        let key = format!("{}.Constraints[{index}]", f.name);
        // Only references by name break; index references keep working, but freeze both for clarity.
        self.check_users(&key, policy)?;
        if let Some(c) = self.sketch_mut(sketch).and_then(|s| s.constraints.get_mut(index)) {
            c.name = None;
        }
        Ok(())
    }

    /// Delete sketch constraints, keeping formulas pointed at the right dimensions
    /// (`Sketch.Constraints[i]` keys are renumbered) and handling users per `policy`.
    pub fn delete_constraints(&mut self, sketch: crate::FeatureId, indices: &[usize], policy: OnUsers) -> Result<(), Vec<String>> {
        let Some(fname) = self.get(sketch).map(|f| f.name.clone()) else { return Ok(()) };
        let mut idx: Vec<usize> = indices.to_vec();
        idx.sort_unstable();
        idx.dedup();
        // Users outside the deleted set must be handled first.
        let mut blocked = Vec::new();
        for &i in &idx {
            let key = format!("{fname}.Constraints[{i}]");
            let users: Vec<String> = self.users_of(&key).into_iter().filter(|u| !idx.iter().any(|j| *u == format!("{fname}.Constraints[{j}]"))).collect();
            if !users.is_empty() {
                match policy {
                    OnUsers::Refuse => blocked.extend(users.iter().map(|u| self.display_name(u))),
                    OnUsers::Freeze => self.freeze_users(&key),
                }
            }
        }
        if !blocked.is_empty() {
            blocked.sort();
            blocked.dedup();
            return Err(blocked);
        }
        // Rewrite index-based references in formulas to the new numbering before deleting.
        let remap = |i: usize| -> Option<usize> {
            if idx.binary_search(&i).is_ok() { None } else { Some(i - idx.iter().filter(|&&j| j < i).count()) }
        };
        let rewrite = |src: &str| -> String { rewrite_constraint_indices(src, &fname, &remap) };
        let mut exprs = std::collections::BTreeMap::new();
        for (k, v) in std::mem::take(&mut self.expressions) {
            let nk = match parse_constraint_key(&k, &fname) {
                Some(i) => match remap(i) {
                    Some(n) => format!("{fname}.Constraints[{n}]"),
                    None => continue, // binding of a deleted constraint
                },
                None => k,
            };
            exprs.insert(nk, rewrite(&v));
        }
        self.expressions = exprs;
        for v in self.parameters.values_mut() {
            *v = rewrite(v);
        }
        if let Some(s) = self.sketch_mut(sketch) {
            for &i in idx.iter().rev() {
                if i < s.constraints.len() {
                    s.remove_constraint(i);
                }
            }
        }
        Ok(())
    }

    /// Delete a sketch geometry together with its constraints (FreeCAD delGeometry).
    pub fn delete_geometry(&mut self, sketch: crate::FeatureId, geo: usize, policy: OnUsers) -> Result<(), Vec<String>> {
        let Some(s) = self.sketch_mut(sketch) else { return Ok(()) };
        let doomed: Vec<usize> = s.constraints.iter().enumerate().filter(|(_, c)| c.kind.geometries().contains(&geo)).map(|(i, _)| i).collect();
        self.delete_constraints(sketch, &doomed, policy)?;
        if let Some(s) = self.sketch_mut(sketch) {
            if geo < s.geometry.len() {
                s.remove_geometry(geo);
            }
        }
        Ok(())
    }
}

fn parse_constraint_key(k: &str, fname: &str) -> Option<usize> {
    k.strip_prefix(fname)?.strip_prefix(".Constraints[")?.strip_suffix(']')?.parse().ok()
}

/// Rewrite `<fname>.Constraints[i]` references in a formula to new indices.
fn rewrite_constraint_indices(src: &str, fname: &str, remap: &dyn Fn(usize) -> Option<usize>) -> String {
    let pat = format!("{fname}.Constraints[");
    let mut out = String::new();
    let mut rest = src;
    while let Some(p) = rest.find(&pat) {
        let boundary = p == 0 || !rest[..p].chars().last().map(|c| c.is_ascii_alphanumeric() || c == '_').unwrap_or(false);
        out.push_str(&rest[..p + pat.len()]);
        rest = &rest[p + pat.len()..];
        if let (true, Some(end)) = (boundary, rest.find(']')) {
            if let Ok(i) = rest[..end].trim().parse::<usize>() {
                // A deleted target can only appear here if its users were frozen; keep text.
                out.push_str(&remap(i).map(|n| n.to_string()).unwrap_or_else(|| rest[..end].to_string()));
                rest = &rest[end..];
            }
        }
    }
    out.push_str(rest);
    out
}

impl Document {
    /// Delete a feature (FreeCAD: removing an object). Fails if other features use it as an
    /// input; formulas that read its values are refused or frozen per `policy`, and formulas
    /// bound to it are dropped.
    pub fn delete_feature(&mut self, id: crate::FeatureId, policy: OnUsers) -> Result<(), Vec<String>> {
        let deps = self.dependents(id);
        if !deps.is_empty() {
            return Err(deps.iter().filter_map(|d| self.get(*d)).map(|f| format!("{} (uses it as input)", f.label)).collect());
        }
        let Some(f) = self.get(id) else { return Ok(()) };
        let prefix = format!("{}.", f.name);
        let mut keys: Vec<String> = feature_props(&f.kind).iter().map(|p| format!("{prefix}{p}")).collect();
        if let FeatureKind::Sketch(s) = &f.kind {
            keys.extend((0..s.sketch.constraints.len()).map(|i| format!("{prefix}Constraints[{i}]")));
        }
        let mut blocked = Vec::new();
        for k in &keys {
            let outside: Vec<String> = self.users_of(k).into_iter().filter(|u| !u.starts_with(&prefix)).collect();
            if !outside.is_empty() {
                match policy {
                    OnUsers::Refuse => blocked.extend(outside.iter().map(|u| self.display_name(u))),
                    OnUsers::Freeze => self.freeze_users(k),
                }
            }
        }
        if !blocked.is_empty() {
            blocked.sort();
            blocked.dedup();
            return Err(blocked);
        }
        self.expressions.retain(|k, _| !k.starts_with(&prefix));
        self.remove(id).map_err(|e| vec![e.to_string()])
    }
}
