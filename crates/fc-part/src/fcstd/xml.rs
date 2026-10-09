//! Reading FreeCAD's Document.xml: objects, their properties and typed property values.

use crate::placement::*;
use roxmltree::Node;
use std::collections::HashMap;

/// One `<Object>` of Document.xml with its properties by name.
pub(crate) struct FcObj<'a> {
    pub name: String,
    pub ty: String,
    pub props: HashMap<String, Node<'a, 'a>>,
}

fn child<'a>(n: Node<'a, 'a>, tag: &str) -> Option<Node<'a, 'a>> {
    n.children().find(|c| c.is_element() && c.tag_name().name() == tag)
}

pub(crate) fn elements<'a>(n: Node<'a, 'a>) -> impl Iterator<Item = Node<'a, 'a>> {
    n.children().filter(|c| c.is_element())
}

pub(crate) fn attr_f64(n: Node, a: &str) -> Option<f64> {
    n.attribute(a)?.trim().parse().ok()
}

pub(crate) fn attr_i64(n: Node, a: &str) -> Option<i64> {
    n.attribute(a)?.trim().parse().ok()
}

/// A FreeCAD Placement in body coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct FcPlacement {
    pub origin: V3,
    /// Quaternion (x, y, z, w).
    pub q: [f64; 4],
}

impl FcPlacement {
    pub const IDENTITY: FcPlacement = FcPlacement { origin: [0.0; 3], q: [0.0, 0.0, 0.0, 1.0] };

    pub fn rotate(&self, v: V3) -> V3 {
        let [x, y, z, w] = self.q;
        // v' = v + 2w (u x v) + 2 u x (u x v), u = (x, y, z)
        let u = [x, y, z];
        let t = scale(cross(u, v), 2.0);
        add(add(v, scale(t, w)), cross(u, t))
    }
    pub fn xdir(&self) -> V3 {
        self.rotate([1.0, 0.0, 0.0])
    }
    pub fn ydir(&self) -> V3 {
        self.rotate([0.0, 1.0, 0.0])
    }
    pub fn normal(&self) -> V3 {
        self.rotate([0.0, 0.0, 1.0])
    }
    pub fn is_identity_rotation(&self) -> bool {
        let [x, y, z, _] = self.q;
        x.abs() < 1e-12 && y.abs() < 1e-12 && z.abs() < 1e-12
    }
    /// self * other (apply `other` first, like Base::Placement multiplication).
    pub fn mul(&self, o: &FcPlacement) -> FcPlacement {
        let [x1, y1, z1, w1] = self.q;
        let [x2, y2, z2, w2] = o.q;
        let q = [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ];
        FcPlacement { origin: add(self.origin, self.rotate(o.origin)), q }
    }
    /// Rotation of `angle` radians about `axis` (unit).
    pub fn rotation(axis: V3, angle: f64) -> FcPlacement {
        let s = (angle / 2.0).sin();
        FcPlacement { origin: [0.0; 3], q: [axis[0] * s, axis[1] * s, axis[2] * s, (angle / 2.0).cos()] }
    }
}

impl<'a> FcObj<'a> {
    pub fn prop(&self, name: &str) -> Option<Node<'a, 'a>> {
        self.props.get(name).copied()
    }
    pub fn has(&self, name: &str) -> bool {
        self.props.contains_key(name)
    }
    fn value_node(&self, name: &str, tag: &str) -> Option<Node<'a, 'a>> {
        child(self.prop(name)?, tag)
    }
    /// PropertyFloat / Length / Angle / Distance / Precision ...
    pub fn float(&self, name: &str) -> Option<f64> {
        let p = self.prop(name)?;
        let n = elements(p).next()?;
        attr_f64(n, "value")
    }
    pub fn bool(&self, name: &str) -> Option<bool> {
        let v = self.value_node(name, "Bool")?.attribute("value")?;
        Some(v == "true" || v == "1")
    }
    pub fn int(&self, name: &str) -> Option<i64> {
        attr_i64(self.value_node(name, "Integer")?, "value")
    }
    pub fn string(&self, name: &str) -> Option<String> {
        Some(self.value_node(name, "String")?.attribute("value")?.to_string())
    }
    /// Enumeration as text: the stored custom enum list if present, else `known[i]`.
    pub fn enumeration(&self, name: &str, known: &[&str]) -> Option<(i64, Option<String>)> {
        let p = self.prop(name)?;
        let i = attr_i64(child(p, "Integer")?, "value")?;
        if let Some(list) = child(p, "CustomEnumList") {
            let s = elements(list).nth(i as usize).and_then(|e| e.attribute("value")).map(str::to_string);
            return Some((i, s));
        }
        Some((i, known.get(i as usize).map(|s| s.trim_start_matches('?').to_string())))
    }
    pub fn link(&self, name: &str) -> Option<String> {
        let v = self.value_node(name, "Link")?.attribute("value")?;
        (!v.is_empty()).then(|| v.to_string())
    }
    pub fn link_list(&self, name: &str) -> Vec<String> {
        let Some(l) = self.value_node(name, "LinkList") else { return vec![] };
        elements(l).filter_map(|e| e.attribute("value")).filter(|v| !v.is_empty()).map(str::to_string).collect()
    }
    /// PropertyLinkSub: (object, sub-element names).
    pub fn link_sub(&self, name: &str) -> Option<(String, Vec<String>)> {
        let l = self.value_node(name, "LinkSub")?;
        let obj = l.attribute("value")?;
        if obj.is_empty() {
            return None;
        }
        let subs = elements(l).filter_map(|e| e.attribute("value")).filter(|v| !v.is_empty()).map(str::to_string).collect();
        Some((obj.to_string(), subs))
    }
    /// PropertyLinkSubList: (object, sub) pairs in order.
    pub fn link_sub_list(&self, name: &str) -> Vec<(String, String)> {
        let Some(l) = self.value_node(name, "LinkSubList") else { return vec![] };
        elements(l)
            .filter_map(|e| Some((e.attribute("obj")?.to_string(), e.attribute("sub").unwrap_or("").to_string())))
            .filter(|(o, _)| !o.is_empty())
            .collect()
    }
    pub fn placement(&self, name: &str) -> Option<FcPlacement> {
        let n = self.value_node(name, "PropertyPlacement")?;
        let g = |a: &str| attr_f64(n, a).unwrap_or(0.0);
        let mut q = [g("Q0"), g("Q1"), g("Q2"), g("Q3")];
        let l = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
        if l > 0.0 {
            for c in &mut q {
                *c /= l;
            }
        } else {
            q = [0.0, 0.0, 0.0, 1.0];
        }
        Some(FcPlacement { origin: [g("Px"), g("Py"), g("Pz")], q })
    }
    /// `(path, expression)` entries of the ExpressionEngine property.
    pub fn expressions(&self) -> Vec<(String, String)> {
        let Some(e) = self.value_node("ExpressionEngine", "ExpressionEngine") else { return vec![] };
        elements(e)
            .filter(|n| n.tag_name().name() == "Expression")
            .filter_map(|n| Some((n.attribute("path")?.to_string(), n.attribute("expression")?.to_string())))
            .collect()
    }
}

/// All objects of the document, in `<Objects>` order.
pub(crate) fn read_objects<'a>(root: Node<'a, 'a>) -> (Vec<String>, HashMap<String, FcObj<'a>>) {
    let mut order = Vec::new();
    let mut objs: HashMap<String, FcObj<'a>> = HashMap::new();
    if let Some(list) = child(root, "Objects") {
        for o in elements(list).filter(|n| n.tag_name().name() == "Object") {
            let (Some(name), Some(ty)) = (o.attribute("name"), o.attribute("type")) else { continue };
            order.push(name.to_string());
            objs.insert(name.to_string(), FcObj { name: name.to_string(), ty: ty.to_string(), props: HashMap::new() });
        }
    }
    if let Some(data) = child(root, "ObjectData") {
        for o in elements(data).filter(|n| n.tag_name().name() == "Object") {
            let Some(name) = o.attribute("name") else { continue };
            let Some(obj) = objs.get_mut(name) else { continue };
            if let Some(props) = child(o, "Properties") {
                for p in elements(props).filter(|n| n.tag_name().name() == "Property") {
                    if let Some(pn) = p.attribute("name") {
                        obj.props.insert(pn.to_string(), p);
                    }
                }
            }
        }
    }
    (order, objs)
}
