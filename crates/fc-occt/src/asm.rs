//! Assembly-level OCCT services (cxx bridge to `cpp/asm.cpp`): XDE documents for
//! structured STEP export/import, located shapes, minimum distance and BREP strings.

use crate::{Result, Shape, P3};
use cxx::UniquePtr;

#[cxx::bridge(namespace = "fcocc")]
pub mod ffi {
    /// A label of an XDE document: a part (simple shape) or an assembly.
    #[derive(Debug, Clone)]
    pub struct AsmNode {
        pub is_assembly: bool,
        pub name: String,
        /// sRGB colour and alpha; alpha < 0 means no colour.
        pub rgba: [f64; 4],
    }

    /// A component (instance) of an assembly node: `child` placed in `parent` by `matrix`
    /// (row-major 3x4 `[R | t]`, relative to the parent).
    #[derive(Debug, Clone)]
    pub struct AsmComp {
        pub parent: usize,
        pub child: usize,
        pub name: String,
        pub matrix: [f64; 12],
    }

    unsafe extern "C++" {
        include!("asm.h");

        type Shape = crate::ffi::ffi::Shape;
        type AsmDoc;

        fn asm_new() -> Result<UniquePtr<AsmDoc>>;
        fn asm_add_part(d: Pin<&mut AsmDoc>, s: &Shape, name: &str, rgba: [f64; 4]) -> Result<usize>;
        fn asm_add_assembly(d: Pin<&mut AsmDoc>, name: &str) -> Result<usize>;
        fn asm_add_component(d: Pin<&mut AsmDoc>, parent: usize, child: usize, m: [f64; 12], name: &str) -> Result<usize>;
        fn asm_write_step(d: Pin<&mut AsmDoc>, path: &str, schema: i32) -> Result<()>;

        fn asm_read_step(path: &str) -> Result<UniquePtr<AsmDoc>>;
        fn asm_nodes(d: &AsmDoc) -> Result<Vec<AsmNode>>;
        fn asm_components(d: &AsmDoc) -> Result<Vec<AsmComp>>;
        fn asm_roots(d: &AsmDoc) -> Result<Vec<usize>>;
        fn asm_node_shape(d: &AsmDoc, node: usize) -> Result<UniquePtr<Shape>>;

        fn shape_moved(s: &Shape, m: [f64; 12]) -> Result<UniquePtr<Shape>>;
        fn min_distance(a: &Shape, b: &Shape) -> Result<[f64; 7]>;
        fn shape_to_brep(s: &Shape) -> Result<String>;
        fn shape_from_brep(text: &str) -> Result<UniquePtr<Shape>>;
    }
}

pub use ffi::{AsmComp, AsmNode};

/// Row-major 3x4 rigid transform `[R | t]`.
pub type Matrix34 = [f64; 12];

/// STEP application protocol for [`XdeDoc::write_step`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum StepSchema {
    #[default]
    Ap214,
    Ap242,
}

/// An XDE (XCAF) document: parts (prototype shapes) and assemblies whose components place
/// them. Written and read through STEPCAFControl, so names, colours and the product
/// structure survive the round trip.
pub struct XdeDoc(UniquePtr<ffi::AsmDoc>);

/// Index of a label in an [`XdeDoc`].
pub type Label = usize;

impl XdeDoc {
    pub fn new() -> Result<Self> {
        Ok(XdeDoc(ffi::asm_new()?))
    }
    /// Adds a part (simple shape). `rgb` is an optional sRGB colour.
    pub fn add_part(&mut self, shape: &Shape, name: &str, rgb: Option<[f64; 3]>) -> Result<Label> {
        let c = rgb.map(|c| [c[0], c[1], c[2], 1.0]).unwrap_or([0.0, 0.0, 0.0, -1.0]);
        Ok(ffi::asm_add_part(self.0.pin_mut(), shape.r(), name, c)?)
    }
    /// Adds an (empty) assembly; it becomes one when its first component is added.
    pub fn add_assembly(&mut self, name: &str) -> Result<Label> {
        Ok(ffi::asm_add_assembly(self.0.pin_mut(), name)?)
    }
    /// Places `child` (a part or assembly) in `parent` at `m` (relative to `parent`).
    pub fn add_component(&mut self, parent: Label, child: Label, m: Matrix34, name: &str) -> Result<Label> {
        Ok(ffi::asm_add_component(self.0.pin_mut(), parent, child, m, name)?)
    }
    pub fn write_step(&mut self, path: &str, schema: StepSchema) -> Result<()> {
        let s = if schema == StepSchema::Ap242 { 242 } else { 214 };
        Ok(ffi::asm_write_step(self.0.pin_mut(), path, s)?)
    }
    /// Reads a STEP file. Labels are the document's top-level shapes (parts and assemblies).
    pub fn read_step(path: &str) -> Result<Self> {
        Ok(XdeDoc(ffi::asm_read_step(path)?))
    }
    pub fn nodes(&self) -> Result<Vec<AsmNode>> {
        Ok(ffi::asm_nodes(&self.0)?)
    }
    /// Components of every assembly label (indices refer to [`XdeDoc::nodes`]).
    pub fn components(&self) -> Result<Vec<AsmComp>> {
        Ok(ffi::asm_components(&self.0)?)
    }
    /// Free (top-level) labels.
    pub fn roots(&self) -> Result<Vec<Label>> {
        Ok(ffi::asm_roots(&self.0)?)
    }
    /// The shape of a label (for assemblies: the compound of its placed components).
    pub fn shape(&self, node: Label) -> Result<Shape> {
        Ok(Shape(ffi::asm_node_shape(&self.0, node)?))
    }
}

/// Minimum distance between two shapes with the closest points.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Distance {
    pub value: f64,
    pub on_a: P3,
    pub on_b: P3,
}

impl Shape {
    /// The shape moved by a rigid transform (shares geometry, like `TopoDS_Shape::Moved`).
    pub fn moved(&self, m: Matrix34) -> Result<Shape> {
        Ok(Shape(ffi::shape_moved(self.r(), m)?))
    }
    /// Minimum distance to `other` (BRepExtrema_DistShapeShape): 0 when they touch or overlap.
    pub fn distance_to(&self, other: &Shape) -> Result<Distance> {
        let d = ffi::min_distance(self.r(), other.r())?;
        Ok(Distance { value: d[0], on_a: [d[1], d[2], d[3]], on_b: [d[4], d[5], d[6]] })
    }
    /// OCCT's BREP text format.
    pub fn to_brep_string(&self) -> Result<String> {
        Ok(ffi::shape_to_brep(self.r())?)
    }
    pub fn from_brep_string(text: &str) -> Result<Shape> {
        Ok(Shape(ffi::shape_from_brep(text)?))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ProfileBuilder;

    fn cube(a: f64) -> Shape {
        let mut b = ProfileBuilder::new();
        b.begin_face();
        let p = [[0.0, 0.0, 0.0], [a, 0.0, 0.0], [a, a, 0.0], [0.0, a, 0.0]];
        for i in 0..4 {
            b.line(p[i], p[(i + 1) % 4]).unwrap();
        }
        b.end_wire().unwrap();
        b.build_faces().unwrap().prism([0.0, 0.0, a]).unwrap()
    }

    fn tr(t: P3) -> Matrix34 {
        [1.0, 0.0, 0.0, t[0], 0.0, 1.0, 0.0, t[1], 0.0, 0.0, 1.0, t[2]]
    }

    #[test]
    fn xde_step_roundtrip() {
        let mut d = XdeDoc::new().unwrap();
        let top = d.add_assembly("Top").unwrap();
        let sub = d.add_assembly("Sub").unwrap();
        let c = d.add_part(&cube(10.0), "Cube", Some([1.0, 0.0, 0.0])).unwrap();
        d.add_component(sub, c, tr([0.0, 0.0, 0.0]), "C1").unwrap();
        d.add_component(sub, c, tr([20.0, 0.0, 0.0]), "C2").unwrap();
        // Rotated 90° about Z and moved up.
        let rz = [0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 50.0];
        d.add_component(top, sub, rz, "SubA").unwrap();
        d.add_component(top, c, tr([0.0, 0.0, -30.0]), "Loose").unwrap();
        let path = std::env::temp_dir().join("fc_occt_xde.step");
        let path = path.to_str().unwrap();
        d.write_step(path, StepSchema::Ap242).unwrap();
        let text = std::fs::read_to_string(path).unwrap();
        assert!(text.contains("NEXT_ASSEMBLY_USAGE_OCCURRENCE"));

        let r = XdeDoc::read_step(path).unwrap();
        let nodes = r.nodes().unwrap();
        let comps = r.components().unwrap();
        let roots = r.roots().unwrap();
        assert_eq!(roots.len(), 1);
        let root = roots[0];
        assert!(nodes[root].is_assembly);
        assert_eq!(nodes[root].name, "Top");
        let parts: Vec<_> = nodes.iter().filter(|n| !n.is_assembly).collect();
        assert_eq!(parts.len(), 1, "{nodes:?}");
        assert_eq!(parts[0].name, "Cube");
        assert!((parts[0].rgba[0] - 1.0).abs() < 1e-3 && parts[0].rgba[3] > 0.0, "{:?}", parts[0].rgba);
        assert_eq!(comps.len(), 4);
        let sub_c = comps.iter().find(|c| c.name == "SubA").unwrap();
        assert_eq!(nodes[sub_c.child].name, "Sub");
        for k in 0..12 {
            assert!((sub_c.matrix[k] - rz[k]).abs() < 1e-9, "{:?}", sub_c.matrix);
        }
        let c2 = comps.iter().find(|c| c.name == "C2").unwrap();
        assert!((c2.matrix[3] - 20.0).abs() < 1e-9);
        // The root's shape is the posed compound: 3 cubes.
        let s = r.shape(root).unwrap();
        assert_eq!(s.solid_count(), 3);
        assert!((s.volume() - 3000.0).abs() < 1e-6);
        let bb = s.bbox_exact();
        assert!((bb[2] + 30.0).abs() < 1e-6 && (bb[5] - 60.0).abs() < 1e-6, "{bb:?}");
    }

    #[test]
    fn moved_distance_and_brep() {
        let a = cube(10.0);
        let b = a.moved(tr([15.0, 0.0, 0.0])).unwrap();
        let d = a.distance_to(&b).unwrap();
        assert!((d.value - 5.0).abs() < 1e-9);
        assert!((d.on_a[0] - 10.0).abs() < 1e-9 && (d.on_b[0] - 15.0).abs() < 1e-9);
        let c = a.moved(tr([5.0, 5.0, 5.0])).unwrap();
        assert!(a.distance_to(&c).unwrap().value < 1e-9);
        let s = b.to_brep_string().unwrap();
        let back = Shape::from_brep_string(&s).unwrap();
        assert!((back.volume() - 1000.0).abs() < 1e-9);
        assert!((back.bbox_exact()[0] - 15.0).abs() < 1e-9);
        assert!(Shape::from_brep_string("garbage").is_err());
    }
}
