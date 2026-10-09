//! Safe Rust wrapper over the OpenCASCADE shim.
//!
//! All geometry is in millimetres, world coordinates, Z up (FreeCAD conventions).

pub mod ffi;

use cxx::UniquePtr;
pub use ffi::ffi::{FaceInfo, MeshData};
use ffi::ffi as sys;
use std::fmt;

pub type P3 = [f64; 3];

#[derive(Debug, Clone, PartialEq)]
pub struct OcctError(pub String);
impl fmt::Display for OcctError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for OcctError {}
impl From<cxx::Exception> for OcctError {
    fn from(e: cxx::Exception) -> Self {
        OcctError(e.what().to_string())
    }
}
pub type Result<T> = std::result::Result<T, OcctError>;

/// OCCT version as 0xMMmmpp.
pub fn version() -> i32 {
    sys::occt_version()
}

/// A B-rep shape (TopoDS_Shape).
pub struct Shape(UniquePtr<sys::Shape>);

// TopoDS_Shape handles are reference counted with non-atomic counters; moving a whole
// Shape between threads is fine as long as it is not shared, which `Send` without `Sync` models.
unsafe impl Send for Shape {}

impl Clone for Shape {
    fn clone(&self) -> Self {
        Shape(sys::clone_shape(&self.0))
    }
}

impl fmt::Debug for Shape {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.is_null() {
            return f.write_str("Shape(null)");
        }
        write!(f, "Shape(solids={}, faces={}, volume={:.3})", self.solid_count(), self.face_count(), self.volume())
    }
}

impl Default for Shape {
    fn default() -> Self {
        Shape(sys::empty_shape())
    }
}

impl Shape {
    fn r(&self) -> &sys::Shape {
        &self.0
    }
    pub fn is_null(&self) -> bool {
        sys::is_null(self.r())
    }
    pub fn is_valid(&self) -> bool {
        sys::is_valid(self.r())
    }
    pub fn volume(&self) -> f64 {
        sys::volume(self.r())
    }
    pub fn area(&self) -> f64 {
        sys::area(self.r())
    }
    /// [xmin, ymin, zmin, xmax, ymax, zmax]
    pub fn bbox(&self) -> [f64; 6] {
        sys::bbox(self.r())
    }
    pub fn solid_count(&self) -> usize {
        sys::count_solids(self.r())
    }
    pub fn face_count(&self) -> usize {
        sys::count_faces(self.r())
    }
    pub fn edge_count(&self) -> usize {
        sys::count_edges(self.r())
    }
    /// Face by 1-based index (FreeCAD's `Face<n>` naming).
    pub fn face_info(&self, index: usize) -> Result<FaceInfo> {
        Ok(sys::face_info(self.r(), index)?)
    }
    pub fn faces(&self) -> Vec<FaceInfo> {
        (1..=self.face_count()).filter_map(|i| self.face_info(i).ok()).collect()
    }

    pub fn prism(&self, d: P3) -> Result<Shape> {
        Ok(Shape(sys::prism(self.r(), d)?))
    }
    /// Revolve by `angle` radians about the axis through `origin` along `dir`.
    pub fn revolve(&self, origin: P3, dir: P3, angle: f64) -> Result<Shape> {
        Ok(Shape(sys::revol(self.r(), origin, dir, angle)?))
    }
    pub fn fuse(&self, other: &Shape) -> Result<Shape> {
        Ok(Shape(sys::fuse(self.r(), other.r())?))
    }
    pub fn cut(&self, other: &Shape) -> Result<Shape> {
        Ok(Shape(sys::cut(self.r(), other.r())?))
    }
    pub fn common(&self, other: &Shape) -> Result<Shape> {
        Ok(Shape(sys::common(self.r(), other.r())?))
    }
    pub fn translated(&self, d: P3) -> Result<Shape> {
        Ok(Shape(sys::translated(self.r(), d)?))
    }
    /// Rotate by `angle` radians about the axis through `origin` along `dir`.
    pub fn rotated(&self, origin: P3, dir: P3, angle: f64) -> Result<Shape> {
        Ok(Shape(sys::rotated(self.r(), origin, dir, angle)?))
    }
    pub fn refine(&self) -> Result<Shape> {
        Ok(Shape(sys::refine(self.r())?))
    }
    /// Compound of self's children (or self) plus `other`.
    pub fn compound_with(&self, other: &Shape) -> Result<Shape> {
        Ok(Shape(sys::make_compound(self.r(), other.r())?))
    }
    pub fn loft(sections: &Shape, solid: bool, ruled: bool) -> Result<Shape> {
        Ok(Shape(sys::loft(sections.r(), solid, ruled)?))
    }
    pub fn sweep(profile: &Shape, spine: &Shape, frenet: bool) -> Result<Shape> {
        Ok(Shape(sys::sweep(profile.r(), spine.r(), frenet)?))
    }

    pub fn mesh(&self, linear_deflection: f64, angular_deflection: f64) -> Result<MeshData> {
        Ok(sys::mesh(self.r(), linear_deflection, angular_deflection)?)
    }
    pub fn write_step(&self, path: &str) -> Result<()> {
        Ok(sys::write_step(self.r(), path)?)
    }
    pub fn write_stl(&self, path: &str, linear_deflection: f64) -> Result<()> {
        Ok(sys::write_stl(self.r(), path, linear_deflection)?)
    }
    pub fn write_brep(&self, path: &str) -> Result<()> {
        Ok(sys::write_brep(self.r(), path)?)
    }
    pub fn read_step(path: &str) -> Result<Shape> {
        Ok(Shape(sys::read_step(path)?))
    }
}

/// Builds wires and planar faces out of lines, arcs and circles in 3D.
pub struct ProfileBuilder(UniquePtr<sys::WireBuilder>);

impl Default for ProfileBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProfileBuilder {
    pub fn new() -> Self {
        ProfileBuilder(sys::new_wire_builder())
    }
    pub fn line(&mut self, a: P3, b: P3) -> Result<()> {
        Ok(self.0.pin_mut().add_line(a, b)?)
    }
    /// Arc on the plane (center, normal, xdir) from angle `a0` to `a1`, CCW about `normal`.
    pub fn arc(&mut self, c: P3, n: P3, x: P3, r: f64, a0: f64, a1: f64) -> Result<()> {
        Ok(self.0.pin_mut().add_arc(c, n, x, r, a0, a1)?)
    }
    pub fn circle(&mut self, c: P3, n: P3, x: P3, r: f64) -> Result<()> {
        Ok(self.0.pin_mut().add_circle(c, n, x, r)?)
    }
    pub fn end_wire(&mut self) -> Result<()> {
        Ok(self.0.pin_mut().end_wire()?)
    }
    /// The next wire is a new face's outer boundary; following wires become its holes.
    pub fn begin_face(&mut self) {
        self.0.pin_mut().begin_face()
    }
    pub fn build_faces(&mut self) -> Result<Shape> {
        Ok(Shape(self.0.pin_mut().build_faces()?))
    }
    pub fn build_wires(&mut self) -> Result<Shape> {
        Ok(Shape(self.0.pin_mut().build_wires()?))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rect(w: f64, h: f64) -> Shape {
        let mut b = ProfileBuilder::new();
        b.begin_face();
        let p = [[0.0, 0.0, 0.0], [w, 0.0, 0.0], [w, h, 0.0], [0.0, h, 0.0]];
        for i in 0..4 {
            b.line(p[i], p[(i + 1) % 4]).unwrap();
        }
        b.end_wire().unwrap();
        b.build_faces().unwrap()
    }

    #[test]
    fn pad_rectangle() {
        assert!(version() >= 0x070800);
        let solid = rect(20.0, 10.0).prism([0.0, 0.0, 5.0]).unwrap();
        assert!(solid.is_valid());
        assert_eq!(solid.solid_count(), 1);
        assert_eq!(solid.face_count(), 6);
        assert!((solid.volume() - 1000.0).abs() < 1e-6);
    }

    #[test]
    fn plate_with_hole_and_pocket() {
        let mut b = ProfileBuilder::new();
        b.begin_face();
        let p = [[0.0, 0.0, 0.0], [40.0, 0.0, 0.0], [40.0, 30.0, 0.0], [0.0, 30.0, 0.0]];
        for i in 0..4 {
            b.line(p[i], p[(i + 1) % 4]).unwrap();
        }
        b.end_wire().unwrap();
        b.circle([20.0, 15.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 5.0).unwrap();
        b.end_wire().unwrap();
        let plate = b.build_faces().unwrap().prism([0.0, 0.0, 10.0]).unwrap();
        let expect = 40.0 * 30.0 * 10.0 - std::f64::consts::PI * 25.0 * 10.0;
        assert!((plate.volume() - expect).abs() < 1e-3, "{}", plate.volume());
        let pocket = rect(10.0, 10.0).translated([0.0, 0.0, 7.0]).unwrap().prism([0.0, 0.0, 3.0]).unwrap();
        let cut = plate.cut(&pocket).unwrap().refine().unwrap();
        assert!(cut.is_valid());
        assert!((cut.volume() - (expect - 300.0)).abs() < 1e-3);
        let m = cut.mesh(0.1, 0.5).unwrap();
        assert!(!m.indices.is_empty());
        assert_eq!(m.indices.len() / 3, m.tri_face.len());
        assert_eq!(m.edge_offsets.len(), cut.edge_count() + 1);
    }

    #[test]
    fn revolve_and_export() {
        // Rectangle in XZ revolved about Z: a tube.
        let mut b = ProfileBuilder::new();
        b.begin_face();
        let p = [[5.0, 0.0, 0.0], [10.0, 0.0, 0.0], [10.0, 0.0, 20.0], [5.0, 0.0, 20.0]];
        for i in 0..4 {
            b.line(p[i], p[(i + 1) % 4]).unwrap();
        }
        b.end_wire().unwrap();
        let tube = b.build_faces().unwrap().revolve([0.0; 3], [0.0, 0.0, 1.0], std::f64::consts::TAU).unwrap();
        let expect = std::f64::consts::PI * (100.0 - 25.0) * 20.0;
        assert!((tube.volume() - expect).abs() < 1e-2, "{}", tube.volume());
        let dir = std::env::temp_dir();
        let step = dir.join("fc_occt_tube.step");
        let stl = dir.join("fc_occt_tube.stl");
        tube.write_step(step.to_str().unwrap()).unwrap();
        tube.write_stl(stl.to_str().unwrap(), 0.05).unwrap();
        let back = Shape::read_step(step.to_str().unwrap()).unwrap();
        assert!((back.volume() - expect).abs() < 1e-2);
        assert!(std::fs::metadata(&stl).unwrap().len() > 1000);
    }

    #[test]
    fn errors_are_reported() {
        let mut b = ProfileBuilder::new();
        b.begin_face();
        b.line([0.0; 3], [1.0, 0.0, 0.0]).unwrap();
        b.line([1.0, 0.0, 0.0], [1.0, 1.0, 0.0]).unwrap();
        b.end_wire().unwrap();
        assert!(b.build_faces().is_err());
        assert!(b.line([0.0; 3], [0.0; 3]).is_err());
    }
}
