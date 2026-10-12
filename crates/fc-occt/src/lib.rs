//! Safe Rust wrapper over the OpenCASCADE shim.
//!
//! All geometry is in millimetres, world coordinates, Z up (FreeCAD conventions).

pub mod ffi;

use cxx::UniquePtr;
pub use ffi::ffi::{EdgeInfo, FaceInfo, MeshData};
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
    /// Tight bounding box from the exact geometry (no tolerance gap).
    pub fn bbox_exact(&self) -> [f64; 6] {
        sys::bbox_exact(self.r())
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
    pub fn vertex_count(&self) -> usize {
        sys::count_vertices(self.r())
    }
    /// Edge by 1-based index (FreeCAD's `Edge<n>`).
    pub fn edge_info(&self, index: usize) -> Result<EdgeInfo> {
        Ok(sys::edge_info(self.r(), index)?)
    }
    pub fn edges(&self) -> Vec<EdgeInfo> {
        (1..=self.edge_count()).filter_map(|i| self.edge_info(i).ok()).collect()
    }
    /// Vertex position by 1-based index (FreeCAD's `Vertex<n>`).
    pub fn vertex_info(&self, index: usize) -> Result<P3> {
        Ok(sys::vertex_info(self.r(), index)?)
    }
    pub fn vertices(&self) -> Vec<P3> {
        (1..=self.vertex_count()).filter_map(|i| self.vertex_info(i).ok()).collect()
    }
    /// 1-based indices of the edges bounding Face<index>.
    pub fn face_edges(&self, index: usize) -> Result<Vec<usize>> {
        Ok(sys::face_edges(self.r(), index)?)
    }
    /// 1-based indices of the faces sharing Edge<index>.
    pub fn edge_faces(&self, index: usize) -> Result<Vec<usize>> {
        Ok(sys::edge_faces(self.r(), index)?)
    }
    /// 1-based indices of the faces touching Vertex<index>.
    pub fn vertex_faces(&self, index: usize) -> Result<Vec<usize>> {
        Ok(sys::vertex_faces(self.r(), index)?)
    }
    /// Axial extent [min, max] of a cylindrical/conical face along its axis
    /// (`FaceInfo::normal`), measured from `FaceInfo::origin`.
    pub fn face_axis_range(&self, index: usize) -> Result<[f64; 2]> {
        Ok(sys::face_axis_range(self.r(), index)?)
    }
    /// Closest point on Face<index> to `p`, and the outward unit normal there.
    pub fn face_point_normal(&self, index: usize, p: P3) -> Result<(P3, P3)> {
        let r = sys::face_point_normal(self.r(), index, p)?;
        Ok(([r[0], r[1], r[2]], [r[3], r[4], r[5]]))
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
    /// `transition`: 0 = transformed, 1 = right corner, 2 = round corner (FreeCAD's Transition).
    pub fn sweep(profile: &Shape, spine: &Shape, frenet: bool, transition: i32) -> Result<Shape> {
        Ok(Shape(sys::sweep(profile.r(), spine.r(), frenet, transition)?))
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
    fn edges_vertices_and_axes() {
        // 40x30x10 plate with a radius-5 through hole at (20, 15).
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
        assert!(plate.vertex_count() >= 8);
        let edges = plate.edges();
        assert_eq!(edges.len(), plate.edge_count());
        let lines: Vec<_> = edges.iter().filter(|e| e.kind == 0).collect();
        assert_eq!(lines.len(), 12 + 1, "box edges + the hole seam");
        for e in &lines {
            let d = [e.end[0] - e.start[0], e.end[1] - e.start[1], e.end[2] - e.start[2]];
            let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
            assert!((l - e.length).abs() < 1e-9);
            for k in 0..3 {
                assert!((e.mid[k] - (e.start[k] + e.end[k]) / 2.0).abs() < 1e-9);
                assert!((e.dir[k] - d[k] / l).abs() < 1e-9);
            }
        }
        let circles: Vec<usize> = (1..=edges.len()).filter(|&i| edges[i - 1].kind == 1).collect();
        assert_eq!(circles.len(), 2);
        for &i in &circles {
            let e = &edges[i - 1];
            assert!((e.radius - 5.0).abs() < 1e-9 && e.closed);
            assert!((e.center[0] - 20.0).abs() < 1e-9 && (e.center[1] - 15.0).abs() < 1e-9);
            assert!(e.axis[2].abs() > 1.0 - 1e-9);
            assert!((e.length - std::f64::consts::TAU * 5.0).abs() < 1e-6);
            // A hole rim edge touches the cylinder and a planar cap.
            let fs = plate.edge_faces(i).unwrap();
            assert_eq!(fs.len(), 2);
            let kinds: Vec<i32> = fs.iter().map(|&f| plate.face_info(f).unwrap().kind).collect();
            assert!(kinds.contains(&0) && kinds.contains(&1));
        }
        // Vertices.
        let vs = plate.vertices();
        assert!(vs.iter().any(|v| v == &[40.0, 30.0, 10.0]));
        let corner = vs.iter().position(|v| v == &[40.0, 30.0, 10.0]).unwrap() + 1;
        assert_eq!(plate.vertex_faces(corner).unwrap().len(), 3);
        // Cylinder face axis range and face/edge adjacency.
        let cyl = (1..=plate.face_count()).find(|&i| plate.face_info(i).unwrap().kind == 1).unwrap();
        let fi = plate.face_info(cyl).unwrap();
        let r = plate.face_axis_range(cyl).unwrap();
        let z0 = fi.origin[2] + fi.normal[2] * r[0];
        let z1 = fi.origin[2] + fi.normal[2] * r[1];
        assert!((z0.min(z1) - 0.0).abs() < 1e-9 && (z0.max(z1) - 10.0).abs() < 1e-9, "{z0} {z1}");
        let fe = plate.face_edges(cyl).unwrap();
        assert!(circles.iter().all(|c| fe.contains(c)));
        // Closest point + outward normal on the top face.
        let top = (1..=plate.face_count()).find(|&i| { let f = plate.face_info(i).unwrap(); f.kind == 0 && f.normal[2] > 0.5 }).unwrap();
        let (q, n) = plate.face_point_normal(top, [3.0, 4.0, 50.0]).unwrap();
        assert!((q[0] - 3.0).abs() < 1e-7 && (q[1] - 4.0).abs() < 1e-7 && (q[2] - 10.0).abs() < 1e-7);
        assert!((n[2] - 1.0).abs() < 1e-9);
        // Inside the hole the closest point is on the rim.
        let (q, _) = plate.face_point_normal(top, [20.0, 15.0, 10.0]).unwrap();
        assert!(((q[0] - 20.0).hypot(q[1] - 15.0) - 5.0).abs() < 1e-6);
        assert!(plate.edge_info(0).is_err() && plate.vertex_info(999).is_err());
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
