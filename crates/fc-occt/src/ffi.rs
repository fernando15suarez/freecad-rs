//! cxx bridge to the C++ shim in `cpp/shim.cpp`.

#[cxx::bridge(namespace = "fcocc")]
pub mod ffi {
    /// Triangulated shape for display. Per-triangle face ids are 1-based (Face1 = 1), as in FreeCAD.
    #[derive(Default, Debug, Clone)]
    pub struct MeshData {
        pub positions: Vec<f32>,
        pub normals: Vec<f32>,
        pub indices: Vec<u32>,
        pub tri_face: Vec<u32>,
        /// Concatenated edge polylines (xyz triples); edge i spans points edge_offsets[i]..edge_offsets[i+1].
        pub edge_points: Vec<f32>,
        pub edge_offsets: Vec<u32>,
    }

    /// Geometric description of one B-rep face (1-based index in `faces()` order).
    #[derive(Default, Debug, Clone)]
    pub struct FaceInfo {
        /// 0 = plane, 1 = cylinder, 2 = cone, 3 = sphere, 4 = torus, 5 = other.
        pub kind: i32,
        pub origin: [f64; 3],
        pub normal: [f64; 3],
        pub xdir: [f64; 3],
        pub radius: f64,
        pub area: f64,
        pub center: [f64; 3],
        pub bbox_min: [f64; 3],
        pub bbox_max: [f64; 3],
    }

    unsafe extern "C++" {
        include!("shim.h");

        type Shape;
        type WireBuilder;

        fn occt_version() -> i32;

        // ---- profile construction ----
        fn new_wire_builder() -> UniquePtr<WireBuilder>;
        fn add_line(self: Pin<&mut WireBuilder>, a: [f64; 3], b: [f64; 3]) -> Result<()>;
        /// Arc of circle on plane (center, normal, xdir) from angle a0 to a1 (CCW about normal).
        fn add_arc(self: Pin<&mut WireBuilder>, c: [f64; 3], n: [f64; 3], x: [f64; 3], r: f64, a0: f64, a1: f64) -> Result<()>;
        fn add_circle(self: Pin<&mut WireBuilder>, c: [f64; 3], n: [f64; 3], x: [f64; 3], r: f64) -> Result<()>;
        /// Finish the current wire (must be closed when building faces).
        fn end_wire(self: Pin<&mut WireBuilder>) -> Result<()>;
        /// Start a new face; the next wire is its outer boundary, following wires are holes.
        fn begin_face(self: Pin<&mut WireBuilder>);
        /// Build a compound of planar faces from the collected wires.
        fn build_faces(self: Pin<&mut WireBuilder>) -> Result<UniquePtr<Shape>>;
        /// Build the collected wires (open or closed) as a compound of wires (for sweeps/lofts).
        fn build_wires(self: Pin<&mut WireBuilder>) -> Result<UniquePtr<Shape>>;

        // ---- features ----
        fn prism(s: &Shape, d: [f64; 3]) -> Result<UniquePtr<Shape>>;
        fn revol(s: &Shape, origin: [f64; 3], dir: [f64; 3], angle: f64) -> Result<UniquePtr<Shape>>;
        fn fuse(a: &Shape, b: &Shape) -> Result<UniquePtr<Shape>>;
        fn cut(a: &Shape, b: &Shape) -> Result<UniquePtr<Shape>>;
        fn common(a: &Shape, b: &Shape) -> Result<UniquePtr<Shape>>;
        fn translated(s: &Shape, d: [f64; 3]) -> Result<UniquePtr<Shape>>;
        fn rotated(s: &Shape, origin: [f64; 3], dir: [f64; 3], angle: f64) -> Result<UniquePtr<Shape>>;
        /// Loft through wire sections (each `Shape` a wire compound with one wire).
        fn loft(sections: &Shape, solid: bool, ruled: bool) -> Result<UniquePtr<Shape>>;
        /// Sweep a profile (wire compound) along a spine (wire compound).
        fn sweep(profile: &Shape, spine: &Shape, frenet: bool, transition: i32) -> Result<UniquePtr<Shape>>;
        fn make_compound(parts: &Shape, add: &Shape) -> Result<UniquePtr<Shape>>;
        fn empty_shape() -> UniquePtr<Shape>;
        fn clone_shape(s: &Shape) -> UniquePtr<Shape>;
        /// Merge coplanar faces / collinear edges left over by booleans (like FreeCAD's Refine).
        fn refine(s: &Shape) -> Result<UniquePtr<Shape>>;

        // ---- queries ----
        fn is_null(s: &Shape) -> bool;
        fn is_valid(s: &Shape) -> bool;
        fn volume(s: &Shape) -> f64;
        fn area(s: &Shape) -> f64;
        fn bbox(s: &Shape) -> [f64; 6];
        fn bbox_exact(s: &Shape) -> [f64; 6];
        fn count_solids(s: &Shape) -> usize;
        fn count_faces(s: &Shape) -> usize;
        fn count_edges(s: &Shape) -> usize;
        fn face_info(s: &Shape, index: usize) -> Result<FaceInfo>;
        fn mesh(s: &Shape, linear_deflection: f64, angular_deflection: f64) -> Result<MeshData>;

        // ---- io ----
        fn write_step(s: &Shape, path: &str) -> Result<()>;
        fn write_stl(s: &Shape, path: &str, linear_deflection: f64) -> Result<()>;
        fn write_brep(s: &Shape, path: &str) -> Result<()>;
        fn read_step(path: &str) -> Result<UniquePtr<Shape>>;
    }
}
