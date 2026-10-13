//! FEM services (cxx bridge to `cpp/fem.cpp`): BooleanFragments of several parts with the
//! history that maps every part's solids and faces to the result, as fc-fem needs it for
//! multi-solid (assembly) analyses.

use crate::{Result, Shape};

#[cxx::bridge(namespace = "fcocc")]
pub mod ffi {
    unsafe extern "C++" {
        include!("fem.h");

        type Shape = crate::ffi::ffi::Shape;

        fn fem_fragments(args: &Shape, fuse: bool, fuzzy: f64, history: &mut Vec<i64>) -> Result<UniquePtr<Shape>>;
        fn fem_solid_faces(s: &Shape, solid: usize) -> Result<Vec<usize>>;
    }
}

/// Result of [`Shape::fragments`]: the combined shape plus, for every input part, the result
/// solids and faces it became (1-based indices in `TopExp::MapShapes` order, FreeCAD's
/// `Solid<n>` / `Face<n>` of the result).
#[derive(Debug, Clone)]
pub struct Fragments {
    pub shape: Shape,
    /// `solids[part]` = result solid indices of that part (sorted).
    pub solids: Vec<Vec<usize>>,
    /// `faces[part]` = (local face index in the part, result face index), sorted.
    pub faces: Vec<Vec<(usize, usize)>>,
}

impl Shape {
    /// FreeCAD's BooleanFragments ("Standard" mode, `generalFuse`) of `parts`: every part is
    /// split by the others, touching parts share the faces they have in common (so a mesh of
    /// the result is conformal across them) and overlapping volumes become separate solids.
    /// `fuzzy` > 0 sets the fuzzy tolerance (mm). With `fuse == false` the parts are only
    /// collected in a compound (each keeps its own faces).
    pub fn fragments(parts: &[Shape], fuse: bool, fuzzy: f64) -> Result<Fragments> {
        let mut c = Shape::default();
        for p in parts {
            c = c.compound_with(p)?;
        }
        let mut hist = Vec::new();
        let shape = Shape(ffi::fem_fragments(c.r(), fuse, fuzzy, &mut hist)?);
        let mut solids = vec![Vec::new(); parts.len()];
        let mut faces = vec![Vec::new(); parts.len()];
        for r in hist.chunks_exact(4) {
            let (part, local, global) = (r[1] as usize, r[2] as usize, r[3] as usize);
            if r[0] == 0 {
                solids[part].push(global);
            } else {
                faces[part].push((local, global));
            }
        }
        for v in &mut solids {
            v.sort_unstable();
            v.dedup();
        }
        for v in &mut faces {
            v.sort_unstable();
            v.dedup();
        }
        Ok(Fragments { shape, solids, faces })
    }

    /// 1-based face indices (of `self`) of the faces bounding `Solid<solid>`.
    pub fn solid_faces(&self, solid: usize) -> Result<Vec<usize>> {
        Ok(ffi::fem_solid_faces(self.r(), solid)?)
    }
}

#[cfg(test)]
mod tests {
    use crate::{ProfileBuilder, Shape};

    fn block(o: [f64; 3], s: [f64; 3]) -> Shape {
        let mut b = ProfileBuilder::new();
        b.begin_face();
        let p = [[o[0], o[1], o[2]], [o[0] + s[0], o[1], o[2]], [o[0] + s[0], o[1] + s[1], o[2]], [o[0], o[1] + s[1], o[2]]];
        for i in 0..4 {
            b.line(p[i], p[(i + 1) % 4]).unwrap();
        }
        b.end_wire().unwrap();
        b.build_faces().unwrap().prism([0.0, 0.0, s[2]]).unwrap()
    }

    #[test]
    fn fragments_share_the_contact_face() {
        let a = block([0.0; 3], [20.0, 20.0, 20.0]);
        let b = block([0.0, 0.0, 20.0], [20.0, 20.0, 20.0]);
        let f = Shape::fragments(&[a.clone(), b.clone()], true, 0.0).unwrap();
        assert_eq!(f.shape.solid_count(), 2);
        assert_eq!(f.shape.face_count(), 11, "the common face is shared");
        assert_eq!(f.solids, vec![vec![1], vec![2]]);
        let fa: std::collections::BTreeSet<usize> = f.shape.solid_faces(1).unwrap().into_iter().collect();
        let fb: std::collections::BTreeSet<usize> = f.shape.solid_faces(2).unwrap().into_iter().collect();
        assert_eq!(fa.intersection(&fb).count(), 1);
        assert_eq!(f.faces[0].len(), 6);
        assert_eq!(f.faces[1].len(), 6);
        // the compound keeps both faces
        let c = Shape::fragments(&[a, b], false, 0.0).unwrap();
        assert_eq!(c.shape.face_count(), 12);
        assert_eq!(c.solids, vec![vec![1], vec![2]]);
        assert_eq!(c.faces[1].iter().map(|x| x.1).collect::<Vec<_>>(), (7..=12).collect::<Vec<_>>());
    }

    #[test]
    fn fragments_split_partial_contact() {
        // lap joint: the upper plate covers part of the lower plate's top face
        let a = block([0.0; 3], [100.0, 40.0, 5.0]);
        let b = block([60.0, 5.0, 5.0], [100.0, 30.0, 5.0]);
        let f = Shape::fragments(&[a, b], true, 0.0).unwrap();
        assert_eq!(f.shape.solid_count(), 2);
        let fa: std::collections::BTreeSet<usize> = f.shape.solid_faces(1).unwrap().into_iter().collect();
        let fb: std::collections::BTreeSet<usize> = f.shape.solid_faces(2).unwrap().into_iter().collect();
        let shared: Vec<_> = fa.intersection(&fb).copied().collect();
        assert_eq!(shared.len(), 1);
        let fi = f.shape.face_info(shared[0]).unwrap();
        assert!((fi.area - 40.0 * 30.0).abs() < 1e-6, "{}", fi.area);
        // the lower plate's top face became two faces
        let top_a: Vec<_> = f.faces[0].iter().filter(|(l, _)| *l == f.faces[0].iter().map(|x| x.0).max().unwrap()).collect();
        assert!(!top_a.is_empty());
        assert!((f.shape.volume() - (100.0 * 40.0 * 5.0 + 100.0 * 30.0 * 5.0)).abs() < 1e-6);
    }
}
