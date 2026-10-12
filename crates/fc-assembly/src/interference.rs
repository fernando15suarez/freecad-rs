//! Interference and clearance checks between posed instances (Onshape/SolidWorks-style
//! "interference detection"; FreeCAD has none built in).

use crate::model::{AssemblyId, InstanceId, Workspace};
use crate::structure::LeafInstance;
use fc_occt::{Distance, Shape};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InterferenceOptions {
    /// Overlaps up to this volume (mm³) are ignored: solids that only touch along faces,
    /// edges or points give a zero-volume common.
    pub min_volume: f64,
    /// Bounding boxes must overlap by more than this (mm) on every axis before the boolean
    /// runs (a touching pair's boxes overlap by ~0).
    pub tolerance: f64,
    /// Skip hidden instances.
    pub visible_only: bool,
}

impl Default for InterferenceOptions {
    fn default() -> Self {
        InterferenceOptions { min_volume: 1e-6, tolerance: 1e-6, visible_only: false }
    }
}

/// Two part instances whose solids overlap.
#[derive(Clone, Debug)]
pub struct Interference {
    pub a: LeafInstance,
    pub b: LeafInstance,
    /// Overlap volume (mm³).
    pub volume: f64,
    /// The overlapping solid (boolean common) in assembly coordinates, for highlighting.
    pub shape: Shape,
}

fn boxes_overlap(a: &[f64; 6], b: &[f64; 6], tol: f64) -> bool {
    (0..3).all(|k| a[k + 3].min(b[k + 3]) - a[k].max(b[k]) > tol)
}

impl Workspace {
    /// Pairwise interference between every two part instances of an assembly (sub-assemblies
    /// flattened), with a bounding-box prefilter.
    pub fn interferences(&self, asm: AssemblyId, opts: &InterferenceOptions) -> Vec<Interference> {
        let posed: Vec<(LeafInstance, Shape, [f64; 6])> = self
            .leaves(asm)
            .into_iter()
            .filter(|l| l.visible || !opts.visible_only)
            .filter_map(|l| {
                let s = self.leaf_shape(&l)?;
                let b = s.bbox_exact();
                Some((l, s, b))
            })
            .collect();
        let mut out = vec![];
        for i in 0..posed.len() {
            for j in i + 1..posed.len() {
                let (la, sa, ba) = &posed[i];
                let (lb, sb, bb) = &posed[j];
                if !boxes_overlap(ba, bb, opts.tolerance) {
                    continue;
                }
                let Ok(c) = sa.common(sb) else { continue };
                if c.is_null() {
                    continue;
                }
                let v = c.volume();
                if v > opts.min_volume {
                    out.push(Interference { a: la.clone(), b: lb.clone(), volume: v, shape: c });
                }
            }
        }
        out
    }

    /// Minimum distance between two instances (instance paths; a sub-assembly instance
    /// counts as all its parts), with the closest points in assembly coordinates. 0 when they
    /// touch or overlap.
    pub fn clearance(&self, asm: AssemblyId, a: &[InstanceId], b: &[InstanceId]) -> Option<Distance> {
        let sa = self.instance_shape(asm, a)?;
        let sb = self.instance_shape(asm, b)?;
        sa.distance_to(&sb).ok()
    }
}
