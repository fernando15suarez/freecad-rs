//! Result fields for the app: per-node displacement, stresses (von Mises, principal), strain,
//! temperature, reactions, eigenmodes, extremes, deformed positions and a colourable surface.

use std::collections::BTreeMap;

use crate::frd::Frd;
use crate::mesh::{FemMesh, SurfaceMesh};
use crate::FemError;

/// One eigenmode of a frequency analysis.
#[derive(Debug, Clone)]
pub struct Mode {
    pub number: u32,
    /// Frequency in Hz.
    pub frequency: f64,
    /// Mass-normalised mode shape per node (mesh node order).
    pub displacement: Vec<[f64; 3]>,
}

/// A scalar extreme with where it happens.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Extreme {
    pub value: f64,
    /// Node id (CalculiX / mesh numbering).
    pub node: u32,
    /// Index into the per-node arrays.
    pub index: usize,
    /// Undeformed node position.
    pub position: [f64; 3],
}

/// Scalar fields a colour map can show (FreeCAD result "Type" choices).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Field {
    DisplacementMagnitude,
    DisplacementX,
    DisplacementY,
    DisplacementZ,
    VonMises,
    PrincipalMax,
    PrincipalMin,
    /// Maximum shear stress (Tresca / 2).
    MaxShear,
    Temperature,
    /// Equivalent (von Mises) strain.
    EquivalentStrain,
}

/// Results of one solve, all per-node arrays in `FemMesh` node order.
#[derive(Debug, Clone, Default)]
pub struct Results {
    pub node_ids: Vec<u32>,
    pub coords: Vec<[f64; 3]>,
    /// Displacement in mm (for a frequency analysis: mode 1).
    pub displacement: Vec<[f64; 3]>,
    /// Stress tensor in MPa: xx, yy, zz, xy, yz, zx.
    pub stress: Vec<[f64; 6]>,
    /// Total strain: xx, yy, zz, xy, yz, zx (engineering shear as written by ccx).
    pub strain: Vec<[f64; 6]>,
    pub von_mises: Vec<f64>,
    /// Principal stresses sorted max, mid, min.
    pub principal: Vec<[f64; 3]>,
    /// Temperature in K (thermomechanical).
    pub temperature: Option<Vec<f64>>,
    /// Heat flux vector (thermomechanical).
    pub heat_flux: Option<Vec<[f64; 3]>>,
    /// Reaction force totals per constraint set name (upper case as printed by ccx).
    pub reactions: BTreeMap<String, [f64; 3]>,
    /// Eigenmodes (frequency analysis).
    pub modes: Vec<Mode>,
    /// Outer surface of the mesh for display.
    pub surface: SurfaceMesh,
}

/// von Mises equivalent of a stress tensor (xx, yy, zz, xy, yz, zx).
pub fn von_mises(s: &[f64; 6]) -> f64 {
    let [xx, yy, zz, xy, yz, zx] = *s;
    (0.5 * ((xx - yy).powi(2) + (yy - zz).powi(2) + (zz - xx).powi(2)) + 3.0 * (xy * xy + yz * yz + zx * zx)).sqrt()
}

/// Eigenvalues of a symmetric 3x3 tensor, sorted descending.
pub fn principal(s: &[f64; 6]) -> [f64; 3] {
    let [a, b, c, d, e, f] = *s; // xx yy zz xy yz zx
    let p1 = d * d + e * e + f * f;
    let q = (a + b + c) / 3.0;
    if p1 <= 1e-30 * (a * a + b * b + c * c).max(1e-300) {
        let mut v = [a, b, c];
        v.sort_by(|x, y| y.total_cmp(x));
        return v;
    }
    let p2 = (a - q).powi(2) + (b - q).powi(2) + (c - q).powi(2) + 2.0 * p1;
    let p = (p2 / 6.0).sqrt();
    // B = (A - qI) / p
    let (ba, bb, bc, bd, be, bf) = ((a - q) / p, (b - q) / p, (c - q) / p, d / p, e / p, f / p);
    let det = ba * (bb * bc - be * be) - bd * (bd * bc - be * bf) + bf * (bd * be - bb * bf);
    let r = (det / 2.0).clamp(-1.0, 1.0);
    let phi = r.acos() / 3.0;
    let e1 = q + 2.0 * p * phi.cos();
    let e3 = q + 2.0 * p * (phi + 2.0 * std::f64::consts::PI / 3.0).cos();
    let e2 = 3.0 * q - e1 - e3;
    [e1, e2, e3]
}

fn equivalent_strain(e: &[f64; 6]) -> f64 {
    // FreeCAD/ccx strain shear components are tensor components (ccx writes EXY = eps_xy)
    let [xx, yy, zz, xy, yz, zx] = *e;
    (2.0 / 3.0 * (xx * xx + yy * yy + zz * zz) + 4.0 / 3.0 * (xy * xy + yz * yz + zx * zx)).sqrt()
}

fn map_block<const N: usize>(mesh: &FemMesh, frd_vals: &std::collections::HashMap<u32, Vec<f64>>) -> Vec<[f64; N]> {
    mesh.node_ids
        .iter()
        .map(|id| {
            let mut a = [0.0; N];
            if let Some(v) = frd_vals.get(id) {
                for (k, x) in v.iter().take(N).enumerate() {
                    a[k] = *x;
                }
            }
            a
        })
        .collect()
}

impl Results {
    /// Build the result arrays from a parsed .frd and the .dat reaction totals.
    pub fn from_frd(mesh: &FemMesh, frd: &Frd, reactions: BTreeMap<String, [f64; 3]>, frequencies: &[f64]) -> Result<Self, FemError> {
        let mut r = Results {
            node_ids: mesh.node_ids.clone(),
            coords: mesh.coords.clone(),
            reactions,
            surface: mesh.boundary(),
            ..Default::default()
        };
        let disp_blocks: Vec<_> = frd.all("DISP").collect();
        let modal = disp_blocks.iter().any(|b| b.kind == 2);
        if modal {
            for (i, b) in disp_blocks.iter().enumerate() {
                let f = frequencies.get(i).copied().unwrap_or(b.value);
                r.modes.push(Mode { number: b.number.max(i as u32 + 1), frequency: f, displacement: map_block::<3>(mesh, &b.values) });
            }
            if let Some(m) = r.modes.first() {
                r.displacement = m.displacement.clone();
            }
            if let Some(s) = frd.all("STRESS").next() {
                r.stress = map_block::<6>(mesh, &s.values);
            }
            if let Some(s) = frd.all("TOSTRAIN").next() {
                r.strain = map_block::<6>(mesh, &s.values);
            }
        } else {
            let d = frd.last("DISP").ok_or_else(|| FemError::Results("no displacement results in the .frd file".into()))?;
            r.displacement = map_block::<3>(mesh, &d.values);
            if let Some(s) = frd.last("STRESS") {
                r.stress = map_block::<6>(mesh, &s.values);
            }
            if let Some(s) = frd.last("TOSTRAIN") {
                r.strain = map_block::<6>(mesh, &s.values);
            }
            if let Some(t) = frd.last("NDTEMP") {
                r.temperature = Some(map_block::<1>(mesh, &t.values).into_iter().map(|v| v[0]).collect());
            }
            if let Some(t) = frd.last("FLUX") {
                r.heat_flux = Some(map_block::<3>(mesh, &t.values));
            }
        }
        if r.stress.is_empty() {
            r.stress = vec![[0.0; 6]; mesh.node_count()];
        }
        if r.strain.is_empty() {
            r.strain = vec![[0.0; 6]; mesh.node_count()];
        }
        r.von_mises = r.stress.iter().map(von_mises).collect();
        r.principal = r.stress.iter().map(principal).collect();
        Ok(r)
    }

    pub fn node_count(&self) -> usize {
        self.node_ids.len()
    }

    /// Scalar field per node.
    pub fn scalar(&self, field: Field) -> Vec<f64> {
        match field {
            Field::DisplacementMagnitude => self.displacement.iter().map(|d| crate::mesh::norm(*d)).collect(),
            Field::DisplacementX => self.displacement.iter().map(|d| d[0]).collect(),
            Field::DisplacementY => self.displacement.iter().map(|d| d[1]).collect(),
            Field::DisplacementZ => self.displacement.iter().map(|d| d[2]).collect(),
            Field::VonMises => self.von_mises.clone(),
            Field::PrincipalMax => self.principal.iter().map(|p| p[0]).collect(),
            Field::PrincipalMin => self.principal.iter().map(|p| p[2]).collect(),
            Field::MaxShear => self.principal.iter().map(|p| 0.5 * (p[0] - p[2])).collect(),
            Field::Temperature => self.temperature.clone().unwrap_or_else(|| vec![0.0; self.node_count()]),
            Field::EquivalentStrain => self.strain.iter().map(equivalent_strain).collect(),
        }
    }

    fn extreme(&self, v: &[f64], max: bool) -> Option<Extreme> {
        let (i, &value) = v.iter().enumerate().reduce(|a, b| if (max && b.1 > a.1) || (!max && b.1 < a.1) { b } else { a })?;
        Some(Extreme { value, node: self.node_ids[i], index: i, position: self.coords[i] })
    }
    pub fn max(&self, field: Field) -> Option<Extreme> {
        self.extreme(&self.scalar(field), true)
    }
    pub fn min(&self, field: Field) -> Option<Extreme> {
        self.extreme(&self.scalar(field), false)
    }
    pub fn max_displacement(&self) -> Option<Extreme> {
        self.max(Field::DisplacementMagnitude)
    }
    pub fn max_von_mises(&self) -> Option<Extreme> {
        self.max(Field::VonMises)
    }

    /// Node positions displaced by `scale` x displacement (FreeCAD's "Show" deformation factor).
    pub fn deformed(&self, scale: f64) -> Vec<[f64; 3]> {
        self.coords.iter().zip(&self.displacement).map(|(p, d)| [p[0] + scale * d[0], p[1] + scale * d[1], p[2] + scale * d[2]]).collect()
    }

    /// Positions displaced by a mode shape (frequency analysis).
    pub fn mode_shape(&self, mode: usize, scale: f64) -> Option<Vec<[f64; 3]>> {
        let m = self.modes.get(mode)?;
        Some(self.coords.iter().zip(&m.displacement).map(|(p, d)| [p[0] + scale * d[0], p[1] + scale * d[1], p[2] + scale * d[2]]).collect())
    }

    /// Display mesh: positions (deformed by `scale`), triangle indices and the per-vertex scalar
    /// of `field` (vertices are the mesh nodes; unused nodes are simply not referenced).
    pub fn surface_mesh(&self, field: Field, scale: f64) -> (Vec<[f64; 3]>, &[[u32; 3]], Vec<f64>) {
        (self.deformed(scale), &self.surface.triangles, self.scalar(field))
    }

    /// A deformation scale that makes the largest displacement `fraction` of the model size.
    pub fn auto_scale(&self, fraction: f64) -> f64 {
        let mut lo = [f64::MAX; 3];
        let mut hi = [f64::MIN; 3];
        for p in &self.coords {
            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
        let size = (0..3).map(|k| hi[k] - lo[k]).fold(0.0, f64::max);
        let dmax = self.max_displacement().map(|e| e.value).unwrap_or(0.0);
        if dmax > 0.0 {
            fraction * size / dmax
        } else {
            1.0
        }
    }

    /// Sum of all reaction totals (N).
    pub fn reaction_sum(&self) -> [f64; 3] {
        let mut s = [0.0; 3];
        for v in self.reactions.values() {
            for k in 0..3 {
                s[k] += v[k];
            }
        }
        s
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn stress_invariants() {
        let uni = [100.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        assert!((von_mises(&uni) - 100.0).abs() < 1e-12);
        let shear = [0.0, 0.0, 0.0, 50.0, 0.0, 0.0];
        assert!((von_mises(&shear) - 50.0 * 3f64.sqrt()).abs() < 1e-9);
        let p = principal(&shear);
        assert!((p[0] - 50.0).abs() < 1e-9 && p[1].abs() < 1e-9 && (p[2] + 50.0).abs() < 1e-9);
        let s = [10.0, 20.0, 30.0, 4.0, 5.0, 6.0];
        let p = principal(&s);
        assert!((p.iter().sum::<f64>() - 60.0).abs() < 1e-9);
        assert!(p[0] >= p[1] && p[1] >= p[2]);
        // det check: product of eigenvalues == det
        let det = 10.0 * (20.0 * 30.0 - 25.0) - 4.0 * (4.0 * 30.0 - 5.0 * 6.0) + 6.0 * (4.0 * 5.0 - 20.0 * 6.0);
        assert!((p[0] * p[1] * p[2] - det).abs() < 1e-6);
    }
}
