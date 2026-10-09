//! Sketch planes and coordinate frames.

use serde::{Deserialize, Serialize};

pub type V3 = [f64; 3];

pub fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
pub fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
pub fn scale(a: V3, s: f64) -> V3 {
    [a[0] * s, a[1] * s, a[2] * s]
}
pub fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
pub fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
pub fn norm(a: V3) -> f64 {
    dot(a, a).sqrt()
}
pub fn normalize(a: V3) -> V3 {
    let n = norm(a);
    if n < 1e-15 {
        a
    } else {
        scale(a, 1.0 / n)
    }
}

/// Right-handed orthonormal frame: sketch X, sketch Y and the plane normal (= X × Y).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Placement {
    pub origin: V3,
    pub xdir: V3,
    pub ydir: V3,
}

impl Default for Placement {
    fn default() -> Self {
        Self::XY
    }
}

impl Placement {
    /// FreeCAD's XY_Plane.
    pub const XY: Placement = Placement { origin: [0.0; 3], xdir: [1.0, 0.0, 0.0], ydir: [0.0, 1.0, 0.0] };
    /// FreeCAD's XZ_Plane: sketch X = +X, sketch Y = +Z, normal = -Y.
    pub const XZ: Placement = Placement { origin: [0.0; 3], xdir: [1.0, 0.0, 0.0], ydir: [0.0, 0.0, 1.0] };
    /// FreeCAD's YZ_Plane: sketch X = +Y, sketch Y = +Z, normal = +X.
    pub const YZ: Placement = Placement { origin: [0.0; 3], xdir: [0.0, 1.0, 0.0], ydir: [0.0, 0.0, 1.0] };

    /// Frame from origin, normal and an X hint (orthogonalised against the normal).
    pub fn from_normal(origin: V3, normal: V3, xhint: V3) -> Placement {
        let n = normalize(normal);
        let mut x = sub(xhint, scale(n, dot(xhint, n)));
        if norm(x) < 1e-9 {
            // Pick any perpendicular.
            let alt = if n[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
            x = sub(alt, scale(n, dot(alt, n)));
        }
        let x = normalize(x);
        let y = cross(n, x);
        Placement { origin, xdir: x, ydir: y }
    }

    pub fn normal(&self) -> V3 {
        cross(self.xdir, self.ydir)
    }

    /// Sketch (u, v) → world.
    pub fn to_world(&self, u: f64, v: f64) -> V3 {
        add(self.origin, add(scale(self.xdir, u), scale(self.ydir, v)))
    }

    /// World → sketch (u, v, w) where w is the height above the plane.
    pub fn to_local(&self, p: V3) -> V3 {
        let d = sub(p, self.origin);
        [dot(d, self.xdir), dot(d, self.ydir), dot(d, self.normal())]
    }

    /// Same plane, moved along its normal.
    pub fn offset(&self, along_normal: f64) -> Placement {
        Placement { origin: add(self.origin, scale(self.normal(), along_normal)), ..*self }
    }

    /// Flip to look from the other side (normal reversed, X kept).
    pub fn reversed(&self) -> Placement {
        Placement { origin: self.origin, xdir: self.xdir, ydir: scale(self.ydir, -1.0) }
    }
}
