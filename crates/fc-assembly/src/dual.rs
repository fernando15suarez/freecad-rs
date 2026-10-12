//! Forward-mode automatic differentiation over the 12 local unknowns of a joint
//! (6 per instance side: scaled rotation vector, then translation). Every residual row
//! gets an exact gradient, which the rank analysis relies on (same approach as
//! fc-sketch's `dual.rs`).

use std::ops::{Add, Mul, Neg, Sub};

pub const NL: usize = 12;

#[derive(Clone, Copy, Debug)]
pub struct D {
    pub v: f64,
    pub g: [f64; NL],
}

pub type DV = [D; 3];

impl D {
    #[inline]
    pub fn c(v: f64) -> D {
        D { v, g: [0.0; NL] }
    }
    #[inline]
    pub fn var(v: f64, slot: usize) -> D {
        let mut g = [0.0; NL];
        g[slot] = 1.0;
        D { v, g }
    }
    #[inline]
    fn map(self, v: f64, dv: f64) -> D {
        let mut g = self.g;
        for e in g.iter_mut() {
            *e *= dv;
        }
        D { v, g }
    }
    #[inline]
    pub fn scale(self, s: f64) -> D {
        self.map(self.v * s, s)
    }
    /// Square root that stays finite (zero gradient) at the origin.
    pub fn sqrt(self) -> D {
        let s = self.v.max(0.0).sqrt();
        if s < 1e-150 {
            return D::c(0.0);
        }
        self.map(s, 0.5 / s)
    }
    /// atan2(self, x)
    pub fn atan2(self, x: D) -> D {
        let y = self;
        let r2 = x.v * x.v + y.v * y.v;
        let v = y.v.atan2(x.v);
        if r2 == 0.0 {
            return D::c(v);
        }
        let mut g = [0.0; NL];
        for i in 0..NL {
            g[i] = (x.v * y.g[i] - y.v * x.g[i]) / r2;
        }
        D { v, g }
    }
    pub fn add_c(self, c: f64) -> D {
        D { v: self.v + c, g: self.g }
    }
}

impl Add for D {
    type Output = D;
    #[inline]
    fn add(self, o: D) -> D {
        let mut g = self.g;
        for i in 0..NL {
            g[i] += o.g[i];
        }
        D { v: self.v + o.v, g }
    }
}
impl Sub for D {
    type Output = D;
    #[inline]
    fn sub(self, o: D) -> D {
        let mut g = self.g;
        for i in 0..NL {
            g[i] -= o.g[i];
        }
        D { v: self.v - o.v, g }
    }
}
impl Mul for D {
    type Output = D;
    #[inline]
    fn mul(self, o: D) -> D {
        let mut g = [0.0; NL];
        for i in 0..NL {
            g[i] = self.g[i] * o.v + o.g[i] * self.v;
        }
        D { v: self.v * o.v, g }
    }
}
impl Neg for D {
    type Output = D;
    #[inline]
    fn neg(self) -> D {
        self.scale(-1.0)
    }
}

pub fn dv_c(v: [f64; 3]) -> DV {
    [D::c(v[0]), D::c(v[1]), D::c(v[2])]
}
pub fn dot(a: &DV, b: &DV) -> D {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
pub fn cross(a: &DV, b: &DV) -> DV {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
pub fn vsub(a: &DV, b: &DV) -> DV {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
pub fn vadd(a: &DV, b: &DV) -> DV {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
pub fn norm(a: &DV) -> D {
    dot(a, a).sqrt()
}
