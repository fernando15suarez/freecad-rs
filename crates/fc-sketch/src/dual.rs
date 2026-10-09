//! Forward-mode automatic differentiation over a small, fixed number of local
//! variables. Every constraint touches at most a handful of geometries, so the
//! gradient of each residual row is stored densely over "local slots" which are
//! later scattered into the global Jacobian. This gives exact derivatives (no
//! finite differences), which matters for the rank-revealing QR diagnosis.

use std::ops::{Add, Div, Mul, Neg, Sub};

/// Maximum number of distinct free parameters a single constraint may touch.
pub const NL: usize = 24;

#[derive(Clone, Copy, Debug)]
pub struct D {
    pub v: f64,
    pub g: [f64; NL],
}

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
    /// Square root that stays finite (zero gradient) at the origin.
    pub fn sqrt(self) -> D {
        let s = (self.v.max(0.0) + 1e-300).sqrt();
        if s < 1e-150 {
            return D::c(0.0);
        }
        self.map(s, 0.5 / s)
    }
    pub fn sin(self) -> D {
        self.map(self.v.sin(), self.v.cos())
    }
    pub fn cos(self) -> D {
        self.map(self.v.cos(), -self.v.sin())
    }
    pub fn sq(self) -> D {
        self * self
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
impl Div for D {
    type Output = D;
    #[inline]
    fn div(self, o: D) -> D {
        let inv = 1.0 / o.v;
        let v = self.v * inv;
        let mut g = [0.0; NL];
        for i in 0..NL {
            g[i] = (self.g[i] - v * o.g[i]) * inv;
        }
        D { v, g }
    }
}
impl Neg for D {
    type Output = D;
    fn neg(self) -> D {
        self.map(-self.v, -1.0)
    }
}
impl Add<f64> for D {
    type Output = D;
    fn add(self, o: f64) -> D {
        D { v: self.v + o, g: self.g }
    }
}
impl Sub<f64> for D {
    type Output = D;
    fn sub(self, o: f64) -> D {
        D { v: self.v - o, g: self.g }
    }
}
impl Mul<f64> for D {
    type Output = D;
    fn mul(self, o: f64) -> D {
        self.map(self.v * o, o)
    }
}
impl Div<f64> for D {
    type Output = D;
    fn div(self, o: f64) -> D {
        self.map(self.v / o, 1.0 / o)
    }
}
impl Mul<D> for f64 {
    type Output = D;
    fn mul(self, o: D) -> D {
        o * self
    }
}
impl Sub<D> for f64 {
    type Output = D;
    fn sub(self, o: D) -> D {
        (-o) + self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn derivatives() {
        let x = D::var(0.3, 0);
        let y = D::var(-1.2, 1);
        let f = (x * y).sin() + y.atan2(x) / (x.sq() + 1.0).sqrt();
        let h = 1e-6;
        let num = |a: f64, b: f64| (a * b).sin() + b.atan2(a) / (a * a + 1.0).sqrt();
        let dx = (num(0.3 + h, -1.2) - num(0.3 - h, -1.2)) / (2.0 * h);
        let dy = (num(0.3, -1.2 + h) - num(0.3, -1.2 - h)) / (2.0 * h);
        assert!((f.g[0] - dx).abs() < 1e-8);
        assert!((f.g[1] - dy).abs() < 1e-8);
    }
}
