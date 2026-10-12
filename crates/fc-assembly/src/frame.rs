//! Rigid frames (FreeCAD's `Base::Placement`): a position and a unit quaternion.
//!
//! The quaternion is stored as `[x, y, z, w]`, the same order as FreeCAD's
//! `Rotation.Q`, so placements copied from FreeCAD (Python or `Document.xml` Q0..Q3)
//! can be used verbatim.

use fc_part::placement::{add, cross, norm, normalize, scale, sub};
use fc_part::{Placement, V3};
use serde::{Deserialize, Serialize};

/// A rigid transform: `p_world = rot * p_local + pos`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Frame {
    pub pos: V3,
    /// Unit quaternion `[x, y, z, w]`.
    pub rot: [f64; 4],
}

impl Default for Frame {
    fn default() -> Self {
        Frame::IDENTITY
    }
}

/// 3x3 matrix stored as columns (the frame's X, Y and Z axes).
pub type M3 = [V3; 3];

fn qmul(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    let [ax, ay, az, aw] = a;
    let [bx, by, bz, bw] = b;
    [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]
}

fn qnormalize(q: [f64; 4]) -> [f64; 4] {
    let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if n < 1e-300 {
        [0.0, 0.0, 0.0, 1.0]
    } else {
        [q[0] / n, q[1] / n, q[2] / n, q[3] / n]
    }
}

impl Frame {
    pub const IDENTITY: Frame = Frame { pos: [0.0; 3], rot: [0.0, 0.0, 0.0, 1.0] };

    pub fn new(pos: V3, rot: [f64; 4]) -> Frame {
        Frame { pos, rot: qnormalize(rot) }
    }
    pub fn translation(pos: V3) -> Frame {
        Frame { pos, rot: [0.0, 0.0, 0.0, 1.0] }
    }
    /// Rotation by `angle` (radians) about `axis` through the origin.
    pub fn rotation(axis: V3, angle: f64) -> Frame {
        Frame { pos: [0.0; 3], rot: quat_axis_angle(axis, angle) }
    }
    /// FreeCAD's `App.Placement(App.Vector(x, y, z), App.Rotation(yaw, pitch, roll))` (degrees):
    /// R = Rz(yaw) * Ry(pitch) * Rx(roll).
    pub fn from_ypr(pos: V3, yaw: f64, pitch: f64, roll: f64) -> Frame {
        let qz = quat_axis_angle([0.0, 0.0, 1.0], yaw.to_radians());
        let qy = quat_axis_angle([0.0, 1.0, 0.0], pitch.to_radians());
        let qx = quat_axis_angle([1.0, 0.0, 0.0], roll.to_radians());
        Frame { pos, rot: qnormalize(qmul(qmul(qz, qy), qx)) }
    }
    /// Frame with the given origin, Z axis and an X hint (orthogonalised against Z).
    pub fn from_z_x(origin: V3, z: V3, xhint: V3) -> Frame {
        let p = Placement::from_normal(origin, z, xhint);
        Frame::from_placement(&p)
    }
    /// Frame from orthonormal X, Y, Z columns.
    pub fn from_axes(origin: V3, m: M3) -> Frame {
        Frame { pos: origin, rot: quat_from_matrix(m) }
    }
    /// fc-part's sketch-plane placement (origin, X, Y; Z = X x Y).
    pub fn from_placement(p: &Placement) -> Frame {
        let x = normalize(p.xdir);
        let z = normalize(cross(p.xdir, p.ydir));
        let y = cross(z, x);
        Frame::from_axes(p.origin, [x, y, z])
    }
    pub fn to_placement(&self) -> Placement {
        let m = self.matrix();
        Placement { origin: self.pos, xdir: m[0], ydir: m[1] }
    }

    /// Rotation matrix as columns [X, Y, Z].
    pub fn matrix(&self) -> M3 {
        quat_matrix(self.rot)
    }
    pub fn x_axis(&self) -> V3 {
        self.matrix()[0]
    }
    pub fn y_axis(&self) -> V3 {
        self.matrix()[1]
    }
    pub fn z_axis(&self) -> V3 {
        self.matrix()[2]
    }
    pub fn rotate(&self, v: V3) -> V3 {
        mat_vec(&self.matrix(), v)
    }
    pub fn transform_point(&self, p: V3) -> V3 {
        add(self.rotate(p), self.pos)
    }
    /// `self ∘ other` (apply `other` first): FreeCAD's `self * other`.
    pub fn mul(&self, other: &Frame) -> Frame {
        Frame { pos: self.transform_point(other.pos), rot: qnormalize(qmul(self.rot, other.rot)) }
    }
    pub fn inverse(&self) -> Frame {
        let qi = [-self.rot[0], -self.rot[1], -self.rot[2], self.rot[3]];
        let f = Frame { pos: [0.0; 3], rot: qi };
        Frame { pos: scale(f.rotate(self.pos), -1.0), rot: qi }
    }
    /// Rotation vector (axis * angle, radians) of the rotation part.
    pub fn rotation_vector(&self) -> V3 {
        let q = if self.rot[3] < 0.0 { [-self.rot[0], -self.rot[1], -self.rot[2], -self.rot[3]] } else { self.rot };
        let s = norm([q[0], q[1], q[2]]);
        if s < 1e-15 {
            return scale([q[0], q[1], q[2]], 2.0);
        }
        let angle = 2.0 * s.atan2(q[3]);
        scale([q[0], q[1], q[2]], angle / s)
    }
    /// Rotation angle (radians, 0..=π).
    pub fn angle(&self) -> f64 {
        norm(self.rotation_vector())
    }
    /// Left-multiply by `exp(w)` (world-frame rotation vector) and translate by `dt`;
    /// the rotation is about the frame's own origin.
    pub fn perturbed(&self, w: V3, dt: V3) -> Frame {
        let th = norm(w);
        let q = if th < 1e-300 { [0.0, 0.0, 0.0, 1.0] } else { quat_axis_angle(scale(w, 1.0 / th), th) };
        Frame { pos: add(self.pos, dt), rot: qnormalize(qmul(q, self.rot)) }
    }
    /// Translation distance and rotation angle between two frames.
    pub fn distance_to(&self, other: &Frame) -> (f64, f64) {
        let d = norm(sub(self.pos, other.pos));
        let r = self.inverse().mul(other).angle();
        (d, r)
    }
    pub fn is_same(&self, other: &Frame, tol: f64) -> bool {
        let (d, r) = self.distance_to(other);
        d <= tol && r <= tol
    }
    /// FreeCAD's `flipPlacement`: a half turn about the frame's own X axis.
    pub fn flipped(&self) -> Frame {
        self.mul(&Frame::rotation([1.0, 0.0, 0.0], std::f64::consts::PI))
    }
    /// Row-major 3x4 `[R | t]` (OCCT's `gp_Trsf` layout, used by fc-occt).
    pub fn to_matrix34(&self) -> [f64; 12] {
        let m = self.matrix();
        let p = self.pos;
        [m[0][0], m[1][0], m[2][0], p[0], m[0][1], m[1][1], m[2][1], p[1], m[0][2], m[1][2], m[2][2], p[2]]
    }
    pub fn from_matrix34(m: &[f64; 12]) -> Frame {
        Frame::from_axes([m[3], m[7], m[11]], [[m[0], m[4], m[8]], [m[1], m[5], m[9]], [m[2], m[6], m[10]]])
    }
    /// The fraction `f` of this rigid motion along its screw axis (`f = 0` is the identity,
    /// `f = 1` is `self`, and `pow(0.5)` applied twice gives `self`). Used to animate
    /// exploded-view steps.
    pub fn pow(&self, f: f64) -> Frame {
        let w = self.rotation_vector();
        let th = norm(w);
        if th < 1e-12 {
            return Frame::translation(scale(self.pos, f));
        }
        let u = scale(w, 1.0 / th);
        let p = self.pos;
        let along = scale(u, fc_part::placement::dot(p, u));
        let perp = sub(p, along);
        // Point on the screw axis: (I - R) c = perp with c ⟂ u.
        let c = scale(add(perp, scale(cross(u, perp), 1.0 / (th * 0.5).tan())), 0.5);
        let r = Frame::rotation(u, th * f);
        Frame { pos: add(sub(c, r.rotate(c)), scale(along, f)), rot: r.rot }
    }
}

pub fn quat_axis_angle(axis: V3, angle: f64) -> [f64; 4] {
    let a = normalize(axis);
    let (s, c) = (angle * 0.5).sin_cos();
    [a[0] * s, a[1] * s, a[2] * s, c]
}

pub fn quat_matrix(q: [f64; 4]) -> M3 {
    let [x, y, z, w] = q;
    [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + z * w), 2.0 * (x * z - y * w)],
        [2.0 * (x * y - z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z + x * w)],
        [2.0 * (x * z + y * w), 2.0 * (y * z - x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]
}

/// Quaternion of a rotation matrix given as columns (Shepperd's method). The columns are
/// re-orthonormalised first.
pub fn quat_from_matrix(m: M3) -> [f64; 4] {
    let x = normalize(m[0]);
    let z = normalize(cross(x, m[1]));
    let y = cross(z, x);
    // r[i][j] = row i, col j
    let r = [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]];
    let tr = r[0][0] + r[1][1] + r[2][2];
    let q = if tr > 0.0 {
        let s = (tr + 1.0).sqrt() * 2.0;
        [(r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s, 0.25 * s]
    } else if r[0][0] > r[1][1] && r[0][0] > r[2][2] {
        let s = (1.0 + r[0][0] - r[1][1] - r[2][2]).sqrt() * 2.0;
        [0.25 * s, (r[0][1] + r[1][0]) / s, (r[0][2] + r[2][0]) / s, (r[2][1] - r[1][2]) / s]
    } else if r[1][1] > r[2][2] {
        let s = (1.0 + r[1][1] - r[0][0] - r[2][2]).sqrt() * 2.0;
        [(r[0][1] + r[1][0]) / s, 0.25 * s, (r[1][2] + r[2][1]) / s, (r[0][2] - r[2][0]) / s]
    } else {
        let s = (1.0 + r[2][2] - r[0][0] - r[1][1]).sqrt() * 2.0;
        [(r[0][2] + r[2][0]) / s, (r[1][2] + r[2][1]) / s, 0.25 * s, (r[1][0] - r[0][1]) / s]
    };
    qnormalize(q)
}

pub fn mat_vec(m: &M3, v: V3) -> V3 {
    add(add(scale(m[0], v[0]), scale(m[1], v[1])), scale(m[2], v[2]))
}

/// OCCT's automatic X direction for a main direction (gp_Ax2(P, V) / gp_Ax3(P, V)),
/// used by FreeCAD's `Part.Plane(origin, normal).Rotation` for line references.
pub fn occt_auto_xdir(v: V3) -> V3 {
    let v = normalize(v);
    let (a, b, c) = (v[0], v[1], v[2]);
    let (aa, ba, ca) = (a.abs(), b.abs(), c.abs());
    let d = if ba <= aa && ba <= ca {
        if aa > ca {
            [-c, 0.0, a]
        } else {
            [c, 0.0, -a]
        }
    } else if aa <= ba && aa <= ca {
        if ba > ca {
            [0.0, -c, b]
        } else {
            [0.0, c, -b]
        }
    } else if aa > ba {
        [-b, a, 0.0]
    } else {
        [b, -a, 0.0]
    };
    normalize(d)
}

/// Frame whose Z is `z` and whose X follows OCCT's automatic rule.
pub fn occt_frame(origin: V3, z: V3) -> Frame {
    let z = normalize(z);
    let x = occt_auto_xdir(z);
    let y = cross(z, x);
    Frame::from_axes(origin, [x, y, z])
}
