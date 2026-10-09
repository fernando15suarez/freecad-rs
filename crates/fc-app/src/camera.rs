//! Orbit camera (orthographic or perspective), Z-up, FreeCAD conventions.
use glam::{DMat3, DMat4, DQuat, DVec3};

pub const FOV_Y: f64 = std::f64::consts::FRAC_PI_4;

#[derive(Clone, Copy, Debug)]
pub struct Camera {
    pub target: DVec3,
    pub dist: f64,
    /// camera-to-world rotation (camera looks down its local -Z, up is +Y)
    pub orient: DQuat,
    pub perspective: bool,
}

pub fn orient_from_dir(fwd: DVec3, up: DVec3) -> DQuat {
    let fwd = fwd.normalize();
    let right = fwd.cross(up).normalize();
    let up = right.cross(fwd);
    DQuat::from_mat3(&DMat3::from_cols(right, up, -fwd)).normalize()
}

impl Camera {
    pub fn new() -> Self {
        Camera {
            target: DVec3::ZERO,
            dist: 100.0,
            orient: orient_from_dir(DVec3::new(-1.0, 1.0, -1.0), DVec3::Z),
            perspective: false,
        }
    }
    pub fn right(&self) -> DVec3 {
        self.orient * DVec3::X
    }
    pub fn up(&self) -> DVec3 {
        self.orient * DVec3::Y
    }
    pub fn fwd(&self) -> DVec3 {
        self.orient * DVec3::NEG_Z
    }
    pub fn eye(&self) -> DVec3 {
        self.target - self.fwd() * self.dist
    }
    pub fn half_height(&self) -> f64 {
        self.dist * (FOV_Y * 0.5).tan()
    }
    /// (view, proj). `center`/`radius` bound the scene and set the depth range.
    pub fn matrices(&self, aspect: f64, center: DVec3, radius: f64) -> (DMat4, DMat4) {
        let eye = self.eye();
        let fwd = self.fwd();
        let view = DMat4::look_to_rh(eye, fwd, self.up());
        let r = radius.max(1e-6) * 1.25;
        let dc = (center - eye).dot(fwd);
        let mut near = dc - r;
        let far = (dc + r).max(1e-3);
        let proj = if self.perspective {
            near = near.max(far * 1e-3);
            DMat4::perspective_rh_gl(FOV_Y, aspect, near, far)
        } else {
            let hh = self.half_height();
            DMat4::orthographic_rh_gl(-hh * aspect, hh * aspect, -hh, hh, near, far)
        };
        (view, proj)
    }
}
