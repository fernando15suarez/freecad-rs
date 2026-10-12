//! Joints (FreeCAD) / mates (Onshape) and their exact constraint equations.
//!
//! A joint relates two joint coordinate systems (JCS): `I` on side `a` and `J` on side
//! `b`, both rigidly attached to an instance (`world = instance.placement ∘ local ∘
//! offset`). Notation below: `rIJ = origin(J) - origin(I)`, `xI, yI, zI` the axes of
//! I, `xJ, yJ, zJ` those of J. The equations are those of the Ondsel solver that FreeCAD
//! 1.1 maps its joints to (`AssemblyObject::makeMbdJointOfType`), except where noted:
//!
//! | Joint | Ondsel joint | Equations | Removes |
//! |---|---|---|---|
//! | Fastened (FreeCAD *Fixed*) | FixedJoint | `rIJ = 0` (3), `R_I = R_J` (3) | 6 |
//! | Revolute | RevoluteJoint | `rIJ = 0` (3), `zI ∥ zJ` (2) | 5 |
//! | Cylindrical | CylindricalJoint | `rIJ·xI = rIJ·yI = 0` (2), `zI ∥ zJ` (2) | 4 |
//! | Slider | TranslationalJoint | `rIJ·xI = rIJ·yI = 0` (2), `R_I = R_J` (3) — slides along zI | 5 |
//! | PinSlot (Onshape) | – | `rIJ·yI = rIJ·zI = 0` (2), `zI ∥ zJ` (2) — slides along xI, turns about zI | 4 |
//! | Planar (Onshape) | PlanarJoint | `rIJ·zI = 0` (1), `zI ∥ zJ` (2) | 3 |
//! | Ball | SphericalJoint | `rIJ = 0` (3) | 3 |
//! | Parallel | ParallelAxesJoint | `zI ∥ zJ` (2) | 2 |
//! | Perpendicular | PerpendicularJoint | `zI·zJ = 0` | 1 |
//! | Angle(θ) | AngleJoint | `zI·zJ = cos θ` (θ ≡ 0 → Parallel, like FreeCAD) | 1 |
//! | Distance(d) | depends on the picked geometry, see [`DistanceMode`] | 1–3 |
//! | RackPinion(r) | RackPinJoint | `rIJ·xI + r·θz = C` (I = rack, J = pinion) | 1 |
//! | Screw(p) | ScrewJoint | `2π·rIJ·zI − p·θz = C` | 1 |
//! | Gears(r1, r2) | GearJoint | `orbit(J around I) + r1/r2 · orbit(I around J) = C` | 1 |
//! | Belt(r1, r2) | GearJoint with `-r2` | same, pulleys turn the same way | 1 |
//!
//! `θz` is the rotation of J about zI (angle of xJ in I's XY plane), unwrapped
//! continuously from the value at solve start. The coupling constants `C` are taken from
//! the placements at solve start, exactly like Ondsel (`aConstant` from `postInput`), so
//! a static solve never moves a part because of a coupling joint; drags then keep it.
//!
//! **Orientation sense.** Ondsel writes `zI ∥ zJ` as `zI·xJ = zI·yJ = 0` and `R_I = R_J`
//! as `zI·xJ = zI·yJ = xI·yJ = 0`, which are also satisfied by the flipped poses
//! (antiparallel Z, or any half turn about an axis); FreeCAD relies on the pre-solve
//! ("match JCS", see [`crate::Assembly::presolve_joint`]) and on the nearest root. We
//! write the alignment as the rotation vector between the frames (exactly zero only at
//! the aligned pose, same Jacobian rank as Ondsel at the solution) and choose the target
//! pose explicitly with [`Sense`]: `Aligned`, `Flipped` (J turned half a turn about its X
//! axis — FreeCAD's "Reverse", Onshape's flip), or `Auto` = the root nearest to the
//! current placements (Ondsel's behaviour; the default).
//!
//! **Offsets.** Each side carries an offset frame applied after the JCS (FreeCAD's
//! `Offset1`/`Offset2`; the task panel's "Offset"/"Rotation" fields set `Offset2`'s Z
//! translation / yaw — [`Joint::with_offset`], [`Joint::with_rotation`]). Onshape's
//! mate offsets map onto the same thing.
//!
//! **Limits.** `length_*` bound `rIJ·zI` and `angle_*` bound `θz` (degrees), on any joint
//! (FreeCAD offers them on Slider/Cylindrical and Revolute/Cylindrical). They are
//! inequality constraints handled with an active set: a violated bound becomes an
//! equality at the bound and the system is solved again.

use crate::frame::Frame;
use crate::geom::{ImplicitPoint, RefKind};
use crate::{ConnectorId, InstanceId, JointId};
use fc_part::FeatureId;
use serde::{Deserialize, Serialize};

/// One side of a joint.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum JointRef {
    /// A mate connector feature inside the instance's part (Onshape style).
    PartConnector { instance: InstanceId, connector: FeatureId },
    /// A mate connector defined in the assembly (anchored to an instance).
    AssemblyConnector(ConnectorId),
    /// Picked geometry (FreeCAD style): "Face3", "Edge5", "Vertex2", plus which point of it.
    Geometry { instance: InstanceId, element: String, #[serde(default)] implicit: ImplicitPoint },
    /// A fixed JCS in the instance's local coordinates (FreeCAD's detached placement, and
    /// what FCStd import produces).
    Frame { instance: InstanceId, local: Frame },
    /// The instance's own origin frame (FreeCAD: a whole object picked without element).
    Origin { instance: InstanceId },
}

impl JointRef {
    /// The instance this reference is attached to (None for assembly connectors, which
    /// are looked up in the assembly).
    pub fn instance(&self) -> Option<InstanceId> {
        match self {
            JointRef::PartConnector { instance, .. }
            | JointRef::Geometry { instance, .. }
            | JointRef::Frame { instance, .. }
            | JointRef::Origin { instance } => Some(*instance),
            JointRef::AssemblyConnector(_) => None,
        }
    }
    pub fn geometry(instance: InstanceId, element: &str) -> JointRef {
        JointRef::Geometry { instance, element: element.to_string(), implicit: ImplicitPoint::Center }
    }
    /// FreeCAD's two-name selection ("Face6", "Vertex7").
    pub fn picked(instance: InstanceId, element: &str, point: &str) -> JointRef {
        let implicit = if point == element || point.is_empty() {
            ImplicitPoint::Center
        } else if point.starts_with("Vertex") {
            ImplicitPoint::Vertex(point.to_string())
        } else {
            ImplicitPoint::EdgeCenter(point.to_string())
        };
        JointRef::Geometry { instance, element: element.to_string(), implicit }
    }
    pub fn frame(instance: InstanceId, local: Frame) -> JointRef {
        JointRef::Frame { instance, local }
    }
}

/// Which pose of J relative to I the orientation equations aim at.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Sense {
    /// The root nearest to the placements at solve start (Ondsel/FreeCAD behaviour).
    #[default]
    Auto,
    Aligned,
    /// J half-turned about its X axis (FreeCAD "Reverse", Onshape "flip primary axis").
    Flipped,
}

/// FreeCAD's Distance joint variants (`AssemblyObject::makeMbdJointDistance`).
/// `Auto` infers the variant from the referenced geometry like FreeCAD's
/// `getDistanceType`; radii of circles/cylinders/spheres are added as FreeCAD does.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum DistanceMode {
    #[default]
    Auto,
    /// `|rIJ| = d` (SphSphJoint); `d = 0` becomes a Ball joint.
    PointPoint,
    /// `rIJ·zI = d`, I = the plane (PointInPlaneJoint).
    PointPlane,
    /// `rIJ·zI = d` and `zI ∥ zJ` (PlanarJoint with offset).
    PlanePlane,
    /// Distance of J's origin from I's Z axis = d (CylSphJoint); `d = 0` → point on line.
    PointLine,
    /// `rIJ·zI = d` and `zI·zJ = 0` (LineInPlaneJoint), I = the plane, J = the line.
    LinePlane,
    /// Parallel lines at distance d: `zI ∥ zJ` and distance of J's origin from I's Z axis (RevCylJoint).
    LineLine,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum JointKind {
    #[serde(alias = "Fixed")]
    Fastened,
    Revolute,
    Cylindrical,
    Slider,
    PinSlot,
    Planar,
    Ball,
    Parallel,
    Perpendicular,
    /// Angle between the Z axes, degrees.
    Angle { angle: f64 },
    Distance { distance: f64, #[serde(default)] mode: DistanceMode },
    RackPinion { pitch_radius: f64 },
    Screw { pitch: f64 },
    Gears { radius1: f64, radius2: f64 },
    Belt { radius1: f64, radius2: f64 },
}

impl JointKind {
    /// FreeCAD's joint type name (Onshape names for the Onshape-only mates).
    pub fn type_name(&self) -> &'static str {
        match self {
            JointKind::Fastened => "Fixed",
            JointKind::Revolute => "Revolute",
            JointKind::Cylindrical => "Cylindrical",
            JointKind::Slider => "Slider",
            JointKind::PinSlot => "PinSlot",
            JointKind::Planar => "Planar",
            JointKind::Ball => "Ball",
            JointKind::Parallel => "Parallel",
            JointKind::Perpendicular => "Perpendicular",
            JointKind::Angle { .. } => "Angle",
            JointKind::Distance { .. } => "Distance",
            JointKind::RackPinion { .. } => "RackPinion",
            JointKind::Screw { .. } => "Screw",
            JointKind::Gears { .. } => "Gears",
            JointKind::Belt { .. } => "Belt",
        }
    }

    /// Number of scalar equations for kinds that do not depend on geometry
    /// (Distance in `Auto` mode needs the references; see [`DistanceMode`]).
    pub fn equation_count(&self) -> usize {
        match self {
            JointKind::Fastened => 6,
            JointKind::Revolute | JointKind::Slider => 5,
            JointKind::Cylindrical | JointKind::PinSlot => 4,
            JointKind::Planar | JointKind::Ball => 3,
            JointKind::Parallel => 2,
            JointKind::Angle { angle } => {
                if is_zero_angle(*angle) {
                    2
                } else {
                    1
                }
            }
            JointKind::Distance { distance, mode } => match mode {
                DistanceMode::PointPoint if distance.abs() < 1e-7 => 3,
                DistanceMode::PointLine if distance.abs() < 1e-7 => 2,
                DistanceMode::PlanePlane | DistanceMode::LineLine => 3,
                DistanceMode::LinePlane => 2,
                _ => 1,
            },
            _ => 1,
        }
    }

    /// Joints whose solve should be preceded by FreeCAD's "match JCS" pre-solve.
    pub fn uses_presolve(&self) -> bool {
        matches!(
            self,
            JointKind::Fastened | JointKind::Revolute | JointKind::Cylindrical | JointKind::Slider | JointKind::Ball
                | JointKind::PinSlot | JointKind::Planar
        )
    }
}

pub(crate) fn is_zero_angle(deg: f64) -> bool {
    let a = deg.to_radians().abs() % (2.0 * std::f64::consts::PI);
    a < 1e-7 || (2.0 * std::f64::consts::PI - a) < 1e-7
}

/// Inequality limits (FreeCAD's LengthMin/Max, AngleMin/Max). Angles in degrees.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Limits {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub length_min: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub length_max: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub angle_min: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub angle_max: Option<f64>,
}

impl Limits {
    pub fn is_empty(&self) -> bool {
        self.length_min.is_none() && self.length_max.is_none() && self.angle_min.is_none() && self.angle_max.is_none()
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Joint {
    pub id: JointId,
    pub name: String,
    pub kind: JointKind,
    pub a: JointRef,
    pub b: JointRef,
    #[serde(default, skip_serializing_if = "is_identity")]
    pub offset_a: Frame,
    #[serde(default, skip_serializing_if = "is_identity")]
    pub offset_b: Frame,
    #[serde(default)]
    pub sense: Sense,
    #[serde(default, skip_serializing_if = "Limits::is_empty")]
    pub limits: Limits,
    #[serde(default)]
    pub suppressed: bool,
}

fn is_identity(f: &Frame) -> bool {
    *f == Frame::IDENTITY
}

impl Joint {
    pub fn new(kind: JointKind, a: JointRef, b: JointRef) -> Joint {
        Joint {
            id: 0,
            name: kind.type_name().to_string(),
            kind,
            a,
            b,
            offset_a: Frame::IDENTITY,
            offset_b: Frame::IDENTITY,
            sense: Sense::Auto,
            limits: Limits::default(),
            suppressed: false,
        }
    }
    /// FreeCAD's "Offset" field: J moved along its Z axis.
    pub fn with_offset(mut self, distance: f64) -> Joint {
        self.offset_b.pos = [self.offset_b.pos[0], self.offset_b.pos[1], distance];
        self
    }
    /// FreeCAD's "Rotation" field: J turned about its Z axis (degrees).
    pub fn with_rotation(mut self, degrees: f64) -> Joint {
        self.offset_b.rot = Frame::rotation([0.0, 0.0, 1.0], degrees.to_radians()).rot;
        self
    }
    pub fn with_sense(mut self, sense: Sense) -> Joint {
        self.sense = sense;
        self
    }
    pub fn with_limits(mut self, limits: Limits) -> Joint {
        self.limits = limits;
        self
    }
}

/// Resolve `DistanceMode::Auto` from the reference kinds (FreeCAD's `getDistanceType`).
/// Returns the mode, whether the sides must be swapped so that I is the plane/line, and
/// the extra distance contributed by radii.
pub(crate) fn distance_mode(mode: DistanceMode, ka: RefKind, kb: RefKind) -> (DistanceMode, bool, f64) {
    use RefKind::*;
    if mode != DistanceMode::Auto {
        return (mode, false, 0.0);
    }
    let r = |k: RefKind| match k {
        Circle { radius } | Cylinder { radius } | Sphere { radius } => radius,
        _ => 0.0,
    };
    let rank = |k: RefKind| match k {
        Plane | Frame | Torus | Surface => 0,
        Line | Cylinder { .. } | Circle { .. } | Cone => 1,
        Curve => 2,
        Point | Sphere { .. } => 3,
    };
    // Order so that I is the "bigger" entity (plane > line > point).
    let swap = rank(ka) > rank(kb);
    let (k1, k2) = if swap { (kb, ka) } else { (ka, kb) };
    let mode = match (k1, k2) {
        (Point, Point) | (Point, Sphere { .. }) | (Sphere { .. }, Sphere { .. }) | (Sphere { .. }, Point) => DistanceMode::PointPoint,
        (Line, Line) | (Line, Circle { .. }) | (Circle { .. }, Circle { .. }) | (Circle { .. }, Line)
        | (Cylinder { .. }, Cylinder { .. }) | (Cylinder { .. }, Torus) | (Line, Cylinder { .. })
        | (Cylinder { .. }, Line) | (Cylinder { .. }, Circle { .. }) | (Circle { .. }, Cylinder { .. }) => DistanceMode::LineLine,
        (Line, Point) | (Cylinder { .. }, Point) | (Cylinder { .. }, Sphere { .. }) | (Circle { .. }, Point)
        | (Line, Sphere { .. }) | (Circle { .. }, Sphere { .. }) => DistanceMode::PointLine,
        (Plane, Line) | (Plane, Cylinder { .. }) => DistanceMode::LinePlane,
        (Plane, Point) | (Plane, Sphere { .. }) | (Plane, Curve) | (Plane, Circle { .. }) => DistanceMode::PointPlane,
        (Curve, Point) => DistanceMode::PointPlane,
        _ => DistanceMode::PlanePlane,
    };
    (mode, swap, r(ka) + r(kb))
}
