#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::indexing_slicing
    )
)]

use brep_math::{nurbs, MathError, Vec2, Vec3};
use serde::{Deserialize, Serialize};
use std::fmt;

#[derive(Debug, Clone, PartialEq)]
pub enum GeomError {
    Math(MathError),
    DegenerateGeometry(&'static str),
}

impl From<MathError> for GeomError {
    fn from(value: MathError) -> Self {
        Self::Math(value)
    }
}

impl fmt::Display for GeomError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Math(e) => write!(f, "bĹ‚Ä…d matematyczny: {e}"),
            Self::DegenerateGeometry(msg) => write!(f, "zdegenerowana geometria: {msg}"),
        }
    }
}

impl std::error::Error for GeomError {}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SurfaceKind {
    Plane,
    Cylinder,
    Nurbs,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum CanonicalSurface {
    Plane { origin: Vec3, normal: Vec3 },
    Cylinder { origin: Vec3, axis: Vec3, radius: f64 },
    Freeform,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Line3 {
    pub start: Vec3,
    pub end: Vec3,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Circle3 {
    pub center: Vec3,
    pub x_dir: Vec3,
    pub y_dir: Vec3,
    pub radius: f64,
    pub t_start: f64,
    pub t_end: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NurbsCurve3 {
    pub degree: usize,
    pub knots: Vec<f64>,
    pub control_points: Vec<Vec3>,
    pub weights: Vec<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Curve3D {
    Line(Line3),
    Circle(Circle3),
    Nurbs(NurbsCurve3),
}

impl Curve3D {
    pub fn domain(&self) -> Result<(f64, f64), GeomError> {
        match self {
            Self::Line(_) => Ok((0.0, 1.0)),
            Self::Circle(c) => Ok((c.t_start, c.t_end)),
            Self::Nurbs(n) => {
                let first = n.knots.get(n.degree).copied().ok_or(GeomError::DegenerateGeometry("brak wÄ™zĹ‚a startowego"))?;
                let last = n.knots.get(n.control_points.len()).copied().ok_or(GeomError::DegenerateGeometry("brak wÄ™zĹ‚a koĹ„cowego"))?;
                Ok((first, last))
            }
        }
    }

    pub fn eval(&self, t: f64) -> Result<Vec3, GeomError> {
        match self {
            Self::Line(l) => Ok(l.start.add(l.end.sub(l.start).scale(t))),
            Self::Circle(c) => Ok(c.center.add(c.x_dir.scale(c.radius * t.cos())).add(c.y_dir.scale(c.radius * t.sin()))),
            Self::Nurbs(n) => Ok(nurbs::eval_rational_curve(n.degree, &n.knots, &n.control_points, &n.weights, t)?),
        }
    }

    pub fn eval_normalized(&self, s: f64, forward: bool) -> Result<Vec3, GeomError> {
        let (t0, t1) = self.domain()?;
        let t = if forward { t0 + (t1 - t0) * s } else { t1 - (t1 - t0) * s };
        self.eval(t)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum PCurve2D {
    Line { start: Vec2, end: Vec2 },
    Circle { center: Vec2, radius: f64, start_angle: f64, sweep: f64 },
}

impl PCurve2D {
    pub fn eval(&self, s: f64) -> Vec2 {
        match self {
            Self::Line { start, end } => start.add(end.sub(*start).scale(s)),
            Self::Circle { center, radius, start_angle, sweep } => {
                let a = start_angle + sweep * s;
                Vec2::new(center.x + radius * a.cos(), center.y + radius * a.sin())
            }
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Plane {
    pub origin: Vec3,
    pub u_dir: Vec3,
    pub v_dir: Vec3,
}

impl Plane {
    pub fn normal(&self) -> Result<Vec3, GeomError> {
        Ok(self.u_dir.cross(self.v_dir).normalized()?)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Cylinder {
    pub origin: Vec3,
    pub axis: Vec3,
    pub x_dir: Vec3,
    pub y_dir: Vec3,
    pub radius: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Surface {
    Plane(Plane),
    Cylinder(Cylinder),
}

impl Surface {
    pub const fn kind(&self) -> SurfaceKind {
        match self {
            Self::Plane(_) => SurfaceKind::Plane,
            Self::Cylinder(_) => SurfaceKind::Cylinder,
        }
    }

    pub fn eval(&self, u: f64, v: f64) -> Vec3 {
        match self {
            Self::Plane(p) => p.origin.add(p.u_dir.scale(u)).add(p.v_dir.scale(v)),
            Self::Cylinder(c) => c.origin.add(c.x_dir.scale(c.radius * u.cos())).add(c.y_dir.scale(c.radius * u.sin())).add(c.axis.scale(v)),
        }
    }

    pub fn partials(&self, u: f64, _v: f64) -> (Vec3, Vec3) {
        match self {
            Self::Plane(p) => (p.u_dir, p.v_dir),
            Self::Cylinder(c) => (
                c.x_dir.scale(-c.radius * u.sin()).add(c.y_dir.scale(c.radius * u.cos())),
                c.axis,
            ),
        }
    }

    pub fn normal(&self, u: f64, v: f64) -> Result<Vec3, GeomError> {
        let (du, dv) = self.partials(u, v);
        Ok(du.cross(dv).normalized()?)
    }

    pub fn canonical_info(&self) -> Result<CanonicalSurface, GeomError> {
        match self {
            Self::Plane(p) => Ok(CanonicalSurface::Plane { origin: p.origin, normal: p.normal()? }),
            Self::Cylinder(c) => Ok(CanonicalSurface::Cylinder { origin: c.origin, axis: c.axis.normalized()?, radius: c.radius }),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cylinder_partials_match_finite_difference() {
        let cyl = Surface::Cylinder(Cylinder {
            origin: Vec3::ZERO,
            axis: Vec3::new(0.0, 0.0, 1.0),
            x_dir: Vec3::new(1.0, 0.0, 0.0),
            y_dir: Vec3::new(0.0, 1.0, 0.0),
            radius: 3.5,
        });
        let (u, v) = (1.1, 2.5);
        let h = 1e-6;
        let (du, dv) = cyl.partials(u, v);
        let du_num = cyl.eval(u + h, v).sub(cyl.eval(u - h, v)).scale(0.5 / h);
        let dv_num = cyl.eval(u, v + h).sub(cyl.eval(u, v - h)).scale(0.5 / h);
        assert!(du.sub(du_num).norm() < 1e-7);
        assert!(dv.sub(dv_num).norm() < 1e-7);
    }
}