#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::indexing_slicing
    )
)]

use serde::{Deserialize, Serialize};
use std::fmt;

#[derive(Debug, Clone, PartialEq)]
pub enum MathError {
    ZeroLengthVector,
    InvalidNurbsParameter { msg: &'static str },
    IndexOutOfBounds,
}

impl fmt::Display for MathError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ZeroLengthVector => write!(f, "wektor ma dĹ‚ugoĹ›Ä‡ bliskÄ… zeru"),
            Self::InvalidNurbsParameter { msg } => write!(f, "bĹ‚Ä…d parametrĂłw NURBS: {msg}"),
            Self::IndexOutOfBounds => write!(f, "indeks poza zakresem bufora"),
        }
    }
}

impl std::error::Error for MathError {}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl Vec2 {
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
    pub fn add(self, other: Self) -> Self {
        Self::new(self.x + other.x, self.y + other.y)
    }
    pub fn sub(self, other: Self) -> Self {
        Self::new(self.x - other.x, self.y - other.y)
    }
    pub fn scale(self, s: f64) -> Self {
        Self::new(self.x * s, self.y * s)
    }
    pub fn dot(self, other: Self) -> f64 {
        self.x * other.x + self.y * other.y
    }
    pub fn norm(self) -> f64 {
        self.dot(self).sqrt()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0);

    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
    pub fn add(self, other: Self) -> Self {
        Self::new(self.x + other.x, self.y + other.y, self.z + other.z)
    }
    pub fn sub(self, other: Self) -> Self {
        Self::new(self.x - other.x, self.y - other.y, self.z - other.z)
    }
    pub fn scale(self, s: f64) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }
    pub fn dot(self, other: Self) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }
    pub fn cross(self, other: Self) -> Self {
        Self::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }
    pub fn norm_sq(self) -> f64 {
        self.dot(self)
    }
    pub fn norm(self) -> f64 {
        self.norm_sq().sqrt()
    }
    pub fn normalized(self) -> Result<Self, MathError> {
        let len = self.norm();
        if len <= 1e-15 {
            return Err(MathError::ZeroLengthVector);
        }
        Ok(self.scale(1.0 / len))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Aabb3 {
    pub min: Vec3,
    pub max: Vec3,
}

impl Aabb3 {
    pub fn from_points(points: &[Vec3]) -> Option<Self> {
        let first = points.first().copied()?;
        let mut min = first;
        let mut max = first;
        for p in points.iter().copied().skip(1) {
            min.x = min.x.min(p.x);
            min.y = min.y.min(p.y);
            min.z = min.z.min(p.z);
            max.x = max.x.max(p.x);
            max.y = max.y.max(p.y);
            max.z = max.z.max(p.z);
        }
        Some(Self { min, max })
    }
}

pub fn orient2d(a: Vec2, b: Vec2, c: Vec2) -> f64 {
    robust::orient2d(
        robust::Coord { x: a.x, y: a.y },
        robust::Coord { x: b.x, y: b.y },
        robust::Coord { x: c.x, y: c.y },
    )
}

pub fn orient3d(a: Vec3, b: Vec3, c: Vec3, d: Vec3) -> f64 {
    robust::orient3d(
        robust::Coord3D { x: a.x, y: a.y, z: a.z },
        robust::Coord3D { x: b.x, y: b.y, z: b.z },
        robust::Coord3D { x: c.x, y: c.y, z: c.z },
        robust::Coord3D { x: d.x, y: d.y, z: d.z },
    )
}

pub mod nurbs {
    use super::{MathError, Vec3};

    pub fn find_span(n: usize, degree: usize, u: f64, knots: &[f64]) -> Result<usize, MathError> {
        if knots.len() != n + degree + 2 {
            return Err(MathError::InvalidNurbsParameter {
                msg: "nieprawidĹ‚owa dĹ‚ugoĹ›Ä‡ wektora wÄ™zĹ‚Ăłw",
            });
        }
        let u_max = knots.get(n + 1).copied().ok_or(MathError::IndexOutOfBounds)?;
        let u_min = knots.get(degree).copied().ok_or(MathError::IndexOutOfBounds)?;
        if u >= u_max {
            return Ok(n);
        }
        if u <= u_min {
            return Ok(degree);
        }
        let mut low = degree;
        let mut high = n + 1;
        let mut mid = (low + high) / 2;
        loop {
            let k_mid = knots.get(mid).copied().ok_or(MathError::IndexOutOfBounds)?;
            let k_next = knots.get(mid + 1).copied().ok_or(MathError::IndexOutOfBounds)?;
            if u >= k_mid && u < k_next {
                return Ok(mid);
            }
            if u < k_mid {
                high = mid;
            } else {
                low = mid;
            }
            mid = (low + high) / 2;
        }
    }

    pub fn basis_functions(
        span: usize,
        u: f64,
        degree: usize,
        knots: &[f64],
    ) -> Result<Vec<f64>, MathError> {
        let mut n_vals = vec![0.0; degree + 1];
        let mut left = vec![0.0; degree + 1];
        let mut right = vec![0.0; degree + 1];
        if let Some(first) = n_vals.get_mut(0) {
            *first = 1.0;
        }
        for j in 1..=degree {
            let k_left = knots.get(span + 1 - j).copied().ok_or(MathError::IndexOutOfBounds)?;
            let k_right = knots.get(span + j).copied().ok_or(MathError::IndexOutOfBounds)?;
            if let Some(l) = left.get_mut(j) {
                *l = u - k_left;
            }
            if let Some(r) = right.get_mut(j) {
                *r = k_right - u;
            }
            let mut saved = 0.0;
            for r in 0..j {
                let r_val = right.get(r + 1).copied().ok_or(MathError::IndexOutOfBounds)?;
                let l_val = left.get(j - r).copied().ok_or(MathError::IndexOutOfBounds)?;
                let n_r = n_vals.get(r).copied().ok_or(MathError::IndexOutOfBounds)?;
                let denom = r_val + l_val;
                let temp = if denom.abs() > 1e-15 { n_r / denom } else { 0.0 };
                if let Some(slot) = n_vals.get_mut(r) {
                    *slot = saved + r_val * temp;
                }
                saved = l_val * temp;
            }
            if let Some(slot) = n_vals.get_mut(j) {
                *slot = saved;
            }
        }
        Ok(n_vals)
    }

    pub fn eval_rational_curve(
        degree: usize,
        knots: &[f64],
        control_points: &[Vec3],
        weights: &[f64],
        u: f64,
    ) -> Result<Vec3, MathError> {
        if control_points.is_empty() || control_points.len() != weights.len() {
            return Err(MathError::InvalidNurbsParameter {
                msg: "niezgodna liczba punktĂłw kontrolnych i wag",
            });
        }
        let n = control_points.len() - 1;
        if n < degree {
            return Err(MathError::InvalidNurbsParameter {
                msg: "stopieĹ„ krzywej wiÄ™kszy niĹĽ liczba punktĂłw - 1",
            });
        }
        let span = find_span(n, degree, u, knots)?;
        let basis = basis_functions(span, u, degree, knots)?;
        let (mut wx, mut wy, mut wz, mut w_sum) = (0.0, 0.0, 0.0, 0.0);
        for (j, &b) in basis.iter().enumerate() {
            let idx = span - degree + j;
            let p = control_points.get(idx).copied().ok_or(MathError::IndexOutOfBounds)?;
            let w = weights.get(idx).copied().ok_or(MathError::IndexOutOfBounds)?;
            let bw = b * w;
            wx += p.x * bw;
            wy += p.y * bw;
            wz += p.z * bw;
            w_sum += bw;
        }
        if w_sum.abs() <= 1e-15 {
            return Err(MathError::InvalidNurbsParameter {
                msg: "suma wag bliska zeru",
            });
        }
        Ok(Vec3::new(wx / w_sum, wy / w_sum, wz / w_sum))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_orient2d_handles_near_collinear() {
        let a = Vec2::new(0.0, 0.0);
        let b = Vec2::new(1.0, 1.0);
        let c_left = Vec2::new(0.5, 0.5 + 1e-15);
        assert!(orient2d(a, b, c_left) > 0.0);
    }

    #[test]
    fn nurbs_quarter_circle_exact() {
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let pts = [
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(1.0, 1.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        ];
        let weights = [1.0, w, 1.0];
        let knots = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        for i in 0..=10 {
            let u = f64::from(i) / 10.0;
            let p = nurbs::eval_rational_curve(2, &knots, &pts, &weights, u).unwrap();
            assert!((p.norm() - 1.0).abs() < 1e-12);
        }
    }
}