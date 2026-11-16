#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::indexing_slicing
    )
)]

use brep_geom::{Curve3D, GeomError, PCurve2D, Surface, SurfaceKind};
use brep_math::{Aabb3, Vec3};
use serde::{Deserialize, Serialize};
use slotmap::{new_key_type, SlotMap};
use std::collections::HashMap;
use std::fmt;

new_key_type! {
    pub struct VertexId;
    pub struct EdgeId;
    pub struct CoedgeId;
    pub struct LoopId;
    pub struct FaceId;
    pub struct ShellId;
    pub struct SolidId;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct OriginTag {
    pub feature_id: u64,
    pub source_id: u64,
    pub role: u16,
}

#[derive(Debug, Clone, PartialEq)]
pub enum TopoError {
    MissingEntity(&'static str),
    Geom(GeomError),
    InvalidLoopCycle { loop_id: String },
    NonManifoldEdge { edge_id: String, coedge_count: usize },
    MissingTwin { coedge_id: String },
    AsymmetricTwin { coedge_id: String },
    DirectionMismatch { coedge_id: String },
    EulerViolation { lhs: isize, rhs: isize },
    ToleranceExceeded { what: &'static str, dist: f64, limit: f64 },
    InvalidVolume { volume: f64 },
}

impl From<GeomError> for TopoError {
    fn from(value: GeomError) -> Self { Self::Geom(value) }
}

impl fmt::Display for TopoError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingEntity(k) => write!(f, "brak encji: {k}"),
            Self::Geom(e) => write!(f, "bĹ‚Ä…d geometrii: {e}"),
            Self::InvalidLoopCycle { loop_id } => write!(f, "pÄ™tla {loop_id} nie jest zamkniÄ™ta"),
            Self::NonManifoldEdge { edge_id, coedge_count } => write!(f, "krawÄ™dĹş {edge_id} ma {coedge_count} pĂłĹ‚krawÄ™dzi"),
            Self::MissingTwin { coedge_id } => write!(f, "coedge {coedge_id} bez twin"),
            Self::AsymmetricTwin { coedge_id } => write!(f, "asymetryczny twin: {coedge_id}"),
            Self::DirectionMismatch { coedge_id } => write!(f, "zĹ‚y kierunek twin: {coedge_id}"),
            Self::EulerViolation { lhs, rhs } => write!(f, "Euler: {lhs} != {rhs}"),
            Self::ToleranceExceeded { what, dist, limit } => write!(f, "tolerancja ({what}): {dist:.3e} > {limit:.3e}"),
            Self::InvalidVolume { volume } => write!(f, "niepoprawna objÄ™toĹ›Ä‡: {volume}"),
        }
    }
}

impl std::error::Error for TopoError {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Vertex { pub pos: Vec3, pub tolerance: f64, pub tag: Option<OriginTag> }

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Edge {
    pub curve: Curve3D,
    pub start_vertex: VertexId,
    pub end_vertex: VertexId,
    pub tolerance: f64,
    pub coedges: [Option<CoedgeId>; 2],
    pub tag: Option<OriginTag>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Coedge {
    pub origin: VertexId,
    pub edge: EdgeId,
    pub forward: bool,
    pub pcurve: PCurve2D,
    pub twin: Option<CoedgeId>,
    pub next: Option<CoedgeId>,
    pub prev: Option<CoedgeId>,
    pub loop_id: LoopId,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Loop { pub face: FaceId, pub first_coedge: Option<CoedgeId>, pub is_outer: bool }

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Face { pub surface: Surface, pub outer_loop: Option<LoopId>, pub inner_loops: Vec<LoopId>, pub tag: Option<OriginTag> }

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Shell { pub faces: Vec<FaceId> }

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Solid { pub shells: Vec<ShellId> }

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ValidationReport {
    pub vertices: usize,
    pub edges: usize,
    pub faces: usize,
    pub inner_loops: usize,
    pub shells: usize,
    pub approx_volume: f64,
    pub surface_histogram: HashMap<SurfaceKind, usize>,
    pub bbox: Option<Aabb3>,
}

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct BRepModel {
    pub vertices: SlotMap<VertexId, Vertex>,
    pub edges: SlotMap<EdgeId, Edge>,
    pub coedges: SlotMap<CoedgeId, Coedge>,
    pub loops: SlotMap<LoopId, Loop>,
    pub faces: SlotMap<FaceId, Face>,
    pub shells: SlotMap<ShellId, Shell>,
    pub solids: SlotMap<SolidId, Solid>,
}

impl BRepModel {
    pub fn new() -> Self { Self::default() }

    pub fn add_vertex(&mut self, pos: Vec3, tolerance: f64, tag: Option<OriginTag>) -> VertexId {
        self.vertices.insert(Vertex { pos, tolerance, tag })
    }

    pub fn add_edge(&mut self, curve: Curve3D, start_vertex: VertexId, end_vertex: VertexId, tolerance: f64, tag: Option<OriginTag>) -> EdgeId {
        self.edges.insert(Edge { curve, start_vertex, end_vertex, tolerance, coedges: [None, None], tag })
    }

    pub fn iter_loop_coedges(&self, loop_id: LoopId) -> Result<Vec<CoedgeId>, TopoError> {
        let lp = self.loops.get(loop_id).ok_or(TopoError::MissingEntity("Loop"))?;
        let Some(start) = lp.first_coedge else { return Ok(Vec::new()); };
        let mut out = Vec::new();
        let mut curr = start;
        for _ in 0..100_000 {
            out.push(curr);
            let ce = self.coedges.get(curr).ok_or(TopoError::MissingEntity("Coedge"))?;
            curr = ce.next.ok_or_else(|| TopoError::InvalidLoopCycle { loop_id: format!("{loop_id:?}") })?;
            if curr == start { return Ok(out); }
        }
        Err(TopoError::InvalidLoopCycle { loop_id: format!("{loop_id:?}") })
    }

    pub fn bind_twins(&mut self, a: CoedgeId, b: CoedgeId) -> Result<(), TopoError> {
        let (edge_a, fwd_a) = {
            let ca = self.coedges.get(a).ok_or(TopoError::MissingEntity("Coedge A"))?;
            (ca.edge, ca.forward)
        };
        let (edge_b, fwd_b) = {
            let cb = self.coedges.get(b).ok_or(TopoError::MissingEntity("Coedge B"))?;
            (cb.edge, cb.forward)
        };
        if edge_a != edge_b || fwd_a == fwd_b {
            return Err(TopoError::DirectionMismatch { coedge_id: format!("{a:?}") });
        }
        if let Some(ca) = self.coedges.get_mut(a) { ca.twin = Some(b); }
        if let Some(cb) = self.coedges.get_mut(b) { cb.twin = Some(a); }
        if let Some(edge) = self.edges.get_mut(edge_a) { edge.coedges = [Some(a), Some(b)]; }
        Ok(())
    }

    pub fn validate_solid(&self, solid_id: SolidId, expected_genus: isize) -> Result<ValidationReport, TopoError> {
        let solid = self.solids.get(solid_id).ok_or(TopoError::MissingEntity("Solid"))?;
        let mut face_ids = Vec::new();
        for &sh_id in &solid.shells {
            let sh = self.shells.get(sh_id).ok_or(TopoError::MissingEntity("Shell"))?;
            face_ids.extend_from_slice(&sh.faces);
        }

        let mut loop_ids = Vec::new();
        let mut inner_loops = 0usize;
        let mut surface_histogram: HashMap<SurfaceKind, usize> = HashMap::new();
        for &f_id in &face_ids {
            let f = self.faces.get(f_id).ok_or(TopoError::MissingEntity("Face"))?;
            *surface_histogram.entry(f.surface.kind()).or_insert(0) += 1;
            let ol = f.outer_loop.ok_or(TopoError::MissingEntity("Face outer_loop"))?;
            loop_ids.push(ol);
            for &il in &f.inner_loops {
                loop_ids.push(il);
                inner_loops += 1;
            }
        }

        let mut coedge_ids = Vec::new();
        for &l_id in &loop_ids {
            for &ce_id in &self.iter_loop_coedges(l_id)? {
                let ce = self.coedges.get(ce_id).ok_or(TopoError::MissingEntity("Coedge"))?;
                let next = ce.next.ok_or(TopoError::MissingEntity("Coedge.next"))?;
                let next_ce = self.coedges.get(next).ok_or(TopoError::MissingEntity("Next Coedge"))?;
                if next_ce.prev != Some(ce_id) {
                    return Err(TopoError::InvalidLoopCycle { loop_id: format!("{l_id:?}") });
                }
                coedge_ids.push(ce_id);
            }
        }

        let mut edge_counts: HashMap<EdgeId, usize> = HashMap::new();
        let mut unique_vertices: HashMap<VertexId, ()> = HashMap::new();
        let mut sample_points = Vec::new();

        for &ce_id in &coedge_ids {
            let ce = self.coedges.get(ce_id).ok_or(TopoError::MissingEntity("Coedge"))?;
            unique_vertices.insert(ce.origin, ());
            *edge_counts.entry(ce.edge).or_insert(0) += 1;

            let twin_id = ce.twin.ok_or_else(|| TopoError::MissingTwin { coedge_id: format!("{ce_id:?}") })?;
            let twin = self.coedges.get(twin_id).ok_or(TopoError::MissingEntity("Twin"))?;
            if twin.twin != Some(ce_id) || twin_id == ce_id {
                return Err(TopoError::AsymmetricTwin { coedge_id: format!("{ce_id:?}") });
            }
            if ce.edge != twin.edge || ce.forward == twin.forward {
                return Err(TopoError::DirectionMismatch { coedge_id: format!("{ce_id:?}") });
            }

            let edge = self.edges.get(ce.edge).ok_or(TopoError::MissingEntity("Edge"))?;
            let lp = self.loops.get(ce.loop_id).ok_or(TopoError::MissingEntity("Loop"))?;
            let face = self.faces.get(lp.face).ok_or(TopoError::MissingEntity("Face"))?;

            for step in 0..=8 {
                let s = f64::from(step) / 8.0;
                let p_3d = edge.curve.eval_normalized(s, ce.forward)?;
                let uv = ce.pcurve.eval(s);
                let p_surf = face.surface.eval(uv.x, uv.y);
                let dist = p_3d.sub(p_surf).norm();
                if dist > edge.tolerance {
                    return Err(TopoError::ToleranceExceeded { what: "3D curve vs S(pcurve)", dist, limit: edge.tolerance });
                }
                sample_points.push(p_3d);
            }

            let v_origin = self.vertices.get(ce.origin).ok_or(TopoError::MissingEntity("Vertex"))?;
            let d_start = edge.curve.eval_normalized(0.0, ce.forward)?.sub(v_origin.pos).norm();
            if d_start > v_origin.tolerance {
                return Err(TopoError::ToleranceExceeded { what: "Vertex vs Edge start", dist: d_start, limit: v_origin.tolerance });
            }
        }

        for (&edge_id, &cnt) in &edge_counts {
            if cnt != 2 {
                return Err(TopoError::NonManifoldEdge { edge_id: format!("{edge_id:?}"), coedge_count: cnt });
            }
        }

        let lhs = unique_vertices.len() as isize - edge_counts.len() as isize + face_ids.len() as isize - inner_loops as isize;
        let rhs = 2 * (solid.shells.len() as isize - expected_genus);
        if lhs != rhs {
            return Err(TopoError::EulerViolation { lhs, rhs });
        }

        let approx_volume = self.approx_signed_volume(&face_ids)?;
        if approx_volume <= 0.0 {
            return Err(TopoError::InvalidVolume { volume: approx_volume });
        }

        Ok(ValidationReport {
            vertices: unique_vertices.len(),
            edges: edge_counts.len(),
            faces: face_ids.len(),
            inner_loops,
            shells: solid.shells.len(),
            approx_volume,
            surface_histogram,
            bbox: Aabb3::from_points(&sample_points),
        })
    }

    fn approx_signed_volume(&self, face_ids: &[FaceId]) -> Result<f64, TopoError> {
        let mut vol6 = 0.0;
        for &f_id in face_ids {
            let f = self.faces.get(f_id).ok_or(TopoError::MissingEntity("Face"))?;
            match &f.surface {
                Surface::Plane(_) => {
                    let mut all_loops = Vec::new();
                    if let Some(ol) = f.outer_loop { all_loops.push(ol); }
                    all_loops.extend_from_slice(&f.inner_loops);
                    for l_id in all_loops {
                        let mut poly = Vec::new();
                        for ce_id in self.iter_loop_coedges(l_id)? {
                            let ce = self.coedges.get(ce_id).ok_or(TopoError::MissingEntity("Coedge"))?;
                            let edge = self.edges.get(ce.edge).ok_or(TopoError::MissingEntity("Edge"))?;
                            let segs = match edge.curve { Curve3D::Line(_) => 1, _ => 64 };
                            for step in 0..segs {
                                poly.push(edge.curve.eval_normalized(f64::from(step) / f64::from(segs), ce.forward)?);
                            }
                        }
                        if let Some(&p0) = poly.first() {
                            for i in 1..poly.len().saturating_sub(1) {
                                let p1 = poly.get(i).copied().ok_or(TopoError::MissingEntity("p1"))?;
                                let p2 = poly.get(i + 1).copied().ok_or(TopoError::MissingEntity("p2"))?;
                                vol6 += p0.dot(p1.cross(p2));
                            }
                        }
                    }
                }
                Surface::Cylinder(_) => {
                    let ol = f.outer_loop.ok_or(TopoError::MissingEntity("outer_loop"))?;
                    let (mut u_min, mut u_max) = (f64::INFINITY, f64::NEG_INFINITY);
                    let (mut v_min, mut v_max) = (f64::INFINITY, f64::NEG_INFINITY);
                    for ce_id in self.iter_loop_coedges(ol)? {
                        let ce = self.coedges.get(ce_id).ok_or(TopoError::MissingEntity("Coedge"))?;
                        for step in 0..=4 {
                            let uv = ce.pcurve.eval(f64::from(step) / 4.0);
                            u_min = u_min.min(uv.x);
                            u_max = u_max.max(uv.x);
                            v_min = v_min.min(uv.y);
                            v_max = v_max.max(uv.y);
                        }
                    }
                    let nu = 64u32;
                    for i in 0..nu {
                        let u0 = u_min + (u_max - u_min) * (f64::from(i) / f64::from(nu));
                        let u1 = u_min + (u_max - u_min) * (f64::from(i + 1) / f64::from(nu));
                        let p00 = f.surface.eval(u0, v_min);
                        let p10 = f.surface.eval(u1, v_min);
                        let p11 = f.surface.eval(u1, v_max);
                        let p01 = f.surface.eval(u0, v_max);
                        vol6 += p00.dot(p10.cross(p11));
                        vol6 += p00.dot(p11.cross(p01));
                    }
                }
            }
        }
        Ok(vol6 / 6.0)
    }
}