#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::indexing_slicing
    )
)]

use brep_geom::{Circle3, Curve3D, Cylinder, Line3, PCurve2D, Plane, Surface};
use brep_math::{Vec2, Vec3};
use brep_topo::{
    BRepModel, Coedge, CoedgeId, EdgeId, Face, FaceId, Loop, LoopId, OriginTag, Shell, Solid,
    SolidId, TopoError, VertexId,
};
use std::collections::HashMap;
use std::f64::consts::TAU;

const DEFAULT_TOL: f64 = 1e-7;

pub fn make_box(
    model: &mut BRepModel,
    dx: f64,
    dy: f64,
    dz: f64,
    feature_id: u64,
) -> Result<SolidId, TopoError> {
    let coords = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(dx, 0.0, 0.0),
        Vec3::new(dx, dy, 0.0),
        Vec3::new(0.0, dy, 0.0),
        Vec3::new(0.0, 0.0, dz),
        Vec3::new(dx, 0.0, dz),
        Vec3::new(dx, dy, dz),
        Vec3::new(0.0, dy, dz),
    ];
    let mut v = Vec::with_capacity(8);
    for (idx, pos) in coords.into_iter().enumerate() {
        v.push(model.add_vertex(pos, DEFAULT_TOL, Some(OriginTag { feature_id, source_id: idx as u64, role: 1 })));
    }
    let get_v = |i: usize| -> Result<VertexId, TopoError> {
        v.get(i).copied().ok_or(TopoError::MissingEntity("Box vertex"))
    };

    let face_defs: [([usize; 4], Vec3, Vec3); 6] = [
        ([0, 3, 2, 1], Vec3::new(0.0, 1.0, 0.0), Vec3::new(1.0, 0.0, 0.0)),
        ([4, 5, 6, 7], Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0)),
        ([0, 1, 5, 4], Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)),
        ([1, 2, 6, 5], Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)),
        ([2, 3, 7, 6], Vec3::new(-1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)),
        ([3, 0, 4, 7], Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)),
    ];

    let mut edge_map: HashMap<(VertexId, VertexId), EdgeId> = HashMap::new();
    let mut half_map: HashMap<(VertexId, VertexId), CoedgeId> = HashMap::new();
    let mut face_ids = Vec::with_capacity(6);

    for (f_idx, (indices, u_dir, v_dir)) in face_defs.into_iter().enumerate() {
        let i0 = indices.first().copied().ok_or(TopoError::MissingEntity("idx0"))?;
        let p0 = model.vertices.get(get_v(i0)?).ok_or(TopoError::MissingEntity("V0"))?.pos;
        let face_id = insert_face(
            model,
            Surface::Plane(Plane { origin: p0, u_dir, v_dir }),
            Some(OriginTag { feature_id, source_id: f_idx as u64, role: 2 }),
        );
        let loop_id = insert_loop(model, face_id);

        let mut ce_ids = Vec::with_capacity(4);
        for k in 0..4 {
            let ia = indices.get(k).copied().ok_or(TopoError::MissingEntity("ia"))?;
            let ib = indices.get((k + 1) % 4).copied().ok_or(TopoError::MissingEntity("ib"))?;
            let a = get_v(ia)?;
            let b = get_v(ib)?;
            let pa = model.vertices.get(a).ok_or(TopoError::MissingEntity("Va"))?.pos;
            let pb = model.vertices.get(b).ok_or(TopoError::MissingEntity("Vb"))?.pos;

            let (edge_id, forward) = if let Some(&existing) = edge_map.get(&(b, a)) {
                (existing, false)
            } else {
                let e = model.add_edge(Curve3D::Line(Line3 { start: pa, end: pb }), a, b, DEFAULT_TOL, None);
                edge_map.insert((a, b), e);
                (e, true)
            };

            let uv_a = Vec2::new(pa.sub(p0).dot(u_dir), pa.sub(p0).dot(v_dir));
            let uv_b = Vec2::new(pb.sub(p0).dot(u_dir), pb.sub(p0).dot(v_dir));
            let ce_id = model.coedges.insert(Coedge {
                origin: a,
                edge: edge_id,
                forward,
                pcurve: PCurve2D::Line { start: uv_a, end: uv_b },
                twin: None,
                next: None,
                prev: None,
                loop_id,
            });
            if let Some(&twin_id) = half_map.get(&(b, a)) {
                model.bind_twins(twin_id, ce_id)?;
            } else {
                half_map.insert((a, b), ce_id);
            }
            ce_ids.push(ce_id);
        }
        link_loop_cycle(model, loop_id, &ce_ids)?;
        face_ids.push(face_id);
    }

    let shell_id = model.shells.insert(Shell { faces: face_ids });
    Ok(model.solids.insert(Solid { shells: vec![shell_id] }))
}

pub fn make_cylinder(
    model: &mut BRepModel,
    radius: f64,
    height: f64,
    feature_id: u64,
) -> Result<SolidId, TopoError> {
    let v_bot = model.add_vertex(Vec3::new(radius, 0.0, 0.0), DEFAULT_TOL, None);
    let v_top = model.add_vertex(Vec3::new(radius, 0.0, height), DEFAULT_TOL, None);
    let (x_dir, y_dir, z_dir) = (Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0));

    let e_bot = model.add_edge(Curve3D::Circle(Circle3 { center: Vec3::ZERO, x_dir, y_dir, radius, t_start: 0.0, t_end: TAU }), v_bot, v_bot, DEFAULT_TOL, None);
    let e_top = model.add_edge(Curve3D::Circle(Circle3 { center: Vec3::new(0.0, 0.0, height), x_dir, y_dir, radius, t_start: 0.0, t_end: TAU }), v_top, v_top, DEFAULT_TOL, None);
    let e_seam = model.add_edge(Curve3D::Line(Line3 { start: Vec3::new(radius, 0.0, 0.0), end: Vec3::new(radius, 0.0, height) }), v_bot, v_top, DEFAULT_TOL, None);

    let f_bot = insert_face(model, Surface::Plane(Plane { origin: Vec3::ZERO, u_dir: x_dir, v_dir: Vec3::new(0.0, -1.0, 0.0) }), Some(OriginTag { feature_id, source_id: 0, role: 2 }));
    let l_bot = insert_loop(model, f_bot);
    let ce_bot_disk = model.coedges.insert(Coedge { origin: v_bot, edge: e_bot, forward: false, pcurve: PCurve2D::Circle { center: Vec2::new(0.0, 0.0), radius, start_angle: 0.0, sweep: TAU }, twin: None, next: None, prev: None, loop_id: l_bot });
    link_loop_cycle(model, l_bot, &[ce_bot_disk])?;

    let f_top = insert_face(model, Surface::Plane(Plane { origin: Vec3::new(0.0, 0.0, height), u_dir: x_dir, v_dir: y_dir }), Some(OriginTag { feature_id, source_id: 1, role: 2 }));
    let l_top = insert_loop(model, f_top);
    let ce_top_disk = model.coedges.insert(Coedge { origin: v_top, edge: e_top, forward: true, pcurve: PCurve2D::Circle { center: Vec2::new(0.0, 0.0), radius, start_angle: 0.0, sweep: TAU }, twin: None, next: None, prev: None, loop_id: l_top });
    link_loop_cycle(model, l_top, &[ce_top_disk])?;

    let f_cyl = insert_face(model, Surface::Cylinder(Cylinder { origin: Vec3::ZERO, axis: z_dir, x_dir, y_dir, radius }), Some(OriginTag { feature_id, source_id: 2, role: 2 }));
    let l_cyl = insert_loop(model, f_cyl);
    let ce_cyl_bot = model.coedges.insert(Coedge { origin: v_bot, edge: e_bot, forward: true, pcurve: PCurve2D::Line { start: Vec2::new(0.0, 0.0), end: Vec2::new(TAU, 0.0) }, twin: None, next: None, prev: None, loop_id: l_cyl });
    let ce_cyl_seam_up = model.coedges.insert(Coedge { origin: v_bot, edge: e_seam, forward: true, pcurve: PCurve2D::Line { start: Vec2::new(TAU, 0.0), end: Vec2::new(TAU, height) }, twin: None, next: None, prev: None, loop_id: l_cyl });
    let ce_cyl_top = model.coedges.insert(Coedge { origin: v_top, edge: e_top, forward: false, pcurve: PCurve2D::Line { start: Vec2::new(TAU, height), end: Vec2::new(0.0, height) }, twin: None, next: None, prev: None, loop_id: l_cyl });
    let ce_cyl_seam_down = model.coedges.insert(Coedge { origin: v_top, edge: e_seam, forward: false, pcurve: PCurve2D::Line { start: Vec2::new(0.0, height), end: Vec2::new(0.0, 0.0) }, twin: None, next: None, prev: None, loop_id: l_cyl });
    link_loop_cycle(model, l_cyl, &[ce_cyl_bot, ce_cyl_seam_up, ce_cyl_top, ce_cyl_seam_down])?;

    model.bind_twins(ce_bot_disk, ce_cyl_bot)?;
    model.bind_twins(ce_top_disk, ce_cyl_top)?;
    model.bind_twins(ce_cyl_seam_up, ce_cyl_seam_down)?;

    let shell_id = model.shells.insert(Shell { faces: vec![f_bot, f_top, f_cyl] });
    Ok(model.solids.insert(Solid { shells: vec![shell_id] }))
}

fn insert_face(model: &mut BRepModel, surface: Surface, tag: Option<OriginTag>) -> FaceId {
    model.faces.insert(Face { surface, outer_loop: None, inner_loops: Vec::new(), tag })
}

fn insert_loop(model: &mut BRepModel, face: FaceId) -> LoopId {
    let loop_id = model.loops.insert(Loop { face, first_coedge: None, is_outer: true });
    if let Some(f) = model.faces.get_mut(face) { f.outer_loop = Some(loop_id); }
    loop_id
}

fn link_loop_cycle(model: &mut BRepModel, loop_id: LoopId, ce_ids: &[CoedgeId]) -> Result<(), TopoError> {
    let n = ce_ids.len();
    if n == 0 { return Err(TopoError::MissingEntity("Empty loop")); }
    if let Some(lp) = model.loops.get_mut(loop_id) { lp.first_coedge = ce_ids.first().copied(); }
    for i in 0..n {
        let curr = ce_ids.get(i).copied().ok_or(TopoError::MissingEntity("curr"))?;
        let next = ce_ids.get((i + 1) % n).copied().ok_or(TopoError::MissingEntity("next"))?;
        let prev = ce_ids.get((i + n - 1) % n).copied().ok_or(TopoError::MissingEntity("prev"))?;
        if let Some(ce) = model.coedges.get_mut(curr) {
            ce.next = Some(next);
            ce.prev = Some(prev);
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use brep_geom::SurfaceKind;

    #[test]
    fn box_passes_full_brep_validation() {
        let mut model = BRepModel::new();
        let box_id = make_box(&mut model, 10.0, 5.0, 2.0, 1).unwrap();
        let report = model.validate_solid(box_id, 0).unwrap();
        assert_eq!(report.vertices, 8);
        assert_eq!(report.edges, 12);
        assert_eq!(report.faces, 6);
        assert_eq!(report.surface_histogram.get(&SurfaceKind::Plane), Some(&6));
        assert!((report.approx_volume - 100.0).abs() < 1e-9);
    }

    #[test]
    fn cylinder_is_exact_analytic_not_polygon() {
        let mut model = BRepModel::new();
        let (r, h) = (3.0, 7.0);
        let cyl_id = make_cylinder(&mut model, r, h, 2).unwrap();
        let report = model.validate_solid(cyl_id, 0).unwrap();
        assert_eq!(report.vertices, 2);
        assert_eq!(report.edges, 3);
        assert_eq!(report.faces, 3);
        assert_eq!(report.surface_histogram.get(&SurfaceKind::Plane), Some(&2));
        assert_eq!(report.surface_histogram.get(&SurfaceKind::Cylinder), Some(&1));
        let exact_vol = std::f64::consts::PI * r * r * h;
        assert!((report.approx_volume - exact_vol).abs() / exact_vol < 0.01);
    }
}