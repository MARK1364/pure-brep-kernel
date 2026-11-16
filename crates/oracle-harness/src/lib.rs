#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::indexing_slicing
    )
)]

use brep_geom::SurfaceKind;
use brep_ops::{make_box, make_cylinder};
use brep_topo::BRepModel;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "op", rename_all = "snake_case")]
pub enum TestCase {
    Box { dx: f64, dy: f64, dz: f64 },
    Cylinder { radius: f64, height: f64 },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OracleReport {
    pub kernel: String,
    pub is_valid: bool,
    pub volume: f64,
    pub has_plane: bool,
    pub has_cylinder: bool,
    pub error: Option<String>,
}

pub fn run_ours(case: &TestCase) -> OracleReport {
    let mut model = BRepModel::new();
    let solid_res = match case {
        TestCase::Box { dx, dy, dz } => make_box(&mut model, *dx, *dy, *dz, 1),
        TestCase::Cylinder { radius, height } => make_cylinder(&mut model, *radius, *height, 1),
    };
    let solid_id = match solid_res {
        Ok(id) => id,
        Err(e) => return OracleReport { kernel: "ours".to_string(), is_valid: false, volume: 0.0, has_plane: false, has_cylinder: false, error: Some(e.to_string()) },
    };
    match model.validate_solid(solid_id, 0) {
        Ok(rep) => OracleReport {
            kernel: "ours".to_string(),
            is_valid: true,
            volume: rep.approx_volume,
            has_plane: rep.surface_histogram.contains_key(&SurfaceKind::Plane),
            has_cylinder: rep.surface_histogram.contains_key(&SurfaceKind::Cylinder),
            error: None,
        },
        Err(e) => OracleReport { kernel: "ours".to_string(), is_valid: false, volume: 0.0, has_plane: false, has_cylinder: false, error: Some(e.to_string()) },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn oracle_report_serializes_cleanly() {
        let rep = run_ours(&TestCase::Cylinder { radius: 2.0, height: 5.0 });
        assert!(rep.is_valid && rep.has_plane && rep.has_cylinder);
        let json = serde_json::to_string_pretty(&rep).unwrap();
        assert!(json.contains("\"has_cylinder\": true"));
    }
}