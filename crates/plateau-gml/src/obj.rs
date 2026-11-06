//! PLATEAU の OBJ 書き出し（PLATEAU SDK for Unity などが出力する `bldg_*` / `tran_*` グループ付き OBJ）の読込。
//!
//! OBJ は局所直交座標 [m] なので、原点の緯度経度と軸の向きを [`ObjFrame`] で与えて
//! CityGML と同じ [`CityModel`] に変換する。
//! - 建物 (`g bldg_…`): LOD1 立体。最も低い面を底面（フットプリント）とし、高さは立体の高さ。
//!   立体の三角形は描画用に [`Feature::solid`] に保持する（高さは底面からの相対値）。
//! - 道路 (`g tran_…`): 路面の三角形群。

use crate::{CityModel, Feature, FeatureKind, GeoPoint};
use anyhow::{Context, Result};
use std::io::BufRead;
use std::path::Path;

const EARTH_R: f64 = 6_378_137.0;

/// OBJ の局所座標系。`x_east` / `z_north` は軸の符号（+1 か -1）。y は上向きとする。
#[derive(Debug, Clone, Copy)]
pub struct ObjFrame {
    pub lat0: f64,
    pub lon0: f64,
    pub x_east: f64,
    pub z_north: f64,
}

impl ObjFrame {
    /// PLATEAU SDK for Unity の書き出し: x が西向き（左手系→右手系変換で反転）、z が北向き。
    pub fn unity_export(lat0: f64, lon0: f64) -> Self {
        Self {
            lat0,
            lon0,
            x_east: -1.0,
            z_north: 1.0,
        }
    }
    fn to_geo(&self, x: f64, y: f64, z: f64) -> GeoPoint {
        let east = x * self.x_east;
        let north = z * self.z_north;
        GeoPoint {
            lat: self.lat0 + (north / EARTH_R).to_degrees(),
            lon: self.lon0 + (east / (EARTH_R * self.lat0.to_radians().cos())).to_degrees(),
            h: y,
        }
    }
}

pub fn parse_file(path: impl AsRef<Path>, frame: ObjFrame) -> Result<CityModel> {
    let path = path.as_ref();
    let f = std::fs::File::open(path).with_context(|| format!("open {}", path.display()))?;
    parse_reader(std::io::BufReader::new(f), frame)
}

pub fn parse_reader<R: BufRead>(src: R, frame: ObjFrame) -> Result<CityModel> {
    let mut model = CityModel::default();
    let mut verts: Vec<[f64; 3]> = vec![];
    let mut group: Option<(FeatureKind, String)> = None;
    let mut tris: Vec<[usize; 3]> = vec![];

    let mut flush =
        |group: &Option<(FeatureKind, String)>, tris: &mut Vec<[usize; 3]>, verts: &[[f64; 3]]| {
            if let Some((kind, id)) = group {
                if let Some(f) = finish(*kind, id, tris, verts, &frame) {
                    model.features.push(f);
                }
            }
            tris.clear();
        };

    for line in src.lines() {
        let line = line?;
        let mut it = line.split_ascii_whitespace();
        match it.next() {
            Some("v") => {
                let v: Vec<f64> = it.take(3).filter_map(|s| s.parse().ok()).collect();
                if v.len() == 3 {
                    verts.push([v[0], v[1], v[2]]);
                }
            }
            Some("g") | Some("o") => {
                flush(&group, &mut tris, &verts);
                let name = it.next().unwrap_or("");
                group = if let Some(id) = name.strip_prefix("bldg_") {
                    Some((FeatureKind::Building, id.to_string()))
                } else if let Some(id) = name.strip_prefix("tran_") {
                    Some((FeatureKind::Road, id.to_string()))
                } else {
                    None
                };
            }
            Some("f") if group.is_some() => {
                // "f v/vt/vn ..." を扇形分割。OBJ の添字は 1 始まり（負なら末尾から）。
                let idx: Vec<usize> = it
                    .filter_map(|s| s.split('/').next()?.parse::<i64>().ok())
                    .map(|i| {
                        if i < 0 {
                            (verts.len() as i64 + i) as usize
                        } else {
                            (i - 1) as usize
                        }
                    })
                    .collect();
                for k in 1..idx.len().saturating_sub(1) {
                    tris.push([idx[0], idx[k], idx[k + 1]]);
                }
            }
            _ => {}
        }
    }
    flush(&group, &mut tris, &verts);
    Ok(model)
}

fn finish(
    kind: FeatureKind,
    id: &str,
    tris: &[[usize; 3]],
    verts: &[[f64; 3]],
    frame: &ObjFrame,
) -> Option<Feature> {
    let tris: Vec<[[f64; 3]; 3]> = tris
        .iter()
        .filter(|t| t.iter().all(|&i| i < verts.len()))
        .map(|t| [verts[t[0]], verts[t[1]], verts[t[2]]])
        .collect();
    if tris.is_empty() {
        return None;
    }
    let (ymin, ymax) = tris
        .iter()
        .flatten()
        .fold((f64::MAX, f64::MIN), |(a, b), v| (a.min(v[1]), b.max(v[1])));
    let geo = |v: &[f64; 3], h: f64| frame.to_geo(v[0], h, v[2]);
    match kind {
        FeatureKind::Road => Some(Feature {
            kind,
            gml_id: Some(id.to_string()),
            name: None,
            areas: vec![],
            rings: tris
                .iter()
                .map(|t| t.iter().map(|v| geo(v, 0.0)).collect())
                .collect(),
            height: None,
            solid: vec![],
        }),
        FeatureKind::Building => {
            // 底面: 3 頂点とも最低高さにある三角形。無ければ（屋根だけの LOD0 など）全三角形。
            let mut bottom: Vec<_> = tris
                .iter()
                .filter(|t| t.iter().all(|v| v[1] - ymin < 0.05))
                .collect();
            if bottom.is_empty() {
                bottom = tris.iter().collect();
            }
            let rings = bottom
                .iter()
                .map(|t| t.iter().map(|v| geo(v, 0.0)).collect())
                .collect();
            let solid = tris
                .iter()
                .map(|t| {
                    [
                        geo(&t[0], t[0][1] - ymin),
                        geo(&t[1], t[1][1] - ymin),
                        geo(&t[2], t[2][1] - ymin),
                    ]
                })
                .collect();
            Some(Feature {
                kind,
                gml_id: Some(id.to_string()),
                name: None,
                areas: vec![],
                rings,
                height: Some((ymax - ymin).max(0.0)),
                solid,
            })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_building_and_road_groups() {
        let src = "g bldg_a\nv 0 10 0\nv 4 10 0\nv 4 10 3\nv 0 10 3\nv 0 22 0\nv 4 22 0\nv 4 22 3\nv 0 22 3\n\
                   f 1 2 3 4\nf 5 6 7 8\nf 1 2 6 5\n\
                   g tran_r\nv 10 0 0\nv 20 0 0\nv 20 0 5\nf 9 10 11\n";
        let m = parse_reader(src.as_bytes(), ObjFrame::unity_export(35.0, 139.0)).unwrap();
        assert_eq!(m.buildings().count(), 1);
        assert_eq!(m.roads().count(), 1);
        let b = m.buildings().next().unwrap();
        assert_eq!(b.rings.len(), 2, "底面は四角形→三角形 2 つ");
        assert!((b.height.unwrap() - 12.0).abs() < 1e-9);
        assert_eq!(b.solid.len(), 6);
        // x が西向きなので x=4 は経度が小さくなる
        let p = &b.rings[0][1];
        assert!(p.lon < 139.0);
    }
}
