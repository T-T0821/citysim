//! PLATEAU CityGML (bldg / tran) の最小パーサ。
//!
//! PLATEAU の座標系は EPSG:6697（JGD2011 緯度・経度・楕円体高）で、
//! `gml:posList` は「緯度 経度 高さ」の順に並ぶ。
//! ここでは人流シミュレーションに必要な 2.5D 情報だけを取り出す:
//! - 建物 (`bldg:Building`) のフットプリント（LOD0 または LOD1 の底面）
//! - 道路 (`tran:Road`) の LOD1 面（歩行可能領域の候補）

use anyhow::{Context, Result};
use quick_xml::events::Event;
use quick_xml::Reader;
use std::io::BufRead;
use std::path::Path;

/// 地理座標（度・メートル）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeoPoint {
    pub lat: f64,
    pub lon: f64,
    pub h: f64,
}

/// 1 つの面の外周リング（閉じたリング、最後の点は先頭と同じとは限らない）。
pub type Ring = Vec<GeoPoint>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FeatureKind {
    Building,
    Road,
}

#[derive(Debug, Clone)]
pub struct Feature {
    pub kind: FeatureKind,
    pub gml_id: Option<String>,
    /// 建物: フットプリント 1 つ（底面）。道路: 路面ポリゴン群。
    pub rings: Vec<Ring>,
    /// 建物の計測高さ (`bldg:measuredHeight`)。
    pub height: Option<f64>,
}

#[derive(Debug, Default, Clone)]
pub struct CityModel {
    pub features: Vec<Feature>,
}

impl CityModel {
    pub fn buildings(&self) -> impl Iterator<Item = &Feature> {
        self.features
            .iter()
            .filter(|f| f.kind == FeatureKind::Building)
    }
    pub fn roads(&self) -> impl Iterator<Item = &Feature> {
        self.features.iter().filter(|f| f.kind == FeatureKind::Road)
    }
    pub fn merge(&mut self, other: CityModel) {
        self.features.extend(other.features);
    }
    /// 全頂点の緯度経度の範囲 (min_lat, min_lon, max_lat, max_lon)。
    pub fn bounds(&self) -> Option<(f64, f64, f64, f64)> {
        let mut it = self.features.iter().flat_map(|f| f.rings.iter().flatten());
        let p = it.next()?;
        let mut b = (p.lat, p.lon, p.lat, p.lon);
        for p in it {
            b.0 = b.0.min(p.lat);
            b.1 = b.1.min(p.lon);
            b.2 = b.2.max(p.lat);
            b.3 = b.3.max(p.lon);
        }
        Some(b)
    }
}

pub fn parse_file(path: impl AsRef<Path>) -> Result<CityModel> {
    let path = path.as_ref();
    let f = std::fs::File::open(path).with_context(|| format!("open {}", path.display()))?;
    parse_reader(std::io::BufReader::new(f))
}

pub fn parse_str(s: &str) -> Result<CityModel> {
    parse_reader(s.as_bytes())
}

/// 走査中の地物の一時状態。
struct Pending {
    kind: FeatureKind,
    gml_id: Option<String>,
    lod0: Vec<Ring>,
    lod1: Vec<Ring>,
    height: Option<f64>,
}

pub fn parse_reader<R: BufRead>(src: R) -> Result<CityModel> {
    let mut reader = Reader::from_reader(src);
    reader.config_mut().trim_text(true);

    let mut model = CityModel::default();
    let mut buf = Vec::new();
    // 開いている要素のローカル名スタック
    let mut stack: Vec<String> = Vec::new();
    let mut cur: Option<Pending> = None;
    let mut cur_depth = 0usize;
    let mut text = String::new();

    loop {
        match reader.read_event_into(&mut buf)? {
            Event::Start(e) => {
                let name = String::from_utf8_lossy(e.local_name().as_ref()).into_owned();
                let qname = String::from_utf8_lossy(e.name().as_ref()).into_owned();
                if cur.is_none() {
                    let kind = match qname.as_str() {
                        "bldg:Building" => Some(FeatureKind::Building),
                        "tran:Road" => Some(FeatureKind::Road),
                        _ => None,
                    };
                    if let Some(kind) = kind {
                        let gml_id = e
                            .attributes()
                            .flatten()
                            .find(|a| a.key.local_name().as_ref() == b"id")
                            .map(|a| String::from_utf8_lossy(&a.value).into_owned());
                        cur = Some(Pending {
                            kind,
                            gml_id,
                            lod0: vec![],
                            lod1: vec![],
                            height: None,
                        });
                        cur_depth = stack.len();
                    }
                }
                stack.push(name);
                text.clear();
            }
            Event::Text(t) => {
                text.push_str(&t.unescape()?);
            }
            Event::End(_) => {
                let name = stack.pop().unwrap_or_default();
                if let Some(p) = cur.as_mut() {
                    match name.as_str() {
                        "posList" => {
                            let ring = parse_pos_list(&text);
                            if ring.len() >= 3 {
                                if stack.iter().any(|s| s.starts_with("lod0")) {
                                    p.lod0.push(ring);
                                } else if stack.iter().any(|s| s.starts_with("lod1")) {
                                    p.lod1.push(ring);
                                }
                            }
                        }
                        "measuredHeight" => p.height = text.trim().parse().ok(),
                        _ => {}
                    }
                    if stack.len() == cur_depth {
                        let p = cur.take().unwrap();
                        if let Some(f) = finish(p) {
                            model.features.push(f);
                        }
                    }
                }
                text.clear();
            }
            Event::Eof => break,
            _ => {}
        }
        buf.clear();
    }
    Ok(model)
}

fn parse_pos_list(s: &str) -> Ring {
    let v: Vec<f64> = s
        .split_ascii_whitespace()
        .filter_map(|x| x.parse().ok())
        .collect();
    v.chunks_exact(3)
        .map(|c| GeoPoint {
            lat: c[0],
            lon: c[1],
            h: c[2],
        })
        .collect()
}

fn finish(p: Pending) -> Option<Feature> {
    let rings = match p.kind {
        FeatureKind::Building => {
            if let Some(r) = p.lod0.into_iter().next() {
                vec![r]
            } else {
                // LOD1 立体の底面 = 平均高さが最小で、面積を持つ面
                let bottom = p
                    .lod1
                    .into_iter()
                    .filter(|r| planar_area_deg(r) > 1e-14)
                    .min_by(|a, b| mean_h(a).total_cmp(&mean_h(b)))?;
                vec![bottom]
            }
        }
        FeatureKind::Road => {
            let mut rs = p.lod1;
            if rs.is_empty() {
                rs = p.lod0;
            }
            rs
        }
    };
    if rings.is_empty() {
        return None;
    }
    Some(Feature {
        kind: p.kind,
        gml_id: p.gml_id,
        rings,
        height: p.height,
    })
}

fn mean_h(r: &Ring) -> f64 {
    r.iter().map(|p| p.h).sum::<f64>() / r.len() as f64
}

fn planar_area_deg(r: &Ring) -> f64 {
    let n = r.len();
    let mut a = 0.0;
    for i in 0..n {
        let (p, q) = (r[i], r[(i + 1) % n]);
        a += p.lon * q.lat - q.lon * p.lat;
    }
    (a * 0.5).abs()
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = include_str!("../../../testdata/sample.gml");

    #[test]
    fn parses_buildings_and_roads() {
        let m = parse_str(SAMPLE).unwrap();
        assert!(m.buildings().count() >= 2);
        assert!(m.roads().count() >= 1);
        let b = m.buildings().next().unwrap();
        assert_eq!(b.rings.len(), 1);
        assert!(b.height.is_some());
        // 底面が選ばれている
        assert!(b.rings[0].iter().all(|p| p.h < 1.0));
    }
}
