//! PLATEAU CityGML (bldg / tran) の最小パーサ。
//!
//! PLATEAU の座標系は EPSG:6697（JGD2011 緯度・経度・楕円体高）で、
//! `gml:posList` は「緯度 経度 高さ」の順に並ぶ。
//! ここでは人流シミュレーションに必要な 2.5D 情報だけを取り出す:
//! - 建物 (`bldg:Building`) のフットプリント（LOD0 または LOD1 の底面）
//! - 道路 (`tran:Road`) の LOD1 面（歩行可能領域の候補）
//! - 道路の LOD2/LOD3 `tran:TrafficArea`（車道部・歩道部・島などの区分面）

pub mod obj;

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

/// 道路の区分面 (`tran:TrafficArea` / `tran:AuxiliaryTrafficArea`)。LOD2 以上の道路にだけ入る。
#[derive(Debug, Clone)]
pub struct TrafficArea {
    /// PLATEAU の TrafficArea_function コード
    /// （1000 車道部, 1010 車道交差部, 1020 島, 2000 歩道部, 3000 自転車道 など）。
    pub function: u32,
    /// `tran:AuxiliaryTrafficArea`（植栽・分離帯など、通行しない部分）か
    pub auxiliary: bool,
    /// 面のポリゴン群（LOD3 があれば LOD3、無ければ LOD2）
    pub rings: Vec<Ring>,
}

impl TrafficArea {
    /// 車が走る部分（車道部・車道交差部・島）。島は車道に挟まれた安全地帯で、道路網の抽出では車道側に含める。
    pub fn is_carriageway(&self) -> bool {
        !self.auxiliary && matches!(self.function, 1000 | 1010 | 1020)
    }
}

#[derive(Debug, Clone)]
pub struct Feature {
    pub kind: FeatureKind,
    pub gml_id: Option<String>,
    /// `gml:name`（道路なら路線名。首都高速などの立体交差の除外に使う）
    pub name: Option<String>,
    /// 道路の区分面。LOD1 だけの道路や建物では空。
    pub areas: Vec<TrafficArea>,
    /// 建物: フットプリント 1 つ（底面）。道路: 路面ポリゴン群。
    pub rings: Vec<Ring>,
    /// 建物の計測高さ (`bldg:measuredHeight`)。
    pub height: Option<f64>,
    /// 建物の立体（描画用）。三角形の頂点、高さは底面からの相対値 [m]。
    /// CityGML からは作らない（押し出しで描く）。OBJ 読込時のみ入る。
    pub solid: Vec<[GeoPoint; 3]>,
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
    /// 歩行者・車の 2D シミュレーションに使う地上の道路。
    /// 高速道路（名前に「高速」を含む路線）は高架・掘割で地上と交わらないので除く。
    pub fn ground_roads(&self) -> impl Iterator<Item = &Feature> {
        self.roads()
            .filter(|f| !f.name.as_deref().is_some_and(|n| n.contains("高速")))
    }
    pub fn merge(&mut self, other: CityModel) {
        self.features.extend(other.features);
    }
    /// 指定した緯度経度の範囲（min_lat, min_lon, max_lat, max_lon）に一部でもかかる地物だけ残す。
    pub fn crop(&mut self, b: (f64, f64, f64, f64)) {
        self.features.retain(|f| {
            f.rings
                .iter()
                .flatten()
                .any(|p| p.lat >= b.0 && p.lat <= b.2 && p.lon >= b.1 && p.lon <= b.3)
                || ring_bbox_intersects(f, b)
        });
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
    name: Option<String>,
    lod0: Vec<Ring>,
    lod1: Vec<Ring>,
    height: Option<f64>,
    areas: Vec<TrafficArea>,
    /// 走査中の TrafficArea（auxiliary, function, lod2 面, lod3 面）
    area: Option<(bool, u32, Vec<Ring>, Vec<Ring>)>,
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
                            name: None,
                            lod0: vec![],
                            lod1: vec![],
                            height: None,
                            areas: vec![],
                            area: None,
                        });
                        cur_depth = stack.len();
                    }
                } else if let Some(p) = cur.as_mut() {
                    match qname.as_str() {
                        "tran:TrafficArea" => p.area = Some((false, 0, vec![], vec![])),
                        "tran:AuxiliaryTrafficArea" => p.area = Some((true, 0, vec![], vec![])),
                        _ => {}
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
                                if let Some(a) = p.area.as_mut() {
                                    if stack.iter().any(|s| s.starts_with("lod3")) {
                                        a.3.push(ring);
                                    } else if stack.iter().any(|s| s.starts_with("lod2")) {
                                        a.2.push(ring);
                                    }
                                } else if stack.iter().any(|s| s.starts_with("lod0")) {
                                    p.lod0.push(ring);
                                } else if stack.iter().any(|s| s.starts_with("lod1")) {
                                    p.lod1.push(ring);
                                }
                            }
                        }
                        "function" => {
                            if let Some(a) = p.area.as_mut() {
                                a.1 = text.trim().parse().unwrap_or(0);
                            }
                        }
                        "TrafficArea" | "AuxiliaryTrafficArea" => {
                            if let Some((auxiliary, function, lod2, lod3)) = p.area.take() {
                                let rings = if lod3.is_empty() { lod2 } else { lod3 };
                                if !rings.is_empty() {
                                    p.areas.push(TrafficArea {
                                        function,
                                        auxiliary,
                                        rings,
                                    });
                                }
                            }
                        }
                        "name" if stack.len() == cur_depth + 1 && p.name.is_none() => {
                            p.name = Some(text.trim().to_string());
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
            if rs.is_empty() {
                // LOD2/LOD3 だけの道路: 区分面を合わせたものを路面とする
                rs = p
                    .areas
                    .iter()
                    .filter(|a| !a.auxiliary)
                    .flat_map(|a| a.rings.iter().cloned())
                    .collect();
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
        name: p.name,
        areas: p.areas,
        rings,
        height: p.height,
        solid: vec![],
    })
}

/// 地物のバウンディングボックスが範囲と交差するか（大きな道路面が範囲をまたぐ場合用）。
fn ring_bbox_intersects(f: &Feature, b: (f64, f64, f64, f64)) -> bool {
    let mut it = f.rings.iter().flatten();
    let Some(p) = it.next() else { return false };
    let mut fb = (p.lat, p.lon, p.lat, p.lon);
    for p in it {
        fb.0 = fb.0.min(p.lat);
        fb.1 = fb.1.min(p.lon);
        fb.2 = fb.2.max(p.lat);
        fb.3 = fb.3.max(p.lon);
    }
    fb.0 <= b.2 && fb.2 >= b.0 && fb.1 <= b.3 && fb.3 >= b.1
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

#[cfg(test)]
mod traffic_area_tests {
    use super::*;

    const ROAD_LOD3: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<core:CityModel xmlns:core="http://www.opengis.net/citygml/2.0" xmlns:gml="http://www.opengis.net/gml" xmlns:tran="http://www.opengis.net/citygml/transportation/2.0">
<core:cityObjectMember><tran:Road gml:id="r1">
<gml:name>都道1号</gml:name>
<tran:function codeSpace="x">3</tran:function>
<tran:lod1MultiSurface><gml:MultiSurface><gml:surfaceMember><gml:Polygon><gml:exterior><gml:LinearRing>
<gml:posList>35.0 139.0 0 35.0 139.001 0 35.0001 139.001 0 35.0001 139.0 0 35.0 139.0 0</gml:posList>
</gml:LinearRing></gml:exterior></gml:Polygon></gml:surfaceMember></gml:MultiSurface></tran:lod1MultiSurface>
<tran:trafficArea><tran:TrafficArea gml:id="t1">
<tran:function codeSpace="x">1000</tran:function>
<tran:lod2MultiSurface><gml:MultiSurface><gml:surfaceMember><gml:Polygon><gml:exterior><gml:LinearRing>
<gml:posList>35.0 139.0 0 35.0 139.001 0 35.00005 139.001 0 35.00005 139.0 0</gml:posList>
</gml:LinearRing></gml:exterior></gml:Polygon></gml:surfaceMember></gml:MultiSurface></tran:lod2MultiSurface>
<tran:lod3MultiSurface><gml:MultiSurface><gml:surfaceMember><gml:Polygon><gml:exterior><gml:LinearRing>
<gml:posList>35.0 139.0 1 35.0 139.001 1 35.00005 139.001 1</gml:posList>
</gml:LinearRing></gml:exterior></gml:Polygon></gml:surfaceMember>
<gml:surfaceMember><gml:Polygon><gml:exterior><gml:LinearRing>
<gml:posList>35.0 139.0 1 35.00005 139.001 1 35.00005 139.0 1</gml:posList>
</gml:LinearRing></gml:exterior></gml:Polygon></gml:surfaceMember></gml:MultiSurface></tran:lod3MultiSurface>
</tran:TrafficArea></tran:trafficArea>
<tran:trafficArea><tran:TrafficArea gml:id="t2">
<tran:function codeSpace="x">2000</tran:function>
<tran:lod2MultiSurface><gml:MultiSurface><gml:surfaceMember><gml:Polygon><gml:exterior><gml:LinearRing>
<gml:posList>35.00005 139.0 0 35.00005 139.001 0 35.0001 139.001 0 35.0001 139.0 0</gml:posList>
</gml:LinearRing></gml:exterior></gml:Polygon></gml:surfaceMember></gml:MultiSurface></tran:lod2MultiSurface>
</tran:TrafficArea></tran:trafficArea>
<tran:auxiliaryTrafficArea><tran:AuxiliaryTrafficArea gml:id="a1">
<tran:function codeSpace="x">1000</tran:function>
<tran:lod2MultiSurface><gml:MultiSurface><gml:surfaceMember><gml:Polygon><gml:exterior><gml:LinearRing>
<gml:posList>35.00005 139.0 0 35.00005 139.0001 0 35.00006 139.0001 0</gml:posList>
</gml:LinearRing></gml:exterior></gml:Polygon></gml:surfaceMember></gml:MultiSurface></tran:lod2MultiSurface>
</tran:AuxiliaryTrafficArea></tran:auxiliaryTrafficArea>
</tran:Road></core:cityObjectMember>
<core:cityObjectMember><tran:Road gml:id="r2">
<gml:name>首都高速都心環状線</gml:name>
<tran:lod1MultiSurface><gml:MultiSurface><gml:surfaceMember><gml:Polygon><gml:exterior><gml:LinearRing>
<gml:posList>35.0 139.0 10 35.0 139.001 10 35.0001 139.001 10</gml:posList>
</gml:LinearRing></gml:exterior></gml:Polygon></gml:surfaceMember></gml:MultiSurface></tran:lod1MultiSurface>
</tran:Road></core:cityObjectMember>
</core:CityModel>"#;

    #[test]
    fn parses_traffic_areas_and_names() {
        let m = parse_str(ROAD_LOD3).unwrap();
        assert_eq!(m.roads().count(), 2);
        let r = m
            .roads()
            .find(|r| r.gml_id.as_deref() == Some("r1"))
            .unwrap();
        assert_eq!(r.name.as_deref(), Some("都道1号"));
        assert_eq!(r.rings.len(), 1, "LOD1 の路面");
        assert_eq!(r.areas.len(), 3);
        let car = &r.areas[0];
        assert_eq!(car.function, 1000);
        assert!(car.is_carriageway());
        // LOD3 があれば LOD3（三角形 2 枚）が使われる
        assert_eq!(car.rings.len(), 2);
        assert_eq!(car.rings[0][0].h, 1.0);
        let walk = &r.areas[1];
        assert_eq!(walk.function, 2000);
        assert!(!walk.is_carriageway());
        assert_eq!(walk.rings.len(), 1);
        assert!(r.areas[2].auxiliary);
        // 首都高は地上の道路から除かれる
        let ground: Vec<_> = m.ground_roads().collect();
        assert_eq!(ground.len(), 1);
        assert_eq!(ground[0].gml_id.as_deref(), Some("r1"));
    }
}
