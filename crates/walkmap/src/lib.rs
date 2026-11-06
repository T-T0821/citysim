//! CityModel から 2D の歩行可能グリッドを作り、経路誘導用の場を計算する。
//!
//! - 座標変換: 緯度経度 → 原点まわりの局所平面 (x=東, y=北, メートル)
//! - 歩行可能領域: 道路面があれば「道路 ∖ 建物」、なければ「範囲内 ∖ 建物」
//! - フローフィールド: 目的地セルからの測地距離 (8近傍 Dijkstra)
//! - 障害物距離場: 壁からの反発力計算用

use glam::Vec2;
use plateau_gml::{CityModel, GeoPoint};
use std::cmp::Ordering;
use std::collections::BinaryHeap;

const EARTH_R: f64 = 6_378_137.0;

/// 局所平面への等距円筒近似投影。街区〜数km 程度の範囲では十分な精度。
#[derive(Debug, Clone, Copy)]
pub struct LocalProjection {
    pub lat0: f64,
    pub lon0: f64,
    cos_lat0: f64,
}

impl LocalProjection {
    pub fn new(lat0: f64, lon0: f64) -> Self {
        Self {
            lat0,
            lon0,
            cos_lat0: lat0.to_radians().cos(),
        }
    }
    pub fn to_local(&self, p: &GeoPoint) -> Vec2 {
        let x = (p.lon - self.lon0).to_radians() * EARTH_R * self.cos_lat0;
        let y = (p.lat - self.lat0).to_radians() * EARTH_R;
        Vec2::new(x as f32, y as f32)
    }
    pub fn to_geo(&self, v: Vec2) -> (f64, f64) {
        let lat = self.lat0 + (v.y as f64 / EARTH_R).to_degrees();
        let lon = self.lon0 + (v.x as f64 / (EARTH_R * self.cos_lat0)).to_degrees();
        (lat, lon)
    }
}

pub type Polygon = Vec<Vec2>;

/// セル単位の歩行可能グリッド。
#[derive(Debug, Clone)]
pub struct WalkGrid {
    pub origin: Vec2,
    pub cell: f32,
    pub w: usize,
    pub h: usize,
    pub walkable: Vec<bool>,
    /// 各セル中心から最寄りの非歩行セルまでの距離 [m]
    pub obstacle_dist: Vec<f32>,
    pub projection: LocalProjection,
    pub buildings: Vec<(Polygon, f32)>,
    /// 道路面データから歩行空間を作ったか
    pub has_roads: bool,
    /// 車道セル（`mark_carriageway` で設定。未設定なら空）
    pub carriageway: Vec<bool>,
    /// 歩行者の経路コスト倍率（車道は高くして歩道を優先させる）
    pub walk_cost: Vec<f32>,
}

impl WalkGrid {
    pub fn from_city(model: &CityModel, cell: f32) -> Option<Self> {
        let (min_lat, min_lon, max_lat, max_lon) = model.bounds()?;
        let proj = LocalProjection::new(min_lat, min_lon);
        let max = proj.to_local(&GeoPoint {
            lat: max_lat,
            lon: max_lon,
            h: 0.0,
        });
        let w = ((max.x / cell).ceil() as usize).max(1);
        let h = ((max.y / cell).ceil() as usize).max(1);

        let to_poly =
            |r: &Vec<GeoPoint>| -> Polygon { r.iter().map(|p| proj.to_local(p)).collect() };
        let roads: Vec<Polygon> = model
            .roads()
            .flat_map(|f| f.rings.iter().map(to_poly))
            .collect();
        let buildings: Vec<(Polygon, f32)> = model
            .buildings()
            .map(|f| (to_poly(&f.rings[0]), f.height.unwrap_or(10.0) as f32))
            .collect();

        let mut walkable = vec![roads.is_empty(); w * h];
        for p in &roads {
            fill_polygon(p, Vec2::ZERO, cell, w, h, |i| walkable[i] = true);
        }
        for (p, _) in &buildings {
            fill_polygon(p, Vec2::ZERO, cell, w, h, |i| walkable[i] = false);
        }
        let mut g = Self {
            origin: Vec2::ZERO,
            cell,
            w,
            h,
            walkable,
            obstacle_dist: vec![],
            projection: proj,
            buildings,
            has_roads: !roads.is_empty(),
            carriageway: vec![],
            walk_cost: vec![1.0; w * h],
        };
        g.obstacle_dist = g.chamfer_obstacle_distance();
        Some(g)
    }

    /// 道路の端（建物・道路外）から `sidewalk_w` [m] 以上離れた歩行可能セルを車道とみなす。
    /// PLATEAU LOD1 の道路面は歩道と車道を区別しないための近似（LOD2 以降は TrafficArea で区別できる）。
    /// 歩行者が車道を通るコストは `crossing_cost` 倍になる。
    pub fn mark_carriageway(&mut self, sidewalk_w: f32, crossing_cost: f32) {
        self.carriageway = (0..self.w * self.h)
            .map(|i| self.walkable[i] && self.obstacle_dist[i] >= sidewalk_w)
            .collect();
        for (c, &cw) in self.walk_cost.iter_mut().zip(&self.carriageway) {
            *c = if cw { crossing_cost } else { 1.0 };
        }
    }

    pub fn is_carriageway(&self, p: Vec2) -> bool {
        !self.carriageway.is_empty()
            && self
                .cell_of(p)
                .map(|(x, y)| self.carriageway[self.idx(x, y)])
                .unwrap_or(false)
    }

    #[inline]
    pub fn idx(&self, x: usize, y: usize) -> usize {
        y * self.w + x
    }
    pub fn cell_of(&self, p: Vec2) -> Option<(usize, usize)> {
        let q = (p - self.origin) / self.cell;
        if q.x < 0.0 || q.y < 0.0 {
            return None;
        }
        let (x, y) = (q.x as usize, q.y as usize);
        (x < self.w && y < self.h).then_some((x, y))
    }
    pub fn center(&self, x: usize, y: usize) -> Vec2 {
        self.origin + Vec2::new(x as f32 + 0.5, y as f32 + 0.5) * self.cell
    }
    pub fn is_walkable(&self, p: Vec2) -> bool {
        self.cell_of(p)
            .map(|(x, y)| self.walkable[self.idx(x, y)])
            .unwrap_or(false)
    }
    pub fn walkable_cells(&self) -> impl Iterator<Item = (usize, usize)> + '_ {
        (0..self.h)
            .flat_map(move |y| (0..self.w).map(move |x| (x, y)))
            .filter(|&(x, y)| self.walkable[self.idx(x, y)])
    }

    /// 双線形補間した障害物距離と、その勾配（障害物から離れる向き）。
    pub fn obstacle_sample(&self, p: Vec2) -> (f32, Vec2) {
        let d = |x: f32, y: f32| self.sample_field(&self.obstacle_dist, Vec2::new(x, y), 0.0);
        let e = self.cell * 0.5;
        let g = Vec2::new(
            d(p.x + e, p.y) - d(p.x - e, p.y),
            d(p.x, p.y + e) - d(p.x, p.y - e),
        ) / (2.0 * e);
        (d(p.x, p.y), g)
    }

    pub fn sample_field(&self, f: &[f32], p: Vec2, outside: f32) -> f32 {
        let q = (p - self.origin) / self.cell - Vec2::splat(0.5);
        let (x0, y0) = (q.x.floor(), q.y.floor());
        let (tx, ty) = (q.x - x0, q.y - y0);
        let get = |x: f32, y: f32| {
            if x < 0.0 || y < 0.0 || x as usize >= self.w || y as usize >= self.h {
                outside
            } else {
                f[self.idx(x as usize, y as usize)]
            }
        };
        let a = get(x0, y0) * (1.0 - tx) + get(x0 + 1.0, y0) * tx;
        let b = get(x0, y0 + 1.0) * (1.0 - tx) + get(x0 + 1.0, y0 + 1.0) * tx;
        a * (1.0 - ty) + b * ty
    }

    /// 2パス chamfer 距離変換（非歩行セル・グリッド外を障害物とする）。
    fn chamfer_obstacle_distance(&self) -> Vec<f32> {
        let (w, h, c) = (self.w, self.h, self.cell);
        let big = f32::MAX / 4.0;
        let mut d: Vec<f32> = self
            .walkable
            .iter()
            .map(|&wk| if wk { big } else { 0.0 })
            .collect();
        // グリッド外周も壁とみなす
        for y in 0..h {
            for x in 0..w {
                let i = y * w + x;
                if d[i] > 0.0 {
                    let edge = x.min(y).min(w - 1 - x).min(h - 1 - y) as f32 + 0.5;
                    d[i] = d[i].min(edge * c);
                }
            }
        }
        let (a, b) = (c, c * std::f32::consts::SQRT_2);
        for y in 0..h {
            for x in 0..w {
                let i = y * w + x;
                let mut v = d[i];
                if x > 0 {
                    v = v.min(d[i - 1] + a);
                }
                if y > 0 {
                    v = v.min(d[i - w] + a);
                    if x > 0 {
                        v = v.min(d[i - w - 1] + b);
                    }
                    if x + 1 < w {
                        v = v.min(d[i - w + 1] + b);
                    }
                }
                d[i] = v;
            }
        }
        for y in (0..h).rev() {
            for x in (0..w).rev() {
                let i = y * w + x;
                let mut v = d[i];
                if x + 1 < w {
                    v = v.min(d[i + 1] + a);
                }
                if y + 1 < h {
                    v = v.min(d[i + w] + a);
                    if x + 1 < w {
                        v = v.min(d[i + w + 1] + b);
                    }
                    if x > 0 {
                        v = v.min(d[i + w - 1] + b);
                    }
                }
                d[i] = v;
            }
        }
        d
    }
}

/// 目的地までの測地距離場。勾配の逆向きが進むべき方向。
#[derive(Debug, Clone)]
pub struct FlowField {
    pub dist: Vec<f32>,
}

impl FlowField {
    pub const UNREACHABLE: f32 = f32::INFINITY;

    /// `goals` のセル群からの 8 近傍 Dijkstra。壁際を避けるため障害物近傍にコストを上乗せ。
    pub fn compute(grid: &WalkGrid, goals: &[(usize, usize)]) -> Self {
        let mut dist = vec![Self::UNREACHABLE; grid.w * grid.h];
        let mut heap = BinaryHeap::new();
        for &(x, y) in goals {
            let i = grid.idx(x, y);
            if grid.walkable[i] {
                dist[i] = 0.0;
                heap.push(Node(0.0, i));
            }
        }
        const N8: [(i32, i32, f32); 8] = [
            (1, 0, 1.0),
            (-1, 0, 1.0),
            (0, 1, 1.0),
            (0, -1, 1.0),
            (1, 1, std::f32::consts::SQRT_2),
            (1, -1, std::f32::consts::SQRT_2),
            (-1, 1, std::f32::consts::SQRT_2),
            (-1, -1, std::f32::consts::SQRT_2),
        ];
        while let Some(Node(d, i)) = heap.pop() {
            if d > dist[i] {
                continue;
            }
            let (x, y) = ((i % grid.w) as i32, (i / grid.w) as i32);
            for (dx, dy, len) in N8 {
                let (nx, ny) = (x + dx, y + dy);
                if nx < 0 || ny < 0 || nx as usize >= grid.w || ny as usize >= grid.h {
                    continue;
                }
                let j = grid.idx(nx as usize, ny as usize);
                if !grid.walkable[j] {
                    continue;
                }
                // 斜め移動で角をすり抜けない
                if dx != 0
                    && dy != 0
                    && (!grid.walkable[grid.idx(nx as usize, y as usize)]
                        || !grid.walkable[grid.idx(x as usize, ny as usize)])
                {
                    continue;
                }
                let wall_pen = 1.0 + 2.0 * (-grid.obstacle_dist[j] / 1.0).exp();
                let nd = d + len * grid.cell * wall_pen * grid.walk_cost[j];
                if nd < dist[j] {
                    dist[j] = nd;
                    heap.push(Node(nd, j));
                }
            }
        }
        Self { dist }
    }

    /// 位置 p における進行方向（単位ベクトル）。到達不能なら None。
    pub fn direction(&self, grid: &WalkGrid, p: Vec2) -> Option<Vec2> {
        let (cx, cy) = grid.cell_of(p)?;
        let here = self.dist[grid.idx(cx, cy)];
        if !here.is_finite() {
            return None;
        }
        // 近傍で最も距離の小さいセル中心へ向かう（補間ノイズに強い）
        let mut best = (here, None);
        for dy in -1i32..=1 {
            for dx in -1i32..=1 {
                let (nx, ny) = (cx as i32 + dx, cy as i32 + dy);
                if nx < 0 || ny < 0 || nx as usize >= grid.w || ny as usize >= grid.h {
                    continue;
                }
                let d = self.dist[grid.idx(nx as usize, ny as usize)];
                if d < best.0 {
                    best = (d, Some((nx as usize, ny as usize)));
                }
            }
        }
        match best.1 {
            Some((x, y)) => (grid.center(x, y) - p).try_normalize(),
            None => Some(Vec2::ZERO), // 目的地セル
        }
    }

    pub fn distance(&self, grid: &WalkGrid, p: Vec2) -> f32 {
        grid.cell_of(p)
            .map(|(x, y)| self.dist[grid.idx(x, y)])
            .unwrap_or(Self::UNREACHABLE)
    }
}

#[derive(PartialEq)]
struct Node(f32, usize);
impl Eq for Node {}
impl Ord for Node {
    fn cmp(&self, o: &Self) -> Ordering {
        o.0.total_cmp(&self.0)
    }
}
impl PartialOrd for Node {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}

/// スキャンラインでポリゴン内部のセル（中心が内側）を塗る。偶奇規則。
pub fn fill_polygon(
    poly: &[Vec2],
    origin: Vec2,
    cell: f32,
    w: usize,
    h: usize,
    mut f: impl FnMut(usize),
) {
    if poly.len() < 3 {
        return;
    }
    let (mut ymin, mut ymax) = (f32::MAX, f32::MIN);
    for p in poly {
        ymin = ymin.min(p.y);
        ymax = ymax.max(p.y);
    }
    let y0 = (((ymin - origin.y) / cell - 0.5).floor().max(0.0)) as usize;
    let y1 = ((((ymax - origin.y) / cell - 0.5).ceil()).max(0.0) as usize).min(h.saturating_sub(1));
    let mut xs = Vec::new();
    for cy in y0..=y1 {
        let sy = origin.y + (cy as f32 + 0.5) * cell;
        xs.clear();
        for i in 0..poly.len() {
            let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
            if (a.y <= sy) != (b.y <= sy) {
                xs.push(a.x + (sy - a.y) / (b.y - a.y) * (b.x - a.x));
            }
        }
        xs.sort_by(f32::total_cmp);
        for pair in xs.chunks_exact(2) {
            let xa = ((pair[0] - origin.x) / cell - 0.5).ceil().max(0.0) as usize;
            let xb = ((pair[1] - origin.x) / cell - 0.5).floor();
            if xb < 0.0 {
                continue;
            }
            let xb = (xb as usize).min(w.saturating_sub(1));
            for cx in xa..=xb {
                f(cy * w + cx);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> WalkGrid {
        let m = plateau_gml::parse_str(include_str!("../../../testdata/sample.gml")).unwrap();
        WalkGrid::from_city(&m, 1.0).unwrap()
    }

    #[test]
    fn grid_matches_sample_layout() {
        let g = sample();
        assert!((199..=201).contains(&g.w), "w={}", g.w);
        // 道路上は歩ける、街区（建物・道路外）は歩けない
        assert!(g.is_walkable(Vec2::new(70.0, 30.0)));
        assert!(g.is_walkable(Vec2::new(70.0, 70.0))); // 交差点
        assert!(!g.is_walkable(Vec2::new(30.0, 30.0)));
    }

    #[test]
    fn flow_field_reaches_goal() {
        let g = sample();
        let goals: Vec<_> = g.walkable_cells().filter(|&(x, _)| x + 2 >= g.w).collect();
        let ff = FlowField::compute(&g, &goals);
        let start = Vec2::new(2.0, 70.0);
        assert!(ff.distance(&g, start).is_finite());
        let d = ff.direction(&g, start).unwrap();
        assert!(d.x > 0.5, "{d:?}");
    }
}
