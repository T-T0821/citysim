//! 通行止め・工事による歩行空間の封鎖。

use crate::{fill_polygon, Polygon, WalkGrid};
use glam::Vec2;

/// 封鎖の種類。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ClosureKind {
    /// 歩道・車道とも通れない（全面通行止め）
    #[default]
    All,
    /// 歩道だけ通れない（歩道工事）
    Sidewalk,
    /// 車道だけ通れない（車線規制・道路工事）。歩行者の横断もできない。
    Road,
}

impl ClosureKind {
    pub const ALL: [ClosureKind; 3] = [ClosureKind::All, ClosureKind::Sidewalk, ClosureKind::Road];

    pub fn label(self) -> &'static str {
        match self {
            ClosureKind::All => "全面通行止め",
            ClosureKind::Sidewalk => "歩道のみ",
            ClosureKind::Road => "車道のみ",
        }
    }
    pub fn key(self) -> &'static str {
        match self {
            ClosureKind::All => "all",
            ClosureKind::Sidewalk => "sidewalk",
            ClosureKind::Road => "road",
        }
    }
    pub fn from_key(s: &str) -> Option<Self> {
        match s {
            "all" | "全面" => Some(ClosureKind::All),
            "sidewalk" | "歩道" => Some(ClosureKind::Sidewalk),
            "road" | "carriageway" | "車道" => Some(ClosureKind::Road),
            _ => None,
        }
    }
    /// 車両の通行も止めるか
    pub fn blocks_vehicles(self) -> bool {
        self != ClosureKind::Sidewalk
    }
}

impl WalkGrid {
    /// 車道セルが未設定なら、道路端から `sidewalk_w` [m] 以上離れた歩行可能セルを車道とする
    /// （`mark_carriageway` と同じ近似。封鎖で壁が増える前に決めておく）。
    pub fn ensure_carriageway(&mut self, sidewalk_w: f32) {
        if self.carriageway.is_empty() && self.has_roads {
            self.carriageway = (0..self.w * self.h)
                .map(|i| self.walkable[i] && self.obstacle_dist[i] >= sidewalk_w)
                .collect();
        }
    }

    /// ポリゴン（局所座標 [m]）の範囲を封鎖する。複数まとめて渡すと距離場の再計算が 1 回で済む。
    /// 戻り値は通行不可にしたセル数。
    pub fn apply_closures<'a>(
        &mut self,
        closures: impl IntoIterator<Item = (&'a Polygon, ClosureKind)>,
        sidewalk_w: f32,
    ) -> usize {
        self.ensure_carriageway(sidewalk_w);
        let (w, h, cell, origin) = (self.w, self.h, self.cell, self.origin);
        let mut closed = 0;
        for (poly, kind) in closures {
            if poly.len() < 3 {
                continue;
            }
            let walkable = &mut self.walkable;
            let cw = &self.carriageway;
            fill_polygon(poly, origin, cell, w, h, |i| {
                let is_road = !cw.is_empty() && cw[i];
                let hit = match kind {
                    ClosureKind::All => true,
                    ClosureKind::Sidewalk => !is_road,
                    ClosureKind::Road => is_road,
                };
                if hit && walkable[i] {
                    walkable[i] = false;
                    closed += 1;
                }
            });
        }
        for (c, &wk) in self.carriageway.iter_mut().zip(&self.walkable) {
            *c &= wk;
        }
        self.obstacle_dist = self.chamfer_obstacle_distance();
        closed
    }
}

/// 長方形（2 隅）をポリゴンにする。
pub fn rect_polygon(a: Vec2, b: Vec2) -> Polygon {
    let (lo, hi) = (a.min(b), a.max(b));
    vec![lo, Vec2::new(hi.x, lo.y), hi, Vec2::new(lo.x, hi.y)]
}

/// 折れ線を幅 `width` [m] の帯にしたポリゴン群（区間ごとの長方形）。
pub fn line_polygons(pts: &[Vec2], width: f32) -> Vec<Polygon> {
    let hw = width * 0.5;
    pts.windows(2)
        .filter_map(|s| {
            let d = (s[1] - s[0]).try_normalize()?;
            let n = d.perp() * hw;
            let (a, b) = (s[0] - d * hw, s[1] + d * hw);
            Some(vec![a - n, b - n, b + n, a + n])
        })
        .collect()
}

/// 点がポリゴン内か（偶奇規則）。
pub fn point_in_polygon(p: Vec2, poly: &[Vec2]) -> bool {
    let mut inside = false;
    let n = poly.len();
    for i in 0..n {
        let (a, b) = (poly[i], poly[(i + n - 1) % n]);
        if (a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x {
            inside = !inside;
        }
    }
    inside
}

/// 折れ線がポリゴンに入るか（頂点を 1m 間隔で調べる簡易判定）。
pub fn polyline_hits_polygon(pts: &[Vec2], poly: &[Vec2]) -> bool {
    pts.windows(2).any(|s| {
        let n = (s[0].distance(s[1]) / 1.0).ceil().max(1.0) as usize;
        (0..=n).any(|k| point_in_polygon(s[0].lerp(s[1], k as f32 / n as f32), poly))
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn closure_blocks_cells() {
        let m = plateau_gml::parse_str(include_str!("../../../testdata/sample.gml")).unwrap();
        let mut g = WalkGrid::from_city(&m, 0.5).unwrap();
        let before = g.walkable.iter().filter(|&&b| b).count();
        let poly = rect_polygon(Vec2::new(90.0, 60.0), Vec2::new(110.0, 80.0));
        let n = g.apply_closures([(&poly, ClosureKind::All)], 3.0);
        let after = g.walkable.iter().filter(|&&b| b).count();
        assert!(n > 0);
        assert_eq!(before - after, n);
        assert!(!g.is_walkable(Vec2::new(100.0, 70.0)));
        assert!(point_in_polygon(Vec2::new(91.0, 61.0), &poly));
        assert!(!point_in_polygon(Vec2::new(60.0, 70.0), &poly));
    }
}
