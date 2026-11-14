//! 負荷テスト用の合成の街（`?city=synthetic`）。
//!
//! 100 万人は銀座の切り出し範囲（歩ける面積 約 8.1 万 m²）には物理的に入らないので、
//! 2km 四方の格子状の街区を乱数で作る。街路は端から端まで通る帯（東西・南北）で、
//! 街区は建物か公園（歩ける）。この形なら歩行可能か・壁までの距離・目的地への向きを
//! GPU のシェーダーで式から求められ、大きな格子データを持たなくて済む。
//!
//! 座標は平面の (x, y) [m]（x = 東, y = 北）で、街は `0..size` の正方形。
//! 帯や街区の境界は整数 [m] にそろえる（1m ごとの参照表で正確に分類できるように）。

use crowd::Rng;

/// 一方向（x か y）の区切り: 帯（街路）と街区が交互に並ぶ。
/// `bands[k]` は街区 k-1 と k の間の街路で、`bands[0]` と最後の帯は街の外周。
#[derive(Debug, Clone)]
pub struct Axis {
    pub bands: Vec<(f32, f32)>,
    pub blocks: Vec<(f32, f32)>,
}

impl Axis {
    fn generate(size: f32, rng: &mut Rng) -> Self {
        let mut bands = vec![];
        let mut blocks = vec![];
        let street = |k: usize, rng: &mut Rng| -> f32 {
            // 4 本に 1 本は広い大通り
            if k % 4 == 0 {
                rng.range(20.0, 27.0).round()
            } else {
                rng.range(10.0, 16.0).round()
            }
        };
        let w0 = street(0, rng);
        bands.push((0.0, w0));
        let mut x = w0;
        loop {
            let bw = rng.range(50.0, 110.0).round();
            let w = street(bands.len(), rng);
            if x + bw + w > size {
                break;
            }
            blocks.push((x, x + bw));
            bands.push((x + bw, x + bw + w));
            x += bw + w;
        }
        let rest = size - x;
        if rest < 40.0 {
            // 狭すぎて街区にならないので、最後の街路を街の端まで広げる
            bands.last_mut().unwrap().1 = size;
        } else {
            let w = 12.0;
            blocks.push((x, size - w));
            bands.push((size - w, size));
        }
        Self { bands, blocks }
    }

    /// v を含む帯の番号か、街区の番号
    pub fn locate(&self, v: f32) -> Cell {
        for (k, &(a, b)) in self.bands.iter().enumerate() {
            if v >= a && v < b {
                return Cell::Band(k);
            }
        }
        for (i, &(a, b)) in self.blocks.iter().enumerate() {
            if v >= a && v < b {
                return Cell::Block(i);
            }
        }
        Cell::Outside
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cell {
    Band(usize),
    Block(usize),
    Outside,
}

#[derive(Debug, Clone)]
pub struct SynthCity {
    pub size: f32,
    pub x: Axis,
    pub y: Axis,
    /// 街区 (ix, iy) が公園か（`iy * nx + ix`）
    pub park: Vec<bool>,
}

impl SynthCity {
    /// 街の一辺 [m]。GPU の密度テクスチャ（1m 格子）が WebGL2 の最小保証 2048 に収まる大きさ
    pub const SIZE: f32 = 2048.0;

    pub fn generate(seed: u64) -> Self {
        let mut rng = Rng::new(seed ^ 0xC17E);
        let size = Self::SIZE;
        let x = Axis::generate(size, &mut rng);
        let y = Axis::generate(size, &mut rng);
        let park = (0..x.blocks.len() * y.blocks.len())
            .map(|_| rng.f32() < 0.06)
            .collect();
        Self { size, x, y, park }
    }

    pub fn nx(&self) -> usize {
        self.x.blocks.len()
    }
    pub fn ny(&self) -> usize {
        self.y.blocks.len()
    }

    pub fn is_park(&self, ix: usize, iy: usize) -> bool {
        self.park[iy * self.nx() + ix]
    }

    pub fn is_walkable(&self, x: f32, y: f32) -> bool {
        match (self.x.locate(x), self.y.locate(y)) {
            (Cell::Outside, _) | (_, Cell::Outside) => false,
            (Cell::Band(_), _) | (_, Cell::Band(_)) => true,
            (Cell::Block(i), Cell::Block(j)) => self.is_park(i, j),
        }
    }

    /// 歩ける面積 [m²]
    pub fn walkable_area(&self) -> f32 {
        let total = self.size * self.size;
        let mut built = 0.0;
        for (j, &(y0, y1)) in self.y.blocks.iter().enumerate() {
            for (i, &(x0, x1)) in self.x.blocks.iter().enumerate() {
                if !self.is_park(i, j) {
                    built += (x1 - x0) * (y1 - y0);
                }
            }
        }
        total - built
    }

    /// 建物（街区を 1〜4 区画に割ったもの）: (x0, y0, x1, y1, 高さ)
    pub fn buildings(&self, seed: u64) -> Vec<[f32; 5]> {
        let mut rng = Rng::new(seed ^ 0xB1D6);
        let mut out = vec![];
        for (j, &(y0, y1)) in self.y.blocks.iter().enumerate() {
            for (i, &(x0, x1)) in self.x.blocks.iter().enumerate() {
                if self.is_park(i, j) {
                    continue;
                }
                // 中心ほど高い建物が多い
                let c = (((x0 + x1) * 0.5 - self.size * 0.5)
                    .hypot((y0 + y1) * 0.5 - self.size * 0.5)
                    / (self.size * 0.7))
                    .min(1.0);
                let sx = if x1 - x0 > 70.0 { 2 } else { 1 };
                let sy = if y1 - y0 > 70.0 { 2 } else { 1 };
                let (dx, dy) = ((x1 - x0) / sx as f32, (y1 - y0) / sy as f32);
                for a in 0..sx {
                    for b in 0..sy {
                        let h = rng.range(6.0, 18.0) + rng.f32().powi(3) * 45.0 * (1.0 - c);
                        let gap = 1.0;
                        out.push([
                            x0 + dx * a as f32 + if a > 0 { gap } else { 0.0 },
                            y0 + dy * b as f32 + if b > 0 { gap } else { 0.0 },
                            x0 + dx * (a + 1) as f32 - if a + 1 < sx { gap } else { 0.0 },
                            y0 + dy * (b + 1) as f32 - if b + 1 < sy { gap } else { 0.0 },
                            h,
                        ]);
                    }
                }
            }
        }
        out
    }

    /// GPU に渡す 1m ごとの参照表（行 0 = x, 行 1 = y）。
    /// 各要素は (種類: 0 = 帯 / 1 = 街区, 番号, 始まり, 終わり)
    pub fn axis_table(&self) -> Vec<[f32; 4]> {
        let n = self.size as usize;
        let mut out = Vec::with_capacity(n * 2);
        for axis in [&self.x, &self.y] {
            for v in 0..n {
                let c = v as f32 + 0.5;
                out.push(match axis.locate(c) {
                    Cell::Band(k) => [0.0, k as f32, axis.bands[k].0, axis.bands[k].1],
                    Cell::Block(i) => [1.0, i as f32, axis.blocks[i].0, axis.blocks[i].1],
                    Cell::Outside => [2.0, 0.0, 0.0, 0.0],
                });
            }
        }
        out
    }

    /// GPU に渡す帯の表（行 0 = x の帯, 行 1 = y の帯）。幅は多い方にそろえる
    pub fn band_table(&self) -> (usize, Vec<[f32; 4]>) {
        let w = self.x.bands.len().max(self.y.bands.len());
        let mut out = vec![[0.0; 4]; w * 2];
        for (row, axis) in [&self.x, &self.y].into_iter().enumerate() {
            for (k, &(a, b)) in axis.bands.iter().enumerate() {
                out[row * w + k] = [a, b, 0.0, 0.0];
            }
        }
        (w, out)
    }

    /// 歩ける場所にランダムに n 人置く: (x, y) の列
    pub fn scatter(&self, n: usize, seed: u64) -> Vec<(f32, f32)> {
        let mut rng = Rng::new(seed ^ 0x5CA7);
        let mut out = Vec::with_capacity(n);
        while out.len() < n {
            let (x, y) = (
                rng.range(0.5, self.size - 0.5),
                rng.range(0.5, self.size - 0.5),
            );
            if self.is_walkable(x, y) {
                out.push((x, y));
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn city_has_room_for_a_million() {
        let c = SynthCity::generate(1);
        // 外周は街路
        assert_eq!(c.x.bands[0].0, 0.0);
        assert_eq!(c.x.bands.last().unwrap().1, c.size);
        assert_eq!(c.x.bands.len(), c.x.blocks.len() + 1);
        // 帯と街区が隙間なく交互に並ぶ
        for (k, b) in c.x.blocks.iter().enumerate() {
            assert_eq!(c.x.bands[k].1, b.0);
            assert_eq!(b.1, c.x.bands[k + 1].0);
            assert!(b.0.fract() == 0.0 && b.1.fract() == 0.0);
        }
        let area = c.walkable_area();
        // 100 万人で 1 人/m² を下回る広さ
        assert!(area > 1.0e6, "歩ける面積 {area}");
        assert!(c.is_walkable(1.0, 1000.0));
        let t = c.axis_table();
        assert_eq!(t.len(), 2 * c.size as usize);
        let pts = c.scatter(1000, 3);
        assert!(pts.iter().all(|&(x, y)| c.is_walkable(x, y)));
        assert!(!c.buildings(1).is_empty());
    }
}
