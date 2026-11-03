//! 建物の押し出しメッシュと歩行面メッシュの生成。
//!
//! 座標系: シミュレーションの局所平面 (x=東, y=北) を bevy の (x, -z) に、高さを y に対応させる。

use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use bevy::render::mesh::{Indices, PrimitiveTopology};
use walkmap::WalkGrid;

#[inline]
pub fn to_world(p: Vec2, h: f32) -> Vec3 {
    Vec3::new(p.x, h, -p.y)
}

#[derive(Default)]
pub struct MeshBuilder {
    pos: Vec<[f32; 3]>,
    nrm: Vec<[f32; 3]>,
    idx: Vec<u32>,
}

impl MeshBuilder {
    fn vert(&mut self, p: Vec3, n: Vec3) -> u32 {
        self.pos.push(p.to_array());
        self.nrm.push(n.to_array());
        (self.pos.len() - 1) as u32
    }
    /// 反時計回り (上/外から見て) の四角形
    fn quad(&mut self, a: Vec3, b: Vec3, c: Vec3, d: Vec3, n: Vec3) {
        let i = [
            self.vert(a, n),
            self.vert(b, n),
            self.vert(c, n),
            self.vert(d, n),
        ];
        self.idx
            .extend_from_slice(&[i[0], i[1], i[2], i[0], i[2], i[3]]);
    }
    pub fn is_empty(&self) -> bool {
        self.idx.is_empty()
    }
    pub fn build(self) -> Mesh {
        Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        )
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, self.pos)
        .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, self.nrm)
        .with_inserted_indices(Indices::U32(self.idx))
    }

    /// フットプリントを高さ `h` まで押し出す（壁 + 屋根）。
    pub fn add_extrusion(&mut self, footprint: &[Vec2], h: f32) {
        let mut poly: Vec<Vec2> = footprint.to_vec();
        if poly.len() > 1 && poly.first().unwrap().distance(*poly.last().unwrap()) < 1e-3 {
            poly.pop();
        }
        poly.dedup_by(|a, b| a.distance(*b) < 1e-3);
        if poly.len() < 3 {
            return;
        }
        if signed_area(&poly) < 0.0 {
            poly.reverse();
        }
        // 壁: 平面上で反時計回りなので外向き法線は (dy, -dx)
        for i in 0..poly.len() {
            let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
            let Some(n2) = Vec2::new(b.y - a.y, a.x - b.x).try_normalize() else {
                continue;
            };
            let n = Vec3::new(n2.x, 0.0, -n2.y);
            self.quad(
                to_world(a, 0.0),
                to_world(b, 0.0),
                to_world(b, h),
                to_world(a, h),
                n,
            );
        }
        // 屋根
        let base: Vec<u32> = poly
            .iter()
            .map(|&p| self.vert(to_world(p, h), Vec3::Y))
            .collect();
        for [a, b, c] in ear_clip(&poly) {
            // +Y から見下ろすと (x, -z) は平面の (x, y) と同じ向きなので、そのまま反時計回り
            self.idx.extend_from_slice(&[base[a], base[b], base[c]]);
        }
    }
}

/// 歩行可能セルを行ごとの連続区間にまとめて板ポリゴン化する。
pub fn walkable_surface(grid: &WalkGrid, h: f32) -> MeshBuilder {
    let mut mb = MeshBuilder::default();
    for y in 0..grid.h {
        let mut x = 0;
        while x < grid.w {
            if !grid.walkable[grid.idx(x, y)] {
                x += 1;
                continue;
            }
            let x0 = x;
            while x < grid.w && grid.walkable[grid.idx(x, y)] {
                x += 1;
            }
            let c = grid.cell;
            let (p0, p1) = (
                grid.origin + Vec2::new(x0 as f32, y as f32) * c,
                grid.origin + Vec2::new(x as f32, y as f32 + 1.0) * c,
            );
            mb.quad(
                to_world(Vec2::new(p0.x, p0.y), h),
                to_world(Vec2::new(p1.x, p0.y), h),
                to_world(Vec2::new(p1.x, p1.y), h),
                to_world(Vec2::new(p0.x, p1.y), h),
                Vec3::Y,
            );
        }
    }
    mb
}

fn signed_area(p: &[Vec2]) -> f32 {
    let n = p.len();
    (0..n).map(|i| p[i].perp_dot(p[(i + 1) % n])).sum::<f32>() * 0.5
}

/// 反時計回り単純多角形の耳切り三角形分割。失敗時は扇形分割にフォールバック。
fn ear_clip(poly: &[Vec2]) -> Vec<[usize; 3]> {
    let mut idx: Vec<usize> = (0..poly.len()).collect();
    let mut tris = Vec::with_capacity(poly.len());
    let mut guard = 0;
    while idx.len() > 3 && guard < poly.len() * poly.len() {
        guard += 1;
        let n = idx.len();
        let mut clipped = false;
        for i in 0..n {
            let (ia, ib, ic) = (idx[(i + n - 1) % n], idx[i], idx[(i + 1) % n]);
            let (a, b, c) = (poly[ia], poly[ib], poly[ic]);
            if (b - a).perp_dot(c - b) <= 1e-9 {
                continue; // 凹頂点
            }
            let inside = idx
                .iter()
                .any(|&j| j != ia && j != ib && j != ic && point_in_tri(poly[j], a, b, c));
            if inside {
                continue;
            }
            tris.push([ia, ib, ic]);
            idx.remove(i);
            clipped = true;
            break;
        }
        if !clipped {
            break;
        }
    }
    if idx.len() == 3 {
        tris.push([idx[0], idx[1], idx[2]]);
    } else if idx.len() > 3 {
        for k in 1..idx.len() - 1 {
            tris.push([idx[0], idx[k], idx[k + 1]]);
        }
    }
    tris
}

fn point_in_tri(p: Vec2, a: Vec2, b: Vec2, c: Vec2) -> bool {
    let d1 = (b - a).perp_dot(p - a);
    let d2 = (c - b).perp_dot(p - b);
    let d3 = (a - c).perp_dot(p - c);
    d1 >= 0.0 && d2 >= 0.0 && d3 >= 0.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ear_clip_l_shape() {
        let l = [
            Vec2::new(0., 0.),
            Vec2::new(2., 0.),
            Vec2::new(2., 1.),
            Vec2::new(1., 1.),
            Vec2::new(1., 2.),
            Vec2::new(0., 2.),
        ];
        let tris = ear_clip(&l);
        assert_eq!(tris.len(), 4);
        let area: f32 = tris
            .iter()
            .map(|t| (l[t[1]] - l[t[0]]).perp_dot(l[t[2]] - l[t[0]]) * 0.5)
            .sum();
        assert!((area - 3.0).abs() < 1e-5);
    }
}
