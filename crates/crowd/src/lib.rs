//! Social Force Model (Helbing & Molnár 1995 / Helbing et al. 2000) による歩行者シミュレーション。
//!
//! 各エージェントに働く力:
//! - 駆動力: 目的方向 (フローフィールド) へ希望速度で歩こうとする
//! - 他者からの反発: 指数関数的な心理的反発 + 接触時の物理力
//! - 壁からの反発: 障害物距離場の勾配方向

pub mod evacuation;
pub mod event;
pub mod png;
pub mod report;
pub mod scenario;

use glam::Vec2;
use walkmap::{FlowField, WalkGrid};

#[derive(Debug, Clone)]
pub struct Params {
    pub tau: f32,     // 緩和時間 [s]
    pub a_agent: f32, // 他者反発の強さ [m/s^2]
    pub b_agent: f32, // 他者反発の到達距離 [m]
    pub a_wall: f32,
    pub b_wall: f32,
    pub k_body: f32, // 接触時の押し返し [1/s^2]
    pub interaction_range: f32,
    pub max_speed_factor: f32,
    /// 視野の異方性 (0=後ろは無視, 1=等方)
    pub lambda: f32,
}

impl Default for Params {
    fn default() -> Self {
        Self {
            tau: 0.5,
            a_agent: 2.1,
            b_agent: 0.3,
            a_wall: 5.0,
            b_wall: 0.2,
            k_body: 120.0,
            interaction_range: 2.5,
            max_speed_factor: 1.3,
            lambda: 0.35,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Agent {
    pub pos: Vec2,
    pub vel: Vec2,
    pub radius: f32,
    pub desired_speed: f32,
    pub goal: usize,
    pub arrived: bool,
}

pub struct Simulation {
    pub grid: WalkGrid,
    pub goals: Vec<FlowField>,
    pub agents: Vec<Agent>,
    pub params: Params,
    pub time: f32,
    /// 動く障害物（車両など）: カプセルの端点 2 つと半径。毎ステップ外から更新する。
    pub obstacles: Vec<(Vec2, Vec2, f32)>,
    /// 歩道から踏み出してはいけないセル（赤信号の横断歩道など）。空なら制限なし。
    /// 車道上にいる人（渡っている途中の人）は対象外で、そのまま渡り切る。
    pub closed: Vec<bool>,
    hash: SpatialHash,
}

impl Simulation {
    pub fn new(grid: WalkGrid, goals: Vec<FlowField>, params: Params) -> Self {
        let hash = SpatialHash::new(params.interaction_range);
        Self {
            grid,
            goals,
            agents: vec![],
            params,
            time: 0.0,
            obstacles: vec![],
            closed: vec![],
            hash,
        }
    }

    /// `grid.walk_cost` を変えたあとに、各目的地のフローフィールドを作り直す。
    pub fn recompute_flows(&mut self) {
        for ff in self.goals.iter_mut() {
            let cells: Vec<_> = self
                .grid
                .walkable_cells()
                .filter(|&(x, y)| ff.dist[self.grid.idx(x, y)] == 0.0)
                .collect();
            *ff = FlowField::compute(&self.grid, &cells);
        }
    }

    /// p のセルが `closed` か
    fn is_closed(&self, p: Vec2) -> bool {
        !self.closed.is_empty()
            && self
                .grid
                .cell_of(p)
                .is_some_and(|(x, y)| self.closed[self.grid.idx(x, y)])
    }

    /// from にいる人が to へ踏み出すのを止めるか（歩道上から閉じたセルへ）
    fn gated(&self, from: Vec2, to: Vec2) -> bool {
        !self.closed.is_empty()
            && !self.grid.is_carriageway(from)
            && !self.is_closed(from)
            && self.is_closed(to)
    }

    pub fn active(&self) -> usize {
        self.agents.iter().filter(|a| !a.arrived).count()
    }

    pub fn step(&mut self, dt: f32) {
        let p = &self.params;
        self.hash
            .rebuild(self.agents.iter().map(|a| (a.pos, !a.arrived)));
        let mut acc = vec![Vec2::ZERO; self.agents.len()];

        for (i, a) in self.agents.iter().enumerate() {
            if a.arrived {
                continue;
            }
            let ff = &self.goals[a.goal];
            let e = ff.direction(&self.grid, a.pos).unwrap_or(Vec2::ZERO);
            // 赤信号の横断歩道の手前では立ち止まって待つ
            let speed = if self.gated(a.pos, a.pos + e * (a.radius + 0.5)) {
                0.0
            } else {
                a.desired_speed
            };
            let mut f = (e * speed - a.vel) / p.tau;

            // 他者
            self.hash.for_neighbors(a.pos, |j| {
                if j == i {
                    return;
                }
                let b = &self.agents[j];
                let d = a.pos - b.pos;
                let dist = d.length().max(1e-4);
                if dist > p.interaction_range {
                    return;
                }
                let n = d / dist;
                let r = a.radius + b.radius;
                let overlap = r - dist;
                // 前方の相手ほど強く意識する
                let cos_phi = e.dot(-n);
                let w = p.lambda + (1.0 - p.lambda) * 0.5 * (1.0 + cos_phi);
                f += n * (p.a_agent * (overlap / p.b_agent).exp() * w);
                if overlap > 0.0 {
                    f += n * (p.k_body * overlap);
                }
            });

            // 壁
            let (dw, grad) = self.grid.obstacle_sample(a.pos);
            if let Some(nw) = grad.try_normalize() {
                let overlap = a.radius - dw;
                f += nw * (p.a_wall * (overlap / p.b_wall).exp());
                if overlap > 0.0 {
                    f += nw * (p.k_body * overlap);
                }
            }
            // 車両など動く障害物（壁より強く、少し遠くから避ける）
            for &(oa, ob, r) in &self.obstacles {
                let ab = ob - oa;
                let t = ((a.pos - oa).dot(ab) / ab.length_squared().max(1e-6)).clamp(0.0, 1.0);
                let d = a.pos - (oa + ab * t);
                let dist = d.length();
                if dist > r + 3.0 {
                    continue;
                }
                let Some(n) = d.try_normalize() else { continue };
                let overlap = a.radius + r - dist;
                let mag =
                    p.a_wall * 2.0 * (overlap / 0.5).exp() + p.k_body * 2.0 * overlap.max(0.0);
                // 押し付けられて壁際で動けなくならないよう上限を設ける
                f += n * mag.min(40.0);
                // 行く手をふさぐ車は車体に沿って回り込む（止まった車の前で押し合って動けなくならない）
                let ahead = e.dot(-n);
                if ahead > 0.0 && dist < r + 1.5 {
                    let axis = ab.try_normalize().unwrap_or(n.perp());
                    let side = if axis.dot(e).abs() > 0.1 {
                        axis.dot(e).signum()
                    } else {
                        1.0
                    };
                    f += axis * side * (ahead * a.desired_speed / p.tau);
                }
            }
            acc[i] = f;
        }

        let mut agents = std::mem::take(&mut self.agents);
        for (a, f) in agents.iter_mut().zip(acc) {
            if a.arrived {
                continue;
            }
            a.vel += f * dt;
            let vmax = a.desired_speed * p.max_speed_factor;
            if a.vel.length() > vmax {
                a.vel = a.vel.normalize() * vmax;
            }
            let next = a.pos + a.vel * dt;
            let from = a.pos;
            let ok = |q: Vec2| self.grid.is_walkable(q) && !self.gated(from, q);
            // 数値的に壁へめり込んだら軸ごとに止める
            if ok(next) {
                a.pos = next;
            } else {
                let nx = Vec2::new(next.x, a.pos.y);
                let ny = Vec2::new(a.pos.x, next.y);
                if ok(nx) {
                    a.pos = nx;
                    a.vel.y = 0.0;
                } else if ok(ny) {
                    a.pos = ny;
                    a.vel.x = 0.0;
                } else {
                    a.vel = Vec2::ZERO;
                }
            }
            if self.goals[a.goal].distance(&self.grid, a.pos) < 1.0 {
                a.arrived = true;
            }
        }
        self.agents = agents;
        self.time += dt;
    }
}

/// 一様グリッドによる近傍探索。
struct SpatialHash {
    cell: f32,
    map: std::collections::HashMap<(i32, i32), Vec<usize>>,
}

impl SpatialHash {
    fn new(cell: f32) -> Self {
        Self {
            cell,
            map: Default::default(),
        }
    }
    fn key(&self, p: Vec2) -> (i32, i32) {
        (
            (p.x / self.cell).floor() as i32,
            (p.y / self.cell).floor() as i32,
        )
    }
    fn rebuild(&mut self, pts: impl Iterator<Item = (Vec2, bool)>) {
        for v in self.map.values_mut() {
            v.clear();
        }
        for (i, (p, on)) in pts.enumerate() {
            if on {
                let k = self.key(p);
                self.map.entry(k).or_default().push(i);
            }
        }
    }
    fn for_neighbors(&self, p: Vec2, mut f: impl FnMut(usize)) {
        let (kx, ky) = self.key(p);
        for dy in -1..=1 {
            for dx in -1..=1 {
                if let Some(v) = self.map.get(&(kx + dx, ky + dy)) {
                    v.iter().for_each(|&j| f(j));
                }
            }
        }
    }
}

/// 依存なしの小さな乱数 (xorshift64*)。再現性のためシード指定。
pub struct Rng(u64);
impl Rng {
    pub fn new(seed: u64) -> Self {
        Self(seed.max(1))
    }
    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    pub fn f32(&mut self) -> f32 {
        (self.next_u64() >> 40) as f32 / (1u64 << 24) as f32
    }
    pub fn range(&mut self, lo: f32, hi: f32) -> f32 {
        lo + (hi - lo) * self.f32()
    }
    /// 平均 mu, 標準偏差 sigma の正規乱数 (Box–Muller)
    pub fn normal(&mut self, mu: f32, sigma: f32) -> f32 {
        let u1 = self.f32().max(1e-7);
        let u2 = self.f32();
        mu + sigma * (-2.0 * u1.ln()).sqrt() * (std::f32::consts::TAU * u2).cos()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn agents_cross_the_sample_town() {
        let m = plateau_gml::parse_str(include_str!("../../../testdata/sample.gml")).unwrap();
        let g = WalkGrid::from_city(&m, 0.5).unwrap();
        let goal: Vec<_> = g.walkable_cells().filter(|&(x, _)| x + 3 >= g.w).collect();
        let ff = FlowField::compute(&g, &goal);
        let mut sim = Simulation::new(g, vec![ff], Params::default());
        let mut rng = Rng::new(7);
        while sim.agents.len() < 40 {
            let p = Vec2::new(rng.range(1.0, 10.0), rng.range(61.0, 79.0));
            if sim.grid.obstacle_sample(p).0 > 0.4 {
                sim.agents.push(Agent {
                    pos: p,
                    vel: Vec2::ZERO,
                    radius: 0.25,
                    desired_speed: 1.34,
                    goal: 0,
                    arrived: false,
                });
            }
        }
        for _ in 0..(300.0 / 0.05) as usize {
            sim.step(0.05);
            if sim.active() == 0 {
                break;
            }
        }
        assert_eq!(sim.active(), 0, "全員が 300 秒以内に到着すること");
        assert!(sim.agents.iter().all(|a| sim.grid.is_walkable(a.pos)));
    }

    #[test]
    fn pedestrians_wait_at_closed_cells() {
        let m = plateau_gml::parse_str(include_str!("../../../testdata/sample.gml")).unwrap();
        let mut g = WalkGrid::from_city(&m, 0.5).unwrap();
        g.mark_carriageway(3.0, 6.0);
        let goal: Vec<_> = g.walkable_cells().filter(|&(x, _)| x + 3 >= g.w).collect();
        let ff = FlowField::compute(&g, &goal);
        let n = g.w * g.h;
        let mut sim = Simulation::new(g, vec![ff], Params::default());
        // 車道をすべて閉じる: 歩道上の人は車道へ出られない
        sim.closed = sim.grid.carriageway.clone();
        assert_eq!(sim.closed.len(), n);
        let mut rng = Rng::new(3);
        while sim.agents.len() < 30 {
            let p = Vec2::new(rng.range(1.0, 60.0), rng.range(61.0, 79.0));
            if sim.grid.obstacle_sample(p).0 > 0.4 && !sim.grid.is_carriageway(p) {
                sim.agents.push(Agent {
                    pos: p,
                    vel: Vec2::ZERO,
                    radius: 0.25,
                    desired_speed: 1.34,
                    goal: 0,
                    arrived: false,
                });
            }
        }
        for _ in 0..1200 {
            sim.step(0.05);
            assert!(
                sim.agents.iter().all(|a| !sim.grid.is_carriageway(a.pos)),
                "閉じた車道に入らない"
            );
        }
        // 開ければ渡れる
        sim.closed.iter_mut().for_each(|c| *c = false);
        for _ in 0..(300.0 / 0.05) as usize {
            sim.step(0.05);
        }
        assert_eq!(sim.active(), 0);
    }
}
