//! 災害避難モード: 指定した避難所へ全員が一斉に向かう。
//!
//! - 避難所ごとにフローフィールドを作り、各人は歩いて最も近い（経路距離が最短の）避難所へ向かう
//! - 避難中は車が止まる前提で、車道を歩くコストを下げる（車道も使って最短で逃げる）
//! - 避難所の周囲 `radius` [m] に入ったら避難完了。時刻を記録して完了率・90% 完了時間などを出す

use crate::Simulation;
use glam::Vec2;
use walkmap::FlowField;

#[derive(Debug, Clone)]
pub struct EvacParams {
    /// 避難所の受け入れ半径 [m]（この範囲に入ったら完了）
    pub radius: f32,
    /// 希望歩行速度の倍率（急ぎ足）
    pub speed_factor: f32,
    /// 避難中に車道を歩くコスト（通常時は 6 倍）
    pub carriageway_cost: f32,
}

impl Default for EvacParams {
    fn default() -> Self {
        Self {
            radius: 4.0,
            speed_factor: 1.2,
            carriageway_cost: 1.5,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Shelter {
    pub name: String,
    pub pos: Vec2,
    /// 避難所に到着した人数
    pub arrived: usize,
}

/// 進行中の避難の状態。
#[derive(Debug, Clone)]
pub struct Evacuation {
    pub shelters: Vec<Shelter>,
    /// `Simulation::goals` のうち避難所の分の先頭インデックス
    pub first_goal: usize,
    pub start_time: f32,
    /// 避難開始時に範囲内にいた人数
    pub total: usize,
    /// どの避難所にもたどり着けない人数（建物に囲まれた中庭など）
    pub unreachable: usize,
    /// 避難完了までの所要時間 [s]（完了順）
    pub times: Vec<f32>,
    tracked: Vec<bool>,
    /// 避難所ごとの到着セル（経路の引き直しに使う）
    cells: Vec<Vec<(usize, usize)>>,
    carriageway_cost: f32,
    /// 止まった車を避ける経路に引き直したか
    pub rerouted: bool,
}

impl Evacuation {
    /// 避難を始める。到着済みでない全員の目的地を最寄りの避難所に切り替える。
    /// 避難所が 1 つも歩行空間の近くに無ければ None。
    pub fn start(
        sim: &mut Simulation,
        shelters: &[(String, Vec2)],
        p: &EvacParams,
    ) -> Option<Self> {
        let first_goal = sim.goals.len();
        let mut list = vec![];
        let mut all_cells = vec![];
        for (name, pos) in shelters {
            let cells = shelter_cells(sim, *pos, p.radius);
            if cells.is_empty() {
                continue;
            }
            all_cells.push(cells);
            list.push(Shelter {
                name: name.clone(),
                pos: *pos,
                arrived: 0,
            });
        }
        if list.is_empty() {
            return None;
        }
        let fields = compute_fields(sim, &all_cells, p.carriageway_cost, &[]);
        // 避難中は信号待ちをしない（車は止まる）
        sim.closed.clear();
        sim.goals.extend(fields);
        let mut total = 0;
        let mut unreachable = 0;
        let mut tracked = vec![false; sim.agents.len()];
        for (i, a) in sim.agents.iter_mut().enumerate() {
            if a.arrived {
                continue;
            }
            total += 1;
            let best = (first_goal..sim.goals.len())
                .map(|g| (g, sim.goals[g].distance(&sim.grid, a.pos)))
                .filter(|(_, d)| d.is_finite())
                .min_by(|a, b| a.1.total_cmp(&b.1));
            match best {
                Some((g, _)) => {
                    a.goal = g;
                    a.desired_speed *= p.speed_factor;
                    tracked[i] = true;
                }
                None => unreachable += 1,
            }
        }
        Some(Self {
            shelters: list,
            first_goal,
            start_time: sim.time,
            total,
            unreachable,
            times: vec![],
            tracked,
            cells: all_cells,
            carriageway_cost: p.carriageway_cost,
            rerouted: false,
        })
    }

    /// 止まった車（カプセル: 端点 2 つと半径）を避けるように経路を引き直す。
    /// 車が止まりきってから一度呼ぶと、車の列の前で押し合って動けなくなるのを防げる。
    pub fn reroute_around(&mut self, sim: &mut Simulation, obstacles: &[(Vec2, Vec2, f32)]) {
        let fields = compute_fields(sim, &self.cells, self.carriageway_cost, obstacles);
        for (k, f) in fields.into_iter().enumerate() {
            sim.goals[self.first_goal + k] = f;
        }
        self.rerouted = true;
    }

    /// 1 ステップごとに呼び、到着した人を数える。
    pub fn update(&mut self, sim: &Simulation) {
        for (i, a) in sim.agents.iter().enumerate() {
            if a.arrived && self.tracked.get(i).copied().unwrap_or(false) {
                self.tracked[i] = false;
                if let Some(s) = self.shelters.get_mut(a.goal - self.first_goal) {
                    s.arrived += 1;
                }
                self.times.push(sim.time - self.start_time);
            }
        }
    }

    pub fn evacuated(&self) -> usize {
        self.times.len()
    }
    /// 避難中（まだ向かっている）人数
    pub fn remaining(&self) -> usize {
        self.tracked.iter().filter(|&&t| t).count()
    }
    pub fn done(&self) -> bool {
        self.remaining() == 0
    }
    /// 避難対象のうち割合 `q` (0〜1) が避難し終えた時刻（開始からの秒数）。まだなら None。
    pub fn time_to_fraction(&self, q: f32) -> Option<f32> {
        let need = ((self.total - self.unreachable) as f32 * q).ceil() as usize;
        if need == 0 {
            return Some(0.0);
        }
        self.times.get(need - 1).copied()
    }
}

/// 車道のコストを下げ、止まった車の周りを通りにくくしてフローフィールドを作る（grid のコストは元に戻す）。
fn compute_fields(
    sim: &mut Simulation,
    cells: &[Vec<(usize, usize)>],
    carriageway_cost: f32,
    obstacles: &[(Vec2, Vec2, f32)],
) -> Vec<FlowField> {
    let saved = sim.grid.walk_cost.clone();
    let g = &mut sim.grid;
    for (c, &cw) in g.walk_cost.iter_mut().zip(g.carriageway.iter()) {
        if cw {
            *c = carriageway_cost;
        }
    }
    for &(a, b, r) in obstacles {
        let r = r + 0.4;
        let lo = ((a.min(b) - Vec2::splat(r) - g.origin) / g.cell).floor().as_ivec2();
        let hi = ((a.max(b) + Vec2::splat(r) - g.origin) / g.cell).ceil().as_ivec2();
        for y in lo.y.max(0)..=hi.y.min(g.h as i32 - 1) {
            for x in lo.x.max(0)..=hi.x.min(g.w as i32 - 1) {
                let c = g.center(x as usize, y as usize);
                let ab = b - a;
                let t = ((c - a).dot(ab) / ab.length_squared().max(1e-6)).clamp(0.0, 1.0);
                if c.distance(a + ab * t) <= r {
                    let i = g.idx(x as usize, y as usize);
                    g.walk_cost[i] = g.walk_cost[i].max(40.0);
                }
            }
        }
    }
    let out = cells
        .iter()
        .map(|c| FlowField::compute(&sim.grid, c))
        .collect();
    sim.grid.walk_cost = saved;
    out
}

/// 避難所の位置から半径 r 内の歩行可能セル。無ければ最も近い歩行可能セル（30m 以内）の周り。
fn shelter_cells(sim: &Simulation, pos: Vec2, r: f32) -> Vec<(usize, usize)> {
    let g = &sim.grid;
    let near = |c: Vec2, r: f32| -> Vec<(usize, usize)> {
        let n = (r / g.cell).ceil() as i64 + 1;
        let q = ((c - g.origin) / g.cell).floor().as_i64vec2();
        let mut out = vec![];
        for y in q.y - n..=q.y + n {
            for x in q.x - n..=q.x + n {
                if x < 0 || y < 0 || x as usize >= g.w || y as usize >= g.h {
                    continue;
                }
                let (x, y) = (x as usize, y as usize);
                if g.walkable[g.idx(x, y)] && g.center(x, y).distance(c) <= r {
                    out.push((x, y));
                }
            }
        }
        out
    };
    let cells = near(pos, r);
    if !cells.is_empty() {
        return cells;
    }
    // 避難所が建物の中（公園の中央・校舎など）を指していたら、最寄りの歩行空間を入口とみなす
    let around = near(pos, 30.0);
    match around
        .iter()
        .min_by(|a, b| {
            g.center(a.0, a.1)
                .distance(pos)
                .total_cmp(&g.center(b.0, b.1).distance(pos))
        }) {
        Some(&(x, y)) => near(g.center(x, y), r),
        None => vec![],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use walkmap::WalkGrid;

    #[test]
    fn everyone_reaches_nearest_shelter() {
        let m = plateau_gml::parse_str(include_str!("../../../testdata/sample.gml")).unwrap();
        let g = WalkGrid::from_city(&m, 0.5).unwrap();
        let (mut sim, _) = crate::scenario::through_traffic(g, 150, 3).unwrap();
        for _ in 0..100 {
            sim.step(0.05);
        }
        // 「井」の字の道路の西の交差点付近と東の端
        let shelters = vec![
            ("西".to_string(), Vec2::new(70.0, 100.0)),
            ("東".to_string(), Vec2::new(195.0, 130.0)),
        ];
        let mut ev = Evacuation::start(&mut sim, &shelters, &EvacParams::default()).unwrap();
        assert_eq!(ev.shelters.len(), 2);
        assert!(ev.total > 100);
        for _ in 0..(400.0 / 0.05) as usize {
            sim.step(0.05);
            ev.update(&sim);
            if ev.done() {
                break;
            }
        }
        assert!(ev.done(), "残り {}", ev.remaining());
        assert_eq!(ev.evacuated() + ev.unreachable, ev.total);
        assert!(ev.shelters.iter().all(|s| s.arrived > 0), "{:?}", ev.shelters);
        let t90 = ev.time_to_fraction(0.9).unwrap();
        let t100 = ev.time_to_fraction(1.0).unwrap();
        assert!(t90 <= t100 && t100 > 0.0);
    }
}
