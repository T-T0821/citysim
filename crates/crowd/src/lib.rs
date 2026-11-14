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
    /// 横断歩道・交差点内で渡っている途中の人は対象外で、そのまま渡り切る。
    pub closed: Vec<bool>,
    /// 歩道から踏み込んではいけない車道セル（信号のある所では、横断歩道・交差点内以外の車道）。
    /// 空なら制限なし。経路がそこを通るしかない人（近くに横断歩道が無いなど）だけは渡れる。
    pub curb: Vec<bool>,
    /// 計算モデル（大人数では他者との反発を密度場で近似する軽量モデルにする）
    pub model: Model,
    hash: SpatialHash,
    density: DensityGrid,
    obstacle_bins: ObstacleBins,
    /// 軽量モデル用: セル中心での障害物距離と、障害物から離れる向き（初回に作る）
    wall_cache: Vec<[f32; 3]>,
}

/// 歩行者どうしの相互作用の計算方法
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Model {
    /// 近傍の 1 人ずつとの反発（Social Force Model）。数万人まで
    #[default]
    Full,
    /// 密度場による近似: 前方の混み具合で歩く速さを落とし（Weidmann の基本図）、
    /// 密度の勾配で空いている方へ押す。1 人あたりの計算が近傍の人数によらず一定で、10 万人〜100 万人向け。
    /// すれ違いでの細かな避け合いは表現しない。
    Lite,
}

impl Model {
    /// これより多い人数では自動で軽量モデルにする
    pub const AUTO_LITE_ABOVE: usize = 20_000;
    pub fn auto(n_agents: usize) -> Self {
        if n_agents > Self::AUTO_LITE_ABOVE {
            Model::Lite
        } else {
            Model::Full
        }
    }
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
            curb: vec![],
            model: Model::Full,
            hash,
            density: DensityGrid::default(),
            obstacle_bins: ObstacleBins::default(),
            wall_cache: vec![],
        }
    }

    /// `grid.walk_cost` を変えたあとに、各目的地のフローフィールドを作り直す。
    /// 目的地ごとの計算は独立なので並列に行う。
    pub fn recompute_flows(&mut self) {
        let (grid, goals) = (&self.grid, &self.goals);
        self.goals = par::map(goals.len(), |k| {
            let cells: Vec<_> = grid
                .walkable_cells()
                .filter(|&(x, y)| goals[k].dist[grid.idx(x, y)] == 0.0)
                .collect();
            FlowField::compute(grid, &cells)
        });
    }

    /// p のセルが `closed` か
    fn is_closed(&self, p: Vec2) -> bool {
        !self.closed.is_empty()
            && self
                .grid
                .cell_of(p)
                .is_some_and(|(x, y)| self.closed[self.grid.idx(x, y)])
    }

    /// p のセルが `curb`（横断歩道以外の車道）か
    fn is_curb(&self, p: Vec2) -> bool {
        !self.curb.is_empty()
            && self
                .grid
                .cell_of(p)
                .is_some_and(|(x, y)| self.curb[self.grid.idx(x, y)])
    }

    /// from にいる人が to へ踏み出せないか（赤信号）: 閉じていないセルから閉じたセルへは入れない。
    /// 渡っている途中で赤になった人は、閉じたセルの中なのでそのまま渡り切れる。
    /// 青の横断歩道から赤の交差点内・横断歩道へ踏み込むこともできない。
    fn gated(&self, from: Vec2, to: Vec2) -> bool {
        !self.closed.is_empty() && self.is_closed(to) && !self.is_closed(from)
    }

    /// 赤信号の手前で立ち止まって待つか（歩道や横断歩道以外の車道から、閉じたセルへ向かうとき）。
    /// 青の横断歩道の上では止まらず、閉じたセルの縁に沿って進む。
    fn waits(&self, from: Vec2, to: Vec2) -> bool {
        self.gated(from, to) && (!self.grid.is_carriageway(from) || self.is_curb(from))
    }

    /// 歩道・横断歩道上（from）にいて目的地 goal へ向かう人が to（横断歩道以外の車道）へ踏み出すのを止めるか。
    /// 押されたり避けたりしても車道へははみ出さない。経路そのものが車道を通る人だけは通す。
    fn off_curb(&self, goal: usize, from: Vec2, to: Vec2) -> bool {
        if !self.is_curb(to) || self.is_curb(from) {
            return false;
        }
        match self.goals[goal].next_cell(&self.grid, from) {
            Some(Some((x, y))) => !self.curb[self.grid.idx(x, y)],
            _ => true,
        }
    }

    pub fn active(&self) -> usize {
        self.agents.iter().filter(|a| !a.arrived).count()
    }

    pub fn step(&mut self, dt: f32) {
        match self.model {
            Model::Full => self.step_full(dt),
            Model::Lite => self.step_lite(dt),
        }
    }

    /// 密度場による軽量モデルで 1 ステップ進める（`Model::Lite`）
    fn step_lite(&mut self, dt: f32) {
        if self.wall_cache.len() != self.grid.w * self.grid.h {
            let g = &self.grid;
            self.wall_cache = (0..g.w * g.h)
                .map(|i| {
                    let (d, grad) = g.obstacle_sample(g.center(i % g.w, i / g.w));
                    let n = grad.try_normalize().unwrap_or(Vec2::ZERO);
                    [d, n.x, n.y]
                })
                .collect();
        }
        self.obstacle_bins.rebuild(&self.obstacles);
        self.density.rebuild(&self.grid, &self.agents);
        // 近くにいる人どうしを配列上でも近くに並べた写しで計算する。格子（障害物距離・
        // フローフィールド・密度）を引く場所がそろい、10 万人規模ではキャッシュミスが大きく減る。
        // 計算は 1 人ずつ独立なので、並べ替えても結果は同じ。
        let order = self.density.order(&self.agents);
        let mut work: Vec<Agent> = order
            .iter()
            .map(|&i| self.agents[i as usize].clone())
            .collect();
        let this = &*self;
        par::for_each_mut(&mut work, |_, a| {
            let f = this.force_lite(a);
            this.integrate(a, f, dt);
        });
        for (a, &i) in work.into_iter().zip(&order) {
            self.agents[i as usize] = a;
        }
        self.time += dt;
    }

    fn force_lite(&self, a: &Agent) -> Vec2 {
        if a.arrived {
            return Vec2::ZERO;
        }
        let p = &self.params;
        let e = self.goals[a.goal]
            .direction(&self.grid, a.pos)
            .unwrap_or(Vec2::ZERO);
        let speed = if self.waits(a.pos, a.pos + e * (a.radius + 0.5)) {
            0.0
        } else {
            // 1 m 先の密度で歩く速さを決める（自分の寄与はほぼ入らない距離）
            let rho = self.density.sample(a.pos + e * DensityGrid::CELL);
            a.desired_speed * weidmann(rho)
        };
        let mut f = (e * speed - a.vel) / p.tau;
        // 混んでいる方から空いている方へ押す
        f -= self.density.gradient(a.pos) * LITE_PRESSURE;
        f + self.wall_force_lite(a) + self.obstacle_force(a, e)
    }

    /// 壁からの反発（軽量モデル）: セルごとに前もって求めた距離と勾配を引くだけで済ませる
    fn wall_force_lite(&self, a: &Agent) -> Vec2 {
        let p = &self.params;
        let Some((x, y)) = self.grid.cell_of(a.pos) else {
            return Vec2::ZERO;
        };
        let [dw, gx, gy] = self.wall_cache[self.grid.idx(x, y)];
        if dw > a.radius + WALL_SKIP || (gx == 0.0 && gy == 0.0) {
            return Vec2::ZERO;
        }
        let nw = Vec2::new(gx, gy);
        let overlap = a.radius - dw;
        let mut f = nw * (p.a_wall * (overlap / p.b_wall).exp());
        if overlap > 0.0 {
            f += nw * (p.k_body * overlap);
        }
        f
    }

    /// 壁からの反発（Full モデル）
    fn wall_force(&self, a: &Agent) -> Vec2 {
        let p = &self.params;
        // 壁から十分離れていれば力はほぼ 0（1.5m 先で 1e-3 倍以下）なので、補間と勾配の計算を省く
        if let Some((x, y)) = self.grid.cell_of(a.pos) {
            if self.grid.obstacle_dist[self.grid.idx(x, y)] > a.radius + WALL_SKIP {
                return Vec2::ZERO;
            }
        }
        let (dw, grad) = self.grid.obstacle_sample(a.pos);
        let Some(nw) = grad.try_normalize() else {
            return Vec2::ZERO;
        };
        let overlap = a.radius - dw;
        let mut f = nw * (p.a_wall * (overlap / p.b_wall).exp());
        if overlap > 0.0 {
            f += nw * (p.k_body * overlap);
        }
        f
    }

    /// 車両など動く障害物からの力（両モデル共通）
    fn obstacle_force(&self, a: &Agent, e: Vec2) -> Vec2 {
        let p = &self.params;
        let mut f = Vec2::ZERO;
        for &k in self.obstacle_bins.near(a.pos) {
            let (oa, ob, r) = self.obstacles[k as usize];
            // 端点の外接矩形で先に弾く（ほとんどの車は遠い）
            let reach = r + 3.0;
            if a.pos.x < oa.x.min(ob.x) - reach
                || a.pos.x > oa.x.max(ob.x) + reach
                || a.pos.y < oa.y.min(ob.y) - reach
                || a.pos.y > oa.y.max(ob.y) + reach
            {
                continue;
            }
            let ab = ob - oa;
            let t = ((a.pos - oa).dot(ab) / ab.length_squared().max(1e-6)).clamp(0.0, 1.0);
            let d = a.pos - (oa + ab * t);
            let dist = d.length();
            if dist > reach {
                continue;
            }
            let Some(n) = d.try_normalize() else { continue };
            let overlap = a.radius + r - dist;
            let mag = p.a_wall * 2.0 * (overlap / 0.5).exp() + p.k_body * 2.0 * overlap.max(0.0);
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
        f
    }

    fn step_full(&mut self, dt: f32) {
        self.obstacle_bins.rebuild(&self.obstacles);
        let p = &self.params;
        self.hash.rebuild(&self.agents);
        let mut acc = vec![Vec2::ZERO; self.agents.len()];

        let force = |i: usize, a: &Agent| -> Vec2 {
            if a.arrived {
                return Vec2::ZERO;
            }
            let ff = &self.goals[a.goal];
            let e = ff.direction(&self.grid, a.pos).unwrap_or(Vec2::ZERO);
            // 赤信号の横断歩道の手前では立ち止まって待つ
            let speed = if self.waits(a.pos, a.pos + e * (a.radius + 0.5)) {
                0.0
            } else {
                a.desired_speed
            };
            let mut f = (e * speed - a.vel) / p.tau;

            // 他者（近傍 3x3 区画。区画内は番号順に並んでいるので、合計の順序も番号順）
            self.hash.for_neighbors(a.pos, |j, bpos, brad| {
                if j == i {
                    return;
                }
                let d = a.pos - bpos;
                let dist = d.length().max(1e-4);
                if dist > p.interaction_range {
                    return;
                }
                let n = d / dist;
                let r = a.radius + brad;
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
            f += self.wall_force(a);
            // 車両など動く障害物（壁より強く、少し遠くから避ける）
            f += self.obstacle_force(a, e);
            f
        };
        par::for_each_mut(&mut acc, |i, out| *out = force(i, &self.agents[i]));

        let mut agents = std::mem::take(&mut self.agents);
        let this = &*self;
        par::for_each_mut(&mut agents, |i, a| this.integrate(a, acc[i], dt));
        self.agents = agents;
        self.time += dt;
    }

    /// 力 f で 1 ステップ動かす（壁・閉じたセル・横断歩道以外の車道にはめり込まない）。
    fn integrate(&self, a: &mut Agent, f: Vec2, dt: f32) {
        if a.arrived {
            return;
        }
        let p = &self.params;
        a.vel += f * dt;
        let vmax = a.desired_speed * p.max_speed_factor;
        if a.vel.length() > vmax {
            a.vel = a.vel.normalize() * vmax;
        }
        let next = a.pos + a.vel * dt;
        let from = a.pos;
        let goal = a.goal;
        let ok = |q: Vec2| {
            self.grid.is_walkable(q) && !self.gated(from, q) && !self.off_curb(goal, from, q)
        };
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
}

/// 並列化（ネイティブは rayon、wasm などスレッドの無い環境では逐次）。
/// 各要素の計算が独立なので、どちらでも結果は同じ。
pub mod par {
    /// 十分な量があるときだけスレッドに分ける（少人数ではかえって遅い）
    #[cfg(not(target_arch = "wasm32"))]
    const MIN_PAR: usize = 512;

    #[cfg(not(target_arch = "wasm32"))]
    pub fn for_each_mut<T: Send>(v: &mut [T], f: impl Fn(usize, &mut T) + Sync + Send) {
        use rayon::prelude::*;
        if v.len() >= MIN_PAR {
            v.par_iter_mut()
                .with_min_len(128)
                .enumerate()
                .for_each(|(i, x)| f(i, x));
        } else {
            v.iter_mut().enumerate().for_each(|(i, x)| f(i, x));
        }
    }

    #[cfg(target_arch = "wasm32")]
    pub fn for_each_mut<T: Send>(v: &mut [T], f: impl Fn(usize, &mut T) + Sync + Send) {
        v.iter_mut().enumerate().for_each(|(i, x)| f(i, x));
    }

    /// `0..n` を並列に map して Vec にする（順序は保つ）
    #[cfg(not(target_arch = "wasm32"))]
    pub fn map<R: Send>(n: usize, f: impl Fn(usize) -> R + Sync + Send) -> Vec<R> {
        use rayon::prelude::*;
        (0..n).into_par_iter().map(f).collect()
    }

    #[cfg(target_arch = "wasm32")]
    pub fn map<R: Send>(n: usize, f: impl Fn(usize) -> R + Sync + Send) -> Vec<R> {
        (0..n).map(f).collect()
    }
}

/// 一様グリッドによる近傍探索。
///
/// 区画ごとに番号順で詰めた配列（CSR）にし、位置・半径も同じ順に並べて持つ。
/// 横に並ぶ 3 区画は配列上で連続するので、近傍 3x3 区画は連続した 3 区間を舐めるだけで済む。
struct SpatialHash {
    cell: f32,
    /// 区画の範囲（エージェントの外接矩形に合わせて毎回決める）
    x0: i32,
    y0: i32,
    nx: i32,
    ny: i32,
    /// 区画 c の要素は `ids[start[c]..start[c + 1]]`
    start: Vec<u32>,
    ids: Vec<u32>,
    pos: Vec<glam::Vec2>,
    rad: Vec<f32>,
    key: Vec<u32>,
}

impl SpatialHash {
    fn new(cell: f32) -> Self {
        Self {
            cell,
            x0: 0,
            y0: 0,
            nx: 0,
            ny: 0,
            start: vec![],
            ids: vec![],
            pos: vec![],
            rad: vec![],
            key: vec![],
        }
    }
    fn coord(&self, p: Vec2) -> (i32, i32) {
        (
            (p.x / self.cell).floor() as i32,
            (p.y / self.cell).floor() as i32,
        )
    }
    fn rebuild(&mut self, agents: &[Agent]) {
        let (mut lo, mut hi) = ((i32::MAX, i32::MAX), (i32::MIN, i32::MIN));
        for a in agents.iter().filter(|a| !a.arrived) {
            let (x, y) = self.coord(a.pos);
            lo = (lo.0.min(x), lo.1.min(y));
            hi = (hi.0.max(x), hi.1.max(y));
        }
        self.ids.clear();
        self.pos.clear();
        self.rad.clear();
        if lo.0 > hi.0 {
            self.nx = 0;
            self.ny = 0;
            return;
        }
        (self.x0, self.y0) = (lo.0, lo.1);
        (self.nx, self.ny) = (hi.0 - lo.0 + 1, hi.1 - lo.1 + 1);
        let n_cells = (self.nx * self.ny) as usize;
        self.start.clear();
        self.start.resize(n_cells + 1, 0);
        self.key.clear();
        for a in agents {
            let k = if a.arrived {
                u32::MAX
            } else {
                let (x, y) = self.coord(a.pos);
                ((y - self.y0) * self.nx + (x - self.x0)) as u32
            };
            self.key.push(k);
            if k != u32::MAX {
                self.start[k as usize + 1] += 1;
            }
        }
        for c in 0..n_cells {
            self.start[c + 1] += self.start[c];
        }
        let total = self.start[n_cells] as usize;
        self.ids.resize(total, 0);
        self.pos.resize(total, Vec2::ZERO);
        self.rad.resize(total, 0.0);
        // 番号順に詰める（区画内の順序が番号順になる）
        let mut fill = self.start[..n_cells].to_vec();
        for (i, a) in agents.iter().enumerate() {
            let k = self.key[i];
            if k == u32::MAX {
                continue;
            }
            let s = &mut fill[k as usize];
            let at = *s as usize;
            *s += 1;
            self.ids[at] = i as u32;
            self.pos[at] = a.pos;
            self.rad[at] = a.radius;
        }
    }
    fn for_neighbors(&self, p: Vec2, mut f: impl FnMut(usize, Vec2, f32)) {
        let (kx, ky) = self.coord(p);
        let (cx, cy) = (kx - self.x0, ky - self.y0);
        let xa = (cx - 1).max(0);
        let xb = (cx + 1).min(self.nx - 1);
        if xa > xb {
            return;
        }
        for y in (cy - 1).max(0)..=(cy + 1).min(self.ny - 1) {
            let row = y * self.nx;
            let s = self.start[(row + xa) as usize] as usize;
            let e = self.start[(row + xb + 1) as usize] as usize;
            for k in s..e {
                f(self.ids[k] as usize, self.pos[k], self.rad[k]);
            }
        }
    }
}

/// 壁からこの距離 [m]（＋自分の半径）より離れていれば壁の力を計算しない
const WALL_SKIP: f32 = 1.5;

/// 動く障害物（車）を粗い格子に振り分けたもの。各歩行者は自分の区画にかかる車だけを調べる。
#[derive(Default)]
struct ObstacleBins {
    x0: f32,
    y0: f32,
    nx: usize,
    ny: usize,
    start: Vec<u32>,
    ids: Vec<u32>,
}

impl ObstacleBins {
    const CELL: f32 = 8.0;
    /// 車から力を受ける範囲（半径 + 3m）より広くとる余裕
    const REACH: f32 = 3.5;

    fn rebuild(&mut self, obs: &[(Vec2, Vec2, f32)]) {
        self.start.clear();
        self.ids.clear();
        if obs.is_empty() {
            self.nx = 0;
            self.ny = 0;
            return;
        }
        let bbox = |&(a, b, r): &(Vec2, Vec2, f32)| {
            let m = r + Self::REACH;
            (a.min(b) - Vec2::splat(m), a.max(b) + Vec2::splat(m))
        };
        let (mut lo, mut hi) = (Vec2::splat(f32::MAX), Vec2::splat(f32::MIN));
        for o in obs {
            let (l, h) = bbox(o);
            lo = lo.min(l);
            hi = hi.max(h);
        }
        self.x0 = lo.x;
        self.y0 = lo.y;
        self.nx = ((hi.x - lo.x) / Self::CELL) as usize + 1;
        self.ny = ((hi.y - lo.y) / Self::CELL) as usize + 1;
        let cells = |l: Vec2, h: Vec2, nx: usize, ny: usize, x0: f32, y0: f32| {
            let cx0 = ((l.x - x0) / Self::CELL) as usize;
            let cy0 = ((l.y - y0) / Self::CELL) as usize;
            let cx1 = (((h.x - x0) / Self::CELL) as usize).min(nx - 1);
            let cy1 = (((h.y - y0) / Self::CELL) as usize).min(ny - 1);
            (cy0..=cy1).flat_map(move |y| (cx0..=cx1).map(move |x| y * nx + x))
        };
        let n = self.nx * self.ny;
        self.start.resize(n + 1, 0);
        for o in obs {
            let (l, h) = bbox(o);
            for c in cells(l, h, self.nx, self.ny, self.x0, self.y0) {
                self.start[c + 1] += 1;
            }
        }
        for c in 0..n {
            self.start[c + 1] += self.start[c];
        }
        self.ids.resize(self.start[n] as usize, 0);
        let mut fill = self.start[..n].to_vec();
        for (k, o) in obs.iter().enumerate() {
            let (l, h) = bbox(o);
            for c in cells(l, h, self.nx, self.ny, self.x0, self.y0) {
                self.ids[fill[c] as usize] = k as u32;
                fill[c] += 1;
            }
        }
    }

    /// p の区画にかかる障害物の番号（番号順）
    fn near(&self, p: Vec2) -> &[u32] {
        let (fx, fy) = ((p.x - self.x0) / Self::CELL, (p.y - self.y0) / Self::CELL);
        if self.nx == 0 || fx < 0.0 || fy < 0.0 {
            return &[];
        }
        let (x, y) = (fx as usize, fy as usize);
        if x >= self.nx || y >= self.ny {
            return &[];
        }
        let c = y * self.nx + x;
        &self.ids[self.start[c] as usize..self.start[c + 1] as usize]
    }
}

/// 軽量モデルで密度の勾配から受ける力の強さ [m^4/s^2]
const LITE_PRESSURE: f32 = 1.2;

/// Weidmann (1993) の基本図: 密度 rho [人/m^2] での歩行速度の自由歩行速度に対する比
fn weidmann(rho: f32) -> f32 {
    const RHO_MAX: f32 = 5.4;
    if rho <= 0.05 {
        return 1.0;
    }
    if rho >= RHO_MAX {
        return 0.05;
    }
    (1.0 - (-1.913 * (1.0 / rho - 1.0 / RHO_MAX)).exp()).max(0.05)
}

/// 軽量モデル用の密度場: 1 m 格子に各人を双線形に配り、人/m^2 にする。
#[derive(Default)]
struct DensityGrid {
    origin: Vec2,
    w: usize,
    h: usize,
    rho: Vec<f32>,
}

impl DensityGrid {
    const CELL: f32 = 1.0;

    fn rebuild(&mut self, grid: &WalkGrid, agents: &[Agent]) {
        self.origin = grid.origin;
        self.w = ((grid.w as f32 * grid.cell) / Self::CELL).ceil() as usize + 1;
        self.h = ((grid.h as f32 * grid.cell) / Self::CELL).ceil() as usize + 1;
        self.rho.clear();
        self.rho.resize(self.w * self.h, 0.0);
        let k = 1.0 / (Self::CELL * Self::CELL);
        for a in agents.iter().filter(|a| !a.arrived) {
            let q = (a.pos - self.origin) / Self::CELL - Vec2::splat(0.5);
            let (x0, y0) = (q.x.floor(), q.y.floor());
            let (tx, ty) = (q.x - x0, q.y - y0);
            let (x0, y0) = (x0 as i32, y0 as i32);
            for (dx, dy, wgt) in [
                (0, 0, (1.0 - tx) * (1.0 - ty)),
                (1, 0, tx * (1.0 - ty)),
                (0, 1, (1.0 - tx) * ty),
                (1, 1, tx * ty),
            ] {
                let (x, y) = (x0 + dx, y0 + dy);
                if x >= 0 && y >= 0 && (x as usize) < self.w && (y as usize) < self.h {
                    self.rho[y as usize * self.w + x as usize] += wgt * k;
                }
            }
        }
    }

    /// 到着していない人の番号を、8m 四方の区画ごとにまとめた順で返す（区画内は番号順）
    fn order(&self, agents: &[Agent]) -> Vec<u32> {
        const TILE: f32 = 8.0;
        let tw = ((self.w as f32 * Self::CELL) / TILE) as usize + 1;
        let th = ((self.h as f32 * Self::CELL) / TILE) as usize + 1;
        let key = |p: Vec2| {
            let q = ((p - self.origin) / TILE).max(Vec2::ZERO);
            (q.y as usize).min(th - 1) * tw + (q.x as usize).min(tw - 1)
        };
        let mut start = vec![0u32; tw * th + 1];
        for a in agents.iter().filter(|a| !a.arrived) {
            start[key(a.pos) + 1] += 1;
        }
        for c in 0..tw * th {
            start[c + 1] += start[c];
        }
        let mut out = vec![0u32; start[tw * th] as usize];
        for (i, a) in agents.iter().enumerate().filter(|(_, a)| !a.arrived) {
            let s = &mut start[key(a.pos)];
            out[*s as usize] = i as u32;
            *s += 1;
        }
        out
    }

    fn sample(&self, p: Vec2) -> f32 {
        let q = (p - self.origin) / Self::CELL - Vec2::splat(0.5);
        let (x0, y0) = (q.x.floor(), q.y.floor());
        let (tx, ty) = (q.x - x0, q.y - y0);
        let get = |x: f32, y: f32| {
            if x < 0.0 || y < 0.0 || x as usize >= self.w || y as usize >= self.h {
                0.0
            } else {
                self.rho[y as usize * self.w + x as usize]
            }
        };
        let a = get(x0, y0) * (1.0 - tx) + get(x0 + 1.0, y0) * tx;
        let b = get(x0, y0 + 1.0) * (1.0 - tx) + get(x0 + 1.0, y0 + 1.0) * tx;
        a * (1.0 - ty) + b * ty
    }

    fn gradient(&self, p: Vec2) -> Vec2 {
        let e = Self::CELL * 0.5;
        Vec2::new(
            self.sample(p + Vec2::new(e, 0.0)) - self.sample(p - Vec2::new(e, 0.0)),
            self.sample(p + Vec2::new(0.0, e)) - self.sample(p - Vec2::new(0.0, e)),
        ) / (2.0 * e)
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
    fn lite_model_crosses_the_sample_town_and_slows_in_crowds() {
        let m = plateau_gml::parse_str(include_str!("../../../testdata/sample.gml")).unwrap();
        let g = WalkGrid::from_city(&m, 0.5).unwrap();
        let goal: Vec<_> = g.walkable_cells().filter(|&(x, _)| x + 3 >= g.w).collect();
        let ff = FlowField::compute(&g, &goal);
        let mut sim = Simulation::new(g, vec![ff], Params::default());
        sim.model = Model::Lite;
        let mut rng = Rng::new(7);
        while sim.agents.len() < 300 {
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
        for _ in 0..(400.0 / 0.1) as usize {
            sim.step(0.1);
            assert!(sim.agents.iter().all(|a| sim.grid.is_walkable(a.pos)));
            if sim.active() == 0 {
                break;
            }
        }
        assert_eq!(sim.active(), 0, "全員が 400 秒以内に到着すること");
        assert_eq!(weidmann(0.0), 1.0);
        assert!(weidmann(2.0) < 0.7 && weidmann(4.0) < weidmann(2.0));
        assert_eq!(Model::auto(1000), Model::Full);
        assert_eq!(Model::auto(100_000), Model::Lite);
    }

    #[test]
    fn spatial_hash_finds_all_neighbors_in_index_order() {
        let mut rng = Rng::new(11);
        let agents: Vec<Agent> = (0..2000)
            .map(|i| Agent {
                pos: Vec2::new(rng.range(-40.0, 60.0), rng.range(-30.0, 30.0)),
                vel: Vec2::ZERO,
                radius: 0.25,
                desired_speed: 1.34,
                goal: 0,
                arrived: i % 7 == 0,
            })
            .collect();
        let range = 2.5;
        let mut h = SpatialHash::new(range);
        h.rebuild(&agents);
        for a in agents.iter().step_by(13) {
            let mut got = vec![];
            h.for_neighbors(a.pos, |j, p, r| {
                assert_eq!(p, agents[j].pos);
                assert_eq!(r, agents[j].radius);
                if a.pos.distance(p) <= range {
                    got.push(j);
                }
            });
            let want: Vec<usize> = (0..agents.len())
                .filter(|&j| !agents[j].arrived && a.pos.distance(agents[j].pos) <= range)
                .collect();
            got.sort_unstable();
            assert_eq!(got, want);
        }
        // 全員到着していても落ちない
        let done: Vec<Agent> = agents
            .iter()
            .cloned()
            .map(|mut a| {
                a.arrived = true;
                a
            })
            .collect();
        h.rebuild(&done);
        h.for_neighbors(Vec2::ZERO, |_, _, _| panic!("誰もいない"));
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
