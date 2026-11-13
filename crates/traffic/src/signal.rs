//! 信号機。
//!
//! - 交差点（近接する分岐ノードはまとめて 1 つ）ごとに、流入路を道路の向きで 2 つの現示グループに分ける。
//!   主道路（車道が最も広い流入路の軸）がグループ 0、それと交差する向きがグループ 1。
//! - 車両現示: 主道路 青 → 黄 → 全赤 → 従道路 青 → 黄 → 全赤（→ 歩行者専用現示）を繰り返す。
//! - 歩行者信号: 各流入路の口に横断歩道を置く。歩行者の方式は 3 通り。
//!   - 歩車同時（標準）: 横断歩道は、並行する車両グループが青の間に青（終わりの数秒は点滅）
//!   - 歩車分離: 車両が全方向赤になる歩行者専用現示でだけ全横断歩道が青
//!   - スクランブル: 歩車分離に加え、歩行者専用現示では交差点内を斜めにも渡れる
//! - 車は赤・黄（止まれる場合）で停止線（横断歩道の手前）に止まる。
//!   歩行者は青でない横断歩道・交差点内へ歩道から踏み出さない（渡り始めた人はそのまま渡り切る）。

use crate::graph::RoadGraph;
use crate::Lane;
use glam::Vec2;
use std::collections::HashMap;
use walkmap::WalkGrid;

/// 歩行者信号の方式。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PedMode {
    /// 歩車同時式（標準）
    Concurrent,
    /// 歩車分離式（歩行者専用現示あり、斜め横断なし）
    Exclusive,
    /// スクランブル（歩行者専用現示で斜め横断も可）
    Scramble,
    /// 広い道路どうしの交差点だけスクランブル、それ以外は歩車同時
    Auto,
}

impl PedMode {
    pub const ALL: [PedMode; 4] = [
        PedMode::Concurrent,
        PedMode::Exclusive,
        PedMode::Scramble,
        PedMode::Auto,
    ];
    pub fn label(self) -> &'static str {
        match self {
            PedMode::Concurrent => "歩車同時（標準）",
            PedMode::Exclusive => "歩車分離",
            PedMode::Scramble => "スクランブル",
            PedMode::Auto => "自動（大交差点はスクランブル）",
        }
    }
    pub fn parse(s: &str) -> Option<Self> {
        Some(match s {
            "concurrent" | "standard" => PedMode::Concurrent,
            "exclusive" => PedMode::Exclusive,
            "scramble" => PedMode::Scramble,
            "auto" => PedMode::Auto,
            _ => return None,
        })
    }
}

#[derive(Debug, Clone)]
pub struct SignalConfig {
    pub mode: PedMode,
    /// サイクル長 [s]
    pub cycle: f32,
    /// 車両の青時間のうち主道路に配る割合
    pub main_split: f32,
    pub yellow: f32,
    pub all_red: f32,
    /// 歩行者専用現示の青 [s]（歩車分離・スクランブル）
    pub ped_green: f32,
    /// 歩行者青点滅 [s]
    pub ped_flash: f32,
    /// 信号を置く交差点の最小流入路数
    pub min_approaches: usize,
    /// 自動モードでスクランブルにする車道幅 [m]（両方向ともこの幅以上の道路がある交差点）
    pub scramble_width: f32,
}

impl Default for SignalConfig {
    fn default() -> Self {
        Self {
            mode: PedMode::Auto,
            cycle: 100.0,
            main_split: 0.6,
            yellow: 3.0,
            all_red: 2.0,
            ped_green: 20.0,
            ped_flash: 6.0,
            min_approaches: 3,
            scramble_width: 14.0,
        }
    }
}

/// 交差点ごとの実際の方式（Auto を解決したもの）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Plan {
    Concurrent,
    Exclusive,
    Scramble,
}

impl Plan {
    pub fn label(self) -> &'static str {
        match self {
            Plan::Concurrent => "歩車同時",
            Plan::Exclusive => "歩車分離",
            Plan::Scramble => "スクランブル",
        }
    }
}

/// サイクル内の時刻割り（秒）。
#[derive(Debug, Clone, Copy)]
pub struct Timing {
    /// グループ 0 / 1 の車両青時間
    pub green: [f32; 2],
    pub yellow: f32,
    pub all_red: f32,
    /// 歩行者専用現示（青, 点滅）。歩車同時なら 0。
    pub ped: (f32, f32),
    pub ped_flash: f32,
}

impl Timing {
    pub fn new(c: &SignalConfig, plan: Plan) -> Self {
        let ped = if plan == Plan::Concurrent {
            (0.0, 0.0)
        } else {
            (c.ped_green.max(3.0), c.ped_flash.max(0.0))
        };
        let ped_total = if plan == Plan::Concurrent {
            0.0
        } else {
            ped.0 + ped.1 + c.all_red
        };
        let lost = 2.0 * (c.yellow + c.all_red) + ped_total;
        let avail = (c.cycle - lost).max(10.0);
        let split = c.main_split.clamp(0.1, 0.9);
        Self {
            green: [(avail * split).max(5.0), (avail * (1.0 - split)).max(5.0)],
            yellow: c.yellow.max(0.0),
            all_red: c.all_red.max(0.0),
            ped,
            ped_flash: c.ped_flash.max(0.0),
        }
    }
    pub fn cycle(&self) -> f32 {
        let veh = self.green[0] + self.green[1] + 2.0 * (self.yellow + self.all_red);
        if self.ped.0 > 0.0 {
            veh + self.ped.0 + self.ped.1 + self.all_red
        } else {
            veh
        }
    }
    /// グループ g の青の開始時刻
    fn green_start(&self, g: usize) -> f32 {
        if g == 0 {
            0.0
        } else {
            self.green[0] + self.yellow + self.all_red
        }
    }
    fn ped_start(&self) -> f32 {
        self.green_start(1) + self.green[1] + self.yellow + self.all_red
    }
    pub fn light(&self, g: usize, t: f32) -> Light {
        let t0 = self.green_start(g);
        let dt = t - t0;
        if (0.0..self.green[g]).contains(&dt) {
            Light::Green
        } else if (self.green[g]..self.green[g] + self.yellow).contains(&dt) {
            Light::Yellow
        } else {
            Light::Red
        }
    }
    /// 歩行者専用現示の歩行者信号
    fn ped_phase(&self, t: f32) -> Walk {
        if self.ped.0 <= 0.0 {
            return Walk::DontWalk;
        }
        let dt = t - self.ped_start();
        if (0.0..self.ped.0).contains(&dt) {
            Walk::Walk
        } else if (self.ped.0..self.ped.0 + self.ped.1).contains(&dt) {
            Walk::Flash
        } else {
            Walk::DontWalk
        }
    }
    /// 歩車同時式で、車両グループ g と並行する横断歩道の信号
    fn parallel_walk(&self, g: usize, t: f32) -> Walk {
        let t0 = self.green_start(g);
        let dt = t - t0;
        let flash = self.ped_flash.min(self.green[g] * 0.5);
        let walk_end = self.green[g] - flash;
        if (0.0..walk_end).contains(&dt) {
            Walk::Walk
        } else if (walk_end..self.green[g]).contains(&dt) {
            Walk::Flash
        } else {
            Walk::DontWalk
        }
    }
}

/// 車両用の灯色。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Light {
    Green,
    Yellow,
    Red,
}

/// 歩行者用の灯色。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Walk {
    Walk,
    Flash,
    DontWalk,
}

/// 交差点への流入路。
#[derive(Debug, Clone)]
pub struct Approach {
    /// 交差点へ入るレーン
    pub lane: Lane,
    /// 現示グループ（0 = 主道路, 1 = 従道路）
    pub group: u8,
    /// 流入レーンの終端ノードから停止線までの距離 [m]
    pub stop_dist: f32,
    /// 停止線の中心（道路中心線上）と、交差点へ向かう向き
    pub stop_pos: Vec2,
    pub dir: Vec2,
    /// 車道の幅 [m]（中心線から左・右の車道端まで）
    pub width: (f32, f32),
    pub crosswalk: Option<usize>,
}

/// 横断歩道。
#[derive(Debug, Clone)]
pub struct Crosswalk {
    pub center: Vec2,
    /// 道路の向き（交差点から外向き）
    pub along: Vec2,
    /// 横断する向き（`along` の左 90°）
    pub across: Vec2,
    /// 道路方向の半幅 [m]
    pub half_len: f32,
    /// `across` 方向の車道端までの距離（正の向き, 負の向き）
    pub extent: (f32, f32),
    /// 横断する道路の現示グループ
    pub group: u8,
    /// 横断歩道に含まれる車道セル
    pub cells: Vec<u32>,
}

#[derive(Debug, Clone)]
pub struct Intersection {
    pub nodes: Vec<usize>,
    pub center: Vec2,
    pub plan: Plan,
    pub timing: Timing,
    /// サイクルの位相のずれ [s]
    pub offset: f32,
    pub approaches: Vec<Approach>,
    pub crosswalks: Vec<Crosswalk>,
    /// 横断歩道に囲まれた交差点内の車道セル
    pub interior: Vec<u32>,
}

impl Intersection {
    fn local_t(&self, t: f32) -> f32 {
        (t + self.offset).rem_euclid(self.timing.cycle())
    }
    pub fn light(&self, group: u8, t: f32) -> Light {
        self.timing.light(group as usize, self.local_t(t))
    }
    /// 横断歩道 k の歩行者信号
    pub fn walk(&self, k: usize, t: f32) -> Walk {
        let t = self.local_t(t);
        match self.plan {
            // 道路を横切る横断歩道は、その道路と交差する向き（もう一方のグループ）の車と並行
            Plan::Concurrent => self
                .timing
                .parallel_walk(1 - self.crosswalks[k].group as usize, t),
            Plan::Exclusive | Plan::Scramble => self.timing.ped_phase(t),
        }
    }
    /// 交差点内（斜め横断）の歩行者信号
    pub fn interior_walk(&self, t: f32) -> Walk {
        match self.plan {
            Plan::Scramble => self.timing.ped_phase(self.local_t(t)),
            _ => Walk::DontWalk,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Signals {
    pub config: SignalConfig,
    pub intersections: Vec<Intersection>,
    /// 流入レーン → (交差点, 流入路)
    by_lane: HashMap<Lane, (usize, usize)>,
}

/// 2 つの交差点ノードがこの距離より近ければ同じ交差点とみなす [m]
const MERGE_DIST: f32 = 24.0;
/// 横断歩道の幅（道路方向）[m]
const CROSSWALK_W: f32 = 4.0;
/// 信号のある所で、横断歩道以外の車道を歩く経路コスト倍率（乱横断より横断歩道へ回る）
const MIDBLOCK_COST: f32 = 30.0;

impl Signals {
    /// 道路網と車道セルから信号を置く。`grid.carriageway` が空なら道路端から 3m を歩道とみなす。
    pub fn build(graph: &RoadGraph, grid: &WalkGrid, config: SignalConfig) -> Self {
        let is_junction = |n: usize| !graph.nodes[n].border && graph.nodes[n].edges.len() >= 3;
        // 近い交差点ノードをまとめる（union-find）
        let mut parent: Vec<usize> = (0..graph.nodes.len()).collect();
        fn find(p: &mut [usize], mut x: usize) -> usize {
            while p[x] != x {
                p[x] = p[p[x]];
                x = p[x];
            }
            x
        }
        for e in &graph.edges {
            if e.a != e.b && is_junction(e.a) && is_junction(e.b) && e.len() < MERGE_DIST {
                let (ra, rb) = (find(&mut parent, e.a), find(&mut parent, e.b));
                parent[ra] = rb;
            }
        }
        let mut clusters: HashMap<usize, Vec<usize>> = HashMap::new();
        for n in (0..graph.nodes.len()).filter(|&n| is_junction(n)) {
            let r = find(&mut parent, n);
            clusters.entry(r).or_default().push(n);
        }
        let mut clusters: Vec<Vec<usize>> = clusters.into_values().collect();
        clusters.iter_mut().for_each(|c| c.sort());
        clusters.sort();

        let cw_mask: Vec<bool> = if grid.carriageway.is_empty() {
            (0..grid.w * grid.h)
                .map(|i| grid.walkable[i] && grid.obstacle_dist[i] >= 3.0)
                .collect()
        } else {
            grid.carriageway.clone()
        };
        let is_cw = |p: Vec2| {
            grid.cell_of(p)
                .map(|(x, y)| cw_mask[grid.idx(x, y)])
                .unwrap_or(false)
        };
        // 点 p から向き v に車道が続く距離
        let extent = |p: Vec2, v: Vec2| {
            let mut d = 0.0;
            while d < 30.0 && is_cw(p + v * (d + 0.25)) {
                d += 0.25;
            }
            d
        };

        let mut out = Signals {
            config: config.clone(),
            intersections: vec![],
            by_lane: HashMap::new(),
        };
        let mut in_crosswalk = vec![false; grid.w * grid.h];
        for (ci, nodes) in clusters.into_iter().enumerate() {
            let center =
                nodes.iter().map(|&n| graph.nodes[n].pos).sum::<Vec2>() / nodes.len() as f32;
            // 外へ出る区間
            struct Raw {
                node: usize,
                edge: usize,
                mouth: f32,
                width_ref: f32,
                out_dir: Vec2,
            }
            let mut raws: Vec<Raw> = vec![];
            for &n in &nodes {
                for &ei in &graph.nodes[n].edges {
                    let e = &graph.edges[ei];
                    if e.a == e.b {
                        continue;
                    }
                    let other = e.other(n);
                    if nodes.contains(&other) {
                        continue;
                    }
                    let len = e.len();
                    // ノードから外向きに距離 s の位置と向き
                    let pose = |s: f32| {
                        if e.a == n {
                            e.at(s)
                        } else {
                            let (p, d) = e.at(len - s);
                            (p, -d)
                        }
                    };
                    let smax = (len * 0.6).min(45.0);
                    let mut widths = vec![];
                    let mut s = 0.0;
                    while s <= smax {
                        let (p, d) = pose(s);
                        let v = d.perp();
                        widths.push((s, extent(p, v) + extent(p, -v)));
                        s += 0.5;
                    }
                    let mut far: Vec<f32> = widths
                        .iter()
                        .filter(|(s, _)| *s >= smax * 0.5)
                        .map(|w| w.1)
                        .collect();
                    far.sort_by(f32::total_cmp);
                    let width_ref = far.get(far.len() / 2).copied().unwrap_or(0.0);
                    let r = graph.nodes[n].radius;
                    // 交差点の口: 車道幅が道路本来の幅に戻る所
                    let mouth = widths
                        .iter()
                        .find(|(s, w)| *s >= r * 0.3 && *w <= width_ref * 1.15 + 1.0)
                        .map_or(r, |w| w.0);
                    let (p, _) = pose((mouth + 6.0).min(len));
                    raws.push(Raw {
                        node: n,
                        edge: ei,
                        mouth,
                        width_ref,
                        out_dir: (p - center).normalize_or(pose(mouth).1),
                    });
                }
            }
            if raws.len() < config.min_approaches.max(2) {
                continue;
            }
            // 主道路の軸 = 最も広い流入路の向き
            let main = raws
                .iter()
                .max_by(|a, b| a.width_ref.total_cmp(&b.width_ref))
                .unwrap()
                .out_dir;
            let mut inter = Intersection {
                nodes: nodes.clone(),
                center,
                plan: Plan::Concurrent,
                timing: Timing::new(&config, Plan::Concurrent),
                offset: 0.0,
                approaches: vec![],
                crosswalks: vec![],
                interior: vec![],
            };
            let mut widest = [0.0f32; 2];
            for r in &raws {
                let e = &graph.edges[r.edge];
                let len = e.len();
                let n = r.node;
                let pose = |s: f32| {
                    if e.a == n {
                        e.at(s)
                    } else {
                        let (p, d) = e.at(len - s);
                        (p, -d)
                    }
                };
                let group = if r.out_dir.dot(main).abs() >= std::f32::consts::FRAC_1_SQRT_2 {
                    0u8
                } else {
                    1u8
                };
                widest[group as usize] = widest[group as usize].max(r.width_ref);
                let s0 = r.mouth + 0.5;
                let s1 = s0 + CROSSWALK_W;
                let lane = Lane {
                    edge: r.edge,
                    fwd: e.b == n,
                };
                let radius = graph.nodes[n].radius;
                // 横断歩道を置く余地がない短い区間は、停止線だけにする
                let has_cw = len > s1 + 8.0;
                let stop_dist = if has_cw {
                    (s1 + 1.5).max(radius)
                } else {
                    radius
                }
                .min((len - 3.0).max(radius));
                let (sp, sd) = pose(stop_dist);
                let (wl, wr) = (extent(sp, -sd.perp()), extent(sp, sd.perp()));
                let mut ap = Approach {
                    lane,
                    group,
                    stop_dist,
                    stop_pos: sp,
                    dir: -sd,
                    width: (wl, wr),
                    crosswalk: None,
                };
                if has_cw {
                    let (c, along) = pose(0.5 * (s0 + s1));
                    let across = along.perp();
                    let ext = (extent(c, across) + 0.5, extent(c, -across) + 0.5);
                    let half_len = 0.5 * CROSSWALK_W;
                    let mut cells = vec![];
                    let corners = [
                        c + along * half_len + across * ext.0,
                        c + along * half_len - across * ext.1,
                        c - along * half_len + across * ext.0,
                        c - along * half_len - across * ext.1,
                    ];
                    let lo = corners.iter().fold(Vec2::INFINITY, |m, p| m.min(*p));
                    let hi = corners.iter().fold(Vec2::NEG_INFINITY, |m, p| m.max(*p));
                    let to_cell = |p: Vec2| ((p - grid.origin) / grid.cell).floor();
                    let (a, b) = (to_cell(lo).max(Vec2::ZERO), to_cell(hi));
                    for y in a.y as usize..=(b.y.max(0.0) as usize).min(grid.h.saturating_sub(1)) {
                        for x in
                            a.x as usize..=(b.x.max(0.0) as usize).min(grid.w.saturating_sub(1))
                        {
                            let i = grid.idx(x, y);
                            let q = grid.center(x, y) - c;
                            let (u, v) = (q.dot(along), q.dot(across));
                            if cw_mask[i] && u.abs() <= half_len && v <= ext.0 && v >= -ext.1 {
                                cells.push(i as u32);
                            }
                        }
                    }
                    if !cells.is_empty() {
                        ap.crosswalk = Some(inter.crosswalks.len());
                        inter.crosswalks.push(Crosswalk {
                            center: c,
                            along,
                            across,
                            half_len,
                            extent: ext,
                            group,
                            cells,
                        });
                    }
                }
                inter.approaches.push(ap);
            }
            inter.plan = match config.mode {
                PedMode::Concurrent => Plan::Concurrent,
                PedMode::Exclusive => Plan::Exclusive,
                PedMode::Scramble => Plan::Scramble,
                PedMode::Auto => {
                    if raws.len() >= 4 && widest[0].min(widest[1]) >= config.scramble_width {
                        Plan::Scramble
                    } else {
                        Plan::Concurrent
                    }
                }
            };
            inter.timing = Timing::new(&config, inter.plan);
            // 位相のずれ（決定的な擬似乱数）
            let h = (ci as u64 + 1).wrapping_mul(0x9E37_79B9_7F4A_7C15);
            inter.offset = ((h >> 40) as f32 / (1u64 << 24) as f32) * inter.timing.cycle();

            // 交差点内: 交差点ノードから横断歩道を越えずに届く車道セル
            for cw in &inter.crosswalks {
                cw.cells
                    .iter()
                    .for_each(|&i| in_crosswalk[i as usize] = true);
            }
            let reach = inter
                .crosswalks
                .iter()
                .map(|c| c.center.distance(center) + c.half_len)
                .fold(0.0f32, f32::max)
                .max(
                    nodes
                        .iter()
                        .map(|&n| graph.nodes[n].radius)
                        .fold(0.0, f32::max),
                );
            let mut seen = std::collections::HashSet::new();
            let mut stack: Vec<(usize, usize)> = nodes
                .iter()
                .filter_map(|&n| grid.cell_of(graph.nodes[n].pos))
                .collect();
            while let Some((x, y)) = stack.pop() {
                let i = grid.idx(x, y);
                if !cw_mask[i] || in_crosswalk[i] || !seen.insert(i) {
                    continue;
                }
                if grid.center(x, y).distance(center) > reach {
                    continue;
                }
                inter.interior.push(i as u32);
                if x > 0 {
                    stack.push((x - 1, y));
                }
                if y > 0 {
                    stack.push((x, y - 1));
                }
                if x + 1 < grid.w {
                    stack.push((x + 1, y));
                }
                if y + 1 < grid.h {
                    stack.push((x, y + 1));
                }
            }
            for cw in &inter.crosswalks {
                cw.cells
                    .iter()
                    .for_each(|&i| in_crosswalk[i as usize] = false);
            }

            let k = out.intersections.len();
            for (j, ap) in inter.approaches.iter().enumerate() {
                out.by_lane.insert(ap.lane, (k, j));
            }
            out.intersections.push(inter);
        }
        out
    }

    /// レーンが信号のある交差点への流入路なら (灯色, 終端ノードから停止線までの距離)。
    pub fn light_for(&self, lane: Lane, t: f32) -> Option<(Light, f32)> {
        let &(k, j) = self.by_lane.get(&lane)?;
        let inter = &self.intersections[k];
        let ap = &inter.approaches[j];
        Some((inter.light(ap.group, t), ap.stop_dist))
    }

    pub fn approach(&self, lane: Lane) -> Option<(&Intersection, &Approach)> {
        let &(k, j) = self.by_lane.get(&lane)?;
        let inter = &self.intersections[k];
        Some((inter, &inter.approaches[j]))
    }

    /// 歩行者が歩道から入ってはいけないセル（青でない横断歩道・交差点内）を書き込む。
    /// `closed` はグリッドと同じ大きさ。
    pub fn write_closed(&self, t: f32, closed: &mut [bool]) {
        for inter in &self.intersections {
            for (k, cw) in inter.crosswalks.iter().enumerate() {
                let c = inter.walk(k, t) != Walk::Walk;
                cw.cells.iter().for_each(|&i| closed[i as usize] = c);
            }
            let c = inter.interior_walk(t) != Walk::Walk;
            inter.interior.iter().for_each(|&i| closed[i as usize] = c);
        }
    }

    /// 歩行者が歩道から踏み込んではいけない車道セル（横断歩道・交差点内以外の車道）。
    /// 信号が 1 つも無ければ空（制限なし）。`crowd::Simulation::curb` に渡す。
    pub fn curb_cells(&self, grid: &WalkGrid) -> Vec<bool> {
        if self.intersections.is_empty() || grid.carriageway.len() != grid.w * grid.h {
            return vec![];
        }
        let mut curb = grid.carriageway.clone();
        for inter in &self.intersections {
            let cells = inter.crosswalks.iter().flat_map(|c| &c.cells);
            for &i in cells.chain(&inter.interior) {
                curb[i as usize] = false;
            }
        }
        curb
    }

    /// 歩行者の経路コストを信号に合わせる: 横断歩道は渡りやすく、
    /// スクランブル以外の交差点内は斜めに突っ切らないよう高くする。
    /// 横断歩道以外の車道も高くし、近くに横断歩道があればそちらを回る。
    pub fn apply_walk_costs(&self, grid: &mut WalkGrid) {
        if grid.walk_cost.len() != grid.w * grid.h {
            return;
        }
        if !self.intersections.is_empty() && grid.carriageway.len() == grid.w * grid.h {
            for (c, &cw) in grid.walk_cost.iter_mut().zip(&grid.carriageway) {
                if cw {
                    *c = c.max(MIDBLOCK_COST);
                }
            }
        }
        for inter in &self.intersections {
            let interior = if inter.plan == Plan::Scramble {
                1.5
            } else {
                40.0
            };
            for &i in &inter.interior {
                grid.walk_cost[i as usize] = interior;
            }
            for cw in &inter.crosswalks {
                for &i in &cw.cells {
                    grid.walk_cost[i as usize] = 1.2;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Traffic;

    fn grid() -> WalkGrid {
        let m = plateau_gml::parse_str(include_str!("../../../testdata/sample.gml")).unwrap();
        let mut g = WalkGrid::from_city(&m, 0.5).unwrap();
        g.mark_carriageway(3.0, 6.0);
        g
    }

    #[test]
    fn builds_four_signalized_crossings() {
        let g = grid();
        let net = RoadGraph::from_grid(&g, 3.0);
        let s = Signals::build(&net, &g, SignalConfig::default());
        assert_eq!(s.intersections.len(), 4);
        for inter in &s.intersections {
            assert_eq!(inter.approaches.len(), 4);
            let g0 = inter.approaches.iter().filter(|a| a.group == 0).count();
            assert_eq!(g0, 2, "対向する 2 本ずつが同じ現示");
            assert_eq!(inter.crosswalks.len(), 4);
            assert!(inter.crosswalks.iter().all(|c| c.cells.len() > 20));
            assert!(!inter.interior.is_empty());
            // 横断歩道セルと交差点内セルは重ならない
            for c in &inter.crosswalks {
                assert!(c.cells.iter().all(|i| !inter.interior.contains(i)));
            }
            // 停止線は交差点の中心から十分離れている
            for a in &inter.approaches {
                assert!(a.stop_pos.distance(inter.center) > 6.0, "{:?}", a.stop_pos);
            }
        }
    }

    #[test]
    fn phases_never_conflict() {
        for mode in [PedMode::Concurrent, PedMode::Exclusive, PedMode::Scramble] {
            let c = SignalConfig {
                mode,
                ..Default::default()
            };
            let plan = match mode {
                PedMode::Concurrent => Plan::Concurrent,
                PedMode::Exclusive => Plan::Exclusive,
                _ => Plan::Scramble,
            };
            let tm = Timing::new(&c, plan);
            assert!(
                (tm.cycle() - c.cycle).abs() < 1e-3,
                "{mode:?} cycle {}",
                tm.cycle()
            );
            let mut saw_walk = false;
            let mut t = 0.0;
            while t < tm.cycle() {
                let l0 = tm.light(0, t);
                let l1 = tm.light(1, t);
                // 交差する 2 方向が同時に青・黄にならない
                assert!(l0 == Light::Red || l1 == Light::Red, "{mode:?} t={t}");
                match plan {
                    Plan::Concurrent => {
                        // 道路 g を渡る横断歩道は、g の車が赤のときだけ青
                        for g in 0..2 {
                            if tm.parallel_walk(1 - g, t) != Walk::DontWalk {
                                assert_eq!(tm.light(g, t), Light::Red);
                                saw_walk = true;
                            }
                        }
                    }
                    _ => {
                        if tm.ped_phase(t) != Walk::DontWalk {
                            assert_eq!((l0, l1), (Light::Red, Light::Red));
                            saw_walk = true;
                        }
                    }
                }
                t += 0.25;
            }
            assert!(saw_walk);
        }
    }

    #[test]
    fn vehicles_obey_red_lights() {
        let g = grid();
        let net = RoadGraph::from_grid(&g, 3.0);
        let sig = Signals::build(
            &net,
            &g,
            SignalConfig {
                cycle: 60.0,
                ..Default::default()
            },
        );
        let mut t = Traffic::new(net, 30, 5);
        t.set_signals(Some(sig));
        // 停止線までの残り距離（流入路にいる車のみ）
        let stop_left = |t: &Traffic, i: usize| {
            let v = &t.vehicles[i];
            let s = t.signals.as_ref()?;
            let (_, ap) = s.approach(v.lane)?;
            Some(t.lane_len(v.lane) - ap.stop_dist - 0.5 * v.length - v.s)
        };
        let mut moved = 0.0;
        let mut stopped_at_red = 0;
        for _ in 0..(240.0 / 0.05) as usize {
            let before: Vec<_> = (0..t.vehicles.len())
                .map(|i| {
                    let v = &t.vehicles[i];
                    let light = t.signals.as_ref().unwrap().light_for(v.lane, t.time);
                    (v.lane, stop_left(&t, i), v.pos, v.v, light.map(|l| l.0))
                })
                .collect();
            t.step(0.05, &[]);
            for (i, (lane, left, pos, speed, light)) in before.into_iter().enumerate() {
                let v = &t.vehicles[i];
                if !v.active {
                    continue;
                }
                moved += v.pos.distance(pos);
                let (Some(left), Some(light)) = (left, light) else {
                    continue;
                };
                let now = if v.lane == lane {
                    stop_left(&t, i).unwrap()
                } else {
                    -1.0
                };
                // 赤で停止線を越えない（止まりきれない距離まで来ていた車は除く）
                if light == Light::Red && left > speed * speed / 12.0 + 0.05 {
                    assert!(
                        now > -0.3,
                        "赤で停止線を越えた t={} left={left} now={now}",
                        t.time
                    );
                }
                if light == Light::Red && left.abs() < 3.0 && v.v < 0.1 {
                    stopped_at_red += 1;
                }
            }
        }
        assert!(stopped_at_red > 0, "赤で止まる車がいる");
        let avg = moved / (30.0 * 240.0);
        assert!(avg > 1.5, "信号があっても流れる avg={avg}");
    }
}
