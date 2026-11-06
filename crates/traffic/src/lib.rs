//! 車両シミュレーション。
//!
//! - 道路網: 車道の中心線グラフ（[`graph::RoadGraph`]）。各エッジは双方向で、左側通行。
//! - 縦方向: Intelligent Driver Model (Treiber et al. 2000) で前方車・停止線・歩行者に追従
//! - 交差点: 進入方向ごとの排他ロック（同じ方向からの車列はまとめて通す、一定時間で交代）
//! - 信号機（任意, [`signal`]）: 赤・黄では停止線で止まり、青の間だけ上の通行権で交差点に入る
//! - 出入口: 範囲の縁の端点から流入し、縁に着いたら別の出入口から再投入
//! - 車種: 乗用車・トラック・バス（停留所で停車）・自転車（車道の左端、車とは別の列）

pub mod graph;
pub mod signal;

use glam::Vec2;
pub use graph::RoadGraph;
pub use signal::{Light, PedMode, SignalConfig, Signals, Walk};
use std::collections::HashMap;

#[derive(Debug, Clone)]
pub struct Params {
    /// 左側通行のレーン中心のずれ [m]
    pub lane_offset: f32,
    /// IDM: 最大加速度・快適減速度 [m/s^2]、最小車間 [m]、車間時間 [s]
    pub a_max: f32,
    pub b_comf: f32,
    pub s0: f32,
    pub t_headway: f32,
    /// 交差点で曲がるときの速度 [m/s]
    pub v_turn: f32,
    /// 待ち車がいるとき、同じ動線が続けて通れる最大時間 [s]
    pub lock_hold: f32,
    /// 歩行者に阻まれてこの秒数止まったら徐行で進む
    pub creep_after: f32,
    /// 交差点でこの秒数待たされたら通行権を待たずに進む
    pub deadlock_after: f32,
    /// 希望速度の上限 [m/s]（各車はこの 70〜100%）
    pub speed_limit: f32,
    /// 車種の構成比（残りは乗用車）
    pub mix: VehicleMix,
}

/// 車種。大きさ・速度・走行位置が違う。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum VehicleKind {
    Car,
    Truck,
    /// 路線バス: 長さ 11m、制限速度の 85% まで、停留所（一定間隔）で停車する
    Bus,
    /// 自転車: 車道の左端を 12〜20km/h で走る。車は自転車を追い越せる（同じ列には並ばない）
    Bicycle,
}

impl VehicleKind {
    pub fn name(self) -> &'static str {
        match self {
            VehicleKind::Car => "car",
            VehicleKind::Truck => "truck",
            VehicleKind::Bus => "bus",
            VehicleKind::Bicycle => "bicycle",
        }
    }
    pub fn label_ja(self) -> &'static str {
        match self {
            VehicleKind::Car => "乗用車",
            VehicleKind::Truck => "トラック",
            VehicleKind::Bus => "バス",
            VehicleKind::Bicycle => "自転車",
        }
    }
    /// (長さ, 幅) [m]
    pub fn size(self) -> (f32, f32) {
        match self {
            VehicleKind::Car => (4.5, 1.8),
            VehicleKind::Truck => (7.5, 2.3),
            VehicleKind::Bus => (11.0, 2.5),
            VehicleKind::Bicycle => (1.8, 0.6),
        }
    }
    pub fn is_bicycle(self) -> bool {
        self == VehicleKind::Bicycle
    }
}

/// 車種の構成比（0〜1）。合計が 1 を超えた分は後ろの種別から削る。
#[derive(Debug, Clone, Copy)]
pub struct VehicleMix {
    pub truck: f32,
    pub bus: f32,
    pub bicycle: f32,
}

impl Default for VehicleMix {
    fn default() -> Self {
        Self {
            truck: 0.1,
            bus: 0.05,
            bicycle: 0.15,
        }
    }
}

impl VehicleMix {
    /// 乗用車だけ（従来の挙動に近い構成）
    pub const CARS_ONLY: Self = Self {
        truck: 0.0,
        bus: 0.0,
        bicycle: 0.0,
    };
    fn pick(&self, u: f32) -> VehicleKind {
        let mut acc = 0.0;
        for (p, k) in [
            (self.truck, VehicleKind::Truck),
            (self.bus, VehicleKind::Bus),
            (self.bicycle, VehicleKind::Bicycle),
        ] {
            acc += p.max(0.0);
            if u < acc {
                return k;
            }
        }
        VehicleKind::Car
    }
}

/// バスの停留所間隔 [m] と停車時間 [s]
const BUS_STOP_SPACING: f32 = 220.0;
const BUS_DWELL: f32 = 15.0;
/// 自転車が車のレーン中心からさらに左へ寄る量 [m]
const BICYCLE_EXTRA_OFFSET: f32 = 1.2;

impl Default for Params {
    fn default() -> Self {
        Self {
            lane_offset: 1.8,
            a_max: 1.5,
            b_comf: 2.5,
            s0: 2.0,
            t_headway: 1.2,
            v_turn: 4.5,
            lock_hold: 6.0,
            creep_after: 5.0,
            deadlock_after: 15.0,
            speed_limit: 13.0,
            mix: VehicleMix::default(),
        }
    }
}

/// エッジ上の進行方向つきレーン。`fwd` なら a→b。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Lane {
    pub edge: usize,
    pub fwd: bool,
}

#[derive(Debug, Clone)]
pub struct Vehicle {
    pub lane: Lane,
    /// レーン始点からの距離 [m]
    pub s: f32,
    pub v: f32,
    pub v0: f32,
    pub length: f32,
    pub width: f32,
    pub next: Option<Lane>,
    pub active: bool,
    /// 通行権を得ている交差点と動線
    pub lock: Option<(usize, Movement)>,
    /// 描画用（平滑化済み）
    pub pos: Vec2,
    pub heading: Vec2,
    lat_err: Vec2,
    ped_wait: f32,
    junction_wait: f32,
    pub style: u32,
    pub kind: VehicleKind,
    /// 中心線から左へのずれ [m]（自転車は車道の左端寄り）
    pub offset: f32,
    /// バス: 前の停留所からの走行距離 [m] と停車の残り時間 [s]
    since_stop: f32,
    pub dwell: f32,
    /// バス: 次に停まる位置（現在のレーン上の s）
    stop_s: Option<f32>,
}

/// 交差点での動線（進入レーン → 退出レーン）。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Movement {
    pub from: Lane,
    pub to: Lane,
}

/// 交差点の通行権。交差しない動線（対向の直進同士など）は同時に通れる。
#[derive(Debug, Clone, Default)]
struct JunctionLock {
    /// 通行中の動線と台数、開始時刻
    active: Vec<(Movement, u32, f32)>,
    /// 直前のステップで待たされた車がいたか
    waiting: bool,
    waiting_next: bool,
}

pub struct Traffic {
    pub graph: RoadGraph,
    pub vehicles: Vec<Vehicle>,
    pub params: Params,
    pub time: f32,
    locks: Vec<JunctionLock>,
    rng: u64,
    entries: Vec<Lane>,
    /// 停止命令（災害避難など）: 全車が減速して止まり、再投入もしない
    pub halted: bool,
    /// 信号機（None なら先着順の通行権だけで制御）
    pub signals: Option<Signals>,
}

impl Traffic {
    pub fn new(graph: RoadGraph, n_vehicles: usize, seed: u64) -> Self {
        Self::new_with(graph, n_vehicles, seed, Params::default())
    }

    pub fn new_with(graph: RoadGraph, n_vehicles: usize, seed: u64, params: Params) -> Self {
        let entries: Vec<Lane> = graph
            .nodes
            .iter()
            .enumerate()
            .filter(|(_, n)| n.border && n.edges.len() == 1)
            .map(|(i, n)| {
                let e = n.edges[0];
                Lane {
                    edge: e,
                    fwd: graph.edges[e].a == i,
                }
            })
            .collect();
        let locks = vec![JunctionLock::default(); graph.nodes.len()];
        let mut t = Self {
            graph,
            vehicles: vec![],
            params,
            time: 0.0,
            locks,
            rng: seed.max(1),
            entries,
            halted: false,
            signals: None,
        };
        // 初期配置: 道路網全体に間隔をあけてばらまく
        let lanes: Vec<Lane> = (0..t.graph.edges.len())
            .flat_map(|e| {
                [
                    Lane { edge: e, fwd: true },
                    Lane {
                        edge: e,
                        fwd: false,
                    },
                ]
            })
            .filter(|l| t.graph.edges[l.edge].len() > 20.0)
            .collect();
        let mut tries = 0;
        while t.vehicles.len() < n_vehicles && !lanes.is_empty() && tries < n_vehicles * 50 {
            tries += 1;
            let lane = lanes[t.rand(lanes.len())];
            let len = t.graph.edges[lane.edge].len();
            let r = t.end_radius(lane).max(t.start_radius(lane));
            if len < 2.0 * r + 8.0 {
                continue;
            }
            let s = r + t.randf() * (len - 2.0 * r);
            if t.vehicles
                .iter()
                .any(|v| v.lane == lane && (v.s - s).abs() < 12.0)
            {
                continue;
            }
            let v = t.make_vehicle(lane, s);
            t.vehicles.push(v);
        }
        t
    }

    /// 信号機を設定する（None で信号なし）。
    pub fn set_signals(&mut self, signals: Option<Signals>) {
        self.signals = signals;
    }

    pub fn active(&self) -> usize {
        self.vehicles.iter().filter(|v| v.active).count()
    }

    fn rand(&mut self, n: usize) -> usize {
        let mut x = self.rng;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.rng = x;
        (x.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 33) as usize % n.max(1)
    }
    fn randf(&mut self) -> f32 {
        self.rand(1 << 24) as f32 / (1u32 << 24) as f32
    }

    fn make_vehicle(&mut self, lane: Lane, s: f32) -> Vehicle {
        let style = self.rand(1 << 16) as u32;
        let u = self.randf();
        let kind = self.params.mix.pick(u);
        let (length, width) = kind.size();
        let r = self.randf();
        let v0 = match kind {
            // 12〜20 km/h（制限速度が低ければそれに従う）
            VehicleKind::Bicycle => (3.3 + 2.2 * r).min(self.params.speed_limit),
            VehicleKind::Bus => self.params.speed_limit * (0.75 + 0.1 * r),
            VehicleKind::Truck => self.params.speed_limit * (0.7 + 0.2 * r),
            VehicleKind::Car => self.params.speed_limit * (0.7 + 0.3 * r),
        };
        let offset = self.params.lane_offset
            + if kind.is_bicycle() {
                BICYCLE_EXTRA_OFFSET
            } else {
                0.0
            };
        let mut v = Vehicle {
            lane,
            s,
            v: v0 * 0.6,
            v0,
            length,
            width,
            next: None,
            active: true,
            lock: None,
            pos: Vec2::ZERO,
            heading: Vec2::X,
            lat_err: Vec2::ZERO,
            ped_wait: 0.0,
            junction_wait: 0.0,
            style,
            kind,
            offset,
            // 停留所の位置がバスごとにずれるように
            since_stop: BUS_STOP_SPACING * self.randf(),
            dwell: 0.0,
            stop_s: None,
        };
        v.next = self.choose_next(lane);
        let (p, d) = self.lane_pose_at(lane, s, offset);
        v.pos = p;
        v.heading = d;
        v
    }

    pub fn lane_len(&self, l: Lane) -> f32 {
        self.graph.edges[l.edge].len()
    }
    pub fn end_node(&self, l: Lane) -> usize {
        let e = &self.graph.edges[l.edge];
        if l.fwd {
            e.b
        } else {
            e.a
        }
    }
    fn start_node(&self, l: Lane) -> usize {
        let e = &self.graph.edges[l.edge];
        if l.fwd {
            e.a
        } else {
            e.b
        }
    }
    fn end_radius(&self, l: Lane) -> f32 {
        self.graph.nodes[self.end_node(l)].radius
    }
    fn start_radius(&self, l: Lane) -> f32 {
        self.graph.nodes[self.start_node(l)].radius
    }

    /// レーン上の距離 s における中心線位置と進行方向（レーンずれ前）。
    fn center_pose(&self, l: Lane, s: f32) -> (Vec2, Vec2) {
        let e = &self.graph.edges[l.edge];
        if l.fwd {
            e.at(s)
        } else {
            let (p, d) = e.at(e.len() - s);
            (p, -d)
        }
    }
    /// 左側通行のレーン上の位置。
    pub fn lane_pose(&self, l: Lane, s: f32) -> (Vec2, Vec2) {
        self.lane_pose_at(l, s, self.params.lane_offset)
    }
    /// 中心線から左へ `offset` [m] ずらした位置。
    pub fn lane_pose_at(&self, l: Lane, s: f32, offset: f32) -> (Vec2, Vec2) {
        let (p, d) = self.center_pose(l, s);
        (p + d.perp() * offset, d)
    }

    fn choose_next(&mut self, l: Lane) -> Option<Lane> {
        let n = self.end_node(l);
        let node = &self.graph.nodes[n];
        if node.border {
            return None;
        }
        let mut cands: Vec<Lane> = node
            .edges
            .iter()
            .filter(|&&e| e != l.edge)
            .map(|&e| Lane {
                edge: e,
                fwd: self.graph.edges[e].a == n,
            })
            .collect();
        // 行き止まりへ入る道は、ほかに選べる道があれば避ける
        let into_dead_end = |g: &RoadGraph, l: &Lane| {
            let e = &g.edges[l.edge];
            g.is_dead_end(if l.fwd { e.b } else { e.a })
        };
        if cands.iter().any(|c| !into_dead_end(&self.graph, c)) {
            cands.retain(|c| !into_dead_end(&self.graph, c));
        }
        if cands.is_empty() {
            // 行き止まり: U ターン
            cands.push(Lane {
                edge: l.edge,
                fwd: !l.fwd,
            });
        }
        Some(cands[self.rand(cands.len())])
    }

    fn turn_cos(&self, from: Lane, to: Lane) -> f32 {
        let (_, d1) = self.center_pose(from, (self.lane_len(from) - 3.0).max(0.0));
        let (_, d2) = self.center_pose(to, 3.0f32.min(self.lane_len(to)));
        d1.dot(d2)
    }

    /// 1 ステップ進める。`peds` は歩行者の位置（車は前方の歩行者に減速する）。
    pub fn step(&mut self, dt: f32, peds: &[Vec2]) {
        let p = self.params.clone();
        // レーンごとの車両リスト（s 昇順）
        let mut by_lane: HashMap<Lane, Vec<usize>> = HashMap::new();
        for (i, v) in self.vehicles.iter().enumerate().filter(|(_, v)| v.active) {
            by_lane.entry(v.lane).or_default().push(i);
        }
        for list in by_lane.values_mut() {
            list.sort_by(|&a, &b| self.vehicles[a].s.total_cmp(&self.vehicles[b].s));
        }
        let ped_bins = PedBins::new(peds, 10.0);
        for lk in self.locks.iter_mut() {
            lk.waiting = lk.waiting_next;
            lk.waiting_next = false;
        }

        let mut accs = vec![0.0f32; self.vehicles.len()];
        for i in 0..self.vehicles.len() {
            if !self.vehicles[i].active {
                continue;
            }
            let v = &self.vehicles[i];
            let len = self.lane_len(v.lane);
            let remaining = len - v.s;
            // 先行車（同じレーン → 次のレーン）
            // 停止命令: ゆっくり止まる
            if self.halted {
                accs[i] = if v.v > 0.0 { -p.b_comf } else { 0.0 };
                continue;
            }
            // 自転車は車道の左端、車はその内側を走るので、同じ列（自転車同士・車同士）だけ追従する
            let bike = v.kind.is_bicycle();
            let same_track = |u: &Vehicle| u.kind.is_bicycle() == bike;
            let mut gap = f32::INFINITY;
            let mut v_lead = v.v;
            if let Some(list) = by_lane.get(&v.lane) {
                let k = list.iter().position(|&j| j == i).unwrap();
                if let Some(&j) = list[k + 1..].iter().find(|&&j| same_track(&self.vehicles[j])) {
                    let u = &self.vehicles[j];
                    gap = u.s - v.s - 0.5 * (u.length + v.length);
                    v_lead = u.v;
                }
            }
            if gap.is_infinite() {
                if let Some(next) = v.next {
                    if let Some(&j) = by_lane
                        .get(&next)
                        .and_then(|l| l.iter().find(|&&j| same_track(&self.vehicles[j])))
                    {
                        let u = &self.vehicles[j];
                        if j != i {
                            gap = remaining + u.s - 0.5 * (u.length + v.length);
                            v_lead = u.v;
                        }
                    }
                }
            }
            // バス: 停留所で停車
            if v.kind == VehicleKind::Bus {
                if v.dwell > 0.0 {
                    accs[i] = if v.v > 0.0 { -p.b_comf * 1.5 } else { 0.0 };
                    self.vehicles[i].dwell = (v.dwell - dt).max(0.0);
                    continue;
                }
                let end_r = self.end_radius(v.lane);
                match v.stop_s {
                    Some(stop) => {
                        // IDM は最小車間 s0 の手前で止まる
                        if stop - v.s < p.s0 + 1.0 && v.v < 0.5 {
                            let b = &mut self.vehicles[i];
                            b.dwell = BUS_DWELL;
                            b.stop_s = None;
                            b.since_stop = 0.0;
                            accs[i] = -p.b_comf;
                            continue;
                        }
                        if stop - v.s < gap {
                            gap = (stop - v.s).max(0.0);
                            v_lead = 0.0;
                        }
                    }
                    None if v.since_stop >= BUS_STOP_SPACING => {
                        // 交差点の手前 8m より前に、無理なく止まれる位置を停留所にする
                        let stop = v.s + v.v * v.v / (2.0 * p.b_comf) + 2.0;
                        if stop < len - end_r - 8.0 {
                            self.vehicles[i].stop_s = Some(stop);
                        }
                    }
                    None => {}
                }
            }
            let v = &self.vehicles[i];
            // 交差点: ロックが取れなければ停止線で止まる
            let end = self.end_node(v.lane);
            let node = &self.graph.nodes[end];
            let mut v0 = v.v0;
            let holds = matches!(v.lock, Some((n, _)) if n == end);
            if let (false, false, Some(next)) = (node.border, holds, v.next) {
                // 信号のある流入路なら停止線は横断歩道の手前
                let sig = self
                    .signals
                    .as_ref()
                    .and_then(|s| s.light_for(v.lane, self.time));
                let stop = remaining - sig.map_or(node.radius, |s| s.1) - 0.5 * v.length;
                // 赤は止まりきれる限り止まる。黄は無理なく止まれるときだけ止まる（ジレンマ回避）
                let halt = match sig {
                    Some((Light::Red, _)) => stop > -1.0 && stop >= v.v * v.v / 12.0 - 1.0,
                    Some((Light::Yellow, _)) => {
                        stop > 0.0 && stop >= v.v * v.v / (2.0 * 1.6 * p.b_comf)
                    }
                    _ => false,
                };
                if halt {
                    if stop.max(0.0) < gap {
                        gap = stop.max(0.0);
                        v_lead = 0.0;
                    }
                    self.vehicles[i].junction_wait = 0.0;
                } else if stop < 25.0 {
                    let mv = Movement {
                        from: v.lane,
                        to: next,
                    };
                    // 出口側のレーンに入る余地がなければ交差点に入らない（交差点内で止まらない）
                    let exit_room = by_lane
                        .get(&next)
                        .and_then(|l| l.iter().find(|&&j| same_track(&self.vehicles[j])))
                        .map_or(true, |&j| {
                            let u = &self.vehicles[j];
                            j == i
                                || u.s - 0.5 * u.length > self.start_radius(next) + v.length + 1.0
                        });
                    // 長く待たされたら（デッドロック回避）強引に入る
                    let stuck = v.junction_wait > p.deadlock_after;
                    let free = stuck || (exit_room && self.can_enter(end, mv));
                    let wants = stop < 6.0 + v.v * v.v / (2.0 * p.b_comf);
                    if free && wants {
                        self.vehicles[i].junction_wait = 0.0;
                        let now = self.time;
                        let lk = &mut self.locks[end];
                        match lk.active.iter_mut().find(|(m, _, _)| *m == mv) {
                            Some(e) => e.1 += 1,
                            None => lk.active.push((mv, 1, now)),
                        }
                        self.vehicles[i].lock = Some((end, mv));
                    } else if !free {
                        self.locks[end].waiting_next = true;
                        if stop.max(0.0) < gap {
                            gap = stop.max(0.0);
                            v_lead = 0.0;
                        }
                        if v.v < 0.3 && stop < 8.0 {
                            self.vehicles[i].junction_wait += dt;
                        }
                    }
                }
            }
            let v = &self.vehicles[i];
            // 曲がる前に減速
            if let Some(next) = v.next {
                if self.turn_cos(v.lane, next) < 0.7 && remaining < 30.0 {
                    v0 = v0.min(p.v_turn + 0.3 * (remaining - node.radius).max(0.0));
                }
            }
            // 前方の歩行者
            let (pos, dir) = (v.pos, v.heading);
            let mut ped_gap = f32::INFINITY;
            ped_bins.for_near(pos + dir * 10.0, 16.0, |q| {
                let r = q - pos;
                let lon = r.dot(dir);
                let lat = r.dot(dir.perp()).abs();
                // 車体の横にいる人は対象外（前バンパーより前だけ見る）
                if lon > 0.5 * v.length - 0.2 && lon < 22.0 && lat < 0.5 * v.width + 0.9 {
                    ped_gap = ped_gap.min((lon - 0.5 * v.length - 1.0).max(0.0));
                }
            });
            // 歩行者に阻まれて長く止まっている車は、徐行で少しずつ進む（にらみ合いの解消）
            let creeping = v.ped_wait > p.creep_after;
            if ped_gap < gap && !creeping {
                gap = ped_gap;
                v_lead = 0.0;
            }
            if creeping {
                v0 = v0.min(1.2);
            }
            let blocked_by_ped = ped_gap < 4.0 && v.v < 0.5;
            accs[i] = idm(v.v, v0, gap, v.v - v_lead, &p);
            let veh = &mut self.vehicles[i];
            veh.ped_wait = if blocked_by_ped {
                veh.ped_wait + dt
            } else {
                0.0
            };
        }

        // 積分とレーン遷移
        for i in 0..self.vehicles.len() {
            if !self.vehicles[i].active {
                continue;
            }
            let a = accs[i];
            let veh = &mut self.vehicles[i];
            veh.v = (veh.v + a * dt).max(0.0);
            veh.s += veh.v * dt;
            if self.halted {
                // 停止命令: 左端に寄せて止める（歩行者が車道の中央を通れるように）
                let edge = self.params.lane_offset + BICYCLE_EXTRA_OFFSET;
                veh.offset = (veh.offset + 0.6 * dt).min(edge.max(veh.offset));
            }
            veh.since_stop += veh.v * dt;
            let mut lane = veh.lane;
            let mut s = veh.s;
            let mut transitioned = false;
            while s >= self.lane_len(lane) {
                s -= self.lane_len(lane);
                match self.vehicles[i].next {
                    Some(n) => {
                        lane = n;
                        transitioned = true;
                        self.vehicles[i].stop_s = None;
                        let nn = self.choose_next(n);
                        self.vehicles[i].next = nn;
                    }
                    None => {
                        self.despawn(i);
                        break;
                    }
                }
            }
            if !self.vehicles[i].active {
                continue;
            }
            // 交差点を抜けたらロックを返す
            if let Some((n, _)) = self.vehicles[i].lock {
                if self.start_node(lane) == n
                    && s > self.graph.nodes[n].radius + 0.5 * self.vehicles[i].length
                {
                    self.release(i);
                }
            }
            let (target, dir) = self.lane_pose_at(lane, s, self.vehicles[i].offset);
            let veh = &mut self.vehicles[i];
            if transitioned {
                // 曲がり角でのレーン中心の飛びを、横ずれとして滑らかに解消
                veh.lat_err = veh.pos - target;
            }
            veh.lat_err *= (-dt * 2.5).exp();
            veh.lane = lane;
            veh.s = s;
            veh.pos = target + veh.lat_err;
            let k = (dt * 6.0).min(1.0);
            veh.heading = (veh.heading + (dir - veh.heading) * k).normalize_or(dir);
        }

        self.respawn();
        self.time += dt;
    }

    fn release(&mut self, i: usize) {
        if let Some((n, mv)) = self.vehicles[i].lock.take() {
            let lk = &mut self.locks[n];
            if let Some(k) = lk.active.iter().position(|(m, _, _)| *m == mv) {
                lk.active[k].1 -= 1;
                if lk.active[k].1 == 0 {
                    lk.active.swap_remove(k);
                }
            }
        }
    }

    fn can_enter(&self, node: usize, mv: Movement) -> bool {
        let lk = &self.locks[node];
        lk.active.iter().all(|&(m, _, since)| {
            if m == mv {
                // 待ち車がいれば一定時間で通行権を譲る
                !(lk.waiting && self.time - since > self.params.lock_hold)
            } else {
                !self.conflicts(node, m, mv)
            }
        })
    }

    /// 2 つの動線の軌跡（2 次ベジェ曲線で近似）が交わるか、同じレーンへ合流するか。
    fn conflicts(&self, node: usize, a: Movement, b: Movement) -> bool {
        if a.to == b.to {
            return true;
        }
        if a.from == b.from {
            return false;
        }
        let pa = self.movement_path(node, a);
        let pb = self.movement_path(node, b);
        pa.windows(2).any(|s| {
            pb.windows(2)
                .any(|t| segments_intersect(s[0], s[1], t[0], t[1]))
        })
    }

    fn movement_path(&self, node: usize, m: Movement) -> [Vec2; 9] {
        let r = self.graph.nodes[node].radius;
        let (p0, d0) = self.lane_pose(m.from, (self.lane_len(m.from) - r).max(0.0));
        let (p2, d2) = self.lane_pose(m.to, r.min(self.lane_len(m.to)));
        let den = d0.perp_dot(d2);
        let c = if den.abs() < 0.2 {
            (p0 + p2) * 0.5
        } else {
            // p0 + t d0 = p2 - u d2 の交点
            let t = (p2 - p0).perp_dot(d2) / den;
            p0 + d0 * t
        };
        std::array::from_fn(|k| {
            let t = k as f32 / 8.0;
            p0 * (1.0 - t) * (1.0 - t) + c * 2.0 * t * (1.0 - t) + p2 * t * t
        })
    }

    fn despawn(&mut self, i: usize) {
        self.release(i);
        self.vehicles[i].active = false;
    }

    /// 縁に出た車を、空いている出入口から再投入する。
    fn respawn(&mut self) {
        if self.entries.is_empty() || self.halted {
            return;
        }
        for i in 0..self.vehicles.len() {
            if self.vehicles[i].active {
                continue;
            }
            let k = self.rand(self.entries.len());
            let lane = self.entries[k];
            let blocked = self
                .vehicles
                .iter()
                .any(|v| v.active && v.lane == lane && v.s < 14.0);
            if blocked {
                continue;
            }
            let style = self.vehicles[i].style;
            let mut v = self.make_vehicle(lane, 0.0);
            v.style = style;
            // 種別は入れ替えない（構成比を保つ）
            let kind = self.vehicles[i].kind;
            if v.kind != kind {
                v.kind = kind;
                (v.length, v.width) = kind.size();
                v.offset = self.vehicles[i].offset;
                v.v0 = self.vehicles[i].v0;
                v.v = v.v0 * 0.6;
                let (p, d) = self.lane_pose_at(lane, 0.0, v.offset);
                v.pos = p;
                v.heading = d;
            }
            self.vehicles[i] = v;
        }
    }

    /// 歩行者から見た障害物（車体を表すカプセル: 端点 2 つと半径）。
    pub fn obstacles(&self) -> Vec<(Vec2, Vec2, f32)> {
        self.vehicles
            .iter()
            .filter(|v| v.active)
            .map(|v| {
                let h = v.heading * (0.5 * (v.length - v.width));
                (v.pos - h, v.pos + h, 0.5 * v.width)
            })
            .collect()
    }
}

fn segments_intersect(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> bool {
    let (r, s) = (b - a, d - c);
    let den = r.perp_dot(s);
    if den.abs() < 1e-6 {
        return false;
    }
    let t = (c - a).perp_dot(s) / den;
    let u = (c - a).perp_dot(r) / den;
    (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u)
}

/// Intelligent Driver Model の加速度。
fn idm(v: f32, v0: f32, gap: f32, dv: f32, p: &Params) -> f32 {
    let free = 1.0 - (v / v0.max(0.1)).powi(4);
    if !gap.is_finite() {
        return p.a_max * free;
    }
    let s_star = p.s0 + (v * p.t_headway + v * dv / (2.0 * (p.a_max * p.b_comf).sqrt())).max(0.0);
    let acc = p.a_max * (free - (s_star / gap.max(0.1)).powi(2));
    acc.max(-9.0)
}

struct PedBins<'a> {
    cell: f32,
    map: HashMap<(i32, i32), Vec<usize>>,
    pts: &'a [Vec2],
}

impl<'a> PedBins<'a> {
    fn new(pts: &'a [Vec2], cell: f32) -> Self {
        let mut map: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
        for (i, p) in pts.iter().enumerate() {
            map.entry(((p.x / cell).floor() as i32, (p.y / cell).floor() as i32))
                .or_default()
                .push(i);
        }
        Self { cell, map, pts }
    }
    fn for_near(&self, c: Vec2, r: f32, mut f: impl FnMut(Vec2)) {
        let (x0, y0) = (
            ((c.x - r) / self.cell).floor() as i32,
            ((c.y - r) / self.cell).floor() as i32,
        );
        let (x1, y1) = (
            ((c.x + r) / self.cell).floor() as i32,
            ((c.y + r) / self.cell).floor() as i32,
        );
        for y in y0..=y1 {
            for x in x0..=x1 {
                if let Some(v) = self.map.get(&(x, y)) {
                    v.iter().for_each(|&i| f(self.pts[i]));
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use walkmap::WalkGrid;

    fn grid() -> WalkGrid {
        let m = plateau_gml::parse_str(include_str!("../../../testdata/sample.gml")).unwrap();
        let mut g = WalkGrid::from_city(&m, 0.5).unwrap();
        g.mark_carriageway(3.0, 6.0);
        g
    }

    #[test]
    fn extracts_grid_network() {
        let g = RoadGraph::from_grid(&grid(), 3.0);
        let junctions = g.nodes.iter().filter(|n| n.edges.len() >= 3).count();
        let borders = g.nodes.iter().filter(|n| n.border).count();
        // 「井」の字の道路: 交差点 4、出入口 8、道路区間 12
        assert_eq!(junctions, 4, "{:#?}", g.nodes);
        assert_eq!(borders, 8, "{:#?}", g.nodes);
        assert_eq!(g.edges.len(), 12);
        // 交差点の位置は道路中心 (70 or 130, 70 or 130) 付近
        for n in g.nodes.iter().filter(|n| n.edges.len() >= 3) {
            for c in [n.pos.x, n.pos.y] {
                assert!(
                    (c - 70.0).abs() < 2.0 || (c - 130.0).abs() < 2.0,
                    "{:?}",
                    n.pos
                );
            }
        }
    }

    #[test]
    fn vehicles_flow_without_collisions() {
        let g = grid();
        let net = RoadGraph::from_grid(&g, 3.0);
        let mut t = Traffic::new(net, 30, 5);
        assert_eq!(t.vehicles.len(), 30);
        let mut moved = 0.0;
        for _ in 0..(180.0 / 0.05) as usize {
            let before: Vec<Vec2> = t.vehicles.iter().map(|v| v.pos).collect();
            t.step(0.05, &[]);
            moved += t
                .vehicles
                .iter()
                .zip(&before)
                .map(|(v, b)| v.pos.distance(*b))
                .sum::<f32>();
            // 同じレーン上で車が重ならない
            for a in t.vehicles.iter().filter(|v| v.active) {
                for b in t.vehicles.iter().filter(|v| v.active) {
                    // 自転車は車道の左端を走り、車に追い越されるので同じ列どうしだけ調べる
                    if !std::ptr::eq(a, b)
                        && a.lane == b.lane
                        && a.kind.is_bicycle() == b.kind.is_bicycle()
                    {
                        assert!(
                            (a.s - b.s).abs() > 0.5 * (a.length + b.length) - 0.5,
                            "overlap at t={}",
                            t.time
                        );
                    }
                }
            }
        }
        // 平均 3 m/s 以上で流れている（渋滞で詰まっていない）
        let avg = moved / (30.0 * 180.0);
        assert!(avg > 3.0, "avg speed {avg}");
    }

    #[test]
    fn mixes_buses_and_bicycles() {
        let g = grid();
        let net = RoadGraph::from_grid(&g, 3.0);
        let params = Params {
            mix: VehicleMix {
                truck: 0.0,
                bus: 0.3,
                bicycle: 0.3,
            },
            ..Default::default()
        };
        let mut t = Traffic::new_with(net, 40, 3, params);
        let count = |t: &Traffic, k| t.vehicles.iter().filter(|v| v.kind == k).count();
        assert!(count(&t, VehicleKind::Bus) >= 4);
        assert!(count(&t, VehicleKind::Bicycle) >= 4);
        for v in &t.vehicles {
            assert_eq!((v.length, v.width), v.kind.size());
            if v.kind.is_bicycle() {
                assert!(v.v0 <= 5.6, "自転車は 20km/h 以下");
                assert!(v.offset > t.params.lane_offset, "自転車は左端寄り");
            }
        }
        let mut dwelled = false;
        let mut bike_speed = (0.0, 0);
        for _ in 0..(120.0 / 0.05) as usize {
            t.step(0.05, &[]);
            dwelled |= t
                .vehicles
                .iter()
                .any(|v| v.active && v.kind == VehicleKind::Bus && v.dwell > 0.0 && v.v < 0.1);
            for v in t.vehicles.iter().filter(|v| v.active && v.kind.is_bicycle()) {
                bike_speed.0 += v.v;
                bike_speed.1 += 1;
            }
        }
        assert!(dwelled, "バスが停留所で停車する");
        let avg = bike_speed.0 / bike_speed.1 as f32;
        assert!(avg > 1.5 && avg < 5.6, "自転車の平均速度 {avg}");
    }

    #[test]
    fn halt_stops_all_vehicles() {
        let g = grid();
        let net = RoadGraph::from_grid(&g, 3.0);
        let mut t = Traffic::new(net, 30, 4);
        for _ in 0..200 {
            t.step(0.05, &[]);
        }
        t.halted = true;
        for _ in 0..(15.0 / 0.05) as usize {
            t.step(0.05, &[]);
        }
        assert!(t.vehicles.iter().filter(|v| v.active).all(|v| v.v == 0.0));
    }

    #[test]
    fn vehicle_stops_for_pedestrian() {
        let g = grid();
        let net = RoadGraph::from_grid(&g, 3.0);
        let mut t = Traffic::new(net, 1, 9);
        let v = &t.vehicles[0];
        let ped = v.pos + v.heading * 15.0;
        for _ in 0..200 {
            t.step(0.05, &[ped]);
        }
        let v = &t.vehicles[0];
        assert!(v.pos.distance(ped) > 0.5 * v.length, "ひかない");
        assert!(v.v < 0.5, "止まって待つ v={}", v.v);
    }
}
