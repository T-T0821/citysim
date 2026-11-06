//! イベント・混雑シナリオと通行止めの設定。
//!
//! TOML の設定ファイル（[`ScenarioFile`]）で次を指定する:
//! - `[[source]]` 発生源（駅出口・会場など）。時刻ごとの発生人数 [人/分] の折れ線と行き先。
//! - `[[destination]]` 行き先（会場・駅など）。範囲の縁の「東西南北」も行き先に使える。
//! - `[[closure]]` 通行止め・工事の範囲（全面 / 歩道のみ / 車道のみ）。
//!
//! 座標は既定で `[緯度, 経度]`。`[scenario] coords = "local"` なら範囲の南西端からの `[x東, y北]` [m]。
//! 時刻は秒数か `"HH:MM"`（`[scenario] start = "17:00"` からの経過時間に直す）。
//!
//! 例は `scenarios/` を参照。

use crate::{Agent, Rng, Simulation};
use glam::Vec2;
use serde::{Deserialize, Serialize};
use walkmap::closure::{line_polygons, polyline_hits_polygon, rect_polygon};
use walkmap::{ClosureKind, FlowField, LocalProjection, Polygon, WalkGrid};

// ---------------------------------------------------------------------------
// 設定ファイル

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ScenarioFile {
    #[serde(default)]
    pub scenario: MetaSpec,
    #[serde(default, rename = "source", skip_serializing_if = "Vec::is_empty")]
    pub sources: Vec<SourceSpec>,
    #[serde(default, rename = "destination", skip_serializing_if = "Vec::is_empty")]
    pub destinations: Vec<DestSpec>,
    #[serde(default, rename = "closure", skip_serializing_if = "Vec::is_empty")]
    pub closures: Vec<ClosureSpec>,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MetaSpec {
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub name: String,
    /// "geo"（[緯度, 経度]、既定）か "local"（[x, y] m）
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub coords: Option<String>,
    /// シミュレーション開始時刻 "HH:MM"
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub start: Option<String>,
    /// 計算する長さ [s]（CLI の --duration の既定値になる）
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub duration: Option<TimeSpec>,
    /// 発生源とは別に範囲内を通り抜ける人数（--agents の既定値）
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub background_agents: Option<usize>,
    /// 車両数（--vehicles の既定値）
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub vehicles: Option<usize>,
    /// 集計区画の一辺 [m]
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub zone_size: Option<f32>,
}

/// 秒数か "HH:MM[:SS]"。
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(untagged)]
pub enum TimeSpec {
    Secs(f32),
    Clock(String),
}

impl TimeSpec {
    /// 開始時刻 `start`（その日の 0 時からの秒）からの経過秒に直す。
    pub fn secs(&self, start: Option<f32>) -> Result<f32, String> {
        match self {
            TimeSpec::Secs(s) => Ok(*s),
            TimeSpec::Clock(c) => Ok(parse_clock(c)? - start.unwrap_or(0.0)),
        }
    }
}

/// "HH:MM" / "HH:MM:SS" を 0 時からの秒に。
pub fn parse_clock(s: &str) -> Result<f32, String> {
    let v: Vec<f32> = s
        .split(':')
        .map(|t| t.trim().parse::<f32>())
        .collect::<Result<_, _>>()
        .map_err(|_| format!("時刻は HH:MM か HH:MM:SS で指定してください: {s}"))?;
    match v[..] {
        [h, m] => Ok(h * 3600.0 + m * 60.0),
        [h, m, sec] => Ok(h * 3600.0 + m * 60.0 + sec),
        _ => Err(format!("時刻は HH:MM か HH:MM:SS で指定してください: {s}")),
    }
}

/// 秒を "HH:MM:SS" に。
pub fn format_clock(secs: f32) -> String {
    let s = secs.max(0.0).round() as u32;
    format!("{:02}:{:02}:{:02}", s / 3600, s / 60 % 60, s % 60)
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SourceSpec {
    pub name: String,
    pub at: [f64; 2],
    /// 発生範囲の半径 [m]
    #[serde(default = "default_source_radius")]
    pub radius: f32,
    /// [[時刻, 人/分], ...]。間は線形補間、範囲外は 0。
    pub rate: Vec<(TimeSpec, f32)>,
    /// 行き先（destination の name か 東/西/南/北）。省略時は全ての行き先から等確率。
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub to: Vec<String>,
    /// `to` と同じ長さの重み
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub weights: Option<Vec<f32>>,
    /// 平均歩行速度 [m/s]
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub speed: Option<f32>,
    #[serde(default = "yes", skip_serializing_if = "is_true")]
    pub enabled: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DestSpec {
    pub name: String,
    pub at: [f64; 2],
    #[serde(default = "default_dest_radius")]
    pub radius: f32,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ClosureSpec {
    #[serde(default)]
    pub name: String,
    /// "all"（全面）/ "sidewalk"（歩道のみ）/ "road"（車道のみ）
    #[serde(default = "default_kind")]
    pub kind: String,
    /// 多角形の頂点
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub polygon: Option<Vec<[f64; 2]>>,
    /// 長方形の対角 2 点
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rect: Option<[[f64; 2]; 2]>,
    /// 折れ線（`width` の帯として封鎖）
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub line: Option<Vec<[f64; 2]>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub width: Option<f32>,
    #[serde(default = "yes", skip_serializing_if = "is_true")]
    pub enabled: bool,
}

fn default_source_radius() -> f32 {
    4.0
}
fn default_dest_radius() -> f32 {
    5.0
}
fn default_kind() -> String {
    "all".into()
}
fn yes() -> bool {
    true
}
fn is_true(b: &bool) -> bool {
    *b
}

impl ScenarioFile {
    pub fn parse(text: &str) -> Result<Self, String> {
        toml::from_str(text).map_err(|e| e.to_string())
    }
    pub fn load(path: &std::path::Path) -> Result<Self, String> {
        let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
        Self::parse(&text).map_err(|e| format!("{}: {e}", path.display()))
    }
    pub fn to_toml(&self) -> String {
        toml::to_string_pretty(self).unwrap_or_default()
    }
}

// ---------------------------------------------------------------------------
// 実行時の表現（局所座標 [m]、時刻は開始からの秒）

#[derive(Debug, Clone)]
pub struct Source {
    pub name: String,
    pub pos: Vec2,
    pub radius: f32,
    /// (経過秒, 人/分)。時刻順。
    pub rate: Vec<(f32, f32)>,
    pub to: Vec<(String, f32)>,
    pub speed: Option<f32>,
    pub enabled: bool,
}

impl Source {
    /// 時刻 t [s] の発生率 [人/分]
    pub fn rate_at(&self, t: f32) -> f32 {
        rate_at(&self.rate, t)
    }
    /// 区間 [0, t] の累計発生人数（台形積分）
    pub fn total_until(&self, t: f32) -> f32 {
        let mut acc = 0.0;
        for w in self.rate.windows(2) {
            let (t0, r0) = w[0];
            let t1 = w[1].0;
            if t <= t0 {
                break;
            }
            let te = t.min(t1);
            let re = rate_at(&self.rate, te);
            acc += (r0 + re) * 0.5 * (te - t0) / 60.0;
        }
        acc
    }
}

pub fn rate_at(curve: &[(f32, f32)], t: f32) -> f32 {
    match curve {
        [] => 0.0,
        [(t0, r)] => {
            if (t - t0).abs() < 1e-3 {
                *r
            } else {
                0.0
            }
        }
        _ => {
            if t < curve[0].0 || t > curve[curve.len() - 1].0 {
                return 0.0;
            }
            for w in curve.windows(2) {
                let ((t0, r0), (t1, r1)) = (w[0], w[1]);
                if t <= t1 {
                    let k = if t1 > t0 { (t - t0) / (t1 - t0) } else { 1.0 };
                    return r0 + (r1 - r0) * k.clamp(0.0, 1.0);
                }
            }
            0.0
        }
    }
}

#[derive(Debug, Clone)]
pub struct Destination {
    pub name: String,
    pub pos: Vec2,
    pub radius: f32,
}

#[derive(Debug, Clone)]
pub struct Closure {
    pub name: String,
    pub kind: ClosureKind,
    pub polys: Vec<Polygon>,
    pub enabled: bool,
}

impl Closure {
    /// 中心付近の正方形の封鎖（ビューアでクリックして置く用）
    pub fn square(name: String, kind: ClosureKind, center: Vec2, size: f32) -> Self {
        let h = Vec2::splat(size * 0.5);
        Self {
            name,
            kind,
            polys: vec![rect_polygon(center - h, center + h)],
            enabled: true,
        }
    }
    pub fn centroid(&self) -> Vec2 {
        let pts: Vec<Vec2> = self.polys.iter().flatten().copied().collect();
        pts.iter().copied().sum::<Vec2>() / pts.len().max(1) as f32
    }
}

#[derive(Debug, Clone, Default)]
pub struct Scenario {
    pub name: String,
    /// 開始時刻（0 時からの秒）
    pub start_clock: Option<f32>,
    pub duration: Option<f32>,
    pub background_agents: Option<usize>,
    pub vehicles: Option<usize>,
    pub zone_size: Option<f32>,
    pub sources: Vec<Source>,
    pub destinations: Vec<Destination>,
    pub closures: Vec<Closure>,
    /// 通行止めをまとめて有効にするか（Before/After 比較用）
    pub closures_on: bool,
    /// 発生人数の倍率
    pub rate_scale: f32,
}

impl Scenario {
    pub fn empty() -> Self {
        Self {
            closures_on: true,
            rate_scale: 1.0,
            ..Default::default()
        }
    }

    pub fn from_file(f: &ScenarioFile, proj: &LocalProjection) -> Result<Self, String> {
        let local = match f.scenario.coords.as_deref() {
            None | Some("geo") => false,
            Some("local") => true,
            Some(c) => return Err(format!("coords は geo か local です: {c}")),
        };
        let pt = |p: [f64; 2]| -> Vec2 {
            if local {
                Vec2::new(p[0] as f32, p[1] as f32)
            } else {
                proj.to_local(&plateau_gml_point(p[0], p[1]))
            }
        };
        let start_clock = f.scenario.start.as_deref().map(parse_clock).transpose()?;
        let mut sources = vec![];
        for s in &f.sources {
            let mut rate = vec![];
            for (t, r) in &s.rate {
                rate.push((t.secs(start_clock)?, r.max(0.0)));
            }
            rate.sort_by(|a, b| a.0.total_cmp(&b.0));
            let w = s.weights.clone().unwrap_or_default();
            if !w.is_empty() && w.len() != s.to.len() {
                return Err(format!(
                    "発生源 {}: weights の数 ({}) が to の数 ({}) と違います",
                    s.name,
                    w.len(),
                    s.to.len()
                ));
            }
            sources.push(Source {
                name: s.name.clone(),
                pos: pt(s.at),
                radius: s.radius.max(0.5),
                rate,
                to: s
                    .to
                    .iter()
                    .enumerate()
                    .map(|(i, n)| (n.clone(), w.get(i).copied().unwrap_or(1.0)))
                    .collect(),
                speed: s.speed,
                enabled: s.enabled,
            });
        }
        let destinations = f
            .destinations
            .iter()
            .map(|d| Destination {
                name: d.name.clone(),
                pos: pt(d.at),
                radius: d.radius.max(1.0),
            })
            .collect();
        let mut closures = vec![];
        for (i, c) in f.closures.iter().enumerate() {
            let kind = ClosureKind::from_key(&c.kind)
                .ok_or_else(|| format!("closure の kind は all/sidewalk/road です: {}", c.kind))?;
            let mut polys = vec![];
            if let Some(p) = &c.polygon {
                polys.push(p.iter().map(|&q| pt(q)).collect());
            }
            if let Some([a, b]) = c.rect {
                polys.push(rect_polygon(pt(a), pt(b)));
            }
            if let Some(l) = &c.line {
                let pts: Vec<Vec2> = l.iter().map(|&q| pt(q)).collect();
                polys.extend(line_polygons(&pts, c.width.unwrap_or(4.0)));
            }
            if polys.is_empty() {
                return Err(format!(
                    "closure {} に polygon / rect / line のどれかが必要です",
                    if c.name.is_empty() {
                        format!("#{}", i + 1)
                    } else {
                        c.name.clone()
                    }
                ));
            }
            closures.push(Closure {
                name: if c.name.is_empty() {
                    format!("通行止め{}", i + 1)
                } else {
                    c.name.clone()
                },
                kind,
                polys,
                enabled: c.enabled,
            });
        }
        Ok(Self {
            name: f.scenario.name.clone(),
            start_clock,
            duration: f
                .scenario
                .duration
                .as_ref()
                .map(|d| d.secs(start_clock))
                .transpose()?,
            background_agents: f.scenario.background_agents,
            vehicles: f.scenario.vehicles,
            zone_size: f.scenario.zone_size,
            sources,
            destinations,
            closures,
            closures_on: true,
            rate_scale: 1.0,
        })
    }

    /// 設定ファイルの形に戻す（座標は緯度経度で書く）。
    pub fn to_file(&self, proj: &LocalProjection) -> ScenarioFile {
        let geo = |p: Vec2| {
            let (lat, lon) = proj.to_geo(p);
            [round6(lat), round6(lon)]
        };
        let time = |t: f32| match self.start_clock {
            Some(s0) => TimeSpec::Clock(format_clock(s0 + t)),
            None => TimeSpec::Secs(t),
        };
        ScenarioFile {
            scenario: MetaSpec {
                name: self.name.clone(),
                coords: None,
                start: self.start_clock.map(|s| format_clock(s)[..5].to_string()),
                duration: self.duration.map(TimeSpec::Secs),
                background_agents: self.background_agents,
                vehicles: self.vehicles,
                zone_size: self.zone_size,
            },
            sources: self
                .sources
                .iter()
                .map(|s| SourceSpec {
                    name: s.name.clone(),
                    at: geo(s.pos),
                    radius: s.radius,
                    rate: s.rate.iter().map(|&(t, r)| (time(t), r)).collect(),
                    to: s.to.iter().map(|(n, _)| n.clone()).collect(),
                    weights: s
                        .to
                        .iter()
                        .any(|(_, w)| (*w - 1.0).abs() > 1e-6)
                        .then(|| s.to.iter().map(|(_, w)| *w).collect()),
                    speed: s.speed,
                    enabled: s.enabled,
                })
                .collect(),
            destinations: self
                .destinations
                .iter()
                .map(|d| DestSpec {
                    name: d.name.clone(),
                    at: geo(d.pos),
                    radius: d.radius,
                })
                .collect(),
            closures: self
                .closures
                .iter()
                .flat_map(|c| {
                    c.polys.iter().enumerate().map(move |(i, p)| ClosureSpec {
                        name: if c.polys.len() > 1 {
                            format!("{}-{}", c.name, i + 1)
                        } else {
                            c.name.clone()
                        },
                        kind: c.kind.key().into(),
                        polygon: Some(p.iter().map(|&q| geo(q)).collect()),
                        enabled: c.enabled,
                        ..Default::default()
                    })
                })
                .collect(),
        }
    }

    pub fn active_closures(&self) -> impl Iterator<Item = &Closure> {
        self.closures
            .iter()
            .filter(move |c| self.closures_on && c.enabled)
    }

    /// 歩行空間に通行止めを反映する。戻り値は通れなくしたセル数。
    pub fn apply_closures(&self, grid: &mut WalkGrid) -> usize {
        let list: Vec<(&Polygon, ClosureKind)> = self
            .active_closures()
            .flat_map(|c| c.polys.iter().map(move |p| (p, c.kind)))
            .collect();
        if list.is_empty() {
            return 0;
        }
        grid.apply_closures(list, crate::scenario::SIDEWALK_W)
    }

    /// 車道の中心線 `pts` が車両の通行止めにかかるか。
    pub fn blocks_road(&self, pts: &[Vec2]) -> bool {
        self.active_closures()
            .filter(|c| c.kind.blocks_vehicles())
            .any(|c| c.polys.iter().any(|p| polyline_hits_polygon(pts, p)))
    }

    /// 時刻 t の全発生源の合計 [人/分]
    pub fn total_rate(&self, t: f32) -> f32 {
        self.sources
            .iter()
            .filter(|s| s.enabled)
            .map(|s| s.rate_at(t))
            .sum::<f32>()
            * self.rate_scale
    }

    /// 発生率の折れ線の最後の時刻
    pub fn last_time(&self) -> f32 {
        self.sources
            .iter()
            .filter_map(|s| s.rate.last().map(|r| r.0))
            .fold(0.0, f32::max)
    }

    /// 開始時刻つきの表示（"17:05:30" か "305 s"）
    pub fn clock_label(&self, t: f32) -> String {
        match self.start_clock {
            Some(s0) => format_clock(s0 + t),
            None => format!("{t:.0} s"),
        }
    }

    /// 行き先の流れ場を `sim.goals` に追加し、発生源を準備する。
    /// `goal_names` は `sim.goals` と同じ順の名前（範囲の縁の 東/西/南/北 など）で、追加分が足される。
    pub fn install(
        &self,
        sim: &mut Simulation,
        goal_names: &mut Vec<String>,
        seed: u64,
    ) -> Result<Spawner, String> {
        for d in &self.destinations {
            let cells = cells_near(&sim.grid, d.pos, d.radius);
            if cells.is_empty() {
                return Err(format!("行き先 {} の近くに歩ける場所がありません", d.name));
            }
            sim.goals.push(FlowField::compute(&sim.grid, &cells));
            goal_names.push(d.name.clone());
        }
        let edge_goals: Vec<usize> = goal_names
            .iter()
            .enumerate()
            .filter(|(_, n)| ["東", "西", "南", "北"].contains(&n.as_str()))
            .map(|(i, _)| i)
            .collect();
        let mut out = vec![];
        for s in self.sources.iter() {
            let cells = cells_near(&sim.grid, s.pos, s.radius);
            if cells.is_empty() {
                return Err(format!("発生源 {} の近くに歩ける場所がありません", s.name));
            }
            let mut targets = vec![];
            if s.to.is_empty() {
                targets.extend((0..goal_names.len()).map(|g| (g, 1.0)));
            }
            for (name, w) in &s.to {
                match name.as_str() {
                    "縁" | "any_edge" => targets.extend(edge_goals.iter().map(|&g| (g, *w))),
                    _ => {
                        let g = goal_names.iter().position(|n| n == name).ok_or_else(|| {
                            format!(
                                "発生源 {} の行き先 {} がありません（候補: {}）",
                                s.name,
                                name,
                                goal_names.join(", ")
                            )
                        })?;
                        targets.push((g, *w));
                    }
                }
            }
            // 発生源から行けない行き先は外す（通行止めで分断された場合など）
            let c = sim.grid.center(cells[0].0, cells[0].1);
            targets.retain(|&(g, w)| w > 0.0 && sim.goals[g].distance(&sim.grid, c).is_finite());
            out.push(SourceRt {
                cells,
                targets,
                acc: 0.0,
                spawned: 0,
            });
        }
        Ok(Spawner {
            first_index: sim.agents.len(),
            sources: out,
            rng: Rng::new(seed ^ 0xA5A5_5A5A),
        })
    }
}

fn round6(x: f64) -> f64 {
    (x * 1e6).round() / 1e6
}

fn plateau_gml_point(lat: f64, lon: f64) -> plateau_gml::GeoPoint {
    plateau_gml::GeoPoint { lat, lon, h: 0.0 }
}

/// 中心から半径内の歩行可能セル（無ければ 30m 以内で最も近い歩行可能セルのまわり）。歩道を優先する。
fn cells_near(g: &WalkGrid, c: Vec2, r: f32) -> Vec<(usize, usize)> {
    let mut v = cells_near_any(g, c, r);
    if !g.carriageway.is_empty() && v.iter().any(|&(x, y)| !g.carriageway[g.idx(x, y)]) {
        v.retain(|&(x, y)| !g.carriageway[g.idx(x, y)]);
    }
    v
}

fn cells_near_any(g: &WalkGrid, c: Vec2, r: f32) -> Vec<(usize, usize)> {
    let near = |r: f32| -> Vec<(usize, usize)> {
        let lo = ((c - Vec2::splat(r) - g.origin) / g.cell)
            .floor()
            .max(Vec2::ZERO);
        let hi = (c + Vec2::splat(r) - g.origin) / g.cell;
        let mut v = vec![];
        for y in lo.y as usize..=(hi.y.max(0.0) as usize).min(g.h.saturating_sub(1)) {
            for x in lo.x as usize..=(hi.x.max(0.0) as usize).min(g.w.saturating_sub(1)) {
                if g.walkable[g.idx(x, y)] && g.center(x, y).distance(c) <= r {
                    v.push((x, y));
                }
            }
        }
        v
    };
    let v = near(r);
    if !v.is_empty() {
        return v;
    }
    let all = near(30.0);
    let Some(&best) = all.iter().min_by(|a, b| {
        g.center(a.0, a.1)
            .distance(c)
            .total_cmp(&g.center(b.0, b.1).distance(c))
    }) else {
        return vec![];
    };
    let p = g.center(best.0, best.1);
    all.into_iter()
        .filter(|&(x, y)| g.center(x, y).distance(p) <= r.max(1.5))
        .collect()
}

#[derive(Debug, Clone)]
struct SourceRt {
    cells: Vec<(usize, usize)>,
    targets: Vec<(usize, f32)>,
    /// 発生待ちの人数（出口が詰まっていると溜まる）
    acc: f32,
    spawned: u32,
}

/// 発生源から時刻に応じて歩行者を出す。`first_index` 以降のエージェントがシナリオ由来。
pub struct Spawner {
    pub first_index: usize,
    sources: Vec<SourceRt>,
    rng: Rng,
}

impl Spawner {
    /// 発生源ごとの累計発生人数
    pub fn spawned(&self) -> Vec<u32> {
        self.sources.iter().map(|s| s.spawned).collect()
    }
    pub fn total_spawned(&self) -> u32 {
        self.sources.iter().map(|s| s.spawned).sum()
    }
    /// 混雑で出られずに待っている人数
    pub fn backlog(&self) -> u32 {
        self.sources
            .iter()
            .map(|s| s.acc.floor().max(0.0) as u32)
            .sum()
    }

    /// `sim.time` から dt 秒ぶんの発生を行う。`speed_scale` は歩行速度の倍率。
    pub fn step(&mut self, sc: &Scenario, sim: &mut Simulation, dt: f32, speed_scale: f32) {
        let t = sim.time;
        for (si, src) in sc.sources.iter().enumerate() {
            let Some(rt) = self.sources.get_mut(si) else {
                break;
            };
            if !src.enabled || rt.targets.is_empty() {
                continue;
            }
            rt.acc += src.rate_at(t) * sc.rate_scale / 60.0 * dt;
            // 1 ステップで出せる人数には上限（出口の容量）を設ける
            let mut tries = 0;
            while rt.acc >= 1.0 && tries < 8 {
                tries += 1;
                let (cx, cy) = rt.cells[(self.rng.next_u64() % rt.cells.len() as u64) as usize];
                let pos = sim.grid.center(cx, cy)
                    + Vec2::new(self.rng.range(-0.4, 0.4), self.rng.range(-0.4, 0.4))
                        * sim.grid.cell;
                if !sim.grid.is_walkable(pos)
                    || sim
                        .agents
                        .iter()
                        .any(|a| !a.arrived && a.pos.distance_squared(pos) < 0.36)
                {
                    continue;
                }
                let total: f32 = rt.targets.iter().map(|t| t.1).sum();
                let mut pick = self.rng.f32() * total;
                let mut goal = rt.targets[0].0;
                for &(g, w) in &rt.targets {
                    if pick < w {
                        goal = g;
                        break;
                    }
                    pick -= w;
                }
                let agent = Agent {
                    pos,
                    vel: Vec2::ZERO,
                    radius: self.rng.range(0.22, 0.28),
                    desired_speed: (self.rng.normal(src.speed.unwrap_or(1.34), 0.26) * speed_scale)
                        .clamp(0.3, 3.0),
                    goal,
                    arrived: false,
                };
                match sim.agents[self.first_index.min(sim.agents.len())..]
                    .iter()
                    .position(|a| a.arrived)
                {
                    Some(k) => sim.agents[self.first_index + k] = agent,
                    None => sim.agents.push(agent),
                }
                rt.acc -= 1.0;
                rt.spawned += 1;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = r#"
[scenario]
name = "テスト"
coords = "local"
start = "17:00"
background_agents = 0

[[source]]
name = "駅出口"
at = [5.0, 70.0]
rate = [["17:00", 0], ["17:01", 120], ["17:02", 0]]
to = ["会場"]

[[destination]]
name = "会場"
at = [195.0, 70.0]
radius = 6

[[closure]]
name = "工事"
kind = "all"
rect = [[90.0, 60.0], [110.0, 80.0]]
"#;

    #[test]
    fn rate_curve_interpolates() {
        let c = [(0.0, 0.0), (60.0, 120.0), (120.0, 0.0)];
        assert_eq!(rate_at(&c, -1.0), 0.0);
        assert!((rate_at(&c, 30.0) - 60.0).abs() < 1e-4);
        assert!((rate_at(&c, 90.0) - 60.0).abs() < 1e-4);
        assert_eq!(rate_at(&c, 121.0), 0.0);
    }

    #[test]
    fn parse_and_roundtrip() {
        let f = ScenarioFile::parse(SAMPLE).unwrap();
        let proj = LocalProjection::new(35.68, 139.76);
        let sc = Scenario::from_file(&f, &proj).unwrap();
        assert_eq!(sc.sources[0].rate[1], (60.0, 120.0));
        assert_eq!(sc.start_clock, Some(17.0 * 3600.0));
        assert!((sc.sources[0].total_until(1e9) - 120.0).abs() < 1e-3);
        let back = Scenario::from_file(
            &ScenarioFile::parse(&sc.to_file(&proj).to_toml()).unwrap(),
            &proj,
        )
        .unwrap();
        assert!(back.sources[0].pos.distance(sc.sources[0].pos) < 0.2);
        assert_eq!(back.sources[0].rate, sc.sources[0].rate);
        assert_eq!(back.closures.len(), 1);
    }

    #[test]
    fn event_crowd_detours_around_closure() {
        let m = plateau_gml::parse_str(include_str!("../../../testdata/sample.gml")).unwrap();
        let proj_grid = walkmap::WalkGrid::from_city(&m, 0.5).unwrap();
        let mut sc =
            Scenario::from_file(&ScenarioFile::parse(SAMPLE).unwrap(), &proj_grid.projection)
                .unwrap();

        let run = |sc: &Scenario| {
            let mut grid = proj_grid.clone();
            sc.apply_closures(&mut grid);
            let (mut sim, names) = crate::scenario::through_traffic(grid, 0, 1).unwrap();
            let mut names: Vec<String> = names.iter().map(|s| s.to_string()).collect();
            let mut sp = sc.install(&mut sim, &mut names, 1).unwrap();
            let mut inside = 0usize;
            for _ in 0..(600.0 / 0.05) as usize {
                sp.step(sc, &mut sim, 0.05, 1.0);
                sim.step(0.05);
                inside += sim
                    .agents
                    .iter()
                    .filter(|a| {
                        !a.arrived
                            && a.pos.x > 90.0
                            && a.pos.x < 110.0
                            && a.pos.y > 60.0
                            && a.pos.y < 80.0
                    })
                    .count();
            }
            (sp.total_spawned(), sim.active(), inside)
        };
        let (spawned, active, inside) = run(&sc);
        assert!(spawned >= 110, "ほぼ 120 人発生すること: {spawned}");
        assert_eq!(active, 0, "全員が会場に着くこと");
        assert_eq!(inside, 0, "通行止めの中を歩かないこと");
        sc.closures_on = false;
        let (_, _, inside_open) = run(&sc);
        assert!(inside_open > 0, "通行止めが無ければその場所を通ること");
    }
}
