//! イベント・混雑シナリオ、通行止め、結果のエクスポートの設定パネルと地図上の表示。
//!
//! - シナリオファイル（TOML）の読込・保存、発生源・行き先・通行止めの編集
//! - 地図をクリックして発生源・行き先・通行止めを置く
//! - 「Before として保存」した結果との差分ヒートマップ
//! - 区画ごとの集計（CSV / PNG）の書き出し

use crate::camera::UiCapture;
use crate::geometry::{to_world, MeshBuilder};
use crate::ui::Settings;
use crate::SimState;
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy_egui::{egui, EguiContexts};
use crowd::event::{
    format_clock, parse_clock, Closure, Destination, Scenario, ScenarioFile, Source, Spawner,
};
use crowd::report::ZoneStats;
use walkmap::closure::line_polygons;
use walkmap::{ClosureKind, WalkGrid};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PlaceMode {
    None,
    Closure,
    Source,
    Destination,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ClosureShape {
    /// ドラッグした線分を幅つきの帯にする（道路を横切るように引く）
    Line,
    /// ドラッグした対角の長方形
    Rect,
}

#[derive(Resource)]
pub struct ScenarioState {
    pub scenario: Scenario,
    /// 通行止めを反映する前の歩行空間
    pub base_grid: WalkGrid,
    pub spawner: Option<Spawner>,
    pub stats: ZoneStats,
    pub before: Option<ZoneStats>,
    /// 通行止めで通れなくなったセル
    pub closed: Vec<bool>,
    /// 行き先の名前（`sim.goals` と同じ順）
    pub goal_names: Vec<String>,
    pub zone_size: f32,
    pub diff_view: bool,
    pub path: String,
    pub export_dir: String,
    pub message: String,
    pub place: PlaceMode,
    pub place_kind: ClosureKind,
    pub place_shape: ClosureShape,
    pub place_width: f32,
    drag_start: Option<Vec2>,
    pub markers_dirty: bool,
    /// 設定が変わって作り直しが必要
    pub pending: bool,
}

impl ScenarioState {
    pub fn new(base_grid: WalkGrid, scenario: Scenario, path: Option<&std::path::Path>) -> Self {
        let zone_size = scenario.zone_size.unwrap_or(20.0);
        let stats = ZoneStats::new(&base_grid, zone_size);
        Self {
            scenario,
            closed: vec![false; base_grid.w * base_grid.h],
            base_grid,
            spawner: None,
            stats,
            before: None,
            goal_names: vec![],
            zone_size,
            diff_view: false,
            path: path.map(|p| p.display().to_string()).unwrap_or_else(|| {
                #[cfg(target_arch = "wasm32")]
                return crate::web::DEFAULT_SCENARIO_PATH.into();
                #[cfg(not(target_arch = "wasm32"))]
                return "scenario.toml".into();
            }),
            export_dir: "citysim-export".into(),
            message: String::new(),
            place: PlaceMode::None,
            place_kind: ClosureKind::All,
            place_shape: ClosureShape::Line,
            place_width: 6.0,
            drag_start: None,
            markers_dirty: true,
            pending: false,
        }
    }

    pub fn load(path: &std::path::Path, grid: &WalkGrid) -> Result<Scenario, String> {
        #[cfg(target_arch = "wasm32")]
        let f = ScenarioFile::parse(crate::web::scenario_text(path)?)?;
        #[cfg(not(target_arch = "wasm32"))]
        let f = ScenarioFile::load(path)?;
        Scenario::from_file(&f, &grid.projection)
    }

    /// 通行止めを反映した歩行空間を作る（`closed` も更新）。
    pub fn closed_grid(&mut self) -> WalkGrid {
        let mut g = self.base_grid.clone();
        self.scenario.apply_closures(&mut g);
        self.closed = self
            .base_grid
            .walkable
            .iter()
            .zip(&g.walkable)
            .map(|(&a, &b)| a && !b)
            .collect();
        g
    }

    /// 1 ステップぶんの発生（`sim.step` の前に呼ぶ）
    pub fn before_step(&mut self, sim: &mut crowd::Simulation, dt: f32, speed_scale: f32) {
        if let Some(sp) = self.spawner.as_mut() {
            sp.step(&self.scenario, sim, dt, speed_scale);
        }
    }

    /// 背景の通り抜け歩行者の数（これ以降の番号は発生源の人）
    pub fn background_limit(&self, n: usize) -> usize {
        self.spawner.as_ref().map_or(n, |s| s.first_index.min(n))
    }
}

/// 差分ヒートマップの RGBA（テクスチャの上端 = 北）
pub fn diff_pixels(st: &ScenarioState, g: &WalkGrid, data: &mut [u8]) {
    let Some(before) = st
        .before
        .as_ref()
        .filter(|b| b.heat.len() == st.stats.heat.len())
    else {
        data.iter_mut().for_each(|v| *v = 0);
        return;
    };
    let (b, a) = (before.heat_rate(), st.stats.heat_rate());
    let diff: Vec<f32> = a.iter().zip(&b).map(|(x, y)| x - y).collect();
    let mut abs: Vec<f32> = diff.iter().map(|d| d.abs()).filter(|d| *d > 0.0).collect();
    abs.sort_by(|x, y| x.total_cmp(y));
    let scale = abs
        .get(abs.len() * 98 / 100)
        .copied()
        .unwrap_or(1.0)
        .max(1e-6);
    for y in 0..g.h {
        for x in 0..g.w {
            let i = g.idx(x, y);
            let o = ((g.h - 1 - y) * g.w + x) * 4;
            let d = (diff[i] / scale).clamp(-1.0, 1.0);
            let px = if st.closed.get(i).copied().unwrap_or(false) {
                [40, 40, 40, 200]
            } else if d.abs() < 0.02 {
                [0, 0, 0, 0]
            } else {
                let k = d.abs().sqrt();
                let al = (80.0 + 170.0 * k) as u8;
                if d > 0.0 {
                    [
                        230,
                        (200.0 * (1.0 - k)) as u8,
                        (200.0 * (1.0 - k)) as u8,
                        al,
                    ]
                } else {
                    [
                        (200.0 * (1.0 - k)) as u8,
                        (120.0 + 80.0 * (1.0 - k)) as u8,
                        240,
                        al,
                    ]
                }
            };
            data[o..o + 4].copy_from_slice(&px);
        }
    }
}

#[derive(Component)]
pub struct ScenarioMarker;

/// 通行止め・発生源・行き先の目印を作り直す。
pub fn sync_markers(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut scen: ResMut<ScenarioState>,
    old: Query<Entity, With<ScenarioMarker>>,
) {
    if !scen.markers_dirty {
        return;
    }
    scen.markers_dirty = false;
    for e in &old {
        commands.entity(e).despawn();
    }
    let mat = |m: &mut Assets<StandardMaterial>, c: Color| {
        m.add(StandardMaterial {
            base_color: c,
            alpha_mode: AlphaMode::Blend,
            unlit: true,
            ..default()
        })
    };
    let on = mat(&mut materials, Color::srgba(1.0, 0.55, 0.0, 0.75));
    let off = mat(&mut materials, Color::srgba(0.6, 0.6, 0.6, 0.35));
    let sc = &scen.scenario;
    for c in &sc.closures {
        let mut mb = MeshBuilder::default();
        for p in &c.polys {
            mb.add_extrusion(p, 1.1);
        }
        if mb.is_empty() {
            continue;
        }
        let active = sc.closures_on && c.enabled;
        commands.spawn((
            ScenarioMarker,
            Mesh3d(meshes.add(mb.build())),
            MeshMaterial3d(if active { on.clone() } else { off.clone() }),
        ));
    }
    let src = mat(&mut materials, Color::srgba(0.1, 0.85, 0.3, 0.55));
    let src_off = mat(&mut materials, Color::srgba(0.5, 0.6, 0.5, 0.3));
    for s in &sc.sources {
        commands.spawn((
            ScenarioMarker,
            Mesh3d(meshes.add(Cylinder::new(s.radius, 0.25))),
            MeshMaterial3d(if s.enabled {
                src.clone()
            } else {
                src_off.clone()
            }),
            Transform::from_translation(to_world(s.pos, 0.15)),
        ));
    }
    let dst = mat(&mut materials, Color::srgba(0.75, 0.2, 0.9, 0.55));
    for d in &sc.destinations {
        commands.spawn((
            ScenarioMarker,
            Mesh3d(meshes.add(Cylinder::new(d.radius, 0.25))),
            MeshMaterial3d(dst.clone()),
            Transform::from_translation(to_world(d.pos, 0.15)),
        ));
    }
}

/// 配置モードで地図をクリック（通行止めはドラッグ）して置く。
pub fn place_on_map(
    buttons: Res<ButtonInput<MouseButton>>,
    windows: Query<&Window, With<PrimaryWindow>>,
    cams: Query<(&Camera, &GlobalTransform)>,
    mut contexts: EguiContexts,
    mut scen: ResMut<ScenarioState>,
    state: Res<SimState>,
) {
    if scen.place == PlaceMode::None {
        scen.drag_start = None;
        return;
    }
    let over_ui = contexts
        .try_ctx_mut()
        .is_some_and(|c| c.wants_pointer_input() || c.is_pointer_over_area());
    let Some(p) = cursor_ground(&windows, &cams) else {
        return;
    };
    if buttons.just_pressed(MouseButton::Left) && !over_ui {
        scen.drag_start = Some(p);
    }
    if !buttons.just_released(MouseButton::Left) {
        return;
    }
    let Some(a) = scen.drag_start.take() else {
        return;
    };
    let t = state.sim.time;
    let scen = &mut *scen;
    let sc = &mut scen.scenario;
    match scen.place {
        PlaceMode::Closure => {
            let n = sc.closures.len() + 1;
            let name = format!("通行止め{n}");
            let c = if a.distance(p) < 1.0 {
                Closure::square(name, scen.place_kind, p, scen.place_width)
            } else {
                let polys = match scen.place_shape {
                    ClosureShape::Line => line_polygons(&[a, p], scen.place_width),
                    ClosureShape::Rect => vec![walkmap::closure::rect_polygon(a, p)],
                };
                Closure {
                    name,
                    kind: scen.place_kind,
                    polys,
                    enabled: true,
                }
            };
            sc.closures.push(c);
        }
        PlaceMode::Source => {
            let n = sc.sources.len() + 1;
            let t0 = (t / 60.0).floor() * 60.0;
            sc.sources.push(Source {
                name: format!("発生源{n}"),
                pos: p,
                radius: 4.0,
                rate: vec![
                    (t0, 0.0),
                    (t0 + 60.0, 60.0),
                    (t0 + 600.0, 60.0),
                    (t0 + 660.0, 0.0),
                ],
                to: vec![],
                speed: None,
                enabled: true,
            });
        }
        PlaceMode::Destination => {
            let n = sc.destinations.len() + 1;
            sc.destinations.push(Destination {
                name: format!("行き先{n}"),
                pos: p,
                radius: 5.0,
            });
        }
        PlaceMode::None => {}
    }
    scen.markers_dirty = true;
    scen.pending = true;
}

fn cursor_ground(
    windows: &Query<&Window, With<PrimaryWindow>>,
    cams: &Query<(&Camera, &GlobalTransform)>,
) -> Option<Vec2> {
    let cursor = windows.single().ok()?.cursor_position()?;
    let (cam, gt) = cams.iter().next()?;
    let ray = cam.viewport_to_world(gt, cursor).ok()?;
    let dy = ray.direction.y;
    if dy.abs() < 1e-5 {
        return None;
    }
    let t = -ray.origin.y / dy;
    if t <= 0.0 {
        return None;
    }
    let w = ray.origin + *ray.direction * t;
    Some(Vec2::new(w.x, -w.z))
}

/// シナリオ・通行止め・結果のパネル（画面左）。
pub fn scenario_panel(
    mut contexts: EguiContexts,
    mut scen: ResMut<ScenarioState>,
    mut settings: ResMut<Settings>,
    state: Res<SimState>,
    mut capture: ResMut<UiCapture>,
    buttons: Res<ButtonInput<MouseButton>>,
    mut overlay: Query<&mut Visibility, With<crate::HeatOverlay>>,
) {
    let Some(ctx) = contexts.try_ctx_mut() else {
        return;
    };
    let ctx = ctx.clone();
    let scen = &mut *scen;
    let mut apply = false;
    egui::Window::new("シナリオ・通行止め・結果")
        .default_pos([12.0, 70.0])
        .default_width(320.0)
        .default_open(true)
        .default_height((ctx.screen_rect().height() - 90.0).max(200.0))
        .max_height((ctx.screen_rect().height() - 90.0).max(200.0))
        .vscroll(true)
        .show(&ctx, |ui| {
            // ---- ファイル
            ui.horizontal(|ui| {
                ui.label("ファイル");
                ui.add(egui::TextEdit::singleline(&mut scen.path).desired_width(150.0));
                if ui.button("読込").clicked() {
                    match ScenarioState::load(std::path::Path::new(&scen.path), &scen.base_grid) {
                        Ok(sc) => {
                            if let Some(n) = sc.background_agents {
                                settings.agents = n;
                            }
                            if let Some(n) = sc.vehicles {
                                settings.vehicles = n;
                            }
                            if let Some(z) = sc.zone_size {
                                scen.zone_size = z;
                            }
                            scen.message = format!(
                                "読み込みました: 発生源 {} / 行き先 {} / 通行止め {}",
                                sc.sources.len(),
                                sc.destinations.len(),
                                sc.closures.len()
                            );
                            scen.scenario = sc;
                            scen.markers_dirty = true;
                            apply = true;
                        }
                        Err(e) => scen.message = format!("読込エラー: {e}"),
                    }
                }
                if ui.button("保存").clicked() {
                    let mut sc = scen.scenario.clone();
                    sc.background_agents = Some(settings.agents);
                    sc.vehicles = Some(settings.vehicles);
                    sc.zone_size = Some(scen.zone_size);
                    let text = sc.to_file(&scen.base_grid.projection).to_toml();
                    scen.message = match std::fs::write(&scen.path, text) {
                        Ok(()) => format!("保存しました: {}", scen.path),
                        Err(e) => format!("保存エラー: {e}"),
                    };
                }
            });
            ui.horizontal(|ui| {
                ui.label("名前");
                ui.text_edit_singleline(&mut scen.scenario.name);
            });
            ui.horizontal(|ui| {
                ui.label("開始時刻");
                let mut s = scen
                    .scenario
                    .start_clock
                    .map(|c| format_clock(c)[..5].to_string())
                    .unwrap_or_default();
                if ui
                    .add(
                        egui::TextEdit::singleline(&mut s)
                            .desired_width(60.0)
                            .hint_text("HH:MM"),
                    )
                    .changed()
                {
                    scen.scenario.start_clock = parse_clock(&s).ok();
                }
                ui.label(format!(
                    "現在 {}",
                    scen.scenario.clock_label(state.sim.time)
                ));
            });

            // ---- 発生源
            let t = state.sim.time;
            ui.separator();
            ui.horizontal(|ui| {
                ui.strong("発生源");
                ui.label(format!(
                    "いま {:.0} 人/分・累計 {} 人{}",
                    scen.scenario.total_rate(t),
                    scen.spawner.as_ref().map_or(0, |s| s.total_spawned()),
                    scen.spawner
                        .as_ref()
                        .map(|s| s.backlog())
                        .filter(|&b| b > 0)
                        .map_or(String::new(), |b| format!("（出口待ち {b}）"))
                ));
            });
            ui.horizontal(|ui| {
                ui.label("人数の倍率");
                ui.add(egui::Slider::new(&mut scen.scenario.rate_scale, 0.0..=5.0).suffix("x"));
            });
            let goal_names = scen.goal_names.clone();
            let start = scen.scenario.start_clock;
            let spawned = scen
                .spawner
                .as_ref()
                .map(|s| s.spawned())
                .unwrap_or_default();
            let mut remove = None;
            let mut changed = false;
            for (i, s) in scen.scenario.sources.iter_mut().enumerate() {
                let title = format!(
                    "{} {} ({:.0} 人/分, 累計 {})",
                    if s.enabled { "●" } else { "○" },
                    s.name,
                    s.rate_at(t),
                    spawned.get(i).copied().unwrap_or(0)
                );
                egui::CollapsingHeader::new(title)
                    .id_salt(("src", i))
                    .show(ui, |ui| {
                        ui.horizontal(|ui| {
                            changed |= ui.checkbox(&mut s.enabled, "有効").changed();
                            ui.text_edit_singleline(&mut s.name);
                        });
                        ui.horizontal(|ui| {
                            ui.label("半径");
                            changed |= ui
                                .add(
                                    egui::DragValue::new(&mut s.radius)
                                        .range(0.5..=50.0)
                                        .suffix(" m"),
                                )
                                .changed();
                            ui.label("歩行速度");
                            let mut v = s.speed.unwrap_or(1.34);
                            if ui
                                .add(
                                    egui::DragValue::new(&mut v)
                                        .range(0.3..=2.5)
                                        .speed(0.01)
                                        .suffix(" m/s"),
                                )
                                .changed()
                            {
                                s.speed = Some(v);
                                changed = true;
                            }
                        });
                        ui.label("時刻ごとの発生人数（間は直線でつなぐ）");
                        let mut del = None;
                        egui::Grid::new(("rate", i)).num_columns(4).show(ui, |ui| {
                            for (k, (tt, r)) in s.rate.iter_mut().enumerate() {
                                changed |= ui
                                    .add(
                                        egui::DragValue::new(tt)
                                            .range(0.0..=86400.0)
                                            .speed(5.0)
                                            .suffix(" s"),
                                    )
                                    .changed();
                                ui.label(match start {
                                    Some(c) => format_clock(c + *tt),
                                    None => String::new(),
                                });
                                changed |= ui
                                    .add(
                                        egui::DragValue::new(r)
                                            .range(0.0..=2000.0)
                                            .suffix(" 人/分"),
                                    )
                                    .changed();
                                if ui.small_button("×").clicked() {
                                    del = Some(k);
                                }
                                ui.end_row();
                            }
                        });
                        if let Some(k) = del {
                            s.rate.remove(k);
                            changed = true;
                        }
                        if ui.small_button("＋ 点を追加").clicked() {
                            let last = s.rate.last().copied().unwrap_or((t, 30.0));
                            s.rate.push((last.0 + 300.0, last.1));
                            changed = true;
                        }
                        s.rate.sort_by(|a, b| a.0.total_cmp(&b.0));
                        ui.label("行き先（未選択なら全部から等確率）");
                        ui.horizontal_wrapped(|ui| {
                            for g in &goal_names {
                                let pos = s.to.iter().position(|(n, _)| n == g);
                                let mut on = pos.is_some();
                                if ui.checkbox(&mut on, g).changed() {
                                    match pos {
                                        Some(p) => {
                                            s.to.remove(p);
                                        }
                                        None => s.to.push((g.clone(), 1.0)),
                                    }
                                    changed = true;
                                }
                            }
                        });
                        for (n, w) in s.to.iter_mut() {
                            ui.horizontal(|ui| {
                                ui.label(format!("  {n} の重み"));
                                changed |= ui
                                    .add(egui::DragValue::new(w).range(0.0..=100.0).speed(0.1))
                                    .changed();
                            });
                        }
                        if ui.button("この発生源を削除").clicked() {
                            remove = Some(i);
                        }
                    });
            }
            if let Some(i) = remove {
                scen.scenario.sources.remove(i);
                changed = true;
            }

            // ---- 行き先
            ui.separator();
            ui.strong("行き先");
            ui.small("範囲の縁の 東・西・南・北 は常に使えます");
            let mut remove = None;
            for (i, d) in scen.scenario.destinations.iter_mut().enumerate() {
                ui.horizontal(|ui| {
                    changed |= ui
                        .add(egui::TextEdit::singleline(&mut d.name).desired_width(120.0))
                        .changed();
                    ui.label("半径");
                    changed |= ui
                        .add(
                            egui::DragValue::new(&mut d.radius)
                                .range(1.0..=50.0)
                                .suffix(" m"),
                        )
                        .changed();
                    if ui.small_button("×").clicked() {
                        remove = Some(i);
                    }
                });
            }
            if let Some(i) = remove {
                scen.scenario.destinations.remove(i);
                changed = true;
            }

            // ---- 通行止め
            ui.separator();
            ui.horizontal(|ui| {
                ui.strong("通行止め・工事");
                changed |= ui
                    .checkbox(&mut scen.scenario.closures_on, "有効にする")
                    .changed();
            });
            let mut remove = None;
            for (i, c) in scen.scenario.closures.iter_mut().enumerate() {
                ui.horizontal(|ui| {
                    changed |= ui.checkbox(&mut c.enabled, "").changed();
                    ui.add(egui::TextEdit::singleline(&mut c.name).desired_width(110.0));
                    egui::ComboBox::from_id_salt(("kind", i))
                        .selected_text(c.kind.label())
                        .show_ui(ui, |ui| {
                            for k in ClosureKind::ALL {
                                changed |= ui.selectable_value(&mut c.kind, k, k.label()).changed();
                            }
                        });
                    if ui.small_button("×").clicked() {
                        remove = Some(i);
                    }
                });
            }
            if let Some(i) = remove {
                scen.scenario.closures.remove(i);
                changed = true;
            }

            // ---- 地図に置く
            ui.separator();
            ui.strong("地図に置く");
            ui.horizontal(|ui| {
                ui.selectable_value(&mut scen.place, PlaceMode::None, "なし");
                ui.selectable_value(&mut scen.place, PlaceMode::Closure, "通行止め");
                ui.selectable_value(&mut scen.place, PlaceMode::Source, "発生源");
                ui.selectable_value(&mut scen.place, PlaceMode::Destination, "行き先");
            });
            if scen.place == PlaceMode::Closure {
                ui.horizontal(|ui| {
                    egui::ComboBox::from_id_salt("place_kind")
                        .selected_text(scen.place_kind.label())
                        .show_ui(ui, |ui| {
                            for k in ClosureKind::ALL {
                                ui.selectable_value(&mut scen.place_kind, k, k.label());
                            }
                        });
                    ui.selectable_value(&mut scen.place_shape, ClosureShape::Line, "線");
                    ui.selectable_value(&mut scen.place_shape, ClosureShape::Rect, "長方形");
                    ui.add(
                        egui::DragValue::new(&mut scen.place_width)
                            .range(1.0..=60.0)
                            .suffix(" m"),
                    );
                });
                ui.small("道路を横切るように左ドラッグ（クリックだけなら正方形）");
            } else if scen.place != PlaceMode::None {
                ui.small("地図を左クリックして置く");
            }
            if changed {
                scen.markers_dirty = true;
                scen.pending = true;
            }
            ui.add_space(4.0);
            let label = if scen.pending {
                "適用して最初から（未反映の変更あり）"
            } else {
                "適用して最初から"
            };
            if ui
                .add_sized([ui.available_width(), 26.0], egui::Button::new(label))
                .clicked()
            {
                apply = true;
            }

            // ---- 比較
            ui.separator();
            ui.strong("Before / After 比較");
            ui.horizontal(|ui| {
                if ui.button("いまの結果を Before に").clicked() {
                    scen.before = Some(scen.stats.clone());
                    scen.message =
                        format!("Before を保存しました（{:.0} 秒分）", scen.stats.elapsed);
                }
                if scen.before.is_some() && ui.button("Before を消す").clicked() {
                    scen.before = None;
                    scen.diff_view = false;
                }
            });
            if let Some(b) = &scen.before {
                ui.small(format!(
                    "Before: {:.0} 秒分 / After: {:.0} 秒分（どちらも 1 秒あたりに直して比べます）",
                    b.elapsed, scen.stats.elapsed
                ));
                if ui
                    .checkbox(&mut scen.diff_view, "差分ヒートマップ（赤=増, 青=減）")
                    .changed()
                    && scen.diff_view
                {
                    for mut v in &mut overlay {
                        *v = Visibility::Visible;
                    }
                }
            } else {
                ui.small("通行止めを無効にして走らせ「Before に」→ 有効にして「適用」→ 差分を表示");
            }

            // ---- エクスポート
            ui.separator();
            ui.strong("結果のエクスポート");
            ui.horizontal(|ui| {
                ui.label("集計区画");
                if ui
                    .add(egui::Slider::new(&mut scen.zone_size, 5.0..=100.0).suffix(" m"))
                    .changed()
                {
                    scen.pending = true;
                }
            });
            ui.horizontal(|ui| {
                ui.label("出力先");
                ui.add(egui::TextEdit::singleline(&mut scen.export_dir).desired_width(170.0));
            });
            if ui.button("CSV と画像を書き出す").clicked() {
                scen.message = export(scen, &state);
            }
            if !scen.message.is_empty() {
                ui.separator();
                ui.small(&scen.message);
            }
        });
    if apply {
        settings.apply_requested = true;
    }
    capture.0 |= ctx.wants_pointer_input()
        || ctx.is_pointer_over_area()
        || (scen.place != PlaceMode::None && buttons.pressed(MouseButton::Left));
}

fn export(scen: &ScenarioState, state: &SimState) -> String {
    let dir = std::path::PathBuf::from(&scen.export_dir);
    let sc = &scen.scenario;
    let res = scen
        .stats
        .export_dir(
            &state.sim.grid,
            &dir,
            scen.before.as_ref(),
            &scen.closed,
            |t| sc.clock_label(t),
        )
        .and_then(|mut files| {
            if let Some(b) = &scen.before {
                b.export_dir(&state.sim.grid, &dir.join("before"), None, &[], |t| {
                    sc.clock_label(t)
                })?;
                files.push("before/".into());
            }
            std::fs::write(
                dir.join("scenario.toml"),
                sc.to_file(&scen.base_grid.projection).to_toml(),
            )?;
            files.push("scenario.toml".into());
            Ok(files)
        });
    match res {
        Ok(files) => format!("{} に書き出しました: {}", dir.display(), files.join(", ")),
        Err(e) => format!("書き出しエラー: {e}"),
    }
}
