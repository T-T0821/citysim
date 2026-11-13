//! plateau-crowd-viewer: PLATEAU の建物を 3D 表示し、その上で歩行者と車をリアルタイムに動かす。
//!
//! 操作: 左ドラッグ=回転 / 右ドラッグ=平行移動 / ホイール=ズーム
//!       Space=一時停止 / ↑↓=再生速度 / H=ヒートマップ表示切替

mod camera;
mod evac;
mod person;
mod place;
mod geometry;
mod scenario;
mod signals;
mod ui;
#[cfg(target_arch = "wasm32")]
mod web;

use anyhow::{bail, Context, Result};
use bevy::prelude::*;
use bevy::render::view::screenshot::{save_to_disk, Screenshot};
use crowd::{Rng, Simulation};
use geometry::{dashed_line, to_world, walkable_surface, MeshBuilder};
use std::path::PathBuf;
use traffic::{RoadGraph, Traffic};
use ui::Settings;
use walkmap::WalkGrid;

const DT: f32 = 0.05;
/// 通常表示で 1 フレームに計算へ使ってよい時間 [s]
const FRAME_SIM_BUDGET: f32 = 0.025;

struct Args {
    gml: Vec<PathBuf>,
    obj: Vec<PathBuf>,
    obj_origin: Option<(f64, f64)>,
    crop: Option<(f64, f64, f64)>,
    agents: usize,
    vehicles: usize,
    cell: f32,
    seed: u64,
    respawn: bool,
    screenshot: Option<(PathBuf, f32)>,
    record: Option<(PathBuf, f32)>,
    fps: f32,
    speed: f32,
    orbit: f32,
    warmup: f32,
    ui_font: Option<PathBuf>,
    scenario: Option<PathBuf>,
    agents_given: bool,
    vehicles_given: bool,
    /// 信号機（None = 信号なし）
    signals: Option<traffic::SignalConfig>,
    place: Option<String>,
    radius: f64,
    data_dir: PathBuf,
    shelters: Vec<(String, f64, f64)>,
    evacuate_at: Option<f32>,
}

fn parse_args() -> Result<Args> {
    let mut a = Args {
        gml: vec![],
        obj: vec![],
        obj_origin: None,
        crop: None,
        agents: 1000,
        vehicles: 40,
        record: None,
        fps: 30.0,
        speed: 2.0,
        orbit: 0.0,
        warmup: 0.0,
        ui_font: None,
        scenario: None,
        agents_given: false,
        vehicles_given: false,
        signals: Some(traffic::SignalConfig::default()),
        place: None,
        radius: 220.0,
        data_dir: PathBuf::from(place::DEFAULT_DATA_DIR),
        shelters: vec![],
        evacuate_at: None,
        cell: 0.5,
        seed: 1,
        respawn: true,
        screenshot: None,
    };
    #[cfg(not(target_arch = "wasm32"))]
    let mut it = std::env::args().skip(1);
    // Web 版は URL のクエリ（?agents=800&signals=scramble など）から
    #[cfg(target_arch = "wasm32")]
    let mut it = web::args().into_iter();
    while let Some(k) = it.next() {
        let mut v = || it.next().with_context(|| format!("{k} に値がありません"));
        match k.as_str() {
            "--gml" => {
                let p = PathBuf::from(v()?);
                if p.is_dir() {
                    for e in std::fs::read_dir(&p)? {
                        let e = e?.path();
                        if e.extension().is_some_and(|x| x == "gml") {
                            a.gml.push(e);
                        }
                    }
                } else {
                    a.gml.push(p);
                }
            }
            // PLATEAU SDK for Unity などが書き出した OBJ（bldg_* / tran_* グループ付き）
            "--obj" => {
                let p = PathBuf::from(v()?);
                if p.is_dir() {
                    for e in std::fs::read_dir(&p)? {
                        let e = e?.path();
                        if e.extension().is_some_and(|x| x == "obj") {
                            a.obj.push(e);
                        }
                    }
                } else {
                    a.obj.push(p);
                }
            }
            // OBJ の原点の緯度,経度
            "--obj-origin" => a.obj_origin = Some(parse_pair(&v()?)?),
            // 切り出し範囲: 中心の緯度,経度,半径[m]
            "--crop" => {
                let s = v()?;
                let x: Vec<f64> = s
                    .split(',')
                    .map(|t| t.trim().parse())
                    .collect::<std::result::Result<_, _>>()?;
                if x.len() != 3 {
                    bail!("--crop は 緯度,経度,半径m");
                }
                a.crop = Some((x[0], x[1], x[2]));
            }
            "--agents" => {
                a.agents = v()?.parse()?;
                a.agents_given = true;
            }
            "--vehicles" => {
                a.vehicles = v()?.parse()?;
                a.vehicles_given = true;
            }
            "--cell" => a.cell = v()?.parse()?,
            "--seed" => a.seed = v()?.parse()?,
            "--no-respawn" => a.respawn = false,
            // 指定秒数シミュレーションを進めた時点で PNG を保存して終了（CI・確認用）
            "--screenshot" => {
                let p = PathBuf::from(v()?);
                let t = v()?.parse()?;
                a.screenshot = Some((p, t));
            }
            // 動画用: 毎フレーム一定のシミュレーション時間だけ進めて連番 PNG を書き出す
            "--record" => {
                let p = PathBuf::from(v()?);
                let t = v()?.parse()?;
                a.record = Some((p, t));
            }
            "--fps" => a.fps = v()?.parse()?,
            "--speed" => a.speed = v()?.parse()?,
            // 録画中のカメラ旋回速度 [rad/s]
            "--orbit" => a.orbit = v()?.parse()?,
            // 録画前に進めておくシミュレーション時間 [s]
            "--warmup" => a.warmup = v()?.parse()?,
            // 設定パネルの日本語フォント（省略時は OS の定番フォントを探す）
            "--ui-font" => a.ui_font = Some(v()?.into()),
            // イベント・混雑シナリオ／通行止めの設定ファイル（TOML, scenarios/ を参照）
            "--scenario" => a.scenario = Some(v()?.into()),
            // 信号機: auto（既定）| concurrent | exclusive | scramble | off
            "--signals" => {
                let m = v()?;
                a.signals = if m == "off" {
                    None
                } else {
                    let mode = traffic::PedMode::parse(&m).with_context(|| {
                        format!("--signals は auto|concurrent|exclusive|scramble|off: {m}")
                    })?;
                    Some(traffic::SignalConfig {
                        mode,
                        ..a.signals.unwrap_or_default()
                    })
                };
            }
            "--cycle" => {
                let x = v()?.parse()?;
                if let Some(c) = a.signals.as_mut() {
                    c.cycle = x;
                }
            }
            // 場所（住所 or 緯度,経度）。--gml が無ければ PLATEAU の CityGML を自動で取得する
            "--place" => a.place = Some(v()?),
            "--radius" => a.radius = v()?.parse()?,
            "--data-dir" => a.data_dir = v()?.into(),
            // 避難所: 緯度,経度[,名前]（複数可）
            "--shelter" => a.shelters.push(evac::parse_shelter(&v()?)?),
            // この時刻 [s] に避難を始める
            "--evacuate-at" => a.evacuate_at = Some(v()?.parse()?),
            "-h" | "--help" => {
                println!(
                    "usage: plateau-crowd-viewer (--gml <file|dir> | --obj <file|dir> --obj-origin 緯度,経度)... [--crop 緯度,経度,半径m] [--agents N] [--vehicles N] [--cell M] [--seed N]\n\
                     \x20                          [--no-respawn] [--screenshot out.png <sim秒>]\n\
                     \x20                          [--record <dir> <動画の秒数>] [--fps 30] [--speed 2] [--orbit rad/s] [--warmup 秒] [--ui-font font.ttf]\n\
                     \x20                          [--scenario scenario.toml]\n\
                     \x20                          [--signals auto|concurrent|exclusive|scramble|off] [--cycle S]\n\
                     \x20                          [--place <住所|緯度,経度>] [--radius m] [--data-dir data/plateau] [--shelter 緯度,経度[,名前]]... [--evacuate-at 秒]"
                );
                std::process::exit(0);
            }
            _ => bail!("不明な引数: {k}"),
        }
    }
    #[cfg(target_arch = "wasm32")]
    {
        if a.place.is_some() {
            bail!("Web 版では場所を指定できません（同梱の銀座 4 丁目交差点のみ）");
        }
        // 入力は同梱の銀座 4 丁目交差点（load_model で読む）
        let (lat, lon) = web::CENTER;
        a.crop = Some((lat, lon, a.radius.clamp(50.0, web::MAX_RADIUS)));
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        if let Some(place) = a.place.clone() {
            let p = if a.gml.is_empty() && a.obj.is_empty() {
                // 入力が無ければ PLATEAU から取得（--data-dir に該当メッシュがあればそれを使う）
                let r = plateau_fetch::prepare(&place, a.radius, &a.data_dir, &mut |m| info!("{m}"))?;
                a.gml.extend(r.files);
                r.place
            } else {
                plateau_fetch::resolve_place(&place)?
            };
            a.crop = Some((p.lat, p.lon, a.radius));
        }
        if a.gml.is_empty() && a.obj.is_empty() {
            bail!("--gml か --obj か --place（例: --place \"東京都中央区銀座4丁目\" または --place 35.67165,139.765）を指定してください (--help)");
        }
        if !a.obj.is_empty() && a.obj_origin.is_none() {
            bail!("--obj には --obj-origin 緯度,経度 が必要です");
        }
    }
    Ok(a)
}

fn parse_pair(s: &str) -> Result<(f64, f64)> {
    let x: Vec<f64> = s
        .split(',')
        .map(|t| t.trim().parse())
        .collect::<std::result::Result<_, _>>()?;
    if x.len() != 2 {
        bail!("緯度,経度 の形式で指定してください: {s}");
    }
    Ok((x[0], x[1]))
}

/// 入力ファイルを読み、必要なら範囲を切り出す。戻り値はモデルとグリッド化する範囲。
fn load_model(args: &Args) -> Result<(plateau_gml::CityModel, Option<(f64, f64, f64, f64)>)> {
    let mut model = plateau_gml::CityModel::default();
    #[cfg(target_arch = "wasm32")]
    for s in web::GML {
        model.merge(plateau_gml::parse_str(s)?);
    }
    for p in &args.gml {
        model.merge(plateau_gml::parse_file(p)?);
    }
    if let Some((lat0, lon0)) = args.obj_origin {
        let frame = plateau_gml::obj::ObjFrame::unity_export(lat0, lon0);
        for p in &args.obj {
            model.merge(plateau_gml::obj::parse_file(p, frame)?);
        }
    }
    let bounds = args.crop.map(|(lat, lon, r)| {
        let dlat = r / 111_320.0;
        let dlon = r / (111_320.0 * lat.to_radians().cos());
        (lat - dlat, lon - dlon, lat + dlat, lon + dlon)
    });
    if let Some(b) = bounds {
        model.crop(b);
    }
    Ok((model, bounds))
}

#[derive(Resource)]
pub struct SimState {
    pub sim: Simulation,
    pub traffic: Option<Traffic>,
    /// 作り直し用に保持する道路網
    road_graph: Option<RoadGraph>,
    rng: Rng,
    /// 目的地ごとの到着セル（再出現位置に使う）
    goal_cells: Vec<Vec<(usize, usize)>>,
    pub respawn: bool,
    pub speed: f32,
    pub paused: bool,
    acc: f32,
    heat: Vec<u32>,
    pub heat_dirty: bool,
    /// 避難所（名前と局所座標）
    pub shelters: Vec<(String, Vec2)>,
    /// 進行中の避難
    pub evac: Option<crowd::evacuation::Evacuation>,
    /// この時刻に避難を自動で始める（--evacuate-at）
    pub evac_at: Option<f32>,
}

impl SimState {
    /// 避難所に向かう目的地の範囲（避難中のみ）
    pub fn evac_goals(&self) -> Option<std::ops::Range<usize>> {
        self.evac
            .as_ref()
            .map(|e| e.first_goal..e.first_goal + e.shelters.len())
    }
}

/// グリッドと道路網から歩行者・車のシミュレーションを組み立てる。
/// シナリオの通行止めを反映し、発生源を準備する（`scen.spawner` などを更新）。
fn build_sim(
    road_graph: Option<&RoadGraph>,
    st: &Settings,
    scen: &mut scenario::ScenarioState,
) -> Result<(Simulation, Option<Traffic>, Vec<Vec<(usize, usize)>>)> {
    let grid = scen.closed_grid();
    let mut traffic = match road_graph {
        Some(net) if st.vehicles > 0 || st.signals => {
            let params = traffic::Params {
                speed_limit: st.speed_limit_kmh / 3.6,
                ..Default::default()
            };
            let net = net.retain_edges(|e| !scen.scenario.blocks_road(&e.pts));
            Some(Traffic::new_with(net, st.vehicles, st.seed, params))
        }
        _ => None,
    };
    let (mut sim, names) = crowd::scenario::through_traffic(grid, st.agents, st.seed)
        .context("範囲の縁に歩行可能領域がありません")?;
    let k = st.walk_speed / 1.34;
    for a in sim.agents.iter_mut() {
        a.desired_speed = (a.desired_speed * k).clamp(0.3, 3.0);
    }
    // 信号機: 横断歩道を通るよう歩行者の経路を引き直し、赤の横断歩道は閉じる
    if let (Some(t), true) = (traffic.as_mut(), st.signals) {
        let s = traffic::Signals::build(&t.graph, &sim.grid, st.signal.clone());
        info!("信号: {} か所", s.intersections.len());
        s.apply_walk_costs(&mut sim.grid);
        sim.recompute_flows();
        sim.closed = vec![false; sim.grid.w * sim.grid.h];
        s.write_closed(t.time, &mut sim.closed);
        t.set_signals(Some(s));
    }
    // 再投入に使うのは範囲の縁（シナリオの行き先を足す前）
    let goal_cells = sim
        .goals
        .iter()
        .map(|ff| {
            sim.grid
                .walkable_cells()
                .filter(|&(x, y)| ff.dist[sim.grid.idx(x, y)] == 0.0)
                .collect()
        })
        .collect();
    let mut names: Vec<String> = names.iter().map(|s| s.to_string()).collect();
    scen.spawner = Some(
        scen.scenario
            .install(&mut sim, &mut names, st.seed)
            .map_err(anyhow::Error::msg)?,
    );
    scen.goal_names = names;
    scen.stats = crowd::report::ZoneStats::new(&sim.grid, scen.zone_size);
    scen.pending = false;
    scen.markers_dirty = true;
    Ok((sim, traffic, goal_cells))
}

#[derive(Resource)]
struct Visuals {
    heat_image: Handle<Image>,
    car_mats: Vec<Handle<StandardMaterial>>,
    glass: Handle<StandardMaterial>,
    bus_mat: Handle<StandardMaterial>,
    bike_mat: Handle<StandardMaterial>,
}

#[derive(Resource)]
struct ScreenshotPlan {
    path: PathBuf,
    at: f32,
    taken_frame: Option<u32>,
}

#[derive(Component)]
struct AgentVis(usize);

#[derive(Component)]
pub struct HeatOverlay;

#[derive(Component)]
struct CarVis(usize);

#[derive(Resource)]
#[cfg_attr(target_arch = "wasm32", allow(dead_code))]
struct RecordPlan {
    dir: PathBuf,
    frames: u32,
    idx: u32,
    sim_per_frame: f32,
    orbit_per_frame: f32,
    done_at: Option<u32>,
    warmup: f32,
}

/// 録画開始前に描画を落ち着かせるフレーム数
const RECORD_WARMUP: u32 = 15;

#[derive(Component)]
struct Hud;

#[cfg(not(target_arch = "wasm32"))]
fn main() -> Result<()> {
    run()
}

#[cfg(target_arch = "wasm32")]
fn main() {
    if let Err(e) = run() {
        error!("{e:#}");
        web::show_error(&format!("{e:#}"));
    }
}

fn run() -> Result<()> {
    let args = parse_args()?;
    let (model, bounds) = load_model(&args)?;
    info!(
        "建物 {} 棟, 道路 {} 件",
        model.buildings().count(),
        model.roads().count()
    );
    let grid = WalkGrid::from_city_bounds(&model, args.cell, bounds)
        .context("座標を含む地物がありません")?;
    let road_graph = grid.has_roads.then(|| {
        let net = RoadGraph::from_grid(&grid, crowd::scenario::SIDEWALK_W);
        info!(
            "道路網: ノード {}, 区間 {}",
            net.nodes.len(),
            net.edges.len()
        );
        net
    });
    let mut settings = Settings::new(args.agents, args.vehicles, args.seed, args.respawn);
    let sc = match &args.scenario {
        Some(p) => {
            let sc = scenario::ScenarioState::load(p, &grid).map_err(anyhow::Error::msg)?;
            if let (Some(n), false) = (sc.background_agents, args.agents_given) {
                settings.agents = n;
            }
            if let (Some(n), false) = (sc.vehicles, args.vehicles_given) {
                settings.vehicles = n;
            }
            sc
        }
        None => crowd::event::Scenario::empty(),
    };
    settings.signals = args.signals.is_some();
    if let Some(c) = &args.signals {
        settings.signal = c.clone();
    }
    let mut scen = scenario::ScenarioState::new(grid, sc, args.scenario.as_deref());
    let (sim, traffic, goal_cells) = build_sim(road_graph.as_ref(), &settings, &mut scen)?;
    info!(
        "エージェント {} 人, 車 {} 台",
        sim.agents.len(),
        traffic.as_ref().map_or(0, |t| t.vehicles.len())
    );

    let heat = vec![0; sim.grid.w * sim.grid.h];
    let state = SimState {
        sim,
        traffic,
        road_graph,
        rng: Rng::new(args.seed ^ 0x9E37_79B9),
        goal_cells,
        respawn: args.respawn,
        speed: args.speed,
        paused: false,
        acc: 0.0,
        heat,
        heat_dirty: true,
        shelters: vec![],
        evac: None,
        evac_at: None,
    };
    let mut state = state;
    state.shelters = args
        .shelters
        .iter()
        .map(|(n, lat, lon)| (n.clone(), evac::geo_to_local(&state.sim.grid, *lat, *lon)))
        .collect();
    state.evac_at = (!state.shelters.is_empty()).then_some(args.evacuate_at).flatten();
    let place_ui = place::PlaceUi::new(&state.sim.grid, args.radius as f32, args.data_dir.clone());

    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: "PLATEAU crowd simulator".into(),
            resolution: (1280.0_f32, 800.0_f32).into(),
            // Web 版: index.html の <canvas id="bevy"> にページいっぱいで描く
            canvas: cfg!(target_arch = "wasm32").then(|| "#bevy".into()),
            fit_canvas_to_parent: true,
            ..default()
        }),
        ..default()
    }))
    .insert_resource(ClearColor(Color::srgb(0.62, 0.72, 0.82)))
    .insert_resource(AmbientLight {
        brightness: 400.0,
        ..default()
    })
    .insert_resource(state)
    .insert_resource(settings)
    .insert_resource(place_ui)
    .init_resource::<evac::EvacUi>()
    .insert_resource(scen)
    .insert_resource(UiFont(args.ui_font.clone()))
    .init_resource::<camera::UiCapture>()
    .add_plugins(bevy_egui::EguiPlugin {
        enable_multipass_for_primary_context: false,
    })
    .add_systems(Startup, setup)
    .add_systems(
        Update,
        (
            install_ui_font,
            evac::tools_window,
            ui::settings_panel,
            evac::pick_shelter,
            scenario::scenario_panel,
            scenario::place_on_map,
            apply_settings,
            camera::orbit_camera,
            controls,
            step_sim,
            spawn_new_agents,
            scenario::sync_markers,
            sync_agents,
            sync_cars,
            signals::sync,
            evac::sync_markers,
            update_heat,
            update_hud,
        )
            .chain(),
    );
    #[cfg(not(target_arch = "wasm32"))]
    if let Some((dir, secs)) = args.record {
        std::fs::create_dir_all(&dir)?;
        app.insert_resource(RecordPlan {
            dir,
            frames: (secs * args.fps).round() as u32,
            idx: 0,
            sim_per_frame: args.speed / args.fps,
            orbit_per_frame: args.orbit / args.fps,
            done_at: None,
            warmup: args.warmup,
        })
        .add_systems(Update, record_frames.after(update_hud));
    }
    if let Some((path, at)) = args.screenshot {
        app.insert_resource(ScreenshotPlan {
            path,
            at,
            taken_frame: None,
        })
        .add_systems(Update, take_screenshot.after(step_sim));
    }
    app.run();
    Ok(())
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut images: ResMut<Assets<Image>>,
    state: Res<SimState>,
) {
    let g = &state.sim.grid;
    let size = Vec2::new(g.w as f32, g.h as f32) * g.cell;
    let center = g.origin + size * 0.5;

    // 地面
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(size.x * 3.0, size.y * 3.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.30, 0.33, 0.30),
            perceptual_roughness: 1.0,
            ..default()
        })),
        Transform::from_translation(to_world(center, -0.05)),
    ));
    // 歩道（明るい灰）と車道（アスファルト）
    let is_road = |i: usize| !g.carriageway.is_empty() && g.carriageway[i];
    let surfaces = [
        (
            walkable_surface(g, 0.0, |i| !is_road(i)),
            Color::srgb(0.78, 0.78, 0.76),
        ),
        (
            walkable_surface(g, 0.0, is_road),
            Color::srgb(0.24, 0.25, 0.27),
        ),
    ];
    for (mb, color) in surfaces {
        if mb.is_empty() {
            continue;
        }
        commands.spawn((
            Mesh3d(meshes.add(mb.build())),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: color,
                perceptual_roughness: 0.95,
                ..default()
            })),
        ));
    }
    // センターライン（交差点内は描かない）
    if let Some(net) = &state.road_graph {
        let mut lines = MeshBuilder::default();
        for e in &net.edges {
            let trim = (net.nodes[e.a].radius + 1.0, net.nodes[e.b].radius + 1.0);
            dashed_line(&mut lines, &e.pts, 0.15, 3.0, 3.0, trim, 0.01);
        }
        if !lines.is_empty() {
            commands.spawn((
                Mesh3d(meshes.add(lines.build())),
                MeshMaterial3d(materials.add(StandardMaterial {
                    base_color: Color::srgb(0.95, 0.95, 0.92),
                    unlit: true,
                    ..default()
                })),
            ));
        }
    }
    // 車: 車体 + 客室の 2 つの箱
    let car_mats = {
        let car_colors = [
            Color::srgb(0.95, 0.95, 0.95),
            Color::srgb(0.12, 0.12, 0.14),
            Color::srgb(0.65, 0.67, 0.70),
            Color::srgb(0.70, 0.10, 0.12),
            Color::srgb(0.10, 0.25, 0.55),
            Color::srgb(0.85, 0.75, 0.20),
        ];
        car_colors
            .iter()
            .map(|&c| {
                materials.add(StandardMaterial {
                    base_color: c,
                    perceptual_roughness: 0.35,
                    metallic: 0.3,
                    ..default()
                })
            })
            .collect::<Vec<_>>()
    };
    let glass = materials.add(StandardMaterial {
        base_color: Color::srgb(0.10, 0.13, 0.18),
        perceptual_roughness: 0.1,
        ..default()
    });
    // 通行量ヒートマップ（歩行面のすぐ上に半透明で重ねる）
    let heat_image = images.add(heat_image_new(g.w as u32, g.h as u32));
    commands.spawn((
        HeatOverlay,
        Mesh3d(meshes.add(Plane3d::default().mesh().size(size.x, size.y))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color_texture: Some(heat_image.clone()),
            alpha_mode: AlphaMode::Blend,
            unlit: true,
            ..default()
        })),
        Transform::from_translation(to_world(center, 0.03)),
        if std::env::var_os("PLATEAU_SHOW_HEAT").is_some() {
            Visibility::Visible
        } else {
            Visibility::Hidden
        },
    ));

    // 建物（高さで色を少し変える）
    let mut by_band: [MeshBuilder; 3] = Default::default();
    for b in &g.buildings {
        let band = if b.height < 15.0 {
            0
        } else if b.height < 40.0 {
            1
        } else {
            2
        };
        if b.solid.is_empty() {
            for poly in &b.footprint {
                by_band[band].add_extrusion(poly, b.height.max(3.0));
            }
        } else {
            by_band[band].add_solid(&b.solid);
        }
    }
    let colors = [
        Color::srgb(0.93, 0.91, 0.86),
        Color::srgb(0.84, 0.86, 0.90),
        Color::srgb(0.70, 0.76, 0.86),
    ];
    for (mb, c) in by_band.into_iter().zip(colors) {
        if mb.is_empty() {
            continue;
        }
        commands.spawn((
            Mesh3d(meshes.add(mb.build())),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: c,
                perceptual_roughness: 0.8,
                ..default()
            })),
        ));
    }

    // 歩行者の色: 目的地ごと
    let palette = [
        Color::srgb(0.90, 0.25, 0.20),
        Color::srgb(0.15, 0.45, 0.90),
        Color::srgb(0.95, 0.70, 0.10),
        Color::srgb(0.20, 0.70, 0.35),
        // シナリオの行き先
        Color::srgb(0.75, 0.25, 0.85),
        Color::srgb(0.10, 0.80, 0.80),
        Color::srgb(0.95, 0.45, 0.70),
        Color::srgb(0.55, 0.35, 0.15),
    ];
    let mut people = person::PersonMeshes::new(
        &mut materials,
        palette.to_vec(),
        Color::srgb(0.10, 0.95, 0.55),
    );
    let visuals = Visuals {
        heat_image,
        car_mats,
        glass,
        bus_mat: materials.add(StandardMaterial {
            base_color: Color::srgb(0.15, 0.55, 0.35),
            perceptual_roughness: 0.4,
            ..default()
        }),
        bike_mat: materials.add(StandardMaterial {
            base_color: Color::srgb(0.85, 0.85, 0.20),
            ..default()
        }),
    };
    spawn_entities(&mut commands, &mut meshes, &visuals, &mut people, &state);
    commands.insert_resource(people);
    let signal_mats = signals::SignalMats::new(&mut meshes, &mut materials);
    signals::spawn(&mut commands, &mut meshes, &signal_mats, &state);
    commands.insert_resource(signal_mats);

    // 光源とカメラ
    commands.spawn((
        DirectionalLight {
            illuminance: 9000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(-0.6, 1.0, 0.4).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    let mut target = to_world(center, 0.0);
    let mut dist = size.max_element().max(60.0) * 1.5;
    let (mut yaw, mut pitch) = (-0.6, 0.9);
    // 確認用: PLATEAU_CAM="yaw,pitch,距離,x,y"（x,y は局所座標 [m]）でカメラ初期位置を指定
    if let Ok(c) = std::env::var("PLATEAU_CAM") {
        let v: Vec<f32> = c.split(',').filter_map(|x| x.trim().parse().ok()).collect();
        if let [y, p, d, x, z] = v[..] {
            (yaw, pitch, dist, target) = (y, p, d, to_world(Vec2::new(x, z), 0.0));
        }
    }
    commands.spawn((
        Camera3d::default(),
        camera::OrbitCamera {
            target,
            yaw,
            pitch,
            distance: dist,
        },
        Transform::default(),
    ));

    commands.spawn((
        Hud,
        Text::new(""),
        TextFont {
            font_size: 16.0,
            ..default()
        },
        TextColor(Color::WHITE),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(10.0),
            left: Val::Px(12.0),
            ..default()
        },
    ));

    commands.insert_resource(visuals);
}

/// 歩行者と車のエンティティを現在の `SimState` に合わせて生成する。
fn spawn_entities(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    visuals: &Visuals,
    people: &mut person::PersonMeshes,
    state: &SimState,
) {
    for (i, a) in state.sim.agents.iter().enumerate() {
        commands.spawn(agent_bundle(meshes, people, i, a));
    }
    let Some(t) = &state.traffic else { return };
    for (i, v) in t.vehicles.iter().enumerate() {
        match v.kind {
            traffic::VehicleKind::Bus => {
                spawn_bus(commands, meshes, visuals, i, v);
                continue;
            }
            traffic::VehicleKind::Bicycle => {
                spawn_bicycle(commands, meshes, visuals, i, v);
                continue;
            }
            _ => {}
        }
        let truck = v.length > 6.0;
        let (body_h, cabin) = if truck {
            (2.6, (1.8, 2.3, v.length * 0.5 - 0.9)) // 荷台の前に運転台
        } else {
            (0.75, (v.length * 0.5, 0.6, -0.2))
        };
        commands
            .spawn((
                CarVis(i),
                Transform::from_translation(to_world(v.pos, 0.0)),
                Visibility::Inherited,
            ))
            .with_children(|c| {
                let body_len = if truck { v.length - 2.0 } else { v.length };
                let body_x = if truck { -1.0 } else { 0.0 };
                c.spawn((
                    Mesh3d(meshes.add(Cuboid::new(body_len, body_h, v.width))),
                    MeshMaterial3d(
                        visuals.car_mats[v.style as usize % visuals.car_mats.len()].clone(),
                    ),
                    Transform::from_xyz(body_x, 0.3 + body_h * 0.5, 0.0),
                ));
                let (cl, ch, cx) = cabin;
                let cy = if truck {
                    0.3 + ch * 0.5
                } else {
                    0.3 + body_h + ch * 0.5
                };
                c.spawn((
                    Mesh3d(meshes.add(Cuboid::new(cl, ch, v.width * 0.92))),
                    MeshMaterial3d(if truck {
                        visuals.car_mats[0].clone()
                    } else {
                        visuals.glass.clone()
                    }),
                    Transform::from_xyz(cx, cy, 0.0),
                ));
            });
    }
}

/// 歩行者 1 人の表示。メッシュ（服装と姿勢）は `sync_agents` が毎フレーム選び直す。
fn agent_bundle(
    meshes: &mut Assets<Mesh>,
    people: &mut person::PersonMeshes,
    i: usize,
    a: &crowd::Agent,
) -> impl Bundle {
    let look = person::look_of(i);
    let gait = person::Gait::new(a.pos, a.vel, i);
    let key = person::Key {
        top: person::Top::Cloth(look.top),
        lower: look.lower,
        phase: 0,
    };
    (
        AgentVis(i),
        Mesh3d(people.get(meshes, key)),
        MeshMaterial3d(people.material.clone()),
        Transform::from_translation(to_world(a.pos, 0.0))
            .with_rotation(gait.rotation())
            .with_scale(look.scale),
        gait,
        if a.arrived {
            Visibility::Hidden
        } else {
            Visibility::Inherited
        },
    )
}

/// バス: 低床の長い車体に窓の帯。
fn spawn_bus(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    visuals: &Visuals,
    i: usize,
    v: &traffic::Vehicle,
) {
    commands
        .spawn((
            CarVis(i),
            Transform::from_translation(to_world(v.pos, 0.0)),
            Visibility::Inherited,
        ))
        .with_children(|c| {
            c.spawn((
                Mesh3d(meshes.add(Cuboid::new(v.length, 2.6, v.width))),
                MeshMaterial3d(visuals.bus_mat.clone()),
                Transform::from_xyz(0.0, 0.3 + 1.3, 0.0),
            ));
            c.spawn((
                Mesh3d(meshes.add(Cuboid::new(v.length - 0.6, 0.8, v.width + 0.04))),
                MeshMaterial3d(visuals.glass.clone()),
                Transform::from_xyz(0.1, 0.3 + 1.9, 0.0),
            ));
            // 前面ガラス
            c.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.06, 1.3, v.width * 0.9))),
                MeshMaterial3d(visuals.glass.clone()),
                Transform::from_xyz(v.length * 0.5, 0.3 + 1.6, 0.0),
            ));
        });
}

/// 自転車: 細い車体と乗っている人。
fn spawn_bicycle(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    visuals: &Visuals,
    i: usize,
    v: &traffic::Vehicle,
) {
    commands
        .spawn((
            CarVis(i),
            Transform::from_translation(to_world(v.pos, 0.0)),
            Visibility::Inherited,
        ))
        .with_children(|c| {
            c.spawn((
                Mesh3d(meshes.add(Cuboid::new(v.length, 0.7, 0.12))),
                MeshMaterial3d(visuals.car_mats[1].clone()),
                Transform::from_xyz(0.0, 0.35, 0.0),
            ));
            c.spawn((
                Mesh3d(meshes.add(Capsule3d::new(0.2, 0.7))),
                MeshMaterial3d(visuals.bike_mat.clone()),
                Transform::from_xyz(-0.1, 1.25, 0.0),
            ));
        });
}

#[derive(Resource)]
struct UiFont(Option<PathBuf>);

/// egui のコンテキストができてから一度だけ日本語フォントを登録する。
fn install_ui_font(
    mut done: Local<bool>,
    mut contexts: bevy_egui::EguiContexts,
    font: Res<UiFont>,
) {
    if *done {
        return;
    }
    if let Some(ctx) = contexts.try_ctx_mut() {
        ui::install_fonts(ctx, font.0.as_deref());
        *done = true;
    }
}

/// 「適用」でシミュレーションを作り直し、エンティティを張り替える。
fn apply_settings(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut settings: ResMut<Settings>,
    mut state: ResMut<SimState>,
    mut scen: ResMut<scenario::ScenarioState>,
    visuals: Res<Visuals>,
    mut people: ResMut<person::PersonMeshes>,
    signal_mats: Res<signals::SignalMats>,
    agents: Query<Entity, With<AgentVis>>,
    cars: Query<Entity, With<CarVis>>,
    signal_ents: Query<Entity, With<signals::SignalEntity>>,
) {
    if !settings.apply_requested {
        return;
    }
    settings.apply_requested = false;
    let st = &mut *state;
    match build_sim(st.road_graph.as_ref(), &settings, &mut scen) {
        Ok((sim, traffic, goal_cells)) => {
            st.sim = sim;
            st.traffic = traffic;
            st.goal_cells = goal_cells;
            st.evac = None;
            st.evac_at = None;
        }
        Err(e) => {
            error!("作り直しに失敗: {e:#}");
            scen.message = format!("作り直しに失敗: {e:#}");
            return;
        }
    }
    st.rng = Rng::new(settings.seed ^ 0x9E37_79B9);
    st.respawn = settings.respawn;
    st.acc = 0.0;
    st.heat.iter_mut().for_each(|h| *h = 0);
    st.heat_dirty = true;
    for e in agents.iter().chain(cars.iter()).chain(signal_ents.iter()) {
        commands.entity(e).despawn();
    }
    spawn_entities(&mut commands, &mut meshes, &visuals, &mut people, st);
    signals::spawn(&mut commands, &mut meshes, &signal_mats, st);
    info!(
        "作り直し: 歩行者 {} 人, 車 {} 台",
        st.sim.agents.len(),
        st.traffic.as_ref().map_or(0, |t| t.vehicles.len())
    );
}

fn heat_image_new(w: u32, h: u32) -> Image {
    use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
    Image::new_fill(
        Extent3d {
            width: w,
            height: h,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        &[0, 0, 0, 0],
        TextureFormat::Rgba8UnormSrgb,
        bevy::asset::RenderAssetUsages::default(),
    )
}

fn controls(
    keys: Res<ButtonInput<KeyCode>>,
    mut state: ResMut<SimState>,
    mut overlay: Query<&mut Visibility, With<HeatOverlay>>,
) {
    if keys.just_pressed(KeyCode::Space) {
        state.paused = !state.paused;
    }
    if keys.just_pressed(KeyCode::ArrowUp) {
        state.speed = (state.speed * 2.0).min(64.0);
    }
    if keys.just_pressed(KeyCode::ArrowDown) {
        state.speed = (state.speed * 0.5).max(0.125);
    }
    if keys.just_pressed(KeyCode::KeyH) {
        for mut v in &mut overlay {
            *v = match *v {
                Visibility::Hidden => Visibility::Visible,
                _ => Visibility::Hidden,
            };
        }
    }
}

fn step_sim(
    time: Res<Time>,
    mut state: ResMut<SimState>,
    mut scen: ResMut<scenario::ScenarioState>,
    settings: Res<Settings>,
    shot: Option<Res<ScreenshotPlan>>,
    rec: Option<Res<RecordPlan>>,
    frames: Res<bevy::diagnostic::FrameCount>,
) {
    let st = &mut *state;
    if st.paused {
        return;
    }
    let scen = &mut *scen;
    let k = settings.walk_speed / 1.34;
    let mut advance = |st: &mut SimState| advance(st, scen, k);
    if let Some(r) = rec {
        // 録画モード: 描画速度に関係なく 1 フレーム = sim_per_frame 秒
        if frames.0 < RECORD_WARMUP || r.idx >= r.frames {
            while st.sim.time < r.warmup {
                advance(st);
            }
            return;
        }
        st.acc += r.sim_per_frame;
        let n = (st.acc / DT + 1e-4) as usize;
        st.acc -= n as f32 * DT;
        for _ in 0..n {
            advance(st);
        }
        return;
    }
    let realtime = shot.is_none();
    // スクリーンショットモードでは描画速度に関係なく決まった時刻まで一気に進める
    let mut budget = match shot {
        Some(s) if st.sim.time < s.at => ((s.at - st.sim.time) / DT).ceil() as usize,
        Some(_) => 0,
        None => {
            st.acc += time.delta_secs().min(0.1) * st.speed;
            let n = (st.acc / DT) as usize;
            st.acc -= n as f32 * DT;
            n.min(40)
        }
    };
    // 通常表示では 1 フレームに計算へ使う時間を区切る。大人数で追いつかないときは
    // 描画を止めずに計算の方を遅らせる（積み残しは捨てる）
    let started = bevy::platform::time::Instant::now();
    while budget > 0 {
        budget -= 1;
        advance(st);
        if realtime && started.elapsed().as_secs_f32() > FRAME_SIM_BUDGET {
            st.acc = 0.0;
            break;
        }
    }
}

fn advance(st: &mut SimState, scen: &mut scenario::ScenarioState, speed_scale: f32) {
    {
        if let Some(t) = st.traffic.as_mut() {
            let peds: Vec<Vec2> = st
                .sim
                .agents
                .iter()
                .filter(|a| !a.arrived)
                .map(|a| a.pos)
                .collect();
            t.step(DT, &peds);
            st.sim.obstacles = t.obstacles();
            // 避難中（車が止まっている間）は信号に関係なく渡る
            if let (Some(s), false) = (&t.signals, t.halted) {
                s.write_closed(t.time, &mut st.sim.closed);
            }
        }
        // 避難中はシナリオの発生・通行止めの切り替えを止める（経路を引き直さない）
        if st.evac.is_none() {
            scen.before_step(&mut st.sim, DT, speed_scale);
        }
        st.sim.step(DT);
        evac::advance(st);
        scen.stats.record(
            &st.sim,
            st.traffic
                .iter()
                .flat_map(|t| t.vehicles.iter().map(|v| (v.pos, v.active))),
            DT,
        );
        for a in st.sim.agents.iter() {
            if !a.arrived {
                if let Some((x, y)) = st.sim.grid.cell_of(a.pos) {
                    let i = st.sim.grid.idx(x, y);
                    st.heat[i] += 1;
                }
            }
        }
        st.heat_dirty = true;
        if st.respawn && st.evac.is_none() {
            let limit = scen.background_limit(st.sim.agents.len());
            respawn_arrived(st, limit);
        }
    }
}

/// 到着した人を別の縁から再投入し、定常的な人の流れを作る（シナリオの発生源の人 `limit..` は除く）。
fn respawn_arrived(st: &mut SimState, limit: usize) {
    let n_goals = st.goal_cells.len();
    if n_goals < 2 {
        return;
    }
    for i in 0..limit {
        if !st.sim.agents[i].arrived {
            continue;
        }
        let from = (st.rng.next_u64() % n_goals as u64) as usize;
        let mut to = (st.rng.next_u64() % (n_goals as u64 - 1)) as usize;
        if to >= from {
            to += 1;
        }
        let cells = &st.goal_cells[from];
        let (cx, cy) = cells[(st.rng.next_u64() % cells.len() as u64) as usize];
        let pos = st.sim.grid.center(cx, cy);
        if !st.sim.goals[to].distance(&st.sim.grid, pos).is_finite() {
            continue;
        }
        let a = &mut st.sim.agents[i];
        a.pos = pos;
        a.vel = Vec2::ZERO;
        a.goal = to;
        a.arrived = false;
    }
}

/// シナリオの発生源で増えた歩行者の表示を足す。
fn spawn_new_agents(
    mut commands: Commands,
    state: Res<SimState>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut people: ResMut<person::PersonMeshes>,
    q: Query<&AgentVis>,
) {
    let have = q.iter().count();
    for (i, a) in state.sim.agents.iter().enumerate().skip(have) {
        commands.spawn(agent_bundle(&mut meshes, &mut people, i, a));
    }
}

fn sync_agents(
    state: Res<SimState>,
    settings: Res<Settings>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut people: ResMut<person::PersonMeshes>,
    mut q: Query<(
        &AgentVis,
        &mut person::Gait,
        &mut Transform,
        &mut Visibility,
        &mut Mesh3d,
    )>,
) {
    let evac = state.evac_goals();
    let n_goals = people.goal_count();
    for (AgentVis(i), mut gait, mut tf, mut vis, mut mesh) in &mut q {
        let Some(a) = state.sim.agents.get(*i) else {
            continue;
        };
        let want = if a.arrived {
            Visibility::Hidden
        } else {
            Visibility::Inherited
        };
        if *vis != want {
            *vis = want;
        }
        if a.arrived {
            gait.prev = a.pos;
            continue;
        }
        let phase = gait.advance(a.pos, a.vel);
        let look = person::look_of(*i);
        let top = if evac.as_ref().is_some_and(|r| r.contains(&a.goal)) {
            person::Top::Evac
        } else if settings.color_by_goal {
            person::Top::Goal((a.goal % n_goals) as u8)
        } else {
            person::Top::Cloth(look.top)
        };
        let key = person::Key {
            top,
            lower: look.lower,
            phase,
        };
        // 変わらないものに書き込むと変更扱いになり、変換の伝播・GPU 転送が毎フレーム走る
        if gait.key != Some(key) {
            gait.key = Some(key);
            mesh.0 = people.get(&mut meshes, key);
        }
        let t = to_world(a.pos, 0.0);
        let r = gait.rotation();
        if tf.translation != t || tf.rotation != r {
            tf.translation = t;
            tf.rotation = r;
        }
    }
}

fn sync_cars(state: Res<SimState>, mut q: Query<(&CarVis, &mut Transform, &mut Visibility)>) {
    let Some(t) = &state.traffic else { return };
    for (CarVis(i), mut tf, mut vis) in &mut q {
        let Some(v) = t.vehicles.get(*i) else {
            continue;
        };
        let want = if v.active {
            Visibility::Inherited
        } else {
            Visibility::Hidden
        };
        if *vis != want {
            *vis = want;
        }
        tf.translation = to_world(v.pos, 0.0);
        // 平面の (x, y) → (x, -z) なので、Y 軸回りに atan2(y, x) 回すと +X が進行方向を向く
        tf.rotation = Quat::from_rotation_y(v.heading.y.atan2(v.heading.x));
    }
}

fn update_heat(
    mut state: ResMut<SimState>,
    scen: Res<scenario::ScenarioState>,
    visuals: Res<Visuals>,
    mut images: ResMut<Assets<Image>>,
    overlay: Query<&Visibility, With<HeatOverlay>>,
) {
    let shown = overlay.iter().any(|v| *v != Visibility::Hidden);
    if !shown || !state.heat_dirty {
        return;
    }
    state.heat_dirty = false;
    let g = &state.sim.grid;
    let max = state.heat.iter().copied().max().unwrap_or(1).max(1) as f32;
    let Some(img) = images.get_mut(&visuals.heat_image) else {
        return;
    };
    let Some(data) = img.data.as_mut() else {
        return;
    };
    if scen.diff_view {
        scenario::diff_pixels(&scen, g, data);
        return;
    }
    for y in 0..g.h {
        for x in 0..g.w {
            let v = state.heat[g.idx(x, y)];
            // テクスチャの上端 = 北
            let o = ((g.h - 1 - y) * g.w + x) * 4;
            let px = if v == 0 {
                [0, 0, 0, 0]
            } else {
                let t = (v as f32).ln_1p() / max.ln_1p();
                [
                    255,
                    (230.0 * (1.0 - t)) as u8,
                    (40.0 * (1.0 - t)) as u8,
                    (90.0 + 150.0 * t) as u8,
                ]
            };
            data[o..o + 4].copy_from_slice(&px);
        }
    }
}

fn update_hud(state: Res<SimState>, mut q: Query<&mut Text, With<Hud>>) {
    let active = state.sim.active();
    for mut t in &mut q {
        **t = format!(
            "t = {:6.1} s   walking {}/{}   cars {}   speed x{}{}\n\
             LMB rotate / RMB pan / wheel zoom   [Space] pause  [Up/Down] speed  [H] heatmap",
            state.sim.time,
            active,
            state.sim.agents.len(),
            state.traffic.as_ref().map_or(0, |t| t.active()),
            state.speed,
            if state.paused { "  (paused)" } else { "" },
        );
    }
}

fn take_screenshot(
    mut commands: Commands,
    mut plan: ResMut<ScreenshotPlan>,
    state: Res<SimState>,
    frames: Res<bevy::diagnostic::FrameCount>,
    mut exit: EventWriter<AppExit>,
) {
    if state.sim.time + 1e-3 < plan.at {
        return;
    }
    match plan.taken_frame {
        // シーンの描画が落ち着くまで数フレーム待ってから撮る
        None if frames.0 > 10 => {
            commands
                .spawn(Screenshot::primary_window())
                .observe(save_to_disk(plan.path.clone()));
            plan.taken_frame = Some(frames.0);
        }
        Some(f) if frames.0 > f + 10 => {
            exit.write(AppExit::Success);
        }
        _ => {}
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn record_frames(
    mut commands: Commands,
    mut plan: ResMut<RecordPlan>,
    frames: Res<bevy::diagnostic::FrameCount>,
    mut cams: Query<&mut camera::OrbitCamera>,
    mut exit: EventWriter<AppExit>,
) {
    if frames.0 < RECORD_WARMUP {
        return;
    }
    if plan.idx < plan.frames {
        let path = plan.dir.join(format!("frame_{:05}.png", plan.idx));
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(path));
        plan.idx += 1;
        for mut c in &mut cams {
            c.yaw += plan.orbit_per_frame;
        }
        if plan.idx % 30 == 0 {
            info!("録画 {}/{} フレーム", plan.idx, plan.frames);
        }
        return;
    }
    // 保存待ちのスクリーンショットが書き終わるまで少し待って終了
    let done = *plan.done_at.get_or_insert(frames.0);
    if frames.0 > done + 30 {
        exit.write(AppExit::Success);
    }
}
