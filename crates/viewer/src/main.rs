//! plateau-crowd-viewer: PLATEAU の建物を 3D 表示し、その上で歩行者をリアルタイムに動かす。
//!
//! 操作: 左ドラッグ=回転 / 右ドラッグ=平行移動 / ホイール=ズーム
//!       Space=一時停止 / ↑↓=再生速度 / H=ヒートマップ表示切替

mod camera;
mod geometry;

use anyhow::{bail, Context, Result};
use bevy::prelude::*;
use bevy::render::view::screenshot::{save_to_disk, Screenshot};
use crowd::{Rng, Simulation};
use geometry::{to_world, walkable_surface, MeshBuilder};
use std::path::PathBuf;
use walkmap::WalkGrid;

const DT: f32 = 0.05;

struct Args {
    gml: Vec<PathBuf>,
    agents: usize,
    cell: f32,
    seed: u64,
    respawn: bool,
    screenshot: Option<(PathBuf, f32)>,
}

fn parse_args() -> Result<Args> {
    let mut a = Args {
        gml: vec![],
        agents: 1000,
        cell: 0.5,
        seed: 1,
        respawn: true,
        screenshot: None,
    };
    let mut it = std::env::args().skip(1);
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
            "--agents" => a.agents = v()?.parse()?,
            "--cell" => a.cell = v()?.parse()?,
            "--seed" => a.seed = v()?.parse()?,
            "--no-respawn" => a.respawn = false,
            // 指定秒数シミュレーションを進めた時点で PNG を保存して終了（CI・確認用）
            "--screenshot" => {
                let p = PathBuf::from(v()?);
                let t = v()?.parse()?;
                a.screenshot = Some((p, t));
            }
            "-h" | "--help" => {
                println!(
                    "usage: plateau-crowd-viewer --gml <file|dir>... [--agents N] [--cell M] [--seed N]\n\
                     \x20                          [--no-respawn] [--screenshot out.png <sim秒>]"
                );
                std::process::exit(0);
            }
            _ => bail!("不明な引数: {k}"),
        }
    }
    if a.gml.is_empty() {
        bail!("--gml を 1 つ以上指定してください (--help)");
    }
    Ok(a)
}

#[derive(Resource)]
struct SimState {
    sim: Simulation,
    rng: Rng,
    /// 目的地ごとの到着セル（再出現位置に使う）
    goal_cells: Vec<Vec<(usize, usize)>>,
    respawn: bool,
    speed: f32,
    paused: bool,
    acc: f32,
    heat: Vec<u32>,
    heat_dirty: bool,
}

#[derive(Resource)]
struct Visuals {
    goal_materials: Vec<Handle<StandardMaterial>>,
    heat_image: Handle<Image>,
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
struct HeatOverlay;

#[derive(Component)]
struct Hud;

fn main() -> Result<()> {
    let args = parse_args()?;
    let mut model = plateau_gml::CityModel::default();
    for p in &args.gml {
        model.merge(plateau_gml::parse_file(p)?);
    }
    info!(
        "建物 {} 棟, 道路 {} 件",
        model.buildings().count(),
        model.roads().count()
    );
    let grid = WalkGrid::from_city(&model, args.cell).context("座標を含む地物がありません")?;
    let (sim, names) = crowd::scenario::through_traffic(grid, args.agents, args.seed)
        .context("範囲の縁に歩行可能領域がありません")?;
    info!("目的地 {:?}, エージェント {} 人", names, sim.agents.len());

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
    let heat = vec![0; sim.grid.w * sim.grid.h];
    let state = SimState {
        sim,
        rng: Rng::new(args.seed ^ 0x9E37_79B9),
        goal_cells,
        respawn: args.respawn,
        speed: 2.0,
        paused: false,
        acc: 0.0,
        heat,
        heat_dirty: true,
    };

    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: "PLATEAU crowd simulator".into(),
            resolution: (1280.0_f32, 800.0_f32).into(),
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
    .add_systems(Startup, setup)
    .add_systems(
        Update,
        (
            camera::orbit_camera,
            controls,
            step_sim,
            sync_agents,
            update_heat,
            update_hud,
        )
            .chain(),
    );
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
    // 歩行可能面
    let walk = walkable_surface(g, 0.0);
    if !walk.is_empty() {
        commands.spawn((
            Mesh3d(meshes.add(walk.build())),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.78, 0.78, 0.76),
                perceptual_roughness: 0.95,
                ..default()
            })),
        ));
    }
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
    for (poly, h) in &g.buildings {
        let band = if *h < 15.0 {
            0
        } else if *h < 40.0 {
            1
        } else {
            2
        };
        by_band[band].add_extrusion(poly, h.max(3.0));
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

    // 歩行者: 目的地ごとに色分けしたカプセル
    let palette = [
        Color::srgb(0.90, 0.25, 0.20),
        Color::srgb(0.15, 0.45, 0.90),
        Color::srgb(0.95, 0.70, 0.10),
        Color::srgb(0.20, 0.70, 0.35),
    ];
    let goal_materials: Vec<_> = (0..state.sim.goals.len())
        .map(|i| {
            materials.add(StandardMaterial {
                base_color: palette[i % 4],
                ..default()
            })
        })
        .collect();
    let body = meshes.add(Capsule3d::new(0.22, 1.2));
    for (i, a) in state.sim.agents.iter().enumerate() {
        commands.spawn((
            AgentVis(i),
            Mesh3d(body.clone()),
            MeshMaterial3d(goal_materials[a.goal].clone()),
            Transform::from_translation(to_world(a.pos, 0.82)),
        ));
    }

    // 光源とカメラ
    commands.spawn((
        DirectionalLight {
            illuminance: 9000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(-0.6, 1.0, 0.4).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    let target = to_world(center, 0.0);
    let dist = size.max_element().max(60.0) * 1.5;
    commands.spawn((
        Camera3d::default(),
        camera::OrbitCamera {
            target,
            yaw: -0.6,
            pitch: 0.9,
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

    commands.insert_resource(Visuals {
        goal_materials,
        heat_image,
    });
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

fn step_sim(time: Res<Time>, mut state: ResMut<SimState>, shot: Option<Res<ScreenshotPlan>>) {
    let st = &mut *state;
    if st.paused {
        return;
    }
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
    while budget > 0 {
        budget -= 1;
        st.sim.step(DT);
        for a in st.sim.agents.iter() {
            if !a.arrived {
                if let Some((x, y)) = st.sim.grid.cell_of(a.pos) {
                    let i = st.sim.grid.idx(x, y);
                    st.heat[i] += 1;
                }
            }
        }
        st.heat_dirty = true;
        if st.respawn {
            respawn_arrived(st);
        }
    }
}

/// 到着した人を別の縁から再投入し、定常的な人の流れを作る。
fn respawn_arrived(st: &mut SimState) {
    let n_goals = st.sim.goals.len();
    if n_goals < 2 {
        return;
    }
    for i in 0..st.sim.agents.len() {
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

fn sync_agents(
    state: Res<SimState>,
    visuals: Res<Visuals>,
    mut q: Query<(
        &AgentVis,
        &mut Transform,
        &mut Visibility,
        &mut MeshMaterial3d<StandardMaterial>,
    )>,
) {
    for (AgentVis(i), mut tf, mut vis, mut mat) in &mut q {
        let a = &state.sim.agents[*i];
        let want = if a.arrived {
            Visibility::Hidden
        } else {
            Visibility::Inherited
        };
        if *vis != want {
            *vis = want;
        }
        tf.translation = to_world(a.pos, 0.82);
        if mat.0 != visuals.goal_materials[a.goal] {
            mat.0 = visuals.goal_materials[a.goal].clone();
        }
    }
}

fn update_heat(
    mut state: ResMut<SimState>,
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
            "t = {:6.1} s   walking {}/{}   speed x{}{}\n\
             LMB rotate / RMB pan / wheel zoom   [Space] pause  [Up/Down] speed  [H] heatmap",
            state.sim.time,
            active,
            state.sim.agents.len(),
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
