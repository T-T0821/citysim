//! 負荷テストモード（`?city=synthetic`）: 合成の街で 100 万人を GPU 上で動かす。
//!
//! CPU で 1 人ずつ計算すると、Web 版（1 スレッド）では 10 万人で 1 ステップ約 50ms かかり、
//! 100 万人は実時間の数十分の一の速さにしかならない。そこで状態（位置・速度）を
//! 1024×1024 の浮動小数点テクスチャに置き、密度の集計と力の計算・移動を GPU の描画パスで行う
//! （WebGL2 には計算シェーダーが無いので、フラグメントシェーダーで 1 テクセル = 1 人を計算する）。
//! 描画も同じテクスチャから位置を読むので、CPU と GPU の間で毎フレーム送るデータは無い。
//!
//! 歩行者モデルは銀座のデモ（2 万人以下）と同じ crowd::Model::Full（Social Force Model）で、
//! 近傍は GPU 上の近傍表（2m マスごとに最大 8 人）から引く。`?model=lite` で密度場の軽量モデルにもできる。
//! 人の見た目も銀座と同じ人型（近く: 歩く人型 / 遠く: 箱 5 つ）で、身長数ピクセル以下の遠方だけ箱 1 つにする。
//!
//! 銀座と違う点:
//! - 車・信号・シナリオは無し。目的地は街の東西南北の端で、着いたら別の端から入り直す
//! - 行き先への向きはフローフィールドの代わりに、格子状の街路から式で求める
//! - 近傍表に入りきらない人（2m マスに 9 人目以降。4.5 人/m² 超の大混雑）は他の人を押さない

use std::borrow::Cow;

use bevy::{
    asset::RenderAssetUsages,
    core_pipeline::core_3d::Transparent3d,
    ecs::{
        query::QueryItem,
        system::{lifetimeless::*, SystemParamItem},
    },
    pbr::{
        MeshPipeline, MeshPipelineKey, RenderMeshInstances, SetMeshBindGroup, SetMeshViewBindGroup,
        ViewKeyCache,
    },
    prelude::*,
    render::{
        extract_component::{ExtractComponent, ExtractComponentPlugin},
        extract_resource::{ExtractResource, ExtractResourcePlugin},
        mesh::{
            allocator::MeshAllocator, Indices, MeshVertexBufferLayoutRef, PrimitiveTopology,
            RenderMesh, RenderMeshBufferInfo,
        },
        render_asset::RenderAssets,
        render_graph::{self, RenderGraph, RenderLabel},
        render_phase::{
            AddRenderCommand, DrawFunctions, PhaseItem, PhaseItemExtraIndex, RenderCommand,
            RenderCommandResult, SetItemPipeline, TrackedRenderPass, ViewSortedRenderPhases,
        },
        render_resource::{
            binding_types::{sampler, texture_2d, uniform_buffer},
            *,
        },
        renderer::{RenderContext, RenderDevice, RenderQueue},
        texture::GpuImage,
        view::ExtractedView,
        Render, RenderApp, RenderSet,
    },
};
use bevy_egui::{egui, EguiContexts};

use crate::synth_city::SynthCity;

/// 状態テクスチャの一辺（最大 1024² = 1,048,576 人）
pub const TEX: u32 = 1024;
pub const MAX_AGENTS: usize = (TEX * TEX) as usize;
/// 既定の人数（PC / スマホ）
const DEFAULT_AGENTS: usize = 1_000_000;
const DEFAULT_AGENTS_MOBILE: usize = 100_000;
/// 1 ステップの最大の時間 [s]（これより長いフレームでは計算を遅らせる）。銀座のデモと同じ刻み
const MAX_DT_FULL: f32 = 0.05;
const MAX_DT_LITE: f32 = 0.1;
/// 近傍表のマスの大きさ [m] と一辺のマス数（stress_sim.wgsl と同じ）
const CELL: f32 = 2.0;
const GRID: u32 = 1024;
/// 近傍表の 1 マスに入る人数
const SLOTS: usize = 8;
/// 画面上の身長がこれ [px] より小さい人は箱 5 つの簡易な人型（銀座の FAR_PERSON_PX と同じ）
const FAR_PERSON_PX: f32 = 24.0;
/// さらにこれより小さい人は箱 1 つ
const TINY_PERSON_PX: f32 = 8.0;
/// 歩く人型・箱 5 つで描く範囲の上限（マス数の半分）。頂点処理の量を抑える
const MAX_NEAR_HALF: i32 = 48;
const MAX_MID_HALF: i32 = 128;

pub struct StressArgs {
    pub agents: Option<usize>,
    pub seed: u64,
    pub speed: f32,
    pub mobile: bool,
    /// 密度場の軽量モデルにする（既定は銀座と同じ Social Force Model）
    pub lite: bool,
}

/// 負荷テストモードのアプリを動かす（Web 版は戻らない）
pub fn run(args: StressArgs) {
    let n = args
        .agents
        .unwrap_or(if args.mobile {
            DEFAULT_AGENTS_MOBILE
        } else {
            DEFAULT_AGENTS
        })
        .clamp(1, MAX_AGENTS);
    let city = SynthCity::generate(args.seed);
    info!(
        "合成の街: {:.0}m 四方, 街区 {}x{}, 歩ける面積 {:.0} m², {} 人（{:.2} 人/m²）",
        city.size,
        city.nx(),
        city.ny(),
        city.walkable_area(),
        n,
        n as f32 / city.walkable_area()
    );
    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: "PLATEAU crowd simulator - 1M stress test".into(),
            resolution: (1280.0_f32, 800.0_f32).into(),
            canvas: cfg!(target_arch = "wasm32").then(|| "#bevy".into()),
            fit_canvas_to_parent: true,
            ..default()
        }),
        ..default()
    }))
    .add_plugins(StressRenderPlugin)
    .insert_resource(ClearColor(Color::srgb(0.62, 0.72, 0.82)))
    .insert_resource(AmbientLight {
        brightness: 500.0,
        ..default()
    })
    .insert_resource(StressSim {
        n,
        size: city.size,
        latest: 0,
        frame: 0,
        step: None,
        dt: 0.0,
        time: 0.0,
        speed: args.speed,
        paused: false,
        mobile: args.mobile,
        lite: args.lite,
        fps: 0.0,
        sim_rate: 0.0,
        log_in: 2.0,
        cam_cell: (0, 0),
        near_half: -1,
        mid_half: -1,
    })
    .insert_resource(CityRes(city))
    .insert_resource(SeedRes(args.seed))
    .init_resource::<crate::camera::UiCapture>()
    .add_plugins(bevy_egui::EguiPlugin {
        enable_multipass_for_primary_context: false,
    })
    .add_systems(Startup, setup)
    .add_systems(
        Update,
        (
            install_font,
            panel,
            crate::camera::orbit_camera,
            keys,
            advance,
            lod_ranges,
        )
            .chain(),
    );
    app.run();
}

#[derive(Resource)]
struct CityRes(SynthCity);

#[derive(Resource)]
struct SeedRes(u64);

/// メインワールド側の状態。毎フレーム描画側へ写す（ExtractResource）
#[derive(Resource, Clone, ExtractResource)]
pub struct StressSim {
    pub n: usize,
    pub size: f32,
    /// 最新の状態が入っている状態テクスチャ（0 / 1）
    pub latest: usize,
    pub frame: u32,
    /// このフレームで進めるなら (読む方, 書く方)
    pub step: Option<(usize, usize)>,
    pub dt: f32,
    pub time: f32,
    pub speed: f32,
    pub paused: bool,
    pub mobile: bool,
    pub lite: bool,
    pub fps: f32,
    pub sim_rate: f32,
    log_in: f32,
    /// カメラの真下の近傍表のマスと、歩く人型・箱 5 つで描く正方形の範囲（マス数の半分。-1 は無し）
    pub cam_cell: (i32, i32),
    pub near_half: i32,
    pub mid_half: i32,
}

/// GPU 側のテクスチャ（メインワールドで Image として作り、描画側では GpuImage を引く）
#[derive(Resource, Clone, ExtractResource)]
pub struct StressImages {
    state: [Handle<Image>; 2],
    /// 歩行の位相・向き・近傍表を作ったときのマス（状態と一緒に入れ替える）
    aux: [Handle<Image>; 2],
    /// 目的地, 希望速度, 人ごとの乱数, 半径
    attr: Handle<Image>,
    /// 見た目（person::look_of）: 横幅・身長の倍率, 上着, 下の組み合わせ
    look: Handle<Image>,
    density: Handle<Image>,
    axis: Handle<Image>,
    blocks: Handle<Image>,
    bands: Handle<Image>,
    nbx: u32,
    nby: u32,
    nbands_x: u32,
    nbands_y: u32,
}

/// 人を描くエンティティ（遠さの段階ごとに 1 つ）
#[derive(Component, Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum StressPeople {
    /// 歩く人型（近く）
    Near,
    /// 箱 5 つ
    Mid,
    /// 箱 1 つ（全員を描き、上の 2 つで描いた人は潰す）
    Distant,
}

impl ExtractComponent for StressPeople {
    type QueryData = &'static StressPeople;
    type QueryFilter = ();
    type Out = Self;
    fn extract_component(item: QueryItem<'_, Self::QueryData>) -> Option<Self> {
        Some(*item)
    }
}

fn float_image(
    w: u32,
    h: u32,
    format: TextureFormat,
    data: Vec<u8>,
    usage: TextureUsages,
) -> Image {
    let mut img = Image::new(
        Extent3d {
            width: w,
            height: h,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        data,
        format,
        RenderAssetUsages::RENDER_WORLD,
    );
    img.texture_descriptor.usage = usage;
    img
}

fn f32_bytes(v: &[[f32; 4]]) -> Vec<u8> {
    bytemuck::cast_slice(v).to_vec()
}

#[allow(clippy::too_many_arguments)]
fn setup(
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    sim: Res<StressSim>,
    city: Res<CityRes>,
    seed: Res<SeedRes>,
) {
    let city = &city.0;
    let n = sim.n;
    // 初期状態: 歩ける所にばらまき、目的地・希望速度・人ごとの乱数を決める
    let mut rng = crowd::Rng::new(seed.0 ^ 0xA77);
    let mut state = vec![[0.0f32; 4]; MAX_AGENTS];
    let mut attr = vec![[0.0f32; 4]; MAX_AGENTS];
    let mut aux = vec![[0.0f32; 4]; MAX_AGENTS];
    let mut look = vec![[0.0f32; 4]; MAX_AGENTS];
    for (i, (x, y)) in city.scatter(n, seed.0).into_iter().enumerate() {
        state[i] = [x, y, 0.0, 0.0];
        // 銀座（crowd::scenario::through_traffic）と同じ半径・希望速度の分布
        let seed_i = rng.f32();
        attr[i] = [
            (rng.next_u64() % 4) as f32,
            rng.normal(1.34, 0.26).clamp(0.6, 2.0),
            seed_i,
            rng.range(0.22, 0.28),
        ];
        // 歩行の位相は番号でずらす（person::Gait::new と同じ）
        aux[i] = [
            (i as f32 * 0.618_034).fract(),
            seed_i * std::f32::consts::TAU,
            0.0,
            0.0,
        ];
        let l = crate::person::look_of(i);
        look[i] = [l.scale.x, l.scale.y, l.top as f32, l.lower as f32];
    }
    let rw =
        TextureUsages::TEXTURE_BINDING | TextureUsages::RENDER_ATTACHMENT | TextureUsages::COPY_DST;
    let ro = TextureUsages::TEXTURE_BINDING | TextureUsages::COPY_DST;
    let state_bytes = f32_bytes(&state);
    drop(state);
    let s0 = images.add(float_image(
        TEX,
        TEX,
        TextureFormat::Rgba32Float,
        state_bytes.clone(),
        rw,
    ));
    let s1 = images.add(float_image(
        TEX,
        TEX,
        TextureFormat::Rgba32Float,
        state_bytes,
        rw,
    ));
    let attr = images.add(float_image(
        TEX,
        TEX,
        TextureFormat::Rgba32Float,
        f32_bytes(&attr),
        ro,
    ));
    let look = images.add(float_image(
        TEX,
        TEX,
        TextureFormat::Rgba32Float,
        f32_bytes(&look),
        ro,
    ));
    let aux_bytes = f32_bytes(&aux);
    drop(aux);
    let a0 = images.add(float_image(
        TEX,
        TEX,
        TextureFormat::Rgba32Float,
        aux_bytes.clone(),
        rw,
    ));
    let a1 = images.add(float_image(
        TEX,
        TEX,
        TextureFormat::Rgba32Float,
        aux_bytes,
        rw,
    ));
    let size = city.size as u32;
    let density = images.add(float_image(
        size,
        size,
        TextureFormat::R16Float,
        vec![0; (size * size * 2) as usize],
        rw,
    ));
    let axis = images.add(float_image(
        size,
        2,
        TextureFormat::Rgba32Float,
        f32_bytes(&city.axis_table()),
        ro,
    ));
    let blocks: Vec<[f32; 4]> = city
        .park
        .iter()
        .map(|&p| [if p { 1.0 } else { 0.0 }, 0.0, 0.0, 0.0])
        .collect();
    let blocks = images.add(float_image(
        city.nx() as u32,
        city.ny() as u32,
        TextureFormat::Rgba32Float,
        f32_bytes(&blocks),
        ro,
    ));
    let (bw, bands) = city.band_table();
    let bands = images.add(float_image(
        bw as u32,
        2,
        TextureFormat::Rgba32Float,
        f32_bytes(&bands),
        ro,
    ));
    commands.insert_resource(StressImages {
        state: [s0, s1],
        aux: [a0, a1],
        attr,
        look,
        density,
        axis,
        blocks,
        bands,
        nbx: city.nx() as u32,
        nby: city.ny() as u32,
        nbands_x: city.x.bands.len() as u32,
        nbands_y: city.y.bands.len() as u32,
    });

    // 人: 遠さの段階ごとのメッシュ（ローカルの +x が前、足裏が原点）
    let mut tiny = Mesh::from(Cuboid::new(0.26, 1.7, 0.44));
    tiny.remove_attribute(Mesh::ATTRIBUTE_UV_0);
    if let Some(bevy::render::mesh::VertexAttributeValues::Float32x3(p)) =
        tiny.attribute_mut(Mesh::ATTRIBUTE_POSITION)
    {
        p.iter_mut().for_each(|v| v[1] += 0.85);
    }
    for (mesh, kind) in [
        (crate::person::build_rigged(), StressPeople::Near),
        (crate::person::build_far(), StressPeople::Mid),
        (tiny, StressPeople::Distant),
    ] {
        commands.spawn((
            Mesh3d(meshes.add(mesh)),
            kind,
            bevy::render::view::NoFrustumCulling,
            Transform::default(),
            Visibility::default(),
        ));
    }

    // 街: 地面（街路）・公園・建物
    let half = city.size * 0.5;
    let to_world = |x: f32, y: f32| Vec3::new(x - half, 0.0, -(y - half));
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(city.size, city.size))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.42, 0.42, 0.44),
            perceptual_roughness: 1.0,
            ..default()
        })),
    ));
    let mut parks = BoxMesh::default();
    for (j, &(y0, y1)) in city.y.blocks.iter().enumerate() {
        for (i, &(x0, x1)) in city.x.blocks.iter().enumerate() {
            if city.is_park(i, j) {
                parks.add(to_world(x0, y0), to_world(x1, y1), 0.0, 0.05);
            }
        }
    }
    commands.spawn((
        Mesh3d(meshes.add(parks.build())),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.36, 0.55, 0.30),
            perceptual_roughness: 1.0,
            ..default()
        })),
    ));
    let mut bldg = BoxMesh::default();
    for [x0, y0, x1, y1, h] in city.buildings(seed.0) {
        bldg.add(to_world(x0, y0), to_world(x1, y1), 0.0, h);
    }
    commands.spawn((
        Mesh3d(meshes.add(bldg.build())),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.86, 0.85, 0.82),
            perceptual_roughness: 0.9,
            ..default()
        })),
    ));

    commands.spawn((
        DirectionalLight {
            illuminance: 9000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_xyz(-400.0, 1000.0, 300.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    let mobile = sim.mobile;
    commands.spawn((
        Camera3d::default(),
        bevy::render::view::NoIndirectDrawing,
        if mobile { Msaa::Off } else { Msaa::default() },
        Projection::Perspective(PerspectiveProjection {
            far: 12_000.0,
            near: 0.5,
            ..default()
        }),
        // 中心に近い交差点を斜め上から見る（ズームアウトで街全体）
        crate::camera::OrbitCamera {
            target: {
                let mid = |a: &crate::synth_city::Axis| {
                    a.bands
                        .iter()
                        .map(|&(x0, x1)| (x0 + x1) * 0.5)
                        .min_by(|p, q| (p - half).abs().total_cmp(&(q - half).abs()))
                        .unwrap_or(half)
                };
                to_world(mid(&city.x), mid(&city.y))
            },
            yaw: -0.6,
            pitch: 0.7,
            distance: 70.0,
        },
        Transform::default(),
    ));
}

/// 直方体を 1 つのメッシュにまとめる（建物・公園）
#[derive(Default)]
struct BoxMesh {
    pos: Vec<[f32; 3]>,
    nrm: Vec<[f32; 3]>,
    idx: Vec<u32>,
}

impl BoxMesh {
    fn add(&mut self, a: Vec3, b: Vec3, y0: f32, y1: f32) {
        let (lo, hi) = (a.min(b), a.max(b));
        let (x0, x1, z0, z1) = (lo.x, hi.x, lo.z, hi.z);
        let faces: [([f32; 3], [[f32; 3]; 4]); 5] = [
            (
                [0.0, 1.0, 0.0],
                [[x0, y1, z0], [x0, y1, z1], [x1, y1, z1], [x1, y1, z0]],
            ),
            (
                [0.0, 0.0, 1.0],
                [[x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]],
            ),
            (
                [0.0, 0.0, -1.0],
                [[x1, y0, z0], [x0, y0, z0], [x0, y1, z0], [x1, y1, z0]],
            ),
            (
                [1.0, 0.0, 0.0],
                [[x1, y0, z1], [x1, y0, z0], [x1, y1, z0], [x1, y1, z1]],
            ),
            (
                [-1.0, 0.0, 0.0],
                [[x0, y0, z0], [x0, y0, z1], [x0, y1, z1], [x0, y1, z0]],
            ),
        ];
        for (n, q) in faces {
            let base = self.pos.len() as u32;
            self.pos.extend(q);
            self.nrm.extend([n; 4]);
            self.idx
                .extend([base, base + 1, base + 2, base, base + 2, base + 3]);
        }
    }

    fn build(self) -> Mesh {
        Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::RENDER_WORLD,
        )
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, self.pos)
        .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, self.nrm)
        .with_inserted_indices(Indices::U32(self.idx))
    }
}

/// 時間を進める（GPU に何ステップ進めさせるかを決める）
fn advance(time: Res<Time>, mut sim: ResMut<StressSim>) {
    let real = time.delta_secs();
    if real > 0.0 {
        let k = (real / 0.5).min(1.0);
        sim.fps += (1.0 / real - sim.fps) * k;
    }
    sim.frame = sim.frame.wrapping_add(1);
    sim.step = None;
    let mut dt = 0.0;
    // 最初の数フレームはパイプラインの準備を待つ
    if !sim.paused && sim.frame > 3 {
        let max_dt = if sim.lite { MAX_DT_LITE } else { MAX_DT_FULL };
        dt = (real * sim.speed).min(max_dt);
        let src = sim.latest;
        sim.step = Some((src, 1 - src));
        sim.latest = 1 - src;
        sim.dt = dt;
        sim.time += dt;
    }
    if real > 0.0 {
        let k = (real / 0.5).min(1.0);
        sim.sim_rate += (dt / real - sim.sim_rate) * k;
    }
    sim.log_in -= real;
    if sim.log_in <= 0.0 {
        sim.log_in = 5.0;
        info!(
            "perf: {:.1} fps, sim x{:.2}, {} agents (GPU, {} model)",
            sim.fps,
            sim.sim_rate,
            sim.n,
            if sim.lite { "lite" } else { "full" }
        );
    }
}

/// 歩く人型・箱 5 つで描く範囲を、銀座のデモと同じ「画面上の身長」の基準で決める
fn lod_ranges(
    mut sim: ResMut<StressSim>,
    camera: Query<(&Transform, &Projection), With<Camera3d>>,
    windows: Query<&Window, With<bevy::window::PrimaryWindow>>,
) {
    let Ok((tf, Projection::Perspective(p))) = camera.single() else {
        return;
    };
    let h = windows.single().map_or(800.0, |w| w.height());
    // 距離 1m あたり身長 1m が何 px になるか
    let px_per_m = h / (2.0 * (p.fov * 0.5).tan());
    let half = sim.size * 0.5;
    let (cx, cy) = (tf.translation.x + half, -tf.translation.z + half);
    let height = tf.translation.y.max(0.0);
    let range = |px: f32, cap: i32| -> i32 {
        let d = 1.7 * px_per_m / px;
        if d <= height {
            return -1;
        }
        let r = (d * d - height * height).sqrt();
        ((r / CELL).ceil() as i32).min(cap)
    };
    sim.cam_cell = (
        ((cx / CELL).floor() as i32).clamp(0, GRID as i32 - 1),
        ((cy / CELL).floor() as i32).clamp(0, GRID as i32 - 1),
    );
    sim.near_half = range(FAR_PERSON_PX, MAX_NEAR_HALF);
    sim.mid_half = range(TINY_PERSON_PX, MAX_MID_HALF).max(sim.near_half);
}

fn keys(keys: Res<ButtonInput<KeyCode>>, mut sim: ResMut<StressSim>) {
    if keys.just_pressed(KeyCode::Space) {
        sim.paused = !sim.paused;
    }
    if keys.just_pressed(KeyCode::ArrowUp) {
        sim.speed = (sim.speed * 2.0).min(8.0);
    }
    if keys.just_pressed(KeyCode::ArrowDown) {
        sim.speed = (sim.speed * 0.5).max(0.125);
    }
}

fn install_font(mut done: Local<bool>, mut contexts: EguiContexts) {
    if *done {
        return;
    }
    if let Some(ctx) = contexts.try_ctx_mut() {
        crate::ui::install_fonts(ctx, None);
        *done = true;
    }
}

fn panel(
    mut contexts: EguiContexts,
    mut sim: ResMut<StressSim>,
    city: Res<CityRes>,
    mut capture: ResMut<crate::camera::UiCapture>,
) {
    let Some(ctx) = contexts.try_ctx_mut() else {
        return;
    };
    let area = city.0.walkable_area();
    egui::Window::new("負荷テスト: 合成の街")
        .default_pos([10.0, 10.0])
        .default_width(280.0)
        .show(ctx, |ui| {
            ui.label(format!(
                "{} 人 · {:.0}m 四方 · {:.2} 人/m²",
                fmt_count(sim.n),
                city.0.size,
                sim.n as f32 / area
            ));
            ui.label(format!(
                "{:.0} fps · 実時間の {:.2} 倍 · t = {:.0} s",
                sim.fps, sim.sim_rate, sim.time
            ));
            ui.horizontal(|ui| {
                if ui
                    .button(if sim.paused {
                        "▶ 再開"
                    } else {
                        "⏸ 一時停止"
                    })
                    .clicked()
                {
                    sim.paused = !sim.paused;
                }
                ui.label("速さ");
                ui.add(egui::Slider::new(&mut sim.speed, 0.125..=8.0).logarithmic(true));
            });
            ui.horizontal_wrapped(|ui| {
                ui.label("人数:");
                for (n, label) in [(100_000, "10万"), (300_000, "30万"), (1_000_000, "100万")] {
                    if ui.selectable_label(sim.n == n, label).clicked() && sim.n != n {
                        reload_with(&format!(
                            "?city=synthetic&agents={n}{}",
                            if sim.lite { "&model=lite" } else { "" }
                        ));
                    }
                }
            });
            ui.horizontal(|ui| {
                ui.label("歩行者モデル:");
                let q = |lite: bool| {
                    format!(
                        "?city=synthetic&agents={}{}",
                        sim.n,
                        if lite { "&model=lite" } else { "" }
                    )
                };
                if ui
                    .selectable_label(!sim.lite, "銀座と同じ（SFM）")
                    .clicked()
                    && sim.lite
                {
                    reload_with(&q(false));
                }
                if ui.selectable_label(sim.lite, "密度場（軽量）").clicked() && !sim.lite {
                    reload_with(&q(true));
                }
            });
            ui.small(
                "計算と描画はすべて GPU 上。人の見た目は銀座と同じ（数ピクセルの遠方だけ箱 1 つ）。\
                 車・信号は無し。",
            );
            if ui.link("← 銀座 4 丁目のデモに戻る").clicked() {
                reload_with("?");
            }
        });
    capture.0 = ctx.wants_pointer_input() || ctx.is_pointer_over_area();
}

fn fmt_count(n: usize) -> String {
    if n >= 10_000 && n.is_multiple_of(10_000) {
        format!("{} 万", n / 10_000)
    } else {
        n.to_string()
    }
}

/// Web 版: URL のクエリを変えて読み込み直す（ネイティブ版では何もしない）
pub fn reload_with(search: &str) {
    #[cfg(target_arch = "wasm32")]
    if let Some(w) = web_sys::window() {
        let _ = w.location().set_search(search);
    }
    #[cfg(not(target_arch = "wasm32"))]
    info!("Web 版では {search} で読み込み直します");
}

// ---------------- 描画側 ----------------

pub struct StressRenderPlugin;

#[derive(Debug, Hash, PartialEq, Eq, Clone, RenderLabel)]
struct StressSimLabel;

#[derive(Resource, Clone)]
struct StressShaders {
    sim: Handle<Shader>,
    people: Handle<Shader>,
}

impl Plugin for StressRenderPlugin {
    fn build(&self, app: &mut App) {
        let people_src = format!(
            "{}\n{}\n{}\n{}",
            crate::person::lowers_wgsl(),
            crate::person::tops_wgsl(),
            crate::person::rig_wgsl(),
            include_str!("stress_people.wgsl")
        );
        let mut shaders = app.world_mut().resource_mut::<Assets<Shader>>();
        let sim = shaders.add(Shader::from_wgsl(
            include_str!("stress_sim.wgsl"),
            "stress_sim.wgsl",
        ));
        let people = shaders.add(Shader::from_wgsl(people_src, "stress_people.wgsl"));
        app.add_plugins((
            ExtractResourcePlugin::<StressSim>::default(),
            ExtractResourcePlugin::<StressImages>::default(),
            ExtractComponentPlugin::<StressPeople>::default(),
        ));
        let Some(render_app) = app.get_sub_app_mut(RenderApp) else {
            return;
        };
        render_app
            .insert_resource(StressShaders { sim, people })
            .add_render_command::<Transparent3d, DrawStressPeople>()
            .init_resource::<SpecializedMeshPipelines<PeoplePipeline>>()
            .add_systems(
                Render,
                (
                    queue_people.in_set(RenderSet::QueueMeshes),
                    prepare_bind_groups.in_set(RenderSet::PrepareBindGroups),
                ),
            );
        let mut graph = render_app.world_mut().resource_mut::<RenderGraph>();
        graph.add_node(StressSimLabel, StressSimNode);
        graph.add_node_edge(StressSimLabel, bevy::render::graph::CameraDriverLabel);
    }

    fn finish(&self, app: &mut App) {
        if let Some(render_app) = app.get_sub_app_mut(RenderApp) {
            render_app
                .init_resource::<SimPipelines>()
                .init_resource::<PeoplePipeline>();
        }
    }
}

use uniforms::{PeopleUniform, SimUniform};

// ShaderType の derive が作る検査用の関数が未使用の警告になるので、モジュールにまとめて抑える
#[allow(dead_code)]
mod uniforms {
    use bevy::render::render_resource::ShaderType;

    #[derive(ShaderType, Clone, Copy, Default)]
    pub struct SimUniform {
        pub dt: f32,
        pub size: f32,
        pub n: u32,
        pub frame: u32,
        pub nbx: u32,
        pub nby: u32,
        pub nbands_x: u32,
        pub nbands_y: u32,
    }

    #[derive(ShaderType, Clone, Copy, Default)]
    pub struct PeopleUniform {
        pub size: f32,
        pub n: u32,
        pub cam_x: i32,
        pub cam_y: i32,
        pub near_half: i32,
        pub mid_half: i32,
        pub _pad0: u32,
        pub _pad1: u32,
    }
}

/// 近傍表のテクスチャ（初期データが要らないので描画側で直接作る）
struct TableTextures {
    slots: Vec<TextureView>,
    depth: TextureView,
    /// 8 枚を 4 人ずつ 2 枚にまとめたもの（Rgba32Uint）。読む側はこちらを使う
    packed: [TextureView; 2],
}

#[derive(Resource)]
struct SimPipelines {
    table_first_layout: BindGroupLayout,
    table_next_layout: BindGroupLayout,
    density_layout: BindGroupLayout,
    update_full_layout: BindGroupLayout,
    update_lite_layout: BindGroupLayout,
    pack: CachedRenderPipelineId,
    pack_group: BindGroup,
    table_first: CachedRenderPipelineId,
    table_next: CachedRenderPipelineId,
    density: CachedRenderPipelineId,
    update_full: CachedRenderPipelineId,
    update_lite: CachedRenderPipelineId,
    sampler: Sampler,
    table: TableTextures,
}

#[allow(clippy::too_many_arguments)]
fn point_pipeline(
    label: &'static str,
    layout: &BindGroupLayout,
    shader: &Handle<Shader>,
    defs: Vec<ShaderDefVal>,
    vertex: &'static str,
    fragment: &'static str,
    target: ColorTargetState,
    depth: Option<DepthStencilState>,
) -> RenderPipelineDescriptor {
    RenderPipelineDescriptor {
        label: Some(label.into()),
        layout: vec![layout.clone()],
        push_constant_ranges: vec![],
        vertex: VertexState {
            shader: shader.clone(),
            shader_defs: defs.clone(),
            entry_point: Cow::from(vertex),
            buffers: vec![],
        },
        primitive: PrimitiveState {
            topology: PrimitiveTopology::PointList,
            ..default()
        },
        depth_stencil: depth,
        multisample: MultisampleState::default(),
        fragment: Some(FragmentState {
            shader: shader.clone(),
            shader_defs: defs,
            entry_point: Cow::from(fragment),
            targets: vec![Some(target)],
        }),
        zero_initialize_workgroup_memory: false,
    }
}

impl FromWorld for SimPipelines {
    fn from_world(world: &mut World) -> Self {
        let device = world.resource::<RenderDevice>();
        let unfilterable = TextureSampleType::Float { filterable: false };
        let uint = TextureSampleType::Uint;
        let stage = ShaderStages::VERTEX_FRAGMENT;
        let table_first_layout = device.create_bind_group_layout(
            "stress table layout",
            &BindGroupLayoutEntries::sequential(
                stage,
                (
                    uniform_buffer::<SimUniform>(false),
                    texture_2d(unfilterable),
                ),
            ),
        );
        let table_next_layout = device.create_bind_group_layout(
            "stress table next layout",
            &BindGroupLayoutEntries::sequential(
                stage,
                (
                    uniform_buffer::<SimUniform>(false),
                    texture_2d(unfilterable),
                    texture_2d(uint),
                ),
            ),
        );
        let density_layout = device.create_bind_group_layout(
            "stress density layout",
            &BindGroupLayoutEntries::sequential(
                stage,
                (
                    uniform_buffer::<SimUniform>(false),
                    texture_2d(unfilterable),
                ),
            ),
        );
        let common = (
            uniform_buffer::<SimUniform>(false),
            texture_2d(unfilterable),
            texture_2d(unfilterable),
            texture_2d(unfilterable),
            texture_2d(unfilterable),
            texture_2d(unfilterable),
            texture_2d(unfilterable),
        );
        let update_full_layout = device.create_bind_group_layout(
            "stress update layout (full)",
            &BindGroupLayoutEntries::sequential(
                stage,
                (
                    common.0,
                    common.1,
                    common.2,
                    common.3,
                    common.4,
                    common.5,
                    common.6,
                    texture_2d(uint),
                    texture_2d(uint),
                ),
            ),
        );
        let update_lite_layout = device.create_bind_group_layout(
            "stress update layout (lite)",
            &BindGroupLayoutEntries::sequential(
                stage,
                (
                    uniform_buffer::<SimUniform>(false),
                    texture_2d(unfilterable),
                    texture_2d(unfilterable),
                    texture_2d(unfilterable),
                    texture_2d(unfilterable),
                    texture_2d(unfilterable),
                    texture_2d(unfilterable),
                    texture_2d(TextureSampleType::Float { filterable: true }),
                    sampler(SamplerBindingType::Filtering),
                ),
            ),
        );
        let sampler = device.create_sampler(&SamplerDescriptor {
            label: Some("stress density sampler"),
            mag_filter: FilterMode::Linear,
            min_filter: FilterMode::Linear,
            address_mode_u: AddressMode::ClampToEdge,
            address_mode_v: AddressMode::ClampToEdge,
            ..default()
        });
        let tex = |label: &'static str, format: TextureFormat| {
            device
                .create_texture(&TextureDescriptor {
                    label: Some(label),
                    size: Extent3d {
                        width: GRID,
                        height: GRID,
                        depth_or_array_layers: 1,
                    },
                    mip_level_count: 1,
                    sample_count: 1,
                    dimension: TextureDimension::D2,
                    format,
                    usage: TextureUsages::RENDER_ATTACHMENT | TextureUsages::TEXTURE_BINDING,
                    view_formats: &[],
                })
                .create_view(&TextureViewDescriptor::default())
        };
        let table = TableTextures {
            slots: (0..SLOTS)
                .map(|_| tex("stress neighbor slot", TextureFormat::R32Uint))
                .collect(),
            depth: tex("stress neighbor depth", TextureFormat::Depth32Float),
            packed: [
                tex("stress neighbor packed", TextureFormat::Rgba32Uint),
                tex("stress neighbor packed", TextureFormat::Rgba32Uint),
            ],
        };
        let pack_layout = device.create_bind_group_layout(
            "stress pack layout",
            &BindGroupLayoutEntries::sequential(
                ShaderStages::FRAGMENT,
                (
                    texture_2d(uint),
                    texture_2d(uint),
                    texture_2d(uint),
                    texture_2d(uint),
                    texture_2d(uint),
                    texture_2d(uint),
                    texture_2d(uint),
                    texture_2d(uint),
                ),
            ),
        );
        let sl = &table.slots;
        let pack_group = device.create_bind_group(
            "stress pack bind group",
            &pack_layout,
            &BindGroupEntries::sequential((
                &sl[0], &sl[1], &sl[2], &sl[3], &sl[4], &sl[5], &sl[6], &sl[7],
            )),
        );

        let shader = world.resource::<StressShaders>().sim.clone();
        let cache = world.resource::<PipelineCache>();
        let table_target = ColorTargetState {
            format: TextureFormat::R32Uint,
            blend: None,
            write_mask: ColorWrites::ALL,
        };
        let table_depth = Some(DepthStencilState {
            format: TextureFormat::Depth32Float,
            depth_write_enabled: true,
            depth_compare: CompareFunction::Less,
            stencil: StencilState::default(),
            bias: DepthBiasState::default(),
        });
        let table_first = cache.queue_render_pipeline(point_pipeline(
            "stress table",
            &table_first_layout,
            &shader,
            vec![],
            "table_vertex",
            "table_fragment",
            table_target.clone(),
            table_depth.clone(),
        ));
        let table_next = cache.queue_render_pipeline(point_pipeline(
            "stress table next",
            &table_next_layout,
            &shader,
            vec!["TABLE_NEXT".into()],
            "table_vertex",
            "table_fragment",
            table_target,
            table_depth,
        ));
        let density = cache.queue_render_pipeline(point_pipeline(
            "stress density",
            &density_layout,
            &shader,
            vec![],
            "density_vertex",
            "density_fragment",
            ColorTargetState {
                format: TextureFormat::R16Float,
                blend: Some(BlendState {
                    color: BlendComponent {
                        src_factor: BlendFactor::One,
                        dst_factor: BlendFactor::One,
                        operation: BlendOperation::Add,
                    },
                    alpha: BlendComponent::OVER,
                }),
                write_mask: ColorWrites::ALL,
            },
            None,
        ));
        let update = |label: &'static str, layout: &BindGroupLayout, defs: Vec<ShaderDefVal>| {
            let target = Some(ColorTargetState {
                format: TextureFormat::Rgba32Float,
                blend: None,
                write_mask: ColorWrites::ALL,
            });
            cache.queue_render_pipeline(RenderPipelineDescriptor {
                label: Some(label.into()),
                layout: vec![layout.clone()],
                push_constant_ranges: vec![],
                vertex: VertexState {
                    shader: shader.clone(),
                    shader_defs: defs.clone(),
                    entry_point: Cow::from("fullscreen_vertex"),
                    buffers: vec![],
                },
                primitive: PrimitiveState::default(),
                depth_stencil: None,
                multisample: MultisampleState::default(),
                fragment: Some(FragmentState {
                    shader: shader.clone(),
                    shader_defs: defs,
                    entry_point: Cow::from("update_fragment"),
                    targets: vec![target.clone(), target],
                }),
                zero_initialize_workgroup_memory: false,
            })
        };
        let pack_target = Some(ColorTargetState {
            format: TextureFormat::Rgba32Uint,
            blend: None,
            write_mask: ColorWrites::ALL,
        });
        let pack = cache.queue_render_pipeline(RenderPipelineDescriptor {
            label: Some("stress pack".into()),
            layout: vec![pack_layout.clone()],
            push_constant_ranges: vec![],
            vertex: VertexState {
                shader: shader.clone(),
                shader_defs: vec!["PACK".into()],
                entry_point: Cow::from("fullscreen_vertex"),
                buffers: vec![],
            },
            primitive: PrimitiveState::default(),
            depth_stencil: None,
            multisample: MultisampleState::default(),
            fragment: Some(FragmentState {
                shader: shader.clone(),
                shader_defs: vec!["PACK".into()],
                entry_point: Cow::from("pack_fragment"),
                targets: vec![pack_target.clone(), pack_target],
            }),
            zero_initialize_workgroup_memory: false,
        });
        let update_full = update(
            "stress update (full)",
            &update_full_layout,
            vec!["UPDATE".into()],
        );
        let update_lite = update(
            "stress update (lite)",
            &update_lite_layout,
            vec!["UPDATE".into(), "LITE".into()],
        );
        Self {
            table_first_layout,
            table_next_layout,
            density_layout,
            update_full_layout,
            update_lite_layout,
            pack,
            pack_group,
            table_first,
            table_next,
            density,
            update_full,
            update_lite,
            sampler,
            table,
        }
    }
}

/// 1 ステップぶんのバインドグループと書き込み先
struct StepGroups {
    table: Vec<BindGroup>,
    density: Option<(BindGroup, TextureView)>,
    update: BindGroup,
    state_dst: TextureView,
    aux_dst: TextureView,
}

/// このフレームで使うバインドグループ
#[derive(Resource)]
struct StressBindGroups {
    step: Option<StepGroups>,
    people: BindGroup,
    // バッファはバインドグループが使い終わるまで保持する
    _buffers: Vec<UniformBuffer<SimUniform>>,
    _people_buf: UniformBuffer<PeopleUniform>,
}

#[allow(clippy::too_many_arguments)]
fn prepare_bind_groups(
    mut commands: Commands,
    sim: Option<Res<StressSim>>,
    images: Option<Res<StressImages>>,
    gpu_images: Res<RenderAssets<GpuImage>>,
    pipelines: Res<SimPipelines>,
    people_pipeline: Res<PeoplePipeline>,
    device: Res<RenderDevice>,
    queue: Res<RenderQueue>,
) {
    let (Some(sim), Some(img)) = (sim, images) else {
        return;
    };
    let get = |h: &Handle<Image>| gpu_images.get(h).map(|g| &g.texture_view);
    let (
        Some(s0),
        Some(s1),
        Some(a0),
        Some(a1),
        Some(attr),
        Some(look),
        Some(dens),
        Some(axis),
        Some(blocks),
        Some(bands),
    ) = (
        get(&img.state[0]),
        get(&img.state[1]),
        get(&img.aux[0]),
        get(&img.aux[1]),
        get(&img.attr),
        get(&img.look),
        get(&img.density),
        get(&img.axis),
        get(&img.blocks),
        get(&img.bands),
    )
    else {
        return;
    };
    let states = [s0, s1];
    let auxes = [a0, a1];
    let slots = &pipelines.table.slots;
    let mut buffers = vec![];
    let step = sim.step.map(|(src, dst)| {
        let mut u = UniformBuffer::from(SimUniform {
            dt: sim.dt,
            size: sim.size,
            n: sim.n as u32,
            frame: sim.frame,
            nbx: img.nbx,
            nby: img.nby,
            nbands_x: img.nbands_x,
            nbands_y: img.nbands_y,
        });
        u.write_buffer(&device, &queue);
        let ub = u.binding().unwrap();
        let mut table = vec![device.create_bind_group(
            "stress table bind group",
            &pipelines.table_first_layout,
            &BindGroupEntries::sequential((ub.clone(), states[src])),
        )];
        for k in 1..SLOTS {
            table.push(device.create_bind_group(
                "stress table bind group",
                &pipelines.table_next_layout,
                &BindGroupEntries::sequential((ub.clone(), states[src], &slots[k - 1])),
            ));
        }
        let (density, update) = if sim.lite {
            let d = device.create_bind_group(
                "stress density bind group",
                &pipelines.density_layout,
                &BindGroupEntries::sequential((ub.clone(), states[src])),
            );
            let up = device.create_bind_group(
                "stress update bind group (lite)",
                &pipelines.update_lite_layout,
                &BindGroupEntries::sequential((
                    ub.clone(),
                    states[src],
                    attr,
                    auxes[src],
                    axis,
                    blocks,
                    bands,
                    dens,
                    &pipelines.sampler,
                )),
            );
            (Some((d, dens.clone())), up)
        } else {
            let up = device.create_bind_group(
                "stress update bind group (full)",
                &pipelines.update_full_layout,
                &BindGroupEntries::sequential((
                    ub.clone(),
                    states[src],
                    attr,
                    auxes[src],
                    axis,
                    blocks,
                    bands,
                    &pipelines.table.packed[0],
                    &pipelines.table.packed[1],
                )),
            );
            (None, up)
        };
        buffers.push(u);
        StepGroups {
            table,
            density,
            update,
            state_dst: states[dst].clone(),
            aux_dst: auxes[dst].clone(),
        }
    });
    let mut pu = UniformBuffer::from(PeopleUniform {
        size: sim.size,
        n: sim.n as u32,
        cam_x: sim.cam_cell.0,
        cam_y: sim.cam_cell.1,
        near_half: sim.near_half,
        mid_half: sim.mid_half,
        ..default()
    });
    pu.write_buffer(&device, &queue);
    let people = device.create_bind_group(
        "stress people bind group",
        &people_pipeline.layout,
        &BindGroupEntries::sequential((
            pu.binding().unwrap(),
            states[sim.latest],
            look,
            auxes[sim.latest],
            &pipelines.table.packed[0],
            &pipelines.table.packed[1],
        )),
    );
    commands.insert_resource(StressBindGroups {
        step,
        people,
        _buffers: buffers,
        _people_buf: pu,
    });
}

/// 描画の前に GPU 上でシミュレーションを 1 ステップ進めるノード
struct StressSimNode;

impl render_graph::Node for StressSimNode {
    fn run(
        &self,
        _graph: &mut render_graph::RenderGraphContext,
        render_context: &mut RenderContext,
        world: &World,
    ) -> Result<(), render_graph::NodeRunError> {
        let (Some(groups), Some(sim)) = (
            world.get_resource::<StressBindGroups>(),
            world.get_resource::<StressSim>(),
        ) else {
            return Ok(());
        };
        let Some(step) = &groups.step else {
            return Ok(());
        };
        let pipelines = world.resource::<SimPipelines>();
        let cache = world.resource::<PipelineCache>();
        let update_id = if sim.lite {
            pipelines.update_lite
        } else {
            pipelines.update_full
        };
        let (Some(table_first), Some(table_next), Some(update)) = (
            cache.get_render_pipeline(pipelines.table_first),
            cache.get_render_pipeline(pipelines.table_next),
            cache.get_render_pipeline(update_id),
        ) else {
            return Ok(());
        };
        let density = cache.get_render_pipeline(pipelines.density);
        if sim.lite && density.is_none() {
            return Ok(());
        }
        let n = sim.n as u32;
        // 1. 近傍表（8 枚）
        for (k, bg) in step.table.iter().enumerate() {
            let mut pass = render_context.begin_tracked_render_pass(RenderPassDescriptor {
                label: Some("stress neighbor table"),
                color_attachments: &[Some(RenderPassColorAttachment {
                    view: &pipelines.table.slots[k],
                    resolve_target: None,
                    ops: Operations {
                        load: LoadOp::Clear(Default::default()),
                        store: StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(RenderPassDepthStencilAttachment {
                    view: &pipelines.table.depth,
                    depth_ops: Some(Operations {
                        load: LoadOp::Clear(1.0),
                        store: StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
            });
            pass.set_render_pipeline(if k == 0 { table_first } else { table_next });
            pass.set_bind_group(0, bg, &[]);
            pass.draw(0..n, 0..1);
        }
        // 近傍表を 2 枚にまとめる
        let Some(pack) = cache.get_render_pipeline(pipelines.pack) else {
            return Ok(());
        };
        {
            let attach = |view| {
                Some(RenderPassColorAttachment {
                    view,
                    resolve_target: None,
                    ops: Operations {
                        load: LoadOp::Clear(Default::default()),
                        store: StoreOp::Store,
                    },
                })
            };
            let mut pass = render_context.begin_tracked_render_pass(RenderPassDescriptor {
                label: Some("stress neighbor pack"),
                color_attachments: &[
                    attach(&pipelines.table.packed[0]),
                    attach(&pipelines.table.packed[1]),
                ],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
            });
            pass.set_render_pipeline(pack);
            pass.set_bind_group(0, &pipelines.pack_group, &[]);
            pass.draw(0..3, 0..1);
        }
        // 2. 密度（軽量モデル）
        if let (Some((bg, view)), Some(density)) = (&step.density, density) {
            let mut pass = render_context.begin_tracked_render_pass(RenderPassDescriptor {
                label: Some("stress density"),
                color_attachments: &[Some(RenderPassColorAttachment {
                    view,
                    resolve_target: None,
                    ops: Operations {
                        load: LoadOp::Clear(Default::default()),
                        store: StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
            });
            pass.set_render_pipeline(density);
            pass.set_bind_group(0, bg, &[]);
            pass.draw(0..n, 0..1);
        }
        // 3. 更新（状態と補助の 2 枚に書く）
        let attach = |view| {
            Some(RenderPassColorAttachment {
                view,
                resolve_target: None,
                ops: Operations {
                    load: LoadOp::Load,
                    store: StoreOp::Store,
                },
            })
        };
        let mut pass = render_context.begin_tracked_render_pass(RenderPassDescriptor {
            label: Some("stress update"),
            color_attachments: &[attach(&step.state_dst), attach(&step.aux_dst)],
            depth_stencil_attachment: None,
            timestamp_writes: None,
            occlusion_query_set: None,
        });
        pass.set_render_pipeline(update);
        pass.set_bind_group(0, &step.update, &[]);
        pass.draw(0..3, 0..1);
        Ok(())
    }
}

// ---- 人の描画（crowd_render と同じ仕組みで、インスタンスの位置をテクスチャから読む） ----

#[derive(Resource)]
struct PeoplePipeline {
    shader: Handle<Shader>,
    mesh_pipeline: MeshPipeline,
    layout: BindGroupLayout,
}

impl FromWorld for PeoplePipeline {
    fn from_world(world: &mut World) -> Self {
        let device = world.resource::<RenderDevice>();
        let unfilterable = TextureSampleType::Float { filterable: false };
        let uint = TextureSampleType::Uint;
        let layout = device.create_bind_group_layout(
            "stress people layout",
            &BindGroupLayoutEntries::sequential(
                ShaderStages::VERTEX,
                (
                    uniform_buffer::<PeopleUniform>(false),
                    texture_2d(unfilterable),
                    texture_2d(unfilterable),
                    texture_2d(unfilterable),
                    texture_2d(uint),
                    texture_2d(uint),
                ),
            ),
        );
        Self {
            shader: world.resource::<StressShaders>().people.clone(),
            mesh_pipeline: world.resource::<MeshPipeline>().clone(),
            layout,
        }
    }
}

impl SpecializedMeshPipeline for PeoplePipeline {
    type Key = (MeshPipelineKey, StressPeople);

    fn specialize(
        &self,
        (key, kind): Self::Key,
        layout: &MeshVertexBufferLayoutRef,
    ) -> Result<RenderPipelineDescriptor, SpecializedMeshPipelineError> {
        let mut desc = self.mesh_pipeline.specialize(key, layout)?;
        let def: ShaderDefVal = match kind {
            StressPeople::Near => "NEAR".into(),
            StressPeople::Mid => "MID".into(),
            StressPeople::Distant => "DISTANT".into(),
        };
        desc.vertex.shader = self.shader.clone();
        desc.vertex.shader_defs.push(def.clone());
        desc.layout.truncate(2);
        desc.layout.push(self.layout.clone());
        let frag = desc
            .fragment
            .as_mut()
            .expect("mesh pipeline has a fragment stage");
        frag.shader = self.shader.clone();
        frag.shader_defs.push(def);
        for t in frag.targets.iter_mut().flatten() {
            t.blend = None;
        }
        if let Some(ds) = desc.depth_stencil.as_mut() {
            ds.depth_write_enabled = true;
        }
        Ok(desc)
    }
}

#[allow(clippy::too_many_arguments)]
fn queue_people(
    draw_functions: Res<DrawFunctions<Transparent3d>>,
    pipeline: Res<PeoplePipeline>,
    mut pipelines: ResMut<SpecializedMeshPipelines<PeoplePipeline>>,
    pipeline_cache: Res<PipelineCache>,
    meshes: Res<RenderAssets<RenderMesh>>,
    render_mesh_instances: Res<RenderMeshInstances>,
    view_key_cache: Res<ViewKeyCache>,
    groups: Query<(Entity, &bevy::render::sync_world::MainEntity, &StressPeople)>,
    mut phases: ResMut<ViewSortedRenderPhases<Transparent3d>>,
    views: Query<&ExtractedView>,
) {
    let draw = draw_functions.read().id::<DrawStressPeople>();
    for view in &views {
        let Some(phase) = phases.get_mut(&view.retained_view_entity) else {
            continue;
        };
        let Some(view_key) = view_key_cache.get(&view.retained_view_entity) else {
            continue;
        };
        for (entity, main_entity, kind) in &groups {
            let Some(mesh_instance) = render_mesh_instances.render_mesh_queue_data(*main_entity)
            else {
                continue;
            };
            let Some(mesh) = meshes.get(mesh_instance.mesh_asset_id) else {
                continue;
            };
            let key =
                *view_key | MeshPipelineKey::from_primitive_topology(mesh.primitive_topology());
            let Ok(pipeline) =
                pipelines.specialize(&pipeline_cache, &pipeline, (key, *kind), &mesh.layout)
            else {
                continue;
            };
            phase.add(Transparent3d {
                entity: (entity, *main_entity),
                pipeline,
                draw_function: draw,
                // 不透明として描くので、半透明の物より先に描かれるよう遠くに置く
                distance: f32::MIN,
                batch_range: 0..1,
                extra_index: PhaseItemExtraIndex::None,
                indexed: true,
            });
        }
    }
}

type DrawStressPeople = (
    SetItemPipeline,
    SetMeshViewBindGroup<0>,
    SetMeshBindGroup<1>,
    DrawPeopleFromTexture,
);

struct DrawPeopleFromTexture;

impl<P: PhaseItem> RenderCommand<P> for DrawPeopleFromTexture {
    type Param = (
        SRes<RenderAssets<RenderMesh>>,
        SRes<RenderMeshInstances>,
        SRes<MeshAllocator>,
        Option<SRes<StressBindGroups>>,
        Option<SRes<StressSim>>,
    );
    type ViewQuery = ();
    type ItemQuery = Read<StressPeople>;

    #[inline]
    fn render<'w>(
        item: &P,
        _view: (),
        kind: Option<&'w StressPeople>,
        (meshes, render_mesh_instances, mesh_allocator, groups, sim): SystemParamItem<
            'w,
            '_,
            Self::Param,
        >,
        pass: &mut TrackedRenderPass<'w>,
    ) -> RenderCommandResult {
        let (Some(groups), Some(sim), Some(kind)) = (groups, sim, kind) else {
            return RenderCommandResult::Skip;
        };
        // 近傍表から引く段階は、カメラの周りの正方形のマス × 8 人ぶん
        let square = |half: i32| {
            if half < 0 {
                0
            } else {
                let side = (2 * half + 1) as u32;
                side * side * SLOTS as u32
            }
        };
        let count = match kind {
            StressPeople::Near => square(sim.near_half),
            StressPeople::Mid => square(sim.mid_half),
            StressPeople::Distant => sim.n as u32,
        };
        if count == 0 {
            return RenderCommandResult::Skip;
        }
        let mesh_allocator = mesh_allocator.into_inner();
        let Some(mesh_instance) = render_mesh_instances.render_mesh_queue_data(item.main_entity())
        else {
            return RenderCommandResult::Skip;
        };
        let Some(gpu_mesh) = meshes.into_inner().get(mesh_instance.mesh_asset_id) else {
            return RenderCommandResult::Skip;
        };
        let Some(vertex_slice) = mesh_allocator.mesh_vertex_slice(&mesh_instance.mesh_asset_id)
        else {
            return RenderCommandResult::Skip;
        };
        pass.set_bind_group(2, &groups.into_inner().people, &[]);
        pass.set_vertex_buffer(0, vertex_slice.buffer.slice(..));
        let instances = 0..count;
        match &gpu_mesh.buffer_info {
            RenderMeshBufferInfo::Indexed {
                index_format,
                count,
            } => {
                let Some(index_slice) =
                    mesh_allocator.mesh_index_slice(&mesh_instance.mesh_asset_id)
                else {
                    return RenderCommandResult::Skip;
                };
                pass.set_index_buffer(index_slice.buffer.slice(..), 0, *index_format);
                pass.draw_indexed(
                    index_slice.range.start..(index_slice.range.start + count),
                    vertex_slice.range.start as i32,
                    instances,
                );
            }
            RenderMeshBufferInfo::NonIndexed => {
                pass.draw(vertex_slice.range, instances);
            }
        }
        RenderCommandResult::Success
    }
}
