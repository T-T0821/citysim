//! 信号機の表示: 横断歩道の縞、車両用・歩行者用の灯器、スクランブル中の交差点内の色付け。

use crate::geometry::to_world;
use crate::SimState;
use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use bevy::render::mesh::{Indices, PrimitiveTopology};
use traffic::signal::Plan;
use traffic::{Light, Walk};

/// 信号関連のエンティティ（作り直すときにまとめて消す）
#[derive(Component)]
pub struct SignalEntity;

/// 車両用灯器: (交差点, 流入路)
#[derive(Component)]
pub struct VehHead(usize, usize);

/// 歩行者用灯器: (交差点, 横断歩道)
#[derive(Component)]
pub struct PedHead(usize, usize);

/// スクランブル交差点内の「青」表示: 交差点
#[derive(Component)]
pub struct ScrambleFloor(usize);

#[derive(Resource)]
pub struct SignalMats {
    green: Handle<StandardMaterial>,
    yellow: Handle<StandardMaterial>,
    red: Handle<StandardMaterial>,
    off: Handle<StandardMaterial>,
    pole: Handle<StandardMaterial>,
    zebra: Handle<StandardMaterial>,
    floor: Handle<StandardMaterial>,
    head: Handle<Mesh>,
    ped_head: Handle<Mesh>,
    pole_mesh: Handle<Mesh>,
}

impl SignalMats {
    pub fn new(meshes: &mut Assets<Mesh>, materials: &mut Assets<StandardMaterial>) -> Self {
        let lamp = |m: &mut Assets<StandardMaterial>, c: Color| {
            m.add(StandardMaterial {
                base_color: c,
                emissive: c.to_linear() * 4.0,
                unlit: true,
                ..default()
            })
        };
        Self {
            green: lamp(materials, Color::srgb(0.15, 0.85, 0.55)),
            yellow: lamp(materials, Color::srgb(1.0, 0.78, 0.1)),
            red: lamp(materials, Color::srgb(0.95, 0.12, 0.10)),
            off: materials.add(StandardMaterial {
                base_color: Color::srgb(0.15, 0.15, 0.17),
                ..default()
            }),
            pole: materials.add(StandardMaterial {
                base_color: Color::srgb(0.55, 0.57, 0.60),
                metallic: 0.5,
                perceptual_roughness: 0.5,
                ..default()
            }),
            zebra: materials.add(StandardMaterial {
                base_color: Color::srgb(0.93, 0.93, 0.90),
                perceptual_roughness: 0.9,
                ..default()
            }),
            floor: materials.add(StandardMaterial {
                base_color: Color::srgba(0.2, 0.85, 0.5, 0.28),
                alpha_mode: AlphaMode::Blend,
                unlit: true,
                ..default()
            }),
            head: meshes.add(Cuboid::new(1.1, 1.1, 1.1)),
            ped_head: meshes.add(Cuboid::new(0.55, 0.55, 0.55)),
            pole_mesh: meshes.add(Cylinder::new(0.08, 1.0)),
        }
    }
}

/// 上向きの四角形を集めた平面メッシュ
#[derive(Default)]
struct Flat {
    pos: Vec<[f32; 3]>,
    idx: Vec<u32>,
}

impl Flat {
    /// 局所座標の四角形（反時計回り）
    fn quad(&mut self, q: [Vec2; 4], h: f32) {
        let i = self.pos.len() as u32;
        self.pos
            .extend(q.iter().map(|&p| to_world(p, h).to_array()));
        self.idx.extend([i, i + 1, i + 2, i, i + 2, i + 3]);
    }
    fn build(self) -> Mesh {
        let n = vec![[0.0, 1.0, 0.0]; self.pos.len()];
        Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        )
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, self.pos)
        .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, n)
        .with_inserted_indices(Indices::U32(self.idx))
    }
}

/// 信号の表示エンティティを作る。信号が無ければ何もしない。
pub fn spawn(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    mats: &SignalMats,
    state: &SimState,
) {
    let Some(sig) = state.traffic.as_ref().and_then(|t| t.signals.as_ref()) else {
        return;
    };
    let g = &state.sim.grid;
    let mut zebra = Flat::default();
    for (k, inter) in sig.intersections.iter().enumerate() {
        for (c, cw) in inter.crosswalks.iter().enumerate() {
            // 横断方向に 0.45m の白線を 0.9m おき
            let (u, v) = (cw.along * cw.half_len, cw.across);
            let mut d = -cw.extent.1 + 0.3;
            while d + 0.45 <= cw.extent.0 - 0.3 {
                let (a, b) = (cw.center + v * d, cw.center + v * (d + 0.45));
                zebra.quad([a - u, a + u, b + u, b - u], 0.015);
                d += 0.9;
            }
            // 両端の歩道に歩行者用灯器
            for side in [cw.extent.0 + 0.4, -(cw.extent.1 + 0.4)] {
                let p = cw.center + v * side;
                spawn_pole(commands, mats, p, 2.4);
                commands.spawn((
                    SignalEntity,
                    PedHead(k, c),
                    Mesh3d(mats.ped_head.clone()),
                    MeshMaterial3d(mats.red.clone()),
                    Transform::from_translation(to_world(p, 2.6)),
                ));
            }
        }
        for (j, ap) in inter.approaches.iter().enumerate() {
            // 停止線の左（歩道側）に車両用灯器
            let p = ap.stop_pos + ap.dir.perp() * (ap.width.0 + 0.6);
            spawn_pole(commands, mats, p, 5.0);
            commands.spawn((
                SignalEntity,
                VehHead(k, j),
                Mesh3d(mats.head.clone()),
                MeshMaterial3d(mats.red.clone()),
                Transform::from_translation(to_world(p, 5.3)),
            ));
            // 停止線
            let n = ap.dir.perp();
            let w = ap.dir * 0.2;
            let (a, b) = (ap.stop_pos, ap.stop_pos + n * (ap.width.0 - 0.3));
            zebra.quad([a - w, a + w, b + w, b - w], 0.015);
        }
        if inter.plan == Plan::Scramble && !inter.interior.is_empty() {
            let mut floor = Flat::default();
            let mut cells: Vec<u32> = inter.interior.clone();
            cells.sort_unstable();
            // 同じ行で連続するセルを 1 枚にまとめる
            let mut k0 = 0;
            while k0 < cells.len() {
                let mut k1 = k0;
                while k1 + 1 < cells.len()
                    && cells[k1 + 1] == cells[k1] + 1
                    && (cells[k1 + 1] as usize) % g.w != 0
                {
                    k1 += 1;
                }
                let (i0, i1) = (cells[k0] as usize, cells[k1] as usize);
                let y = (i0 / g.w) as f32;
                let (x0, x1) = ((i0 % g.w) as f32, (i1 % g.w) as f32 + 1.0);
                let p = |x: f32, y: f32| g.origin + Vec2::new(x, y) * g.cell;
                floor.quad([p(x0, y), p(x1, y), p(x1, y + 1.0), p(x0, y + 1.0)], 0.025);
                k0 = k1 + 1;
            }
            commands.spawn((
                SignalEntity,
                ScrambleFloor(k),
                Mesh3d(meshes.add(floor.build())),
                MeshMaterial3d(mats.floor.clone()),
                Visibility::Hidden,
            ));
        }
    }
    if !zebra.pos.is_empty() {
        commands.spawn((
            SignalEntity,
            Mesh3d(meshes.add(zebra.build())),
            MeshMaterial3d(mats.zebra.clone()),
        ));
    }
}

fn spawn_pole(commands: &mut Commands, mats: &SignalMats, p: Vec2, h: f32) {
    commands.spawn((
        SignalEntity,
        Mesh3d(mats.pole_mesh.clone()),
        MeshMaterial3d(mats.pole.clone()),
        Transform::from_translation(to_world(p, h * 0.5)).with_scale(Vec3::new(1.0, h, 1.0)),
    ));
}

/// 灯色を現在時刻に合わせる。点滅は 0.5 秒ごとに点灯・消灯。
#[allow(clippy::type_complexity)]
pub fn sync(
    state: Res<SimState>,
    mats: Res<SignalMats>,
    mut veh: Query<(&VehHead, &mut MeshMaterial3d<StandardMaterial>), Without<PedHead>>,
    mut ped: Query<(&PedHead, &mut MeshMaterial3d<StandardMaterial>), Without<VehHead>>,
    mut floor: Query<(&ScrambleFloor, &mut Visibility)>,
) {
    let Some(t) = &state.traffic else { return };
    let Some(sig) = &t.signals else { return };
    let now = t.time;
    let blink_on = (now * 2.0).floor() as i64 % 2 == 0;
    let set = |m: &mut MeshMaterial3d<StandardMaterial>, h: &Handle<StandardMaterial>| {
        if m.0 != *h {
            m.0 = h.clone();
        }
    };
    for (VehHead(k, j), mut m) in &mut veh {
        let Some(inter) = sig.intersections.get(*k) else {
            continue;
        };
        let Some(ap) = inter.approaches.get(*j) else {
            continue;
        };
        let h = match inter.light(ap.group, now) {
            Light::Green => &mats.green,
            Light::Yellow => &mats.yellow,
            Light::Red => &mats.red,
        };
        set(&mut m, h);
    }
    for (PedHead(k, c), mut m) in &mut ped {
        let Some(inter) = sig.intersections.get(*k) else {
            continue;
        };
        if *c >= inter.crosswalks.len() {
            continue;
        }
        let h = match inter.walk(*c, now) {
            Walk::Walk => &mats.green,
            Walk::Flash if blink_on => &mats.green,
            Walk::Flash => &mats.off,
            Walk::DontWalk => &mats.red,
        };
        set(&mut m, h);
    }
    for (ScrambleFloor(k), mut vis) in &mut floor {
        let Some(inter) = sig.intersections.get(*k) else {
            continue;
        };
        let want = match inter.interior_walk(now) {
            Walk::Walk => Visibility::Visible,
            Walk::Flash if blink_on => Visibility::Visible,
            _ => Visibility::Hidden,
        };
        if *vis != want {
            *vis = want;
        }
    }
}
