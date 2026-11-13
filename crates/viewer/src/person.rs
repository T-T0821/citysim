//! 歩行者の見た目: 頭・胴・腕・脚のある簡単な人型を、服や髪の色違いと歩行の姿勢ごとに用意する。
//!
//! 何千人も並ぶので骨格アニメーションは使わず、歩行周期を `PHASES` 段階に分けた姿勢の
//! メッシュを作っておき、歩いた距離に応じて差し替える。色は頂点カラーで持ち、
//! マテリアルは全員で 1 つを共有する（同じメッシュの人はまとめて描画される）。
//! 体格差は Transform の拡大率で付ける。

use bevy::asset::RenderAssetUsages;
use bevy::math::Affine3A;
use bevy::platform::collections::HashMap;
use bevy::prelude::*;
use bevy::render::mesh::{Indices, PrimitiveTopology};

/// 歩行 1 周期（左右 1 歩ずつ）を何段階の姿勢で表すか
pub const PHASES: u8 = 8;
/// 脚の振りの最大角 [rad]
const LEG_SWING: f32 = 0.42;
/// 股関節の高さ [m]（足裏から）
const HIP: f32 = 0.86;

/// 上着の色（普段の服装）
const TOPS: [(f32, f32, f32); 10] = [
    (0.92, 0.92, 0.90), // 白
    (0.12, 0.12, 0.13), // 黒
    (0.16, 0.20, 0.36), // 紺
    (0.50, 0.51, 0.53), // 灰
    (0.78, 0.70, 0.56), // ベージュ
    (0.62, 0.18, 0.18), // えんじ
    (0.36, 0.40, 0.26), // オリーブ
    (0.55, 0.70, 0.86), // 水色
    (0.60, 0.44, 0.28), // キャメル
    (0.18, 0.34, 0.28), // 深緑
];

/// 上着以外の組み合わせ: ズボン・靴・肌・髪・長髪か
struct Lower {
    pants: (f32, f32, f32),
    shoes: (f32, f32, f32),
    skin: (f32, f32, f32),
    hair: (f32, f32, f32),
    long_hair: bool,
}

const LOWERS: [Lower; 6] = [
    Lower {
        pants: (0.18, 0.24, 0.38),
        shoes: (0.90, 0.90, 0.88),
        skin: (0.93, 0.78, 0.66),
        hair: (0.06, 0.05, 0.05),
        long_hair: false,
    },
    Lower {
        pants: (0.10, 0.10, 0.11),
        shoes: (0.08, 0.08, 0.08),
        skin: (0.95, 0.80, 0.70),
        hair: (0.22, 0.14, 0.09),
        long_hair: true,
    },
    Lower {
        pants: (0.70, 0.62, 0.48),
        shoes: (0.35, 0.22, 0.12),
        skin: (0.84, 0.66, 0.50),
        hair: (0.05, 0.05, 0.05),
        long_hair: false,
    },
    Lower {
        pants: (0.32, 0.32, 0.34),
        shoes: (0.10, 0.10, 0.10),
        skin: (0.92, 0.77, 0.66),
        hair: (0.55, 0.54, 0.52),
        long_hair: false,
    },
    Lower {
        pants: (0.24, 0.28, 0.40),
        shoes: (0.92, 0.92, 0.90),
        skin: (0.96, 0.82, 0.72),
        hair: (0.30, 0.20, 0.12),
        long_hair: true,
    },
    Lower {
        pants: (0.50, 0.46, 0.34),
        shoes: (0.85, 0.85, 0.82),
        skin: (0.72, 0.54, 0.40),
        hair: (0.08, 0.06, 0.05),
        long_hair: false,
    },
];

/// 上着の色の決め方
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum Top {
    /// 普段の服（`TOPS` の番号）
    Cloth(u8),
    /// 目的地ごとの色（目的地で色分けする表示）
    Goal(u8),
    /// 避難中
    Evac,
}

/// 1 人ぶんの見た目のメッシュを決める鍵
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct Key {
    pub top: Top,
    pub lower: u8,
    pub phase: u8,
}

/// 歩行者ごとの固定の見た目（服装と体格）。番号から決まるので乱数状態を持たない。
pub struct Look {
    pub top: u8,
    pub lower: u8,
    pub scale: Vec3,
}

pub fn look_of(i: usize) -> Look {
    // splitmix64
    let mut z = (i as u64).wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^= z >> 31;
    let u = |shift: u32| ((z >> shift) & 0xFFFF) as f32 / 65535.0;
    // 身長 1.55〜1.85 m 程度、肩幅は身長と少し連動
    let h = 0.92 + 0.17 * u(16);
    let w = 0.90 + 0.12 * u(32) + 0.4 * (h - 1.0);
    Look {
        top: (z % TOPS.len() as u64) as u8,
        lower: ((z >> 8) % LOWERS.len() as u64) as u8,
        scale: Vec3::new(w, h, w),
    }
}

/// 歩き方の状態（歩行者の表示エンティティごと）
#[derive(Component)]
pub struct Gait {
    /// 前回のフレームの位置
    pub prev: Vec2,
    /// 歩行周期の位相 0..1
    pub phase: f32,
    /// 向き（シミュレーション平面での角度, x 軸から反時計回り）
    pub heading: f32,
    pub key: Option<Key>,
}

impl Gait {
    pub fn new(pos: Vec2, vel: Vec2, i: usize) -> Self {
        Self {
            prev: pos,
            // 全員が足並みをそろえないように、位相を番号でずらしておく
            phase: (i as f32 * 0.618_034).fract(),
            heading: if vel.length_squared() > 1e-4 {
                vel.y.atan2(vel.x)
            } else {
                0.0
            },
            key: None,
        }
    }

    /// 位置と速度から位相と向きを進め、使う姿勢の番号を返す。
    pub fn advance(&mut self, pos: Vec2, vel: Vec2) -> u8 {
        let step = pos - self.prev;
        self.prev = pos;
        // 再投入などで瞬間移動したときは歩いたことにしない
        let dist = if step.length_squared() > 9.0 {
            0.0
        } else {
            step.length()
        };
        let speed = vel.length();
        if speed > 0.15 {
            // 1 周期で進む距離（2 歩ぶん）。速く歩くほど歩幅が伸びる。
            let cycle = 1.0 + 0.35 * speed.min(2.5);
            self.phase = (self.phase + dist / cycle).fract();
        } else {
            // 立ち止まったら脚をそろえた姿勢（位相 0 か 0.5）へ寄せる
            let target = (self.phase * 2.0).round() * 0.5;
            let d = (target - self.phase).clamp(-0.04, 0.04);
            self.phase = (self.phase + d).rem_euclid(1.0);
        }
        if speed > 0.2 {
            let want = vel.y.atan2(vel.x);
            let mut d = want - self.heading;
            d = (d + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI;
            // 急に向きを変えず、少しずつ振り向く
            self.heading += d * 0.25;
        }
        ((self.phase * PHASES as f32).round() as u32 % PHASES as u32) as u8
    }

    /// 表示用の回転（人型はローカルの +x を向いて作ってある）
    pub fn rotation(&self) -> Quat {
        Quat::from_rotation_y(self.heading)
    }
}

/// 人型メッシュの置き場（必要になった組み合わせだけ作る）と共有マテリアル
#[derive(Resource)]
pub struct PersonMeshes {
    map: HashMap<Key, Handle<Mesh>>,
    pub material: Handle<StandardMaterial>,
    goal_colors: Vec<Color>,
    evac_color: Color,
}

impl PersonMeshes {
    pub fn new(
        materials: &mut Assets<StandardMaterial>,
        goal_colors: Vec<Color>,
        evac_color: Color,
    ) -> Self {
        Self {
            map: HashMap::default(),
            material: materials.add(StandardMaterial {
                base_color: Color::WHITE,
                perceptual_roughness: 0.85,
                ..default()
            }),
            goal_colors,
            evac_color,
        }
    }

    pub fn goal_count(&self) -> usize {
        self.goal_colors.len()
    }

    pub fn get(&mut self, meshes: &mut Assets<Mesh>, key: Key) -> Handle<Mesh> {
        if let Some(h) = self.map.get(&key) {
            return h.clone();
        }
        let top = match key.top {
            Top::Cloth(i) => {
                let (r, g, b) = TOPS[i as usize % TOPS.len()];
                Color::srgb(r, g, b)
            }
            Top::Goal(i) => self.goal_colors[i as usize % self.goal_colors.len()],
            Top::Evac => self.evac_color,
        };
        let lower = &LOWERS[key.lower as usize % LOWERS.len()];
        let h = meshes.add(build(top, lower, key.phase));
        self.map.insert(key, h.clone());
        h
    }
}

#[derive(Default)]
struct Builder {
    pos: Vec<[f32; 3]>,
    nrm: Vec<[f32; 3]>,
    col: Vec<[f32; 4]>,
    idx: Vec<u32>,
}

fn lin(c: (f32, f32, f32)) -> [f32; 4] {
    LinearRgba::from(Color::srgb(c.0, c.1, c.2)).to_f32_array()
}

impl Builder {
    /// 直方体。`half` は半分の大きさ、上面・底面の水平方向の縮尺で先細りにできる。
    fn cuboid(&mut self, tf: Affine3A, half: Vec3, bottom: f32, top: f32, color: [f32; 4]) {
        let c = |sx: f32, sy: f32, sz: f32| {
            let k = if sy > 0.0 { top } else { bottom };
            tf.transform_point3(Vec3::new(sx * half.x * k, sy * half.y, sz * half.z * k))
        };
        let center = tf.transform_point3(Vec3::ZERO);
        let faces = [
            [c(1., -1., -1.), c(1., 1., -1.), c(1., 1., 1.), c(1., -1., 1.)],
            [c(-1., -1., 1.), c(-1., 1., 1.), c(-1., 1., -1.), c(-1., -1., -1.)],
            [c(-1., 1., -1.), c(-1., 1., 1.), c(1., 1., 1.), c(1., 1., -1.)],
            [c(-1., -1., 1.), c(-1., -1., -1.), c(1., -1., -1.), c(1., -1., 1.)],
            [c(-1., -1., 1.), c(1., -1., 1.), c(1., 1., 1.), c(-1., 1., 1.)],
            [c(1., -1., -1.), c(-1., -1., -1.), c(-1., 1., -1.), c(1., 1., -1.)],
        ];
        for mut f in faces {
            let mut n = (f[1] - f[0]).cross(f[2] - f[0]).normalize_or_zero();
            let mid = (f[0] + f[1] + f[2] + f[3]) * 0.25;
            if n.dot(mid - center) < 0.0 {
                f.reverse();
                n = -n;
            }
            let base = self.pos.len() as u32;
            for p in f {
                self.pos.push(p.to_array());
                self.nrm.push(n.to_array());
                self.col.push(color);
            }
            self.idx
                .extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
        }
    }

    /// 頭: 少し縦長の低ポリゴン球。法線の向きで髪と肌を塗り分ける。
    fn head(&mut self, center: Vec3, r: f32, skin: [f32; 4], hair: [f32; 4], long_hair: bool) {
        const LAT: u32 = 6;
        const LON: u32 = 8;
        let base = self.pos.len() as u32;
        for i in 0..=LAT {
            let th = std::f32::consts::PI * i as f32 / LAT as f32;
            for j in 0..=LON {
                let ph = std::f32::consts::TAU * j as f32 / LON as f32;
                let n = Vec3::new(th.sin() * ph.cos(), th.cos(), th.sin() * ph.sin());
                let p = center + Vec3::new(n.x * r, n.y * r * 1.15, n.z * r * 0.95);
                // 頭頂と後頭部は髪（顔は +x 側）。長髪は後ろを首まで覆う。
                let back_limit = if long_hair { -0.9 } else { -0.55 };
                let is_hair = n.y > 0.3 || (n.x < -0.2 && n.y > back_limit);
                self.pos.push(p.to_array());
                self.nrm.push(n.to_array());
                self.col.push(if is_hair { hair } else { skin });
            }
        }
        for i in 0..LAT {
            for j in 0..LON {
                let a = base + i * (LON + 1) + j;
                let b = a + LON + 1;
                self.idx.extend_from_slice(&[a, a + 1, b, a + 1, b + 1, b]);
            }
        }
    }

    fn build(self) -> Mesh {
        Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        )
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, self.pos)
        .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, self.nrm)
        .with_inserted_attribute(Mesh::ATTRIBUTE_COLOR, self.col)
        .with_inserted_indices(Indices::U32(self.idx))
    }
}

/// 足裏を原点、+x を前、+y を上にした身長 1.7 m ほどの人型。`phase` 番目の歩行姿勢。
fn build(top: Color, lower: &Lower, phase: u8) -> Mesh {
    let top = LinearRgba::from(top).to_f32_array();
    let pants = lin(lower.pants);
    let shoes = lin(lower.shoes);
    let skin = lin(lower.skin);
    let hair = lin(lower.hair);

    let s = (std::f32::consts::TAU * phase as f32 / PHASES as f32).sin();
    let leg = s * LEG_SWING;
    // 脚を開くと腰が下がる（上下の揺れ）
    let drop = HIP * (1.0 - leg.cos());
    let root = Affine3A::from_translation(Vec3::new(0.0, -drop, 0.0));
    let at = |x: f32, y: f32, z: f32| root * Affine3A::from_translation(Vec3::new(x, y, z));

    let mut b = Builder::default();
    // 脚（左右逆に振る）と靴
    for (side, angle) in [(1.0f32, leg), (-1.0, -leg)] {
        let hip = at(0.0, HIP, side * 0.09) * Affine3A::from_rotation_z(angle);
        let thigh = HIP - 0.08;
        b.cuboid(
            hip * Affine3A::from_translation(Vec3::new(0.0, -thigh * 0.5, 0.0)),
            Vec3::new(0.07, thigh * 0.5, 0.065),
            0.8,
            1.0,
            pants,
        );
        b.cuboid(
            hip * Affine3A::from_translation(Vec3::new(0.035, -HIP + 0.04, 0.0)),
            Vec3::new(0.12, 0.04, 0.055),
            1.0,
            1.0,
            shoes,
        );
    }
    // 腰
    b.cuboid(at(0.0, HIP + 0.02, 0.0), Vec3::new(0.10, 0.08, 0.165), 1.0, 1.0, pants);
    // 胴（肩に向かって少し広がる）
    b.cuboid(at(0.0, 1.14, 0.0), Vec3::new(0.11, 0.28, 0.19), 0.85, 1.0, top);
    // 腕（脚と逆に振る）と手
    for (side, angle) in [(1.0f32, -leg * 0.7), (-1.0, leg * 0.7)] {
        let sh = at(0.0, 1.37, side * 0.235) * Affine3A::from_rotation_z(angle);
        b.cuboid(
            sh * Affine3A::from_translation(Vec3::new(0.0, -0.25, 0.0)),
            Vec3::new(0.055, 0.25, 0.05),
            0.85,
            1.0,
            top,
        );
        b.cuboid(
            sh * Affine3A::from_translation(Vec3::new(0.0, -0.55, 0.0)),
            Vec3::new(0.045, 0.05, 0.035),
            1.0,
            1.0,
            skin,
        );
    }
    // 首と頭
    b.cuboid(at(0.0, 1.45, 0.0), Vec3::new(0.045, 0.04, 0.05), 1.0, 1.0, skin);
    let head_c = root.transform_point3(Vec3::new(0.01, 1.585, 0.0));
    b.head(head_c, 0.105, skin, hair, lower.long_hair);
    if lower.long_hair {
        // 肩にかかる後ろ髪
        b.cuboid(at(-0.07, 1.47, 0.0), Vec3::new(0.03, 0.09, 0.09), 1.0, 1.0, hair);
    }
    // 踏み出した足のかかとが地面に沈まないよう、いちばん低い点を地面にそろえる
    let min_y = b.pos.iter().map(|p| p[1]).fold(f32::MAX, f32::min);
    for p in &mut b.pos {
        p[1] -= min_y;
    }
    b.build()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn person_mesh_stands_on_ground() {
        for phase in 0..PHASES {
            let m = build(Color::WHITE, &LOWERS[1], phase);
            let pos = m.attribute(Mesh::ATTRIBUTE_POSITION).unwrap().as_float3().unwrap();
            let min_y = pos.iter().map(|p| p[1]).fold(f32::MAX, f32::min);
            let max_y = pos.iter().map(|p| p[1]).fold(f32::MIN, f32::max);
            assert!(min_y.abs() < 1e-4, "phase {phase}: 足が地面から {min_y} m");
            assert!((1.6..1.8).contains(&max_y), "phase {phase}: 身長 {max_y} m");
            assert!(pos.len() < 400, "頂点数 {}", pos.len());
        }
    }

    #[test]
    fn gait_advances_with_distance_and_rests_when_stopped() {
        let mut g = Gait::new(Vec2::ZERO, Vec2::X, 0);
        g.phase = 0.0;
        let mut p = Vec2::ZERO;
        for _ in 0..10 {
            p.x += 0.07;
            g.advance(p, Vec2::new(1.4, 0.0));
        }
        assert!(g.phase > 0.2 && g.phase < 0.6, "phase {}", g.phase);
        for _ in 0..40 {
            g.advance(p, Vec2::ZERO);
        }
        assert!((g.phase * 2.0 - (g.phase * 2.0).round()).abs() < 1e-4);
        // 北向きに歩くと向きが +90° に寄る
        for _ in 0..40 {
            p.y += 0.07;
            g.advance(p, Vec2::new(0.0, 1.4));
        }
        assert!((g.heading - std::f32::consts::FRAC_PI_2).abs() < 0.05);
    }
}
