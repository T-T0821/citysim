// 合成の街での大人数シミュレーション（stress.rs）を GPU で 1 ステップ進める。
// WebGL2 でも動くよう、計算シェーダーは使わずに描画パスで行う:
//   1. density: 全員を 1 ピクセルの点として密度テクスチャ（1m 格子）に加算で描く
//   2. update:  状態テクスチャ（1 テクセル = 1 人: 位置 xy, 速度 zw）の全テクセルを
//               フラグメントシェーダーで計算して、もう一方の状態テクスチャへ書く
// 力の模型は crowd::Model::Lite と同じ（前方の密度で速さを落とす + 密度の勾配で押す + 壁）。

struct SimUniform {
    dt: f32,
    size: f32,
    n: u32,
    frame: u32,
    nbx: u32,
    nby: u32,
    nbands_x: u32,
    nbands_y: u32,
};

const TEX: u32 = 1024u;
const TAU: f32 = 0.5;
const A_WALL: f32 = 5.0;
const B_WALL: f32 = 0.2;
const K_BODY: f32 = 120.0;
const RADIUS: f32 = 0.25;
const PRESSURE: f32 = 1.2;
const RHO_MAX: f32 = 5.4;

@group(0) @binding(0) var<uniform> u: SimUniform;
@group(0) @binding(1) var state_src: texture_2d<f32>;

#ifdef UPDATE
@group(0) @binding(2) var attr_tex: texture_2d<f32>;
@group(0) @binding(3) var density_tex: texture_2d<f32>;
@group(0) @binding(4) var density_sampler: sampler;
@group(0) @binding(5) var axis_tex: texture_2d<f32>;
@group(0) @binding(6) var blocks_tex: texture_2d<f32>;
@group(0) @binding(7) var bands_tex: texture_2d<f32>;
#endif

fn texel_of(i: u32) -> vec2<i32> {
    return vec2<i32>(i32(i % TEX), i32(i / TEX));
}

// ---- 1. 密度 ----

struct PointOut {
    @builtin(position) pos: vec4<f32>,
};

@vertex
fn density_vertex(@builtin(vertex_index) i: u32) -> PointOut {
    var out: PointOut;
    if i >= u.n {
        out.pos = vec4<f32>(2.0, 2.0, 0.0, 1.0);
        return out;
    }
    let p = textureLoad(state_src, texel_of(i), 0).xy;
    // テクスチャの行 r が y = r..r+1 [m]（行 0 = クリップ空間の上端）
    out.pos = vec4<f32>(p.x / u.size * 2.0 - 1.0, 1.0 - p.y / u.size * 2.0, 0.0, 1.0);
    return out;
}

@fragment
fn density_fragment() -> @location(0) vec4<f32> {
    // 1m 格子なので 1 人 = 1 人/m²
    return vec4<f32>(1.0, 0.0, 0.0, 0.0);
}

// ---- 2. 更新 ----

struct FullOut {
    @builtin(position) pos: vec4<f32>,
};

@vertex
fn fullscreen_vertex(@builtin(vertex_index) i: u32) -> FullOut {
    let uv = vec2<f32>(f32((i << 1u) & 2u), f32(i & 2u));
    var out: FullOut;
    out.pos = vec4<f32>(uv * 2.0 - 1.0, 0.0, 1.0);
    return out;
}

#ifdef UPDATE

// (種類 0 = 街路の帯 / 1 = 街区 / 2 = 外, 番号, 始まり, 終わり)
fn axis_at(v: f32, row: i32) -> vec4<f32> {
    let n = i32(u.size);
    if v < 0.0 || v >= u.size {
        return vec4<f32>(2.0, 0.0, 0.0, 0.0);
    }
    return textureLoad(axis_tex, vec2<i32>(clamp(i32(floor(v)), 0, n - 1), row), 0);
}

fn is_building(ix: i32, iy: i32) -> bool {
    if ix < 0 || iy < 0 || ix >= i32(u.nbx) || iy >= i32(u.nby) {
        return false;
    }
    return textureLoad(blocks_tex, vec2<i32>(ix, iy), 0).r < 0.5;
}

fn walkable(p: vec2<f32>) -> bool {
    let ax = axis_at(p.x, 0);
    let ay = axis_at(p.y, 1);
    if ax.x > 1.5 || ay.x > 1.5 {
        return false;
    }
    if ax.x < 0.5 || ay.x < 0.5 {
        return true;
    }
    return !is_building(i32(ax.y), i32(ay.y));
}

// 最寄りの建物の面までの距離 (x) と、建物から離れる向き (yz)
fn wall_at(p: vec2<f32>) -> vec3<f32> {
    let ax = axis_at(p.x, 0);
    let ay = axis_at(p.y, 1);
    var best = vec3<f32>(1e9, 0.0, 0.0);
    let kx = i32(ax.y);
    let ky = i32(ay.y);
    if ax.x < 0.5 && ay.x < 0.5 {
        // 交差点: 4 つの角
        for (var c = 0; c < 4; c++) {
            let ix = kx - 1 + (c & 1);
            let iy = ky - 1 + (c >> 1);
            if is_building(ix, iy) {
                let corner = vec2<f32>(select(ax.z, ax.w, (c & 1) == 1), select(ay.z, ay.w, (c >> 1) == 1));
                let d = p - corner;
                let l = length(d);
                if l < best.x && l > 1e-4 {
                    best = vec3<f32>(l, d / l);
                }
            }
        }
    } else if ax.x < 0.5 {
        // 南北の街路: 左右の街区の面
        let iy = i32(ay.y);
        if is_building(kx - 1, iy) && p.x - ax.z < best.x {
            best = vec3<f32>(p.x - ax.z, 1.0, 0.0);
        }
        if is_building(kx, iy) && ax.w - p.x < best.x {
            best = vec3<f32>(ax.w - p.x, -1.0, 0.0);
        }
    } else if ay.x < 0.5 {
        let ix = i32(ax.y);
        if is_building(ix, ky - 1) && p.y - ay.z < best.x {
            best = vec3<f32>(p.y - ay.z, 0.0, 1.0);
        }
        if is_building(ix, ky) && ay.w - p.y < best.x {
            best = vec3<f32>(ay.w - p.y, 0.0, -1.0);
        }
    }
    return best;
}

// 目的地（0 = 東端, 1 = 西端, 2 = 北端, 3 = 南端）へ向かう向き。
// 目的の向きに通る街路（か公園）にいればそのまま進み、交わる街路にいれば近い方の通りへ出る。
// 左側通行の癖として、進む向きの左へ少し寄る（すれ違う流れが分かれる）。
fn direction(p: vec2<f32>, goal: u32, seed: f32) -> vec2<f32> {
    let ax = axis_at(p.x, 0);
    let ay = axis_at(p.y, 1);
    let in_park = ax.x > 0.5 && ay.x > 0.5 && ax.x < 1.5 && ay.x < 1.5;
    if goal < 2u {
        let s = select(-1.0, 1.0, goal == 0u);
        if ay.x < 0.5 || in_park {
            return normalize(vec2<f32>(s, 0.15 * s));
        }
        // 南北の街路の中: 近い方の東西の通りへ（真ん中付近は人ごとに分ける）
        let lo = p.y - ay.z;
        let hi = ay.w - p.y;
        let up = hi + (seed - 0.5) * 20.0 < lo;
        return vec2<f32>(0.0, select(-1.0, 1.0, up));
    } else {
        let s = select(-1.0, 1.0, goal == 2u);
        if ax.x < 0.5 || in_park {
            return normalize(vec2<f32>(-0.15 * s, s));
        }
        let lo = p.x - ax.z;
        let hi = ax.w - p.x;
        let right = hi + (seed - 0.5) * 20.0 < lo;
        return vec2<f32>(select(-1.0, 1.0, right), 0.0);
    }
}

fn arrived(p: vec2<f32>, goal: u32) -> bool {
    switch goal {
        case 0u: { return p.x > u.size - 1.5; }
        case 1u: { return p.x < 1.5; }
        case 2u: { return p.y > u.size - 1.5; }
        default: { return p.y < 1.5; }
    }
}

fn hash(x: u32) -> u32 {
    // PCG
    let s = x * 747796405u + 2891336453u;
    let w = ((s >> ((s >> 28u) + 4u)) ^ s) * 277803737u;
    return (w >> 22u) ^ w;
}

fn rand01(x: u32) -> f32 {
    return f32(hash(x) & 0xFFFFFFu) / 16777216.0;
}

// 目的地以外の 3 つの端のどれかの街路の入口に置き直す
fn respawn(i: u32, goal: u32) -> vec2<f32> {
    let h = hash(i * 9781u + u.frame * 6271u);
    var edge = h % 3u;
    if edge >= goal {
        edge += 1u;
    }
    let r = rand01(h ^ 0x5bd1e995u);
    if edge < 2u {
        // 東端か西端: 東西の通りのどれか
        let k = i32(hash(h + 1u) % u.nbands_y);
        let b = textureLoad(bands_tex, vec2<i32>(k, 1), 0);
        let y = mix(b.x + 0.5, b.y - 0.5, r);
        return vec2<f32>(select(1.0, u.size - 1.0, edge == 0u), y);
    }
    let k = i32(hash(h + 1u) % u.nbands_x);
    let b = textureLoad(bands_tex, vec2<i32>(k, 0), 0);
    let x = mix(b.x + 0.5, b.y - 0.5, r);
    return vec2<f32>(x, select(1.0, u.size - 1.0, edge == 2u));
}

fn density(p: vec2<f32>) -> f32 {
    return textureSampleLevel(density_tex, density_sampler, p / u.size, 0.0).r;
}

fn weidmann(rho: f32) -> f32 {
    if rho <= 0.05 {
        return 1.0;
    }
    if rho >= RHO_MAX {
        return 0.05;
    }
    return max(1.0 - exp(-1.913 * (1.0 / rho - 1.0 / RHO_MAX)), 0.05);
}

@fragment
fn update_fragment(in: FullOut) -> @location(0) vec4<f32> {
    let t = vec2<i32>(floor(in.pos.xy));
    let i = u32(t.y) * TEX + u32(t.x);
    let st = textureLoad(state_src, t, 0);
    if i >= u.n {
        return st;
    }
    let attr = textureLoad(attr_tex, t, 0);
    let goal = u32(attr.x + 0.5);
    let v0 = attr.y;
    var p = st.xy;
    var vel = st.zw;
    if arrived(p, goal) || !walkable(p) {
        return vec4<f32>(respawn(i, goal), 0.0, 0.0);
    }
    let e = direction(p, goal, attr.z);
    let speed = v0 * weidmann(density(p + e));
    var f = (e * speed - vel) / TAU;
    let g = vec2<f32>(
        density(p + vec2<f32>(0.5, 0.0)) - density(p - vec2<f32>(0.5, 0.0)),
        density(p + vec2<f32>(0.0, 0.5)) - density(p - vec2<f32>(0.0, 0.5)),
    );
    f -= g * PRESSURE;
    let w = wall_at(p);
    if w.x < RADIUS + 1.5 {
        let overlap = RADIUS - w.x;
        f += w.yz * (A_WALL * exp(overlap / B_WALL) + K_BODY * max(overlap, 0.0));
    }
    vel += f * u.dt;
    let vmax = v0 * 1.3;
    let sp = length(vel);
    if sp > vmax {
        vel *= vmax / sp;
    }
    let next = p + vel * u.dt;
    if walkable(next) {
        p = next;
    } else if walkable(vec2<f32>(next.x, p.y)) {
        p = vec2<f32>(next.x, p.y);
        vel.y = 0.0;
    } else if walkable(vec2<f32>(p.x, next.y)) {
        p = vec2<f32>(p.x, next.y);
        vel.x = 0.0;
    } else {
        vel = vec2<f32>(0.0);
    }
    return vec4<f32>(p, vel);
}

#endif
