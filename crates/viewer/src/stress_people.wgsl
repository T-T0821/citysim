// 合成の街の大人数（stress.rs）の描画。位置・歩行の位相・向きは GPU 上のテクスチャから直接読むので、
// CPU から毎フレーム送るデータは無い。銀座のデモ（crowd_render.rs）と同じ人型・同じ色分けで、
// 遠さで描き分ける:
//   NEAR:    画面上で身長 24px 以上に見える範囲。銀座と同じ歩く人型（person::build_rigged を
//            歩行の位相に応じてシェーダーで手足を回す）
//   MID:     その外側。銀座の遠景と同じ箱 5 つの簡易な人型（person::build_far）
//   DISTANT: さらに外側（身長数ピクセル以下）。箱 1 つを高さで塗り分ける（100 万人ぶんの頂点処理を抑えるため）
// NEAR / MID は近傍表（2m マスごとに最大 8 人）をカメラの周りのマスだけ引いて描き、
// DISTANT は全員を描いて、NEAR / MID で描いた人は潰す。
// 先頭に LOWER_PANTS などの表（person::lowers_wgsl）と TOPS（person::tops_wgsl）、
// 関節の定数（person::rig_wgsl）が差し込まれる。

#import bevy_pbr::mesh_view_bindings::{view, lights}
#ifdef TONEMAP_IN_SHADER
#import bevy_core_pipeline::tonemapping::tone_mapping
#endif

struct PeopleUniform {
    size: f32,
    n: u32,
    // カメラの下のマスと、NEAR / MID の範囲（マス数の半分）
    cam_x: i32,
    cam_y: i32,
    near_half: i32,
    mid_half: i32,
    _pad0: u32,
    _pad1: u32,
};

@group(2) @binding(0) var<uniform> pu: PeopleUniform;
@group(2) @binding(1) var state_tex: texture_2d<f32>;
@group(2) @binding(2) var look_tex: texture_2d<f32>;
@group(2) @binding(3) var aux_tex: texture_2d<f32>;
// 近傍表（4 人ずつ 2 枚）
@group(2) @binding(4) var packed0: texture_2d<u32>;
@group(2) @binding(5) var packed1: texture_2d<u32>;

const TEX: u32 = 1024u;
const GRID: i32 = 1024;
const PI: f32 = 3.14159265;

struct Vertex {
    @builtin(instance_index) instance: u32,
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
#ifndef DISTANT
    // r に部位の番号（person::part）、g に手足の番号（person::limb）
    @location(5) part: vec4<f32>,
#endif
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) world_normal: vec3<f32>,
    @location(1) color: vec3<f32>,
};

fn slot(k: u32, c: vec2<i32>) -> u32 {
    if k < 4u {
        return textureLoad(packed0, c, 0)[k];
    }
    return textureLoad(packed1, c, 0)[k - 4u];
}

fn in_square(c: vec2<i32>, half: i32) -> bool {
    return abs(c.x - pu.cam_x) <= half && abs(c.y - pu.cam_y) <= half;
}

fn hidden() -> VertexOutput {
    var out: VertexOutput;
    out.clip_position = vec4<f32>(2.0, 2.0, 2.0, 1.0);
    return out;
}

// y 軸まわりの回転（ローカル +x を向きの方向へ）
fn rot_y(v: vec3<f32>, c: f32, s: f32) -> vec3<f32> {
    return vec3<f32>(c * v.x + s * v.z, v.y, -s * v.x + c * v.z);
}

// z 軸まわりに pivot を中心に回す（手足の振り）
fn swing(p: vec3<f32>, pivot: vec3<f32>, a: f32) -> vec3<f32> {
    let d = p - pivot;
    let c = cos(a);
    let s = sin(a);
    return pivot + vec3<f32>(c * d.x - s * d.y, s * d.x + c * d.y, d.z);
}

@vertex
fn vertex(v: Vertex) -> VertexOutput {
    var id: u32;
#ifdef DISTANT
    id = v.instance;
    if id >= pu.n {
        return hidden();
    }
#else
    // インスタンス番号 → (カメラの周りの正方形の中のマス, そのマスの k 番目の人)
#ifdef NEAR
    let half = pu.near_half;
#else
    let half = pu.mid_half;
#endif
    let side = u32(2 * half + 1);
    let k = v.instance % 8u;
    let ci = v.instance / 8u;
    let cc = vec2<i32>(pu.cam_x - half + i32(ci % side), pu.cam_y - half + i32(ci / side));
    if cc.x < 0 || cc.y < 0 || cc.x >= GRID || cc.y >= GRID {
        return hidden();
    }
#ifdef MID
    if in_square(cc, pu.near_half) {
        return hidden();
    }
#endif
    let id1 = slot(k, cc);
    if id1 == 0u {
        return hidden();
    }
    id = id1 - 1u;
#endif
    let t = vec2<i32>(i32(id % TEX), i32(id / TEX));
    let aux = textureLoad(aux_tex, t, 0);
#ifdef DISTANT
    // 近傍表に入っていて NEAR / MID の範囲のマスにいる人は、そちらで描かれている
    let cell = u32(aux.w);
    let oc = vec2<i32>(i32(cell % u32(GRID)), i32(cell / u32(GRID)));
    if in_square(oc, pu.mid_half) {
        let p0 = textureLoad(packed0, oc, 0);
        let p1 = textureLoad(packed1, oc, 0);
        let me = vec4<u32>(id + 1u);
        if any(p0 == me) || any(p1 == me) {
            return hidden();
        }
    }
#endif
    let st = textureLoad(state_tex, t, 0);
    // person::look_of と同じ見た目（CPU で求めておいたもの）: 横幅, 身長の倍率, 上着, 下の組み合わせ
    let look = textureLoad(look_tex, t, 0);
    let top = u32(look.z + 0.5);
    let lower = u32(look.w + 0.5);

    var p = v.position;
    var n = v.normal;
#ifdef DISTANT
    // 箱 1 つ（高さ 0〜1.7m）を高さで部位に塗り分ける
    let hy = v.position.y / 1.7;
    var color = TOPS[top];
    if hy < 0.04 {
        color = LOWER_SHOES[lower];
    } else if hy < 0.5 {
        color = LOWER_PANTS[lower];
    } else if hy > 0.95 {
        color = LOWER_HAIR[lower];
    } else if hy > 0.84 {
        color = LOWER_SKIN[lower];
    }
#else
    let part = i32(v.part.r + 0.5);
    let long_hair = LOWER_LONG_HAIR[lower] > 0.5;
    var color = TOPS[top];
    switch part {
        case 1: { color = LOWER_PANTS[lower]; }
        case 2: { color = LOWER_SHOES[lower]; }
        case 3: { color = LOWER_SKIN[lower]; }
        case 4, 6: { color = LOWER_HAIR[lower]; }
        case 5: { color = select(LOWER_SKIN[lower], LOWER_HAIR[lower], long_hair); }
        default: {}
    }
    // 長髪でない人の後ろ髪は大きさ 0 に潰す
    if part == 6 && !long_hair {
        p = vec3<f32>(0.0);
    }
#endif
#ifdef NEAR
    // 歩行の位相に応じて手足を振る（person::build と同じ角度）
    let leg = sin(2.0 * PI * aux.x) * LEG_SWING;
    let limb = i32(v.part.g + 0.5);
    var a = 0.0;
    var pivot = vec3<f32>(0.0, HIP, HIP_Z);
    switch limb {
        case 1: { a = leg; }
        case 2: { a = -leg; pivot.z = -HIP_Z; }
        case 3: { a = -leg * 0.7; pivot = vec3<f32>(0.0, SHOULDER_Y, SHOULDER_Z); }
        case 4: { a = leg * 0.7; pivot = vec3<f32>(0.0, SHOULDER_Y, -SHOULDER_Z); }
        default: {}
    }
    if limb > 0 {
        p = swing(p, pivot, a);
        n = swing(n, vec3<f32>(0.0), a);
    }
#endif
    let scale = vec3<f32>(look.x, look.y, look.x);
    p = p * scale;
    n = n / scale;
    let c = cos(aux.y);
    let s = sin(aux.y);
    // 平面 (x, y) → ワールド (x, -z)。街の中心をワールドの原点に置く
    let base = vec3<f32>(st.x - pu.size * 0.5, 0.0, -(st.y - pu.size * 0.5));
    var out: VertexOutput;
    out.clip_position = view.clip_from_world * vec4<f32>(rot_y(p, c, s) + base, 1.0);
    out.world_normal = rot_y(n, c, s);
    out.color = color;
    return out;
}

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    let n = normalize(in.world_normal);
    var light = lights.ambient_color.rgb;
    if lights.n_directional_lights > 0u {
        let sun = lights.directional_lights[0];
        light += sun.color.rgb * max(dot(n, sun.direction_to_light), 0.0) / PI;
    }
    var out = vec4<f32>(in.color * light * view.exposure, 1.0);
#ifdef TONEMAP_IN_SHADER
    out = tone_mapping(out, view.color_grading);
#endif
    return out;
}
