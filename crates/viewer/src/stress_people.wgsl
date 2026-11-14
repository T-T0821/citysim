// 合成の街の大人数（stress.rs）の描画。位置は GPU 上の状態テクスチャから直接読むので、
// CPU から毎フレーム送るデータは無い。100 万人を描くため、人は箱 1 つ（12 三角形）で表し、
// 高さで塗り分けて靴・ズボン・上着・顔・髪に見せる。

#import bevy_pbr::mesh_view_bindings::{view, lights}
#ifdef TONEMAP_IN_SHADER
#import bevy_core_pipeline::tonemapping::tone_mapping
#endif

struct PeopleUniform {
    size: f32,
    n: u32,
    _pad0: u32,
    _pad1: u32,
};

@group(2) @binding(0) var<uniform> pu: PeopleUniform;
@group(2) @binding(1) var state_tex: texture_2d<f32>;
@group(2) @binding(2) var attr_tex: texture_2d<f32>;

const TEX: u32 = 1024u;
const PI: f32 = 3.14159265;

struct Vertex {
    @builtin(instance_index) instance: u32,
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) world_normal: vec3<f32>,
    @location(1) height: f32,
    @location(2) top: vec3<f32>,
    @location(3) lower: vec3<f32>,
};

fn hash(x: u32) -> u32 {
    let s = x * 747796405u + 2891336453u;
    let w = ((s >> ((s >> 28u) + 4u)) ^ s) * 277803737u;
    return (w >> 22u) ^ w;
}

fn palette_top(k: u32) -> vec3<f32> {
    switch k % 8u {
        case 0u: { return vec3<f32>(0.80, 0.80, 0.78); }
        case 1u: { return vec3<f32>(0.05, 0.05, 0.06); }
        case 2u: { return vec3<f32>(0.08, 0.14, 0.35); }
        case 3u: { return vec3<f32>(0.45, 0.06, 0.06); }
        case 4u: { return vec3<f32>(0.30, 0.30, 0.32); }
        case 5u: { return vec3<f32>(0.40, 0.28, 0.15); }
        case 6u: { return vec3<f32>(0.10, 0.30, 0.18); }
        default: { return vec3<f32>(0.60, 0.50, 0.30); }
    }
}

fn palette_lower(k: u32) -> vec3<f32> {
    switch k % 4u {
        case 0u: { return vec3<f32>(0.04, 0.05, 0.10); }
        case 1u: { return vec3<f32>(0.10, 0.10, 0.10); }
        case 2u: { return vec3<f32>(0.25, 0.22, 0.18); }
        default: { return vec3<f32>(0.12, 0.16, 0.28); }
    }
}

@vertex
fn vertex(v: Vertex) -> VertexOutput {
    var out: VertexOutput;
    if v.instance >= pu.n {
        out.clip_position = vec4<f32>(2.0, 2.0, 2.0, 1.0);
        return out;
    }
    let t = vec2<i32>(i32(v.instance % TEX), i32(v.instance / TEX));
    let st = textureLoad(state_tex, t, 0);
    let attr = textureLoad(attr_tex, t, 0);
    let h = hash(v.instance);
    let stature = 0.92 + 0.17 * f32(h & 255u) / 255.0;
    let width = 0.95 + 0.1 * f32((h >> 8u) & 255u) / 255.0;
    let sp = length(st.zw);
    // 止まっている人は人ごとの向き、歩いている人は進む向き
    var heading = attr.z * 2.0 * PI;
    if sp > 0.1 {
        heading = atan2(st.w, st.z);
    }
    let p = v.position * vec3<f32>(width, stature, width);
    let c = cos(heading);
    let s = sin(heading);
    // 平面 (x, y) → ワールド (x, -z)。街の中心をワールドの原点に置く
    let base = vec3<f32>(st.x - pu.size * 0.5, 0.0, -(st.y - pu.size * 0.5));
    let world = vec3<f32>(c * p.x + s * p.z, p.y, -s * p.x + c * p.z) + base;
    out.clip_position = view.clip_from_world * vec4<f32>(world, 1.0);
    out.world_normal = vec3<f32>(c * v.normal.x + s * v.normal.z, v.normal.y, -s * v.normal.x + c * v.normal.z);
    out.height = v.position.y / 1.7;
    out.top = palette_top(h >> 16u);
    out.lower = palette_lower(h >> 20u);
    return out;
}

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    // 箱の高さ 0..1 を部位に塗り分ける
    var color = in.top;
    if in.height < 0.04 {
        color = vec3<f32>(0.03, 0.03, 0.03);
    } else if in.height < 0.5 {
        color = in.lower;
    } else if in.height > 0.95 {
        color = vec3<f32>(0.03, 0.02, 0.02);
    } else if in.height > 0.84 {
        color = vec3<f32>(0.75, 0.55, 0.42);
    }
    let n = normalize(in.world_normal);
    var light = lights.ambient_color.rgb;
    if lights.n_directional_lights > 0u {
        let sun = lights.directional_lights[0];
        light += sun.color.rgb * max(dot(n, sun.direction_to_light), 0.0) / PI;
    }
    var out = vec4<f32>(color * light * view.exposure, 1.0);
#ifdef TONEMAP_IN_SHADER
    out = tone_mapping(out, view.color_grading);
#endif
    return out;
}
