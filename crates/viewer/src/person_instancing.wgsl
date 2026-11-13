// 歩行者のインスタンス描画（crowd_render.rs）。
// 先頭に LOWER_PANTS などの色の表が person::lowers_wgsl() から差し込まれる。

#import bevy_pbr::mesh_view_bindings::{view, lights}
#ifdef TONEMAP_IN_SHADER
#import bevy_core_pipeline::tonemapping::tone_mapping
#endif

struct Vertex {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    // r に部位の番号（person::part）
    @location(5) part: vec4<f32>,
    @location(10) i_pos_heading: vec4<f32>,
    @location(11) i_scale_lower: vec4<f32>,
    @location(12) i_top: vec4<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) world_normal: vec3<f32>,
    @location(1) color: vec3<f32>,
};

const PI: f32 = 3.14159265;

@vertex
fn vertex(v: Vertex) -> VertexOutput {
    let part = i32(v.part.r + 0.5);
    let lower = i32(v.i_scale_lower.z + 0.5);
    let long_hair = LOWER_LONG_HAIR[lower] > 0.5;

    var color = v.i_top.rgb;
    switch part {
        case 1: { color = LOWER_PANTS[lower]; }
        case 2: { color = LOWER_SHOES[lower]; }
        case 3: { color = LOWER_SKIN[lower]; }
        case 4, 6: { color = LOWER_HAIR[lower]; }
        case 5: { color = select(LOWER_SKIN[lower], LOWER_HAIR[lower], long_hair); }
        default: {}
    }

    let scale = vec3<f32>(v.i_scale_lower.x, v.i_scale_lower.y, v.i_scale_lower.x);
    // 長髪でない人の後ろ髪は大きさ 0 に潰す
    let keep = select(1.0, 0.0, part == 6 && !long_hair);
    let p = v.position * scale * keep;
    let n = v.normal / scale;
    // y 軸まわりの回転（ローカル +x を向きの方向へ）
    let c = cos(v.i_pos_heading.w);
    let s = sin(v.i_pos_heading.w);
    let world = vec3<f32>(c * p.x + s * p.z, p.y, -s * p.x + c * p.z) + v.i_pos_heading.xyz;
    let wn = vec3<f32>(c * n.x + s * n.z, n.y, -s * n.x + c * n.z);

    var out: VertexOutput;
    out.clip_position = view.clip_from_world * vec4<f32>(world, 1.0);
    out.world_normal = wn;
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
