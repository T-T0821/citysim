//! マウス・タッチ操作のオービットカメラ。

use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll, MouseScrollUnit};
use bevy::input::touch::Touches;
use bevy::platform::collections::HashSet;
use bevy::prelude::*;

#[derive(Component)]
pub struct OrbitCamera {
    pub target: Vec3,
    pub yaw: f32,
    pub pitch: f32,
    pub distance: f32,
}

/// UI がマウスを使っている間はカメラ操作を止める
#[derive(Resource, Default)]
pub struct UiCapture(pub bool);

pub fn orbit_camera(
    ui: Res<UiCapture>,
    buttons: Res<ButtonInput<MouseButton>>,
    keys: Res<ButtonInput<KeyCode>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    touches: Res<Touches>,
    // UI の上で始まったタッチ（パネルのスクロールなど）はカメラ操作に使わない
    mut on_ui: Local<HashSet<u64>>,
    mut q: Query<(&mut OrbitCamera, &mut Transform)>,
) {
    for t in touches.iter_just_pressed() {
        if ui.0 {
            on_ui.insert(t.id());
        }
    }
    on_ui.retain(|id| touches.get_pressed(*id).is_some());
    let fingers: Vec<_> = touches
        .iter()
        .filter(|t| !on_ui.contains(&t.id()))
        .collect();
    for (mut cam, mut tf) in &mut q {
        match fingers[..] {
            // 1 本指: 回転
            [a] => {
                let d = a.delta();
                cam.yaw -= d.x * 0.008;
                cam.pitch = (cam.pitch + d.y * 0.008).clamp(0.05, 1.55);
            }
            // 2 本指: 中点の移動で平行移動、指の間隔でズーム
            [a, b, ..] => {
                let prev = a.previous_position().distance(b.previous_position());
                let now = a.position().distance(b.position());
                if prev > 1.0 && now > 1.0 {
                    cam.distance = (cam.distance * prev / now).clamp(5.0, 20_000.0);
                }
                let d = (a.delta() + b.delta()) * 0.5;
                let right = Vec3::new(cam.yaw.cos(), 0.0, -cam.yaw.sin());
                let fwd = Vec3::new(-cam.yaw.sin(), 0.0, -cam.yaw.cos());
                let k = cam.distance * 0.0025;
                cam.target += (-right * d.x + fwd * d.y) * k;
            }
            _ => {}
        }
        let d = if ui.0 { Vec2::ZERO } else { motion.delta };
        // Shift + 左クリックは避難所の追加に使うので回転しない
        let shift = keys.any_pressed([KeyCode::ShiftLeft, KeyCode::ShiftRight]);
        if buttons.pressed(MouseButton::Left) && !shift {
            cam.yaw -= d.x * 0.005;
            cam.pitch = (cam.pitch + d.y * 0.005).clamp(0.05, 1.55);
        }
        if buttons.pressed(MouseButton::Right) || buttons.pressed(MouseButton::Middle) {
            // 画面上の移動量を地面上の移動量に変換
            let right = Vec3::new(cam.yaw.cos(), 0.0, -cam.yaw.sin());
            let fwd = Vec3::new(-cam.yaw.sin(), 0.0, -cam.yaw.cos());
            let k = cam.distance * 0.0015;
            cam.target += (-right * d.x + fwd * d.y) * k;
        }
        let s = if ui.0 {
            0.0
        } else {
            match scroll.unit {
                MouseScrollUnit::Line => scroll.delta.y,
                MouseScrollUnit::Pixel => scroll.delta.y / 40.0,
            }
        };
        if s != 0.0 {
            cam.distance = (cam.distance * (1.0 - s * 0.1)).clamp(5.0, 20_000.0);
        }
        let offset = Vec3::new(
            cam.pitch.cos() * cam.yaw.sin(),
            cam.pitch.sin(),
            cam.pitch.cos() * cam.yaw.cos(),
        ) * cam.distance;
        *tf = Transform::from_translation(cam.target + offset).looking_at(cam.target, Vec3::Y);
    }
}
