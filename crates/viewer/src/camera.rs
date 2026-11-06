//! マウス操作のオービットカメラ。

use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll, MouseScrollUnit};
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
    mut q: Query<(&mut OrbitCamera, &mut Transform)>,
) {
    for (mut cam, mut tf) in &mut q {
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
