//! 災害避難モードと場所選択のウィンドウ、避難所の配置（Shift + クリック）と表示。

use crate::camera::UiCapture;
use crate::geometry::to_world;
use crate::place::PlaceUi;
use crate::SimState;
use anyhow::{bail, Result};
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy_egui::{egui, EguiContexts};
use crowd::evacuation::{EvacParams, Evacuation};
use walkmap::WalkGrid;

/// "緯度,経度[,名前]"
pub fn parse_shelter(s: &str) -> Result<(String, f64, f64)> {
    let t: Vec<&str> = s.splitn(3, ',').collect();
    if t.len() < 2 {
        bail!("--shelter は 緯度,経度[,名前]: {s}");
    }
    let (lat, lon): (f64, f64) = (t[0].trim().parse()?, t[1].trim().parse()?);
    let name = t.get(2).map_or_else(
        || format!("避難所({lat:.5},{lon:.5})"),
        |n| n.trim().to_string(),
    );
    Ok((name, lat, lon))
}

pub fn geo_to_local(g: &WalkGrid, lat: f64, lon: f64) -> Vec2 {
    g.projection
        .to_local(&plateau_gml::GeoPoint { lat, lon, h: 0.0 })
}

/// 避難パネルの入力欄など。
#[derive(Resource, Default)]
pub struct EvacUi {
    pub input: String,
    pub message: String,
    pub speed_factor: Option<f32>,
}

/// 避難を始める（車は止め、全員を最寄りの避難所へ向かわせる）。
pub fn start(st: &mut SimState, speed_factor: f32) -> Result<(), String> {
    if st.shelters.is_empty() {
        return Err("避難所がありません".into());
    }
    let p = EvacParams {
        speed_factor,
        ..Default::default()
    };
    let ev = Evacuation::start(&mut st.sim, &st.shelters, &p)
        .ok_or("避難所の近くに歩行空間がありません（範囲外か建物の中）")?;
    info!(
        "避難開始: t={:.1}s, 対象 {} 人, 避難所 {} か所",
        st.sim.time,
        ev.total,
        ev.shelters.len()
    );
    if let Some(t) = st.traffic.as_mut() {
        t.halted = true;
    }
    st.evac = Some(ev);
    st.evac_at = None;
    Ok(())
}

/// シミュレーションを 1 ステップ進めた後に呼ぶ。
pub fn advance(st: &mut SimState) {
    if let Some(at) = st.evac_at {
        if st.sim.time + 1e-4 >= at {
            if let Err(e) = start(st, EvacParams::default().speed_factor) {
                warn!("{e}");
                st.evac_at = None;
            }
        }
    }
    let Some(ev) = st.evac.as_mut() else { return };
    // 車が止まりきったら、止まった車を避ける経路に引き直す
    if let (false, Some(t)) = (ev.rerouted, &st.traffic) {
        if t.vehicles.iter().all(|v| !v.active || v.v == 0.0) {
            ev.reroute_around(&mut st.sim, &t.obstacles());
        }
    }
    ev.update(&st.sim);
}

/// 3D 表示の右下の「場所・避難」ウィンドウ。
pub fn tools_window(
    mut contexts: EguiContexts,
    mut state: ResMut<SimState>,
    mut ui_state: ResMut<EvacUi>,
    mut place: ResMut<PlaceUi>,
    mut settings: ResMut<crate::ui::Settings>,
    mut exit: EventWriter<AppExit>,
) {
    let Some(ctx) = contexts.try_ctx_mut() else {
        return;
    };
    let ctx = ctx.clone();
    egui::Window::new("場所・避難")
        // 左上はシナリオのウィンドウ、右は設定パネルなので、3D 表示の右下に置く
        .anchor(egui::Align2::RIGHT_BOTTOM, [-350.0, -10.0])
        .default_width(300.0)
        .resizable(false)
        .collapsible(true)
        .show(&ctx, |ui| {
            egui::CollapsingHeader::new("場所を選ぶ")
                .default_open(false)
                .show(ui, |ui| {
                    if crate::place::panel(ui, &mut place) {
                        exit.write(AppExit::Success);
                    }
                });
            egui::CollapsingHeader::new("災害避難モード")
                .default_open(true)
                .show(ui, |ui| {
                    evac_panel(ui, &mut state, &mut ui_state, &mut settings)
                });
        });
}

fn evac_panel(
    ui: &mut egui::Ui,
    state: &mut SimState,
    ui_state: &mut EvacUi,
    settings: &mut crate::ui::Settings,
) {
    let st = state;
    ui.label("避難所");
    let mut remove = None;
    let running = st.evac.is_some();
    for (k, (name, pos)) in st.shelters.iter().enumerate() {
        let (lat, lon) = st.sim.grid.projection.to_geo(*pos);
        ui.horizontal(|ui| {
            if !running && ui.small_button("✕").clicked() {
                remove = Some(k);
            }
            ui.label(format!("{name}  ({lat:.5}, {lon:.5})"));
        });
    }
    if let Some(k) = remove {
        st.shelters.remove(k);
    }
    if st.shelters.is_empty() {
        ui.weak("（未設定）");
    }
    if !running {
        ui.horizontal(|ui| {
            ui.add(
                egui::TextEdit::singleline(&mut ui_state.input)
                    .hint_text("緯度,経度[,名前]")
                    .desired_width(190.0),
            );
            if ui.button("追加").clicked() {
                match parse_shelter(&ui_state.input) {
                    Ok((n, lat, lon)) => {
                        let p = geo_to_local(&st.sim.grid, lat, lon);
                        st.shelters.push((n, p));
                        ui_state.input.clear();
                        ui_state.message.clear();
                    }
                    Err(e) => ui_state.message = format!("{e}"),
                }
            }
        });
        ui.small("Shift + 左クリックで地図上に追加できます");
    }
    let factor = ui_state
        .speed_factor
        .get_or_insert(EvacParams::default().speed_factor);
    ui.add_enabled(
        !running,
        egui::Slider::new(factor, 1.0..=1.6).text("歩行速度の倍率"),
    );
    let factor = *factor;
    ui.horizontal(|ui| {
        if !running {
            if ui
                .add_enabled(!st.shelters.is_empty(), egui::Button::new("🚨 避難開始"))
                .clicked()
            {
                ui_state.message = match start(st, factor) {
                    Ok(()) => String::new(),
                    Err(e) => e,
                };
            }
        } else if ui.button("通常に戻す（最初から）").clicked() {
            settings.apply_requested = true;
        }
    });
    if !ui_state.message.is_empty() {
        ui.colored_label(egui::Color32::LIGHT_RED, &ui_state.message);
    }
    let Some(ev) = &st.evac else {
        ui.small("開始すると車は左に寄せて止まり、全員が最寄りの避難所へ向かいます（信号は無視、車道も歩く）。");
        return;
    };
    let fmt = |t: Option<f32>| t.map_or("—".to_string(), |t| format!("{t:.0} 秒"));
    let target = ev.total - ev.unreachable;
    egui::Grid::new("evac_status")
        .num_columns(2)
        .spacing([8.0, 3.0])
        .show(ui, |ui| {
            ui.label("経過");
            ui.label(format!("{:.0} 秒", st.sim.time - ev.start_time));
            ui.end_row();
            ui.label("避難完了");
            ui.label(format!(
                "{} / {} 人 ({:.0}%)",
                ev.evacuated(),
                target,
                100.0 * ev.evacuated() as f32 / target.max(1) as f32
            ));
            ui.end_row();
            if ev.unreachable > 0 {
                ui.label("到達できない");
                ui.label(format!("{} 人", ev.unreachable));
                ui.end_row();
            }
            ui.label("50% / 90% / 全員");
            ui.label(format!(
                "{} / {} / {}",
                fmt(ev.time_to_fraction(0.5)),
                fmt(ev.time_to_fraction(0.9)),
                fmt(ev.time_to_fraction(1.0))
            ));
            ui.end_row();
            for s in &ev.shelters {
                ui.label(&s.name);
                ui.label(format!("{} 人", s.arrived));
                ui.end_row();
            }
        });
}

/// Shift + 左クリックした地面の位置に避難所を追加する。
pub fn pick_shelter(
    buttons: Res<ButtonInput<MouseButton>>,
    keys: Res<ButtonInput<KeyCode>>,
    capture: Res<UiCapture>,
    windows: Query<&Window, With<PrimaryWindow>>,
    cams: Query<(&Camera, &GlobalTransform)>,
    mut state: ResMut<SimState>,
) {
    if capture.0
        || state.evac.is_some()
        || !buttons.just_pressed(MouseButton::Left)
        || !keys.any_pressed([KeyCode::ShiftLeft, KeyCode::ShiftRight])
    {
        return;
    }
    let Some(cursor) = windows.iter().next().and_then(|w| w.cursor_position()) else {
        return;
    };
    let Some((cam, gt)) = cams.iter().next() else {
        return;
    };
    let Ok(ray) = cam.viewport_to_world(gt, cursor) else {
        return;
    };
    if ray.direction.y.abs() < 1e-4 {
        return;
    }
    let t = -ray.origin.y / ray.direction.y;
    if t <= 0.0 {
        return;
    }
    let p = ray.origin + *ray.direction * t;
    let local = Vec2::new(p.x, -p.z);
    let n = state.shelters.len() + 1;
    state.shelters.push((format!("避難所{n}"), local));
}

#[derive(Component)]
pub struct ShelterMarker;

/// 避難所の目印（緑の柱と受け入れ範囲の半透明の円柱）を避難所の一覧に合わせて置き直す。
pub fn sync_markers(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    state: Res<SimState>,
    markers: Query<Entity, With<ShelterMarker>>,
    mut last: Local<Vec<Vec2>>,
    mut mats: Local<Option<(Handle<StandardMaterial>, Handle<StandardMaterial>)>>,
) {
    let now: Vec<Vec2> = state.shelters.iter().map(|s| s.1).collect();
    if *last == now {
        return;
    }
    *last = now.clone();
    for e in &markers {
        commands.entity(e).despawn();
    }
    let (pole, disk) = mats
        .get_or_insert_with(|| {
            (
                materials.add(StandardMaterial {
                    base_color: Color::srgb(0.1, 0.85, 0.3),
                    emissive: LinearRgba::rgb(0.0, 0.6, 0.15),
                    ..default()
                }),
                materials.add(StandardMaterial {
                    base_color: Color::srgba(0.1, 0.9, 0.3, 0.28),
                    alpha_mode: AlphaMode::Blend,
                    unlit: true,
                    ..default()
                }),
            )
        })
        .clone();
    let r = EvacParams::default().radius;
    for (name, p) in &state.shelters {
        debug!("避難所 {name}: 局所座標 ({:.1}, {:.1})", p.x, p.y);
    }
    for p in now {
        commands.spawn((
            ShelterMarker,
            Mesh3d(meshes.add(Cylinder::new(1.2, 30.0))),
            MeshMaterial3d(pole.clone()),
            Transform::from_translation(to_world(p, 15.0)),
        ));
        // 受け入れ範囲を半透明の円柱で示す（遠くからでも見える）
        commands.spawn((
            ShelterMarker,
            Mesh3d(meshes.add(Cylinder::new(r, 20.0))),
            MeshMaterial3d(disk.clone()),
            Transform::from_translation(to_world(p, 10.0)),
        ));
    }
}
