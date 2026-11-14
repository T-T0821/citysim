//! egui による設定パネル。

use crate::camera::UiCapture;
use crate::SimState;
use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};

/// 作り直しに使う設定（「適用」で `SimState` に反映される）。
#[derive(Resource, Clone, Debug)]
pub struct Settings {
    pub agents: usize,
    pub vehicles: usize,
    /// 平均歩行速度 [m/s]
    pub walk_speed: f32,
    /// 車の制限速度 [km/h]
    pub speed_limit_kmh: f32,
    pub respawn: bool,
    pub seed: u64,
    /// 信号機を置くか（オフなら交差点は先着順）
    pub signals: bool,
    pub signal: traffic::SignalConfig,
    /// 歩行者の上着を目的地ごとの色にする（オフなら普段の服の色）
    pub color_by_goal: bool,
    /// 「適用」が押された
    pub apply_requested: bool,
}

impl Settings {
    pub fn new(agents: usize, vehicles: usize, seed: u64, respawn: bool) -> Self {
        Self {
            agents,
            vehicles,
            walk_speed: 1.34,
            speed_limit_kmh: 47.0,
            respawn,
            seed,
            signals: true,
            signal: traffic::SignalConfig::default(),
            color_by_goal: false,
            apply_requested: false,
        }
    }
}

/// 日本語が表示できるフォントを探して egui に登録する。
/// `PLATEAU_UI_FONT` 環境変数か `--ui-font` で指定したものを優先し、無ければ OS の定番フォントを探す。
pub fn install_fonts(ctx: &egui::Context, explicit: Option<&std::path::Path>) {
    let candidates: Vec<std::path::PathBuf> = explicit
        .map(|p| p.to_path_buf())
        .into_iter()
        .chain(std::env::var_os("PLATEAU_UI_FONT").map(Into::into))
        .chain(
            [
                // Linux
                "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
                "/usr/share/fonts/opentype/noto/NotoSansCJKjp-Regular.otf",
                "/usr/share/fonts/truetype/noto/NotoSansCJKjp-Regular.otf",
                "/usr/share/fonts/opentype/ipafont-gothic/ipag.ttf",
                "/usr/share/fonts/truetype/fonts-japanese-gothic.ttf",
                "/usr/share/fonts/opentype/ipaexfont-gothic/ipaexg.ttf",
                // macOS
                "/System/Library/Fonts/ヒラギノ角ゴシック W3.ttc",
                "/System/Library/Fonts/Hiragino Sans GB.ttc",
                "/Library/Fonts/Arial Unicode.ttf",
                // Windows
                "C:\\Windows\\Fonts\\meiryo.ttc",
                "C:\\Windows\\Fonts\\YuGothM.ttc",
                "C:\\Windows\\Fonts\\msgothic.ttc",
            ]
            .iter()
            .map(Into::into),
        )
        .collect();
    // Web 版はファイルを読めないので同梱のフォントを使う
    #[cfg(target_arch = "wasm32")]
    let found = {
        let _ = candidates;
        Some((
            std::path::PathBuf::from("Noto Sans JP (同梱)"),
            crate::web::FONT.to_vec(),
        ))
    };
    #[cfg(not(target_arch = "wasm32"))]
    let found = candidates
        .iter()
        .find_map(|p| std::fs::read(p).ok().map(|b| (p.clone(), b)));
    let Some((path, bytes)) = found else {
        warn!("日本語フォントが見つかりません。パネルの日本語は表示されないことがあります (PLATEAU_UI_FONT で指定できます)");
        return;
    };
    info!("UI フォント: {}", path.display());
    let mut fonts = egui::FontDefinitions::default();
    fonts.font_data.insert(
        "jp".into(),
        std::sync::Arc::new(egui::FontData::from_owned(bytes)),
    );
    for family in [egui::FontFamily::Proportional, egui::FontFamily::Monospace] {
        fonts
            .families
            .entry(family)
            .or_default()
            .insert(0, "jp".into());
    }
    ctx.set_fonts(fonts);
}

/// スマホなど狭い画面向けのタブ（同時に開くのは 1 つだけ）
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Tab {
    Settings,
    Scenario,
    Tools,
}

/// 画面の幅に応じたレイアウト。
/// 幅が狭い（スマホの縦持ちなど）ときは、上のバーのボタンで開くシートに各パネルをまとめる。
#[derive(Resource, Default)]
pub struct Layout {
    pub compact: bool,
    pub tab: Option<Tab>,
    /// 広い画面に戻したときに元に戻すための egui の間隔設定
    saved_spacing: Option<egui::style::Spacing>,
}

/// これより狭い（egui の論理ピクセル）と狭い画面用のレイアウトにする
const COMPACT_WIDTH: f32 = 720.0;

impl Layout {
    /// 狭い画面でこのタブを表示中か
    pub fn shows(&self, tab: Tab) -> bool {
        self.compact && self.tab == Some(tab)
    }
}

/// 狭い画面では、パネルを画面下のシートとして出す（高さは画面の 45%、はみ出す分はスクロール）。
pub fn sheet(ctx: &egui::Context, id: &str, body: impl FnOnce(&mut egui::Ui)) {
    let h = (ctx.screen_rect().height() * 0.45).max(160.0);
    egui::TopBottomPanel::bottom(id.to_string())
        .resizable(false)
        .exact_height(h)
        .show(ctx, |ui| {
            egui::ScrollArea::vertical()
                .auto_shrink([false, false])
                .show(ui, |ui| {
                    ui.add_space(4.0);
                    body(ui);
                    // 下端はページの出典表示に重なるので空けておく
                    ui.add_space(24.0);
                });
        });
}

/// 狭い画面かどうかを判定し、狭い画面なら上のバー（再生と各パネルの開閉）を出す。
pub fn top_bar(
    mut contexts: EguiContexts,
    mut layout: ResMut<Layout>,
    mut state: ResMut<SimState>,
) {
    let Some(ctx) = contexts.try_ctx_mut() else {
        return;
    };
    let ctx = ctx.clone();
    let compact = ctx.screen_rect().width() < COMPACT_WIDTH;
    if compact != layout.compact {
        layout.compact = compact;
        if compact {
            // 指で押しやすいようにボタン・スライダーを大きくする
            layout.saved_spacing = Some(ctx.style().spacing.clone());
            ctx.all_styles_mut(|s| {
                let sp = &mut s.spacing;
                sp.interact_size.y = sp.interact_size.y.max(30.0);
                sp.button_padding = egui::vec2(10.0, 6.0);
                sp.item_spacing = egui::vec2(8.0, 8.0);
                sp.slider_width = 150.0;
                sp.icon_width = 20.0;
                sp.icon_width_inner = 12.0;
                sp.combo_height = 300.0;
            });
        } else if let Some(sp) = layout.saved_spacing.take() {
            ctx.all_styles_mut(|s| s.spacing = sp.clone());
        }
    }
    if !compact {
        return;
    }
    egui::TopBottomPanel::top("compact_bar").show(&ctx, |ui| {
        ui.horizontal_wrapped(|ui| {
            let label = if state.paused { "▶" } else { "⏸" };
            if ui.button(label).clicked() {
                state.paused = !state.paused;
            }
            for (tab, name) in [
                (Tab::Settings, "設定"),
                (Tab::Scenario, "シナリオ"),
                (Tab::Tools, "場所・避難"),
            ] {
                let on = layout.tab == Some(tab);
                if ui.selectable_label(on, name).clicked() {
                    layout.tab = if on { None } else { Some(tab) };
                }
            }
            ui.label(
                egui::RichText::new(format!(
                    "{:.0} s · 歩行 {}",
                    state.sim.time,
                    state.sim.active()
                ))
                .small(),
            );
        });
    });
}

pub fn settings_panel(
    mut contexts: EguiContexts,
    mut settings: ResMut<Settings>,
    mut state: ResMut<SimState>,
    mut capture: ResMut<UiCapture>,
    layout: Res<Layout>,
    mut overlay: Query<&mut Visibility, With<crate::HeatOverlay>>,
) {
    let Some(ctx) = contexts.try_ctx_mut() else {
        return;
    };
    let ctx = ctx.clone();

    let body = |ui: &mut egui::Ui| {
        ui.heading("シミュレーション設定");
        ui.add_space(4.0);
        egui::Grid::new("params")
            .num_columns(2)
            .spacing([8.0, 6.0])
            .show(ui, |ui| {
                ui.label("歩行者数");
                ui.add(egui::Slider::new(&mut settings.agents, 0..=100_000).logarithmic(true));
                ui.end_row();
                ui.label("車両数");
                ui.add(egui::Slider::new(&mut settings.vehicles, 0..=300));
                ui.end_row();
                ui.label("平均歩行速度");
                ui.add(egui::Slider::new(&mut settings.walk_speed, 0.6..=2.0).suffix(" m/s"));
                ui.end_row();
                ui.label("制限速度");
                ui.add(
                    egui::Slider::new(&mut settings.speed_limit_kmh, 10.0..=80.0).suffix(" km/h"),
                );
                ui.end_row();
                ui.label("乱数シード");
                ui.add(egui::DragValue::new(&mut settings.seed));
                ui.end_row();
                ui.label("到着後に再投入");
                ui.checkbox(&mut settings.respawn, "");
                ui.end_row();
            });
        ui.add_space(4.0);
        signal_settings(ui, &mut settings);
        ui.add_space(6.0);
        if ui
            .add_sized(
                [ui.available_width(), 28.0],
                egui::Button::new("適用して最初から"),
            )
            .clicked()
        {
            settings.apply_requested = true;
        }
        ui.small("人数・台数・速度・シード・信号は「適用」で作り直したときに反映されます。");

        ui.separator();
        ui.heading("再生");
        ui.horizontal(|ui| {
            let label = if state.paused {
                "▶ 再開"
            } else {
                "⏸ 一時停止"
            };
            if ui.button(label).clicked() {
                state.paused = !state.paused;
            }
            if ui.button("⟲ 時刻 0 に戻す").clicked() {
                settings.apply_requested = true;
            }
        });
        ui.horizontal(|ui| {
            ui.label("再生速度");
            ui.add(
                egui::Slider::new(&mut state.speed, 0.25..=16.0)
                    .logarithmic(true)
                    .suffix("x"),
            );
        });
        let mut show_heat = overlay.iter().any(|v| *v != Visibility::Hidden);
        if ui.checkbox(&mut show_heat, "通行量ヒートマップ").changed() {
            for mut v in &mut overlay {
                *v = if show_heat {
                    Visibility::Visible
                } else {
                    Visibility::Hidden
                };
            }
            state.heat_dirty = true;
        }
        ui.checkbox(&mut settings.color_by_goal, "歩行者の服を目的地で色分け");
        state.respawn = settings.respawn;

        ui.separator();
        ui.heading("状態");
        let walking = state.sim.active();
        let total = state.sim.agents.len();
        egui::Grid::new("status")
            .num_columns(2)
            .spacing([8.0, 4.0])
            .show(ui, |ui| {
                ui.label("時刻");
                ui.label(format!("{:.1} s", state.sim.time));
                ui.end_row();
                ui.label("歩行中 / 到着");
                ui.label(format!("{} / {}", walking, total - walking));
                ui.end_row();
                ui.label("走行中の車");
                ui.label(format!(
                    "{}",
                    state.traffic.as_ref().map_or(0, |t| t.active())
                ));
                ui.end_row();
                ui.label("歩行者の平均速度");
                let n = walking.max(1) as f32;
                let v: f32 = state
                    .sim
                    .agents
                    .iter()
                    .filter(|a| !a.arrived)
                    .map(|a| a.vel.length())
                    .sum::<f32>()
                    / n;
                ui.label(format!("{v:.2} m/s"));
                ui.end_row();
                if let Some(s) = state.traffic.as_ref().and_then(|t| t.signals.as_ref()) {
                    ui.label("信号のある交差点");
                    let scr = s
                        .intersections
                        .iter()
                        .filter(|i| i.plan == traffic::signal::Plan::Scramble)
                        .count();
                    ui.label(format!(
                        "{} か所（スクランブル {}）",
                        s.intersections.len(),
                        scr
                    ));
                    ui.end_row();
                }
                if let Some(t) = &state.traffic {
                    ui.label("車の平均速度");
                    let n = t.active().max(1) as f32;
                    let v: f32 = t
                        .vehicles
                        .iter()
                        .filter(|v| v.active)
                        .map(|v| v.v)
                        .sum::<f32>()
                        / n
                        * 3.6;
                    ui.label(format!("{v:.1} km/h"));
                    ui.end_row();
                }
            });
        ui.add_space(8.0);
        if layout.compact {
            ui.small("1 本指ドラッグ: 回転 / 2 本指: 移動・ピンチでズーム");
        } else {
            ui.small("左ドラッグ: 回転 / 右ドラッグ: 移動 / ホイール: ズーム\nSpace: 一時停止  ↑↓: 再生速度  H: ヒートマップ\n1 本指ドラッグ: 回転 / 2 本指: 移動・ピンチでズーム");
        }
    };

    if layout.compact {
        if layout.shows(Tab::Settings) {
            sheet(&ctx, "settings_sheet", body);
        }
    } else {
        let resp = egui::SidePanel::right("settings")
            .default_width(260.0)
            .show(&ctx, body);
        if std::env::var_os("PLATEAU_UI_DEBUG").is_some() {
            info!(
                "panel rect {:?} screen {:?} pixels_per_point {}",
                resp.response.rect,
                ctx.screen_rect(),
                ctx.pixels_per_point()
            );
        }
    }
    capture.0 = ctx.wants_pointer_input() || ctx.is_pointer_over_area();
}

/// 信号機の設定欄。
fn signal_settings(ui: &mut egui::Ui, settings: &mut Settings) {
    egui::CollapsingHeader::new("信号機")
        .default_open(true)
        .show(ui, |ui| {
            ui.checkbox(
                &mut settings.signals,
                "信号機を置く（オフ: 交差点は先着順）",
            );
            ui.add_enabled_ui(settings.signals, |ui| {
                let c = &mut settings.signal;
                egui::ComboBox::from_label("歩行者の方式")
                    .selected_text(c.mode.label())
                    .show_ui(ui, |ui| {
                        for m in traffic::PedMode::ALL {
                            ui.selectable_value(&mut c.mode, m, m.label());
                        }
                    });
                egui::Grid::new("signal")
                    .num_columns(2)
                    .spacing([8.0, 6.0])
                    .show(ui, |ui| {
                        ui.label("サイクル長");
                        ui.add(egui::Slider::new(&mut c.cycle, 40.0..=180.0).suffix(" s"));
                        ui.end_row();
                        ui.label("主道路の青の割合");
                        ui.add(egui::Slider::new(&mut c.main_split, 0.2..=0.8));
                        ui.end_row();
                        ui.label("歩行者専用の青");
                        ui.add(egui::Slider::new(&mut c.ped_green, 5.0..=60.0).suffix(" s"));
                        ui.end_row();
                        ui.label("歩行者の青点滅");
                        ui.add(egui::Slider::new(&mut c.ped_flash, 0.0..=15.0).suffix(" s"));
                        ui.end_row();
                        ui.label("黄 / 全赤");
                        ui.horizontal(|ui| {
                            ui.add(
                                egui::DragValue::new(&mut c.yellow)
                                    .range(1.0..=6.0)
                                    .speed(0.1)
                                    .suffix(" s"),
                            );
                            ui.add(
                                egui::DragValue::new(&mut c.all_red)
                                    .range(0.0..=6.0)
                                    .speed(0.1)
                                    .suffix(" s"),
                            );
                        });
                        ui.end_row();
                    });
                // 実際の青時間（サイクル長から黄・全赤・歩行者専用現示を引いて主従に配分）
                let show = |plan| {
                    let t = traffic::signal::Timing::new(c, plan);
                    format!("主 {:.0} s / 従 {:.0} s", t.green[0], t.green[1])
                };
                let (std, excl) = (
                    show(traffic::signal::Plan::Concurrent),
                    show(traffic::signal::Plan::Exclusive),
                );
                let text = match c.mode {
                    traffic::PedMode::Concurrent => format!("車の青: {std}"),
                    traffic::PedMode::Exclusive | traffic::PedMode::Scramble => {
                        format!(
                            "車の青: {excl}（歩行者専用 {:.0} s）",
                            c.ped_green + c.ped_flash
                        )
                    }
                    traffic::PedMode::Auto => {
                        format!("車の青: 標準 {std}\nスクランブル {excl}")
                    }
                };
                ui.small(text);
            });
        });
}
