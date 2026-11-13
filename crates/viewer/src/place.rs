//! 場所を選ぶ UI: 住所か緯度経度と半径を入れると、PLATEAU の CityGML を用意して、その場所でビューアを開き直す。
//!
//! データの取得（`plateau_fetch::prepare`）は別スレッドで行い、進み具合をパネルに出す。
//! 建物・道路のメッシュを作り直すより確実なので、用意できたら同じ設定のまま新しいプロセスで起動し直す。

use bevy::prelude::*;
use bevy_egui::egui;
use std::path::PathBuf;
#[cfg(not(target_arch = "wasm32"))]
use std::sync::{Arc, Mutex};
use walkmap::WalkGrid;

pub const DEFAULT_DATA_DIR: &str = "data/plateau";

#[cfg(not(target_arch = "wasm32"))]
#[derive(Default)]
struct Job {
    log: Vec<String>,
    result: Option<Result<plateau_fetch::Prepared, String>>,
}

#[derive(Resource)]
pub struct PlaceUi {
    pub query: String,
    pub radius: f32,
    pub data_dir: String,
    /// いま表示している範囲の中心
    center: (f64, f64),
    #[cfg(not(target_arch = "wasm32"))]
    job: Option<Arc<Mutex<Job>>>,
    error: Option<String>,
}

impl PlaceUi {
    pub fn new(grid: &WalkGrid, radius: f32, data_dir: PathBuf) -> Self {
        let size = Vec2::new(grid.w as f32, grid.h as f32) * grid.cell;
        let center = grid.projection.to_geo(grid.origin + size * 0.5);
        Self {
            query: String::new(),
            radius,
            data_dir: data_dir.display().to_string(),
            center,
            #[cfg(not(target_arch = "wasm32"))]
            job: None,
            error: None,
        }
    }
}

/// Web 版: データの取得とプロセスの起動ができないので、同梱の場所だけを案内する。
#[cfg(target_arch = "wasm32")]
pub fn panel(ui: &mut egui::Ui, st: &mut PlaceUi) -> bool {
    ui.small(format!(
        "いまの中心: {:.5}, {:.5}",
        st.center.0, st.center.1
    ));
    let _ = (&st.query, st.radius, &st.data_dir, &st.error);
    ui.label("Web 版は同梱の銀座 4 丁目交差点だけを表示します。");
    ui.small("ほかの場所（住所・緯度経度から PLATEAU を自動取得）はネイティブ版で使えます。");
    ui.hyperlink_to("ソースコードとネイティブ版の使い方", crate::web::REPO_URL);
    false
}

/// パネルを描く。開き直す準備ができたら新しいプロセスを起動して true を返す（呼び出し側で終了する）。
#[cfg(not(target_arch = "wasm32"))]
pub fn panel(ui: &mut egui::Ui, st: &mut PlaceUi) -> bool {
    ui.small(format!(
        "いまの中心: {:.5}, {:.5}",
        st.center.0, st.center.1
    ));
    let busy = st.job.is_some();
    ui.add_enabled(
        !busy,
        egui::TextEdit::singleline(&mut st.query)
            .hint_text("住所 または 緯度,経度")
            .desired_width(f32::INFINITY),
    );
    ui.add_enabled(
        !busy,
        egui::Slider::new(&mut st.radius, 80.0..=600.0)
            .suffix(" m")
            .text("半径"),
    );
    ui.horizontal(|ui| {
        ui.label("保存先");
        ui.add_enabled(
            !busy,
            egui::TextEdit::singleline(&mut st.data_dir).desired_width(170.0),
        );
    });
    if ui
        .add_enabled(
            !busy && !st.query.trim().is_empty(),
            egui::Button::new("この場所を読み込む"),
        )
        .clicked()
    {
        st.error = None;
        let job = Arc::new(Mutex::new(Job::default()));
        st.job = Some(job.clone());
        let (q, r, dir) = (
            st.query.trim().to_string(),
            st.radius as f64,
            PathBuf::from(&st.data_dir),
        );
        std::thread::spawn(move || {
            let j2 = job.clone();
            let res = plateau_fetch::prepare(&q, r, &dir, &mut |m| {
                info!("{m}");
                j2.lock().unwrap().log.push(m.to_string());
            });
            job.lock().unwrap().result = Some(res.map_err(|e| format!("{e:#}")));
        });
    }
    ui.small("住所は国土地理院の住所検索、データは PLATEAU の配信 API から取得します。保存先に該当メッシュの GML があればそれを使います（オフライン可）。");

    let mut relaunch = None;
    if let Some(job) = &st.job {
        let mut j = job.lock().unwrap();
        for l in j
            .log
            .iter()
            .rev()
            .take(4)
            .collect::<Vec<_>>()
            .into_iter()
            .rev()
        {
            ui.small(l);
        }
        match j.result.take() {
            Some(Ok(p)) => relaunch = Some(p),
            Some(Err(e)) => st.error = Some(e),
            None => {
                ui.spinner();
            }
        }
    }
    if st.error.is_some() || relaunch.is_some() {
        st.job = None;
    }
    if let Some(e) = &st.error {
        ui.colored_label(egui::Color32::LIGHT_RED, e);
    }
    if let Some(p) = relaunch {
        match relaunch_with(&p) {
            Ok(()) => return true,
            Err(e) => st.error = Some(format!("開き直せませんでした: {e:#}")),
        }
    }
    false
}

#[cfg(not(target_arch = "wasm32"))]
/// 入力データと範囲の指定だけ差し替えて、同じ設定で自分自身を起動し直す。
fn relaunch_with(p: &plateau_fetch::Prepared) -> anyhow::Result<()> {
    let args = relaunch_args(std::env::args().skip(1), p);
    info!("開き直し: {}", args.join(" "));
    std::process::Command::new(std::env::current_exe()?)
        .args(&args)
        .spawn()?;
    Ok(())
}

#[cfg(not(target_arch = "wasm32"))]
/// 場所に依存する引数（入力ファイル・範囲・避難所・録画など）を落とし、新しい GML と範囲を足す。
fn relaunch_args(old: impl Iterator<Item = String>, p: &plateau_fetch::Prepared) -> Vec<String> {
    // 値を 1 つ取る引数のうち、場所に依存するもの
    const DROP1: [&str; 9] = [
        "--gml",
        "--obj",
        "--obj-origin",
        "--crop",
        "--place",
        "--radius",
        "--data-dir",
        "--shelter",
        "--evacuate-at",
    ];
    // 値を 2 つ取る引数（確認・録画用。開き直しでは使わない）
    const DROP2: [&str; 2] = ["--screenshot", "--record"];
    let mut out = vec![];
    let mut it = old.peekable();
    while let Some(a) = it.next() {
        if DROP1.contains(&a.as_str()) {
            it.next();
        } else if DROP2.contains(&a.as_str()) {
            it.next();
            it.next();
        } else {
            out.push(a);
        }
    }
    for f in &p.files {
        out.push("--gml".into());
        out.push(f.display().to_string());
    }
    out.push("--crop".into());
    out.push(format!(
        "{:.6},{:.6},{}",
        p.place.lat, p.place.lon, p.radius
    ));
    out
}

#[cfg(all(test, not(target_arch = "wasm32")))]
mod tests {
    use super::*;

    #[test]
    fn replaces_location_args() {
        let old =
            "--gml a.gml --agents 900 --crop 1,2,3 --shelter 35,139,x --record rec 20 --seed 4"
                .split(' ')
                .map(String::from);
        let p = plateau_fetch::Prepared {
            place: plateau_fetch::Place {
                lat: 35.5,
                lon: 139.5,
                label: None,
            },
            radius: 200.0,
            meshes: vec![],
            files: vec!["x/b.gml".into(), "x/t.gml".into()],
            downloaded: 0,
        };
        assert_eq!(
            relaunch_args(old, &p).join(" "),
            "--agents 900 --seed 4 --gml x/b.gml --gml x/t.gml --crop 35.500000,139.500000,200"
        );
    }
}
