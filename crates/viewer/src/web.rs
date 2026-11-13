//! Web 版（wasm32）だけで使うもの: 同梱データ、URL のクエリからの設定、エラー表示。
//!
//! ブラウザではファイルの読み書きもネットワークからの PLATEAU 取得もできないので、
//! 銀座 4 丁目交差点の CityGML とシナリオ例・日本語フォントを wasm に埋め込む。

use anyhow::Result;

pub const REPO_URL: &str = "https://github.com/T-T0821/citysim";

/// 同梱の銀座 4 丁目交差点（testdata/ginza）
pub const GML: [&str; 2] = [
    include_str!("../../../testdata/ginza/53394601_bldg_6697_op.gml"),
    include_str!("../../../testdata/ginza/53394601_tran_6697_op.gml"),
];

/// 切り出しの中心（銀座 4 丁目交差点）。半径は `--radius`（既定 220m）
pub const CENTER: (f64, f64) = (35.67165, 139.76500);
/// 同梱データは半径 260m までなので、それより少し内側まで
pub const MAX_RADIUS: f64 = 250.0;

/// 同梱のシナリオ（ファイル名の拡張子なし, 本文）
const SCENARIOS: [(&str, &str); 1] = [(
    "ginza-evening",
    include_str!("../../../scenarios/ginza-evening.toml"),
)];

pub const DEFAULT_SCENARIO_PATH: &str = "scenarios/ginza-evening.toml";

/// 日本語 UI 用フォント（Noto Sans JP を JIS 第 1 水準 + UI の文字にサブセット化, SIL OFL 1.1）
pub const FONT: &[u8] = include_bytes!("../web/fonts/NotoSansJP-subset.ttf");

/// シナリオの本文をパスかファイル名（拡張子なし）で探す
pub fn scenario_text(path: &std::path::Path) -> Result<&'static str, String> {
    let stem = path.file_stem().and_then(|s| s.to_str()).unwrap_or("");
    SCENARIOS
        .iter()
        .find(|(n, _)| *n == stem)
        .map(|(_, t)| *t)
        .ok_or_else(|| {
            let names: Vec<_> = SCENARIOS.iter().map(|(n, _)| *n).collect();
            format!(
                "Web 版で読み込めるのは同梱のシナリオだけです: {}",
                names.join(", ")
            )
        })
}

/// URL のクエリで受け付ける設定（値なしは `no-respawn` だけ）
const KEYS: [&str; 12] = [
    "agents",
    "vehicles",
    "seed",
    "signals",
    "cycle",
    "speed",
    "cell",
    "scenario",
    "shelter",
    "evacuate-at",
    "radius",
    "no-respawn",
];

/// `?agents=800&signals=scramble` を `["--agents", "800", "--signals", "scramble"]` にする。
pub fn args() -> Vec<String> {
    let search = web_sys::window()
        .and_then(|w| w.location().search().ok())
        .unwrap_or_default();
    query_to_args(&search, |s| {
        js_sys::decode_uri_component(&s.replace('+', " "))
            .ok()
            .and_then(|v| v.as_string())
            .unwrap_or_else(|| s.to_string())
    })
}

fn query_to_args(search: &str, decode: impl Fn(&str) -> String) -> Vec<String> {
    let mut out = vec![];
    for kv in search.trim_start_matches('?').split('&') {
        if kv.is_empty() {
            continue;
        }
        let (k, v) = kv.split_once('=').unwrap_or((kv, ""));
        let k = decode(k);
        if !KEYS.contains(&k.as_str()) {
            continue;
        }
        out.push(format!("--{k}"));
        if k != "no-respawn" {
            out.push(decode(v));
        }
    }
    out
}

/// 起動できなかったときにページ上に理由を出す
pub fn show_error(msg: &str) {
    let el = web_sys::window()
        .and_then(|w| w.document())
        .and_then(|d| d.get_element_by_id("status"));
    if let Some(el) = el {
        el.set_text_content(Some(&format!("起動できませんでした: {msg}")));
        let _ = el.set_attribute("class", "error");
    }
}

/// スマホ・タブレットなど（指で操作する端末か、幅の狭い画面）。描画を軽くするのに使う
pub fn is_mobile() -> bool {
    let Some(w) = web_sys::window() else {
        return false;
    };
    let coarse = w
        .match_media("(pointer: coarse)")
        .ok()
        .flatten()
        .is_some_and(|m| m.matches());
    let narrow = w
        .inner_width()
        .ok()
        .and_then(|v| v.as_f64())
        .is_some_and(|x| x < 720.0);
    coarse || narrow
}

/// ページの表示領域の縦横比（幅 / 高さ）
pub fn viewport_aspect() -> Option<f32> {
    let w = web_sys::window()?;
    let x = w.inner_width().ok()?.as_f64()?;
    let y = w.inner_height().ok()?.as_f64()?;
    (x > 0.0 && y > 0.0).then(|| (x / y) as f32)
}
