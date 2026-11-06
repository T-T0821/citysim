//! 場所（住所 or 緯度経度）から PLATEAU の CityGML を用意する。
//!
//! 1. 住所なら国土地理院のジオコーダ（AddressSearch）で緯度経度にする
//! 2. 中心と半径から、範囲にかかる 3 次メッシュ（1km 四方, JIS X 0410）のコードを求める
//! 3. データディレクトリに該当メッシュの `*_bldg_*.gml` / `*_tran_*.gml` があればそれを使う（オフライン可）
//! 4. 無ければ PLATEAU の CityGML 配信 API（`api.plateau.reearth.io/datacatalog/citygml/m:<メッシュ>`）で
//!    メッシュ単位の GML の URL を調べてダウンロードし、データディレクトリに保存する
//!
//! 取得した GML はそのままでは広すぎる（1km 四方）ので、呼び出し側が中心・半径で切り出す（`--crop` と同じ）。

use anyhow::{bail, Context, Result};
use std::path::{Path, PathBuf};
use std::time::Duration;

pub const GEOCODER_URL: &str = "https://msearch.gsi.go.jp/address-search/AddressSearch";
pub const CITYGML_API_URL: &str = "https://api.plateau.reearth.io/datacatalog/citygml";
/// 手動で取得するときの案内先
pub const MANUAL_URL: &str = "https://www.geospatial.jp/ckan/dataset/plateau";

/// 解決した場所。
#[derive(Debug, Clone)]
pub struct Place {
    pub lat: f64,
    pub lon: f64,
    /// 住所検索の結果名（緯度経度で指定したときは None）
    pub label: Option<String>,
}

/// "35.67165,139.765" のような緯度経度ならそのまま、そうでなければ住所として検索する。
pub fn resolve_place(s: &str) -> Result<Place> {
    if let Some((lat, lon)) = parse_latlon(s) {
        return Ok(Place {
            lat,
            lon,
            label: None,
        });
    }
    geocode(s)
}

/// "緯度,経度"（空白・全角カンマも可）を読む。日本付近の値でなければ None。
pub fn parse_latlon(s: &str) -> Option<(f64, f64)> {
    let s = s.replace(['，', '、'], ",");
    let v: Vec<f64> = s
        .split([',', ' '])
        .filter(|t| !t.trim().is_empty())
        .map(|t| t.trim().parse().ok())
        .collect::<Option<_>>()?;
    match v[..] {
        [lat, lon] if (20.0..50.0).contains(&lat) && (120.0..155.0).contains(&lon) => {
            Some((lat, lon))
        }
        // 経度,緯度 の順で書かれていても受け付ける
        [lon, lat] if (20.0..50.0).contains(&lat) && (120.0..155.0).contains(&lon) => {
            Some((lat, lon))
        }
        _ => None,
    }
}

fn agent() -> ureq::Agent {
    ureq::AgentBuilder::new()
        .timeout_connect(Duration::from_secs(15))
        .timeout_read(Duration::from_secs(120))
        .try_proxy_from_env(true)
        .user_agent(concat!("plateau-crowd/", env!("CARGO_PKG_VERSION")))
        .build()
}

/// 国土地理院の住所検索で緯度経度を得る（最初の候補）。
pub fn geocode(addr: &str) -> Result<Place> {
    let body = agent()
        .get(GEOCODER_URL)
        .query("q", addr.trim())
        .call()
        .with_context(|| format!("住所検索に失敗しました（{GEOCODER_URL}）"))?
        .into_string()?;
    parse_geocode(&body).with_context(|| format!("住所が見つかりません: {addr}"))
}

/// AddressSearch の応答（GeoJSON Feature の配列）から最初の候補を取り出す。
pub fn parse_geocode(body: &str) -> Option<Place> {
    let v: serde_json::Value = serde_json::from_str(body).ok()?;
    let f = v.as_array()?.first()?;
    let c = f.pointer("/geometry/coordinates")?.as_array()?;
    let (lon, lat) = (c.first()?.as_f64()?, c.get(1)?.as_f64()?);
    let label = f
        .pointer("/properties/title")
        .and_then(|t| t.as_str())
        .map(String::from);
    Some(Place { lat, lon, label })
}

/// 3 次メッシュコード（8 桁、約 1km 四方）。
pub fn mesh3(lat: f64, lon: f64) -> String {
    let y = lat * 1.5;
    let x = lon - 100.0;
    let (p, u) = (y.floor(), x.floor());
    let (y2, x2) = ((y - p) * 8.0, (x - u) * 8.0);
    let (q, v) = (y2.floor(), x2.floor());
    let (r, w) = (((y2 - q) * 10.0).floor(), ((x2 - v) * 10.0).floor());
    format!("{:02}{:02}{}{}{}{}", p, u, q, v, r, w)
}

/// 中心 (lat, lon) から半径 r [m] の正方形にかかる 3 次メッシュ。
pub fn meshes_covering(lat: f64, lon: f64, r: f64) -> Vec<String> {
    let dlat = r / 111_320.0;
    let dlon = r / (111_320.0 * lat.to_radians().cos());
    // 3 次メッシュは緯度 30 秒・経度 45 秒。その半分の刻みで格子点を取れば漏れない
    let n = ((2.0 * dlat) / (30.0 / 3600.0 / 2.0)).ceil().max(1.0) as usize;
    let m = ((2.0 * dlon) / (45.0 / 3600.0 / 2.0)).ceil().max(1.0) as usize;
    let mut out: Vec<String> = vec![];
    for i in 0..=n {
        for j in 0..=m {
            let la = lat - dlat + 2.0 * dlat * i as f64 / n as f64;
            let lo = lon - dlon + 2.0 * dlon * j as f64 / m as f64;
            let c = mesh3(la, lo);
            if !out.contains(&c) {
                out.push(c);
            }
        }
    }
    out.sort();
    out
}

/// GML の種類（このアプリが使うもの）。
const KINDS: [&str; 2] = ["bldg", "tran"];

/// データディレクトリ以下（4 階層まで）から、指定メッシュの bldg / tran GML を探す。
pub fn find_local(dir: &Path, meshes: &[String]) -> Vec<PathBuf> {
    let mut out = vec![];
    walk(dir, 0, &mut |p| {
        let Some(name) = p.file_name().and_then(|n| n.to_str()) else {
            return;
        };
        if !name.ends_with(".gml") {
            return;
        }
        let mut parts = name.split('_');
        let (Some(code), Some(kind)) = (parts.next(), parts.next()) else {
            return;
        };
        if meshes.iter().any(|m| m == code) && KINDS.contains(&kind) {
            out.push(p.to_path_buf());
        }
    });
    out.sort();
    out
}

fn walk(dir: &Path, depth: usize, f: &mut impl FnMut(&Path)) {
    let Ok(rd) = std::fs::read_dir(dir) else {
        return;
    };
    for e in rd.flatten() {
        let p = e.path();
        if p.is_dir() {
            if depth < 4 {
                walk(&p, depth + 1, f);
            }
        } else {
            f(&p);
        }
    }
}

/// 配信 API で見つかった 1 ファイル。
#[derive(Debug, Clone, PartialEq)]
pub struct RemoteFile {
    pub url: String,
    /// "bldg" / "tran"
    pub kind: String,
    pub mesh: String,
    /// データセット名（例: 13101_chiyoda-ku_city_2023_citygml_1_op）。保存先のサブディレクトリにする
    pub dataset: String,
    pub year: u32,
}

/// CityGML 配信 API の応答から、指定メッシュの bldg / tran GML の URL を拾う。
/// 応答の形が変わっても動くよう、JSON 中の `.gml` で終わる文字列をすべて調べる。
/// 同じ自治体のデータが複数年度あれば最新の年度だけ残す（区境のメッシュは区ごとに別ファイルなので両方残す）。
pub fn parse_citygml_api(body: &str, mesh: &str) -> Vec<RemoteFile> {
    let Ok(v) = serde_json::from_str::<serde_json::Value>(body) else {
        return vec![];
    };
    let mut urls = vec![];
    collect_strings(&v, &mut urls);
    let mut files: Vec<RemoteFile> = urls
        .into_iter()
        .filter_map(|u| remote_file(&u))
        .filter(|f| f.mesh == mesh)
        .collect();
    files.sort_by(|a, b| a.url.cmp(&b.url));
    files.dedup_by(|a, b| a.url == b.url);
    // 自治体コード（データセット名の先頭 5 桁）ごとに最新年度
    let city = |f: &RemoteFile| f.dataset.split('_').next().unwrap_or("").to_string();
    let latest: Vec<(String, u32)> = {
        let mut m: Vec<(String, u32)> = vec![];
        for f in &files {
            let c = city(f);
            match m.iter_mut().find(|(k, _)| *k == c) {
                Some(e) => e.1 = e.1.max(f.year),
                None => m.push((c, f.year)),
            }
        }
        m
    };
    files.retain(|f| latest.iter().any(|(c, y)| *c == city(f) && *y == f.year));
    files
}

fn collect_strings(v: &serde_json::Value, out: &mut Vec<String>) {
    match v {
        serde_json::Value::String(s) if s.ends_with(".gml") => out.push(s.clone()),
        serde_json::Value::Array(a) => a.iter().for_each(|x| collect_strings(x, out)),
        serde_json::Value::Object(o) => o.values().for_each(|x| collect_strings(x, out)),
        _ => {}
    }
}

/// URL `.../<dataset>/udx/<kind>/<mesh>_<kind>_6697_op.gml` を読む。
fn remote_file(url: &str) -> Option<RemoteFile> {
    if !url.starts_with("http") {
        return None;
    }
    let segs: Vec<&str> = url.split('/').collect();
    let name = *segs.last()?;
    let mut parts = name.split('_');
    let (mesh, kind) = (parts.next()?, parts.next()?);
    if !KINDS.contains(&kind) || mesh.len() < 6 || !mesh.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    let udx = segs.iter().rposition(|s| *s == "udx")?;
    let dataset = segs[..udx]
        .iter()
        .rev()
        .find(|s| !s.is_empty())
        .map(|s| s.to_string())
        .unwrap_or_default();
    // データセット名の 4 桁の数字（2020〜2099）を年度とみなす
    let year = dataset
        .split('_')
        .filter_map(|t| t.parse::<u32>().ok())
        .find(|y| (2019..2100).contains(y))
        .unwrap_or(0);
    Some(RemoteFile {
        url: url.to_string(),
        kind: kind.to_string(),
        mesh: mesh.to_string(),
        dataset,
        year,
    })
}

/// 配信 API にメッシュの GML を問い合わせる。
pub fn query_citygml(mesh: &str) -> Result<Vec<RemoteFile>> {
    let url = format!("{CITYGML_API_URL}/m:{mesh}");
    let body = agent()
        .get(&url)
        .call()
        .with_context(|| format!("PLATEAU 配信 API に接続できません（{url}）"))?
        .into_string()?;
    Ok(parse_citygml_api(&body, mesh))
}

/// URL をファイルに保存する（途中で切れたら消す）。
pub fn download(url: &str, dst: &Path, progress: &mut dyn FnMut(&str)) -> Result<()> {
    if let Some(p) = dst.parent() {
        std::fs::create_dir_all(p)?;
    }
    let resp = agent()
        .get(url)
        .call()
        .with_context(|| format!("ダウンロードに失敗しました: {url}"))?;
    let total: Option<u64> = resp.header("Content-Length").and_then(|v| v.parse().ok());
    let tmp = dst.with_extension("gml.part");
    let mut f = std::io::BufWriter::new(std::fs::File::create(&tmp)?);
    let mut r = resp.into_reader();
    let mut buf = vec![0u8; 1 << 16];
    let mut got = 0u64;
    let mut last = 0u64;
    let name = dst.file_name().and_then(|n| n.to_str()).unwrap_or("");
    loop {
        let n = match std::io::Read::read(&mut r, &mut buf) {
            Ok(0) => break,
            Ok(n) => n,
            Err(e) => {
                drop(f);
                let _ = std::fs::remove_file(&tmp);
                return Err(e).context("ダウンロードが途中で切れました");
            }
        };
        std::io::Write::write_all(&mut f, &buf[..n])?;
        got += n as u64;
        if got - last > 4 << 20 {
            last = got;
            progress(&match total {
                Some(t) => format!("{name}: {:.0} / {:.0} MB", got as f64 / 1e6, t as f64 / 1e6),
                None => format!("{name}: {:.0} MB", got as f64 / 1e6),
            });
        }
    }
    std::io::Write::flush(&mut f)?;
    drop(f);
    std::fs::rename(&tmp, dst)?;
    Ok(())
}

/// 用意できたデータ。
#[derive(Debug, Clone)]
pub struct Prepared {
    pub place: Place,
    pub radius: f64,
    pub meshes: Vec<String>,
    pub files: Vec<PathBuf>,
    /// ダウンロードしたファイル数（0 ならすべて手元にあった）
    pub downloaded: usize,
}

/// 場所と半径から、読み込む GML ファイルを用意する。`progress` に途中経過を渡す。
pub fn prepare(
    place: &str,
    radius: f64,
    data_dir: &Path,
    progress: &mut dyn FnMut(&str),
) -> Result<Prepared> {
    progress("場所を調べています…");
    let place = resolve_place(place)?;
    if let Some(l) = &place.label {
        progress(&format!("{l} ({:.5}, {:.5})", place.lat, place.lon));
    }
    let meshes = meshes_covering(place.lat, place.lon, radius);
    progress(&format!("3 次メッシュ: {}", meshes.join(", ")));
    let mut files = vec![];
    let mut downloaded = 0;
    let mut errors = vec![];
    for mesh in &meshes {
        let local = find_local(data_dir, std::slice::from_ref(mesh));
        if !local.is_empty() {
            progress(&format!("{mesh}: 手元のファイルを使います（{} 件）", local.len()));
            files.extend(local);
            continue;
        }
        progress(&format!("{mesh}: PLATEAU 配信 API で検索中…"));
        let remote = match query_citygml(mesh) {
            Ok(r) => r,
            Err(e) => {
                errors.push(format!("{mesh}: {e:#}"));
                continue;
            }
        };
        if remote.is_empty() {
            errors.push(format!("{mesh}: このメッシュの建物・道路データは配信されていません"));
            continue;
        }
        for f in remote {
            let name = f.url.rsplit('/').next().unwrap_or("x.gml");
            let dst = data_dir.join(&f.dataset).join(&f.kind).join(name);
            if !dst.exists() {
                progress(&format!("ダウンロード: {}/{name}", f.dataset));
                if let Err(e) = download(&f.url, &dst, progress) {
                    errors.push(format!("{e:#}"));
                    continue;
                }
                downloaded += 1;
            }
            files.push(dst);
        }
    }
    if files.is_empty() {
        bail!(
            "データを用意できませんでした。\n{}\n\n\
             手動で取得する場合: {MANUAL_URL} から該当する市区町村の CityGML を落とし、\n\
             udx/bldg と udx/tran にある {} のファイルを {} に置いてください（同じ操作で手元のファイルを使います）。",
            errors.join("\n"),
            meshes
                .iter()
                .map(|m| format!("{m}_*"))
                .collect::<Vec<_>>()
                .join(" / "),
            data_dir.display()
        );
    }
    for e in &errors {
        progress(&format!("注意: {e}"));
    }
    Ok(Prepared {
        place,
        radius,
        meshes,
        files,
        downloaded,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mesh_codes() {
        // 銀座 4 丁目交差点（同梱データのメッシュ）
        assert_eq!(mesh3(35.67165, 139.76500), "53394601");
        // 渋谷スクランブル交差点
        assert_eq!(mesh3(35.65949, 139.70055), "53393596");
        // 東京駅
        assert_eq!(mesh3(35.68124, 139.76712), "53394611");
        let m = meshes_covering(35.65949, 139.70055, 220.0);
        assert!(m.contains(&"53393596".to_string()));
        // 渋谷スクランブルの 220m 四方は 4 メッシュの接点付近
        assert!(m.len() >= 2 && m.len() <= 4, "{m:?}");
        let one = meshes_covering(35.6625, 139.7703, 50.0);
        assert_eq!(one.len(), 1);
    }

    #[test]
    fn latlon_input() {
        assert_eq!(parse_latlon("35.67165,139.765"), Some((35.67165, 139.765)));
        assert_eq!(parse_latlon(" 35.67165 , 139.765 "), Some((35.67165, 139.765)));
        assert_eq!(parse_latlon("35.67165，139.765"), Some((35.67165, 139.765)));
        assert_eq!(parse_latlon("139.765, 35.67165"), Some((35.67165, 139.765)));
        assert_eq!(parse_latlon("東京都中央区銀座4丁目"), None);
        assert_eq!(parse_latlon("1,2"), None);
    }

    #[test]
    fn geocoder_response() {
        let body = r#"[{"geometry":{"coordinates":[139.765,35.671],"type":"Point"},"type":"Feature","properties":{"addressCode":"","title":"東京都中央区銀座四丁目"}}]"#;
        let p = parse_geocode(body).unwrap();
        assert_eq!((p.lat, p.lon), (35.671, 139.765));
        assert_eq!(p.label.as_deref(), Some("東京都中央区銀座四丁目"));
        assert!(parse_geocode("[]").is_none());
    }

    #[test]
    fn citygml_api_response() {
        let base = "https://assets.cms.plateau.reearth.io/assets/aa/bb";
        let body = format!(
            r#"{{"cities":[
              {{"cityCode":"13101","year":2023,"files":{{
                 "bldg":[{{"code":"53394601","maxLod":2,"url":"{base}/13101_chiyoda-ku_city_2023_citygml_1_op/udx/bldg/53394601_bldg_6697_op.gml"}}],
                 "tran":[{{"code":"53394601","maxLod":3,"url":"{base}/13101_chiyoda-ku_city_2023_citygml_1_op/udx/tran/53394601_tran_6697_op.gml"}}],
                 "brid":[{{"code":"53394601","url":"{base}/13101_chiyoda-ku_city_2023_citygml_1_op/udx/brid/53394601_brid_6697_op.gml"}}]}}}},
              {{"cityCode":"13101","year":2022,"files":{{
                 "bldg":[{{"code":"53394601","url":"{base}/13101_chiyoda-ku_city_2022_citygml_1_op/udx/bldg/53394601_bldg_6697_op.gml"}}]}}}},
              {{"cityCode":"13102","year":2023,"files":{{
                 "bldg":[{{"code":"53394601","url":"{base}/13102_chuo-ku_city_2023_citygml_1_op/udx/bldg/53394601_bldg_6697_op.gml"}},
                         {{"code":"53394602","url":"{base}/13102_chuo-ku_city_2023_citygml_1_op/udx/bldg/53394602_bldg_6697_op.gml"}}]}}}}
            ]}}"#
        );
        let f = parse_citygml_api(&body, "53394601");
        let got: Vec<_> = f.iter().map(|f| (f.dataset.as_str(), f.kind.as_str(), f.year)).collect();
        assert_eq!(
            got,
            vec![
                ("13101_chiyoda-ku_city_2023_citygml_1_op", "bldg", 2023),
                ("13101_chiyoda-ku_city_2023_citygml_1_op", "tran", 2023),
                ("13102_chuo-ku_city_2023_citygml_1_op", "bldg", 2023),
            ]
        );
    }

    #[test]
    fn finds_bundled_ginza() {
        let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata");
        let f = find_local(&dir, &["53394601".to_string()]);
        assert_eq!(f.len(), 2, "{f:?}");
        assert!(find_local(&dir, &["53393596".to_string()]).is_empty());
    }
}
