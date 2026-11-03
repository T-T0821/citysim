# plateau-crowd

Project PLATEAU（国交省 3D 都市モデル, CityGML）上で歩行者の流れをシミュレートする pure Rust アプリ。

## 構成

```
crates/
  plateau-gml/  CityGML (bldg / tran) のストリーミングパーサ（quick-xml）
  walkmap/      緯度経度→局所平面、歩行可能グリッド、フローフィールド、障害物距離場
  crowd/        Social Force Model によるエージェント計算
  cli/          ヘッドレス実行。軌跡 CSV と通行量ヒートマップ PNG を出力
  viewer/       bevy 0.16 による 3D ビューア（建物の押し出し表示 + 歩行者 + ヒートマップ）
testdata/       合成テストデータ（東京駅付近 120m 四方の十字路と 5 棟）
```

処理の流れ:

1. **読込**: `bldg:Building` の LOD0 フットプリント（無ければ LOD1 立体の底面）と `tran:Road` の LOD1 路面を取得。座標は EPSG:6697（緯度・経度・楕円体高）。
2. **歩行空間**: 道路面があれば「道路 ∖ 建物」、無ければ「範囲 ∖ 建物」を 0.5m グリッドに塗る。
3. **経路誘導**: 目的地ごとに 8 近傍 Dijkstra で距離場（フローフィールド）を作る。壁際はコスト増。
4. **歩行者モデル**: 駆動力 + 他者反発（視野異方性つき）+ 壁反発 + 接触力。近傍探索は空間ハッシュ。
5. **出力**: `t,id,lat,lon,x,y,vx,vy` の CSV（kepler.gl / QGIS / deck.gl でそのまま可視化可）と PNG。

## 使い方

```sh
cargo run --release -- --gml testdata/sample.gml --agents 400 --duration 300 \
    --csv out.csv --png heatmap.png

# PLATEAU の 3 次メッシュ単位の GML を複数／ディレクトリで渡せる
cargo run --release -- --gml udx/bldg/53394611_bldg_6697_op.gml \
    --gml udx/tran/53394611_tran_6697_op.gml --agents 2000
```

## 3D ビューア

```sh
cargo run --release -p plateau-crowd-viewer -- --gml testdata/sample.gml --agents 600
```

| 操作 | 内容 |
|---|---|
| 左ドラッグ / 右ドラッグ / ホイール | 回転 / 平行移動 / ズーム |
| Space | 一時停止 |
| ↑ / ↓ | 再生速度 ×2 / ×½ |
| H | 通行量ヒートマップの表示切替 |

歩行者は目的地（東西南北の縁）ごとに色分けされ、到着すると別の縁から再投入されます（`--no-respawn` で無効）。
`--screenshot out.png 60` でシミュレーション 60 秒時点の画面を保存して終了します（`PLATEAU_SHOW_HEAT=1` でヒートマップ付き）。

Linux では `libxkbcommon-x11-0` と Vulkan ドライバ（GPU のもの、または `mesa-vulkan-drivers`）が必要です。

![viewer](docs/viewer.png)

## ロードマップ

- [x] MVP: CityGML 読込 → グリッド → フローフィールド → Social Force → CSV/PNG
- [ ] 実データ検証（PLATEAU の bldg + tran を 1 メッシュ分）、大きなファイルの高速化
- [ ] シナリオ定義（TOML）: 出発地・目的地（駅出口など）・発生レート・時間帯
- [x] 可視化: bevy で 3D 建物 + エージェントのリアルタイム表示
- [ ] 可視化の拡張: LOD2 の屋根形状・テクスチャ、地形 (dem)、wasm + WebGPU でブラウザ表示
- [ ] 速度向上: rayon による並列化、密度に応じた計算（数万人規模）
- [ ] 評価指標: 密度（人/m²）、流量、サービス水準 (LOS)、避難完了時間
- [ ] 歩道 (`tran:TrafficArea` の function)、横断歩道、階段・建物入口の扱い
