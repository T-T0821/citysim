# plateau-crowd

Project PLATEAU（国交省 3D 都市モデル, CityGML）上で歩行者の流れをシミュレートする pure Rust アプリ。

## 構成

```
crates/
  plateau-gml/  CityGML (bldg / tran) のストリーミングパーサ（quick-xml）
  walkmap/      緯度経度→局所平面、歩行可能グリッド、フローフィールド、障害物距離場
  crowd/        Social Force Model による歩行者の計算
  traffic/      車道の中心線ネットワーク抽出と車両の計算（IDM・左側通行・交差点制御）
  cli/          ヘッドレス実行。軌跡 CSV と通行量ヒートマップ PNG、区画別の集計 CSV・画像を出力
  viewer/       bevy 0.16 による 3D ビューア（建物の押し出し表示 + 歩行者 + ヒートマップ）
scenarios/      イベント・混雑シナリオと通行止めの設定例（TOML）
scripts/        公式 CityGML の取得 (fetch-ginza.sh) と範囲切り出し (extract_gml.py)
testdata/
  sample.gml    合成テストデータ（東京駅付近 200m 四方、幅 20m の「井」の字の道路と 15 棟）
  ginza/        公式 PLATEAU CityGML から切り出した銀座 4 丁目交差点周辺（建物 457 棟、LOD3 道路 160 件）
```

処理の流れ:

1. **読込**: `bldg:Building` の LOD0 フットプリント（無ければ LOD1 立体の底面）と `tran:Road` の LOD1 路面を取得。座標は EPSG:6697（緯度・経度・楕円体高）。
2. **歩行空間**: 道路面があれば「道路 ∖ 建物」、無ければ「範囲 ∖ 建物」を 0.5m グリッドに塗る。LOD2/LOD3 の道路なら `tran:TrafficArea` の区分（1000 車道部・1020 島 → 車道、2000 歩道部・3000 自転車道 → 歩道、`AuxiliaryTrafficArea` の植栽・分離帯 → 通行不可）をそのまま使う。LOD1 だけの道路では道路の端から 3m を歩道、それより内側を車道とみなす近似を使う。名前に「高速」を含む路線（高架・掘割の首都高）は地上と交わらないので除く。
3. **経路誘導**: 目的地ごとに 8 近傍 Dijkstra で距離場（フローフィールド）を作る。壁際はコスト増。
4. **歩行者モデル**: 駆動力 + 他者反発（視野異方性つき）+ 壁反発 + 車からの反発 + 接触力。近傍探索は空間ハッシュ。車道の通行コストを 6 倍にして、歩道を歩き車道は短く横断するようにしている。
5. **車両モデル**:
   - 車道を Zhang–Suen 法で細線化して中心線を取り出し、交差点ノードと道路区間のグラフにする。範囲の縁で途切れる道路は出入口になる。
   - 左側通行で中心線から 1.8m ずらして走る。縦方向は Intelligent Driver Model（前の車・停止線・前方の歩行者に追従）。
   - 交差点は「動線（進入レーン→退出レーン）」単位の通行権で制御する。軌跡が交わらない動線（対向の直進同士など）は同時に通れ、待ち車がいれば 6 秒で交代。出口側が詰まっていれば交差点に入らない。
   - **信号機**（既定でオン、`--signals off` で上の先着順だけに戻せる）: 分岐ノードを交差点ごとにまとめ（広い交差点で割れた節点は 1 つに）、流入路を道路の向きで主道路・従道路の 2 現示に分ける。各流入路の口（車道幅が道路本来の幅に戻る所）に幅 4m の横断歩道と、その手前に停止線を置く。サイクルは 主道路 青→黄→全赤 → 従道路 青→黄→全赤（→ 歩行者専用現示）。車は赤と、無理なく止まれる黄で停止線に止まり、青の間だけ上の通行権で交差点に入る。
   - 歩行者信号の方式は `--signals` で選ぶ: `concurrent`（歩車同時: 並行する車が青の間に青、終わりは点滅）、`exclusive`（歩車分離: 歩行者専用現示で全横断歩道が青）、`scramble`（スクランブル: 歩行者専用現示で交差点内を斜めにも渡れる）、`auto`（既定: 両方向とも車道幅 14m 以上の 4 方向交差点だけスクランブル、他は歩車同時。銀座 4 丁目・数寄屋橋などが該当）。サイクル長・主道路の青の割合・歩行者専用の青は `--cycle` `--main-split` `--ped-green` か、ビューアの設定パネルで変えられる。
   - 歩行者は青でない横断歩道・交差点内へ歩道から踏み出さず、縁石で待つ（渡り始めた人は渡り切る）。信号があるときは横断歩道の通行コストを下げ、スクランブル以外の交差点内は高くして、横断歩道を渡るように経路を引き直す。
   - 歩行者が前にいれば止まり、5 秒以上にらみ合ったら徐行で進む。縁に着いた車は別の出入口から再投入する。
6. **出力**: `t,kind,id,lat,lon,x,y,vx,vy` の CSV（kind は `ped` / `car`）（kepler.gl / QGIS / deck.gl でそのまま可視化可）と PNG。

## 使い方

```sh
cargo run --release -- --gml testdata/sample.gml --agents 400 --vehicles 30 --duration 300 \
    --csv out.csv --png heatmap.png

# PLATEAU の 3 次メッシュ単位の GML を複数／ディレクトリで渡せる
cargo run --release -- --gml udx/bldg/53394611_bldg_6697_op.gml \
    --gml udx/tran/53394611_tran_6697_op.gml --agents 2000
```

## 実際の PLATEAU データで動かす

### まず試す: 銀座 4 丁目交差点（同梱の公式 CityGML 抜粋）

東京都 23 区（2023 年度）の CityGML から銀座 4 丁目交差点（和光・三越前）を中心とする半径 260m を切り出したものを `testdata/ginza/` に同梱しています。
LOD3 の道路（車道部・歩道部・島）が入っているので、中央通り・晴海通りの広い歩道と車道が実際の形で区別されます。

```sh
cargo run --release -p plateau-crowd-viewer -- --gml testdata/ginza/53394601_bldg_6697_op.gml \
    --gml testdata/ginza/53394601_tran_6697_op.gml --crop 35.67165,139.76500,220 --agents 1500 --vehicles 40

# ヘッドレスで CSV / ヒートマップ / 確認用地図
cargo run --release -- --gml testdata/ginza/53394601_bldg_6697_op.gml --gml testdata/ginza/53394601_tran_6697_op.gml \
    --crop 35.67165,139.76500,220 --agents 1000 --vehicles 30 --duration 300 --csv ginza.csv --png ginza-heat.png --map-png ginza-map.png
```

![ginza 3d](docs/ginza-3d.png)

![ginza map](docs/ginza-map.png)

地図の明るい灰色が歩道部、暗い灰色が車道部（LOD3 の TrafficArea 由来）、白線が抽出した車道の中心線です。

メッシュ全体（銀座・有楽町・新橋・日比谷、建物 2297 棟・道路 1007 件、約 290MB）は `scripts/fetch-ginza.sh` で取得できます。
別の場所を切り出して小さな GML にするには `scripts/extract_gml.py 入力.gml 出力.gml 緯度 経度 半径m` を使います（建物は LOD0/LOD1 だけ、道路は区分面を残し、LOD3 の三角形メッシュは shapely があれば平面ポリゴンにまとめます）。

出典: 国土交通省 Project PLATEAU 3D 都市モデル（東京都 23 区、2023 年度）を加工して作成（[PLATEAU サイトポリシー](https://www.mlit.go.jp/plateau/site-policy/)、CC BY 4.0 互換）。

### CityGML（公式配布データ）

[G空間情報センター](https://www.geospatial.jp/ckan/dataset/plateau) から都市の CityGML を取得し、
`udx/bldg/` と `udx/tran/` の 3 次メッシュ単位のファイルを渡します。`--crop 緯度,経度,半径m` で範囲を切り出せます。

```sh
cargo run --release -- --gml udx/bldg/53393596_bldg_6697_op.gml --gml udx/tran/53393596_tran_6697_op.gml \
    --crop 35.65949,139.70055,220 --agents 1500 --vehicles 30 --map-png map.png
```

### OBJ（PLATEAU SDK for Unity などの書き出し）

`g bldg_…` / `g tran_…` のグループ名が付いた OBJ も読めます（PLATEAU SDK for Unity の書き出し形式）。
局所座標なので原点の緯度経度を `--obj-origin` で指定します。SDK の書き出しは x が西向き・z が北向き・y が上向きで、
原点は取り込み範囲の基準点（複数メッシュなら中心）です。建物は LOD1/LOD2 の立体をそのまま描き、道路は LOD1〜3 を重ねて使います。

```sh
# 渋谷スクランブル交差点周辺 440m 四方（4 メッシュ 533935-85/86/95/96、原点はその接点）
cargo run --release -p plateau-crowd-viewer -- --obj data/shibuya --obj-origin 35.6583333333,139.7 \
    --crop 35.65949,139.70055,220 --agents 1500 --vehicles 30
```

![shibuya](docs/shibuya-scramble.png)

出典: 国土交通省 Project PLATEAU 3D 都市モデル（渋谷区）を加工して作成（[PLATEAU サイトポリシー](https://www.mlit.go.jp/plateau/site-policy/)、CC BY 4.0 互換）。

## 3D ビューア

```sh
cargo run --release -p plateau-crowd-viewer -- --gml testdata/sample.gml --agents 600 --vehicles 30
```

画面右の設定パネル（egui）で、歩行者数・車両数・平均歩行速度・制限速度・乱数シード・到着後の再投入を変えて
「適用して最初から」で作り直せます。一時停止、再生速度、ヒートマップ表示、時刻リセットと、歩行者・車の平均速度などの状態表示もここにあります。
パネルの日本語表示には OS の日本語フォント（Noto Sans CJK、IPA ゴシック、ヒラギノ、メイリオなど）を自動で探します。
見つからない場合は `--ui-font <フォントファイル>` か環境変数 `PLATEAU_UI_FONT` で指定してください。

![settings](docs/viewer-settings.png)

キーボード・マウス:

| 操作 | 内容 |
|---|---|
| 左ドラッグ / 右ドラッグ / ホイール | 回転 / 平行移動 / ズーム |
| Space | 一時停止 |
| ↑ / ↓ | 再生速度 ×2 / ×½ |
| H | 通行量ヒートマップの表示切替 |

歩行者は目的地（東西南北の縁）ごとに色分けされ、車は乗用車（4.5m）とトラック（7.5m、1 割）が走ります。歩行者は到着すると別の縁から再投入されます（`--no-respawn` で無効）。
`--map-png` で歩行空間・車道・抽出した道路網を描いた確認用の地図を出力します。`--screenshot out.png 60` でシミュレーション 60 秒時点の画面を保存して終了します（`PLATEAU_SHOW_HEAT=1` でヒートマップ付き、`PLATEAU_CAM="yaw,pitch,距離,x,y"` でカメラ位置指定）。

動画は `--record <出力ディレクトリ> <秒数>` で連番 PNG を書き出し、ffmpeg でつなぎます（`--speed` 再生倍率、`--orbit` カメラ旋回 rad/s、`--warmup` 録画前に進める秒数）。

```sh
cargo run --release -p plateau-crowd-viewer -- --gml testdata/sample.gml --agents 700 --vehicles 30 \
    --record rec 20 --speed 1.5 --orbit 0.04 --warmup 30
ffmpeg -framerate 30 -i rec/frame_%05d.png -c:v libx264 -pix_fmt yuv420p sim.mp4
```

外部依存は quick-xml / glam / bevy / bevy_egui です。Linux では `libxkbcommon-x11-0` と Vulkan ドライバ（GPU のもの、または `mesa-vulkan-drivers`）が必要です。

![viewer](docs/viewer.png)

![cars](docs/viewer-cars.png)

## シナリオ・通行止め・結果のエクスポート

### イベント・混雑シナリオ（`--scenario`）

TOML の設定ファイルで、駅出口や会場などの**発生源**と時刻ごとの発生人数、**行き先**、**通行止め・工事**を指定できます。
例: [`scenarios/ginza-evening.toml`](scenarios/ginza-evening.toml)（銀座の夕方の帰宅ラッシュ＋終演後の観客＋晴海通りの工事）。

```toml
[scenario]
name = "銀座 夕方"
start = "18:00"            # 時刻を "HH:MM" で書けるようにする基準
duration = "18:20"         # 計算の終わり（秒数でも可）
background_agents = 300    # 発生源とは別に範囲を通り抜ける人
vehicles = 30
zone_size = 20             # 集計区画の一辺 [m]

[[source]]                 # 発生源
name = "銀座駅 交差点北西口"
at = [35.671542, 139.764735]   # [緯度, 経度]（coords = "local" なら [x, y] m）
radius = 6
rate = [["18:00", 10], ["18:05", 40], ["18:15", 40], ["18:20", 10]]  # [時刻, 人/分]、間は直線
to = ["有楽町方面", "西", "北"]  # 行き先（destination の名前か、範囲の縁の 東/西/南/北）
weights = [3, 1, 1]

[[destination]]            # 行き先
name = "有楽町方面"
at = [35.672863, 139.762733]
radius = 8

[[closure]]                # 通行止め・工事
name = "晴海通りの工事"
kind = "all"               # all（全面）/ sidewalk（歩道のみ）/ road（車道のみ。車も通れない）
line = [[35.671723, 139.764004], [35.672124, 139.764448]]  # polygon / rect / line(+width) のどれか
width = 8
```

- 発生源の人は時刻に応じて出現し（出口が混んでいると出口で待つ）、行き先に着くと消えます。
- 通行止めは歩行空間（グリッド）を塞ぎ、`all` / `road` は車道の中心線グラフの区間も外します。車は迂回するか U ターンします。

### 結果のエクスポート（`--export`）と Before/After 比較（`--compare`）

```sh
cargo run --release -- --gml testdata/ginza/53394601_bldg_6697_op.gml --gml testdata/ginza/53394601_tran_6697_op.gml \
    --crop 35.67165,139.76500,220 --scenario scenarios/ginza-evening.toml --compare --export out/ginza-evening
```

`--compare` は同じシナリオを通行止めなし（Before）とあり（After）で続けて計算します。`--export` のディレクトリには次を書き出します。

| ファイル | 内容 |
|---|---|
| `zones.csv` | 区画（既定 20m 四方）ごとの 通過人数（延べ・1 時間あたり）、平均滞留時間、平均待ち時間（0.3 m/s 未満で止まっていた時間）、最大人数・最大密度 [人/m²]、車の通過台数。比較時は `before_*` と `diff_*` 列も |
| `timeline.csv` | 1 分ごとの歩行者数・平均速度・停止している人の割合・車の台数 |
| `heatmap.png` / `zones_*.png` | 通行量のヒートマップと、区画ごとの通過人数・待ち時間・滞留時間の地図（下端に凡例） |
| `diff_heatmap.png` / `diff_zones_passes_per_h.png` | After − Before（1 秒あたりに正規化）。赤=増えた、青=減った、黄黒の斜線=通行止め |
| `before/` | Before 側の同じ一式 |

`--map-png` の確認用地図にも、通行止め（黄黒の斜線）・発生源（緑の円）・行き先（紫の円）が描かれます。

![通行止めの差分](docs/ginza-closure-diff.png)

晴海通りの有楽町寄りを全面通行止めにしたときの差分。晴海通り（青）を通っていた人が、北側の並行する通り（赤）へ迂回しています。

![区画ごとの平均待ち時間](docs/ginza-zones-wait.png)

### ビューアのシナリオパネル

`--scenario` を付けて起動するか、左の「シナリオ・通行止め・結果」パネルでファイルを読み込みます。

- 発生源ごとに有効/無効、時刻と人数の折れ線（点の追加・削除）、行き先と重み、歩行速度を編集
- 「地図に置く」で、地図をクリックして発生源・行き先を、ドラッグして通行止め（線＋幅、または長方形）を置ける
- 通行止めの種類（全面・歩道のみ・車道のみ）の変更、全体の有効/無効の切り替え
- Before/After: 通行止めを無効にして走らせ「いまの結果を Before に」→ 有効にして「適用」→「差分ヒートマップ」
- 「CSV と画像を書き出す」で上の一式と、編集後のシナリオ（`scenario.toml`）を書き出す。「保存」で編集内容を TOML に保存

![scenario panel](docs/viewer-scenario.png)

## ロードマップ

- [x] MVP: CityGML 読込 → グリッド → フローフィールド → Social Force → CSV/PNG
- [x] 実データ検証（渋谷 4 メッシュの OBJ、CityGML 抜粋）
- [x] CityGML LOD2/LOD3 の `tran:TrafficArea`（車道部・歩道部・島）を使った歩道判定（銀座で検証）
- [ ] 横断歩道の位置（PLATEAU の道路データに横断歩道の面が無いので別データが要る）
- [x] シナリオ定義（TOML）: 出発地・目的地（駅出口など）・発生レート・時間帯、通行止め・工事、区画別の集計と Before/After 比較
- [x] 可視化: bevy で 3D 建物 + エージェントのリアルタイム表示
- [ ] 可視化の拡張: LOD2 の屋根形状・テクスチャ、地形 (dem)、wasm + WebGPU でブラウザ表示
- [ ] 速度向上: rayon による並列化、密度に応じた計算（数万人規模）
- [ ] 評価指標: 密度（人/m²）、流量、サービス水準 (LOS)、避難完了時間
- [x] 車両: 道路網抽出、IDM、交差点の通行権、歩行者との相互作用
- [ ] 信号機・横断歩道、車線数（多車線・右左折レーン）、駐車・路線バス
- [ ] 階段・建物入口の扱い、歩道の植栽・街路樹（AuxiliaryTrafficArea の種別）の反映
