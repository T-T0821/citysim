# 銀座 4 丁目交差点周辺の PLATEAU CityGML（抜粋）

東京都 23 区（2023 年度）の 3D 都市モデル、3 次メッシュ 53394601 から、銀座 4 丁目交差点
（北緯 35.67165, 東経 139.76500）を中心とする半径 260m を `scripts/extract_gml.py` で切り出したもの。

- `53394601_bldg_6697_op.gml`: 建物 457 棟。LOD0 のフットプリントと計測高さ・階数など。LOD2 の形状とテクスチャは省いた。
- `53394601_tran_6697_op.gml`: 道路 160 件。LOD2/LOD3 の `tran:TrafficArea`（車道部 1000・島 1020・歩道部 2000）と
  `tran:AuxiliaryTrafficArea`。LOD3 しかない区分面は三角形メッシュを平面ポリゴンに結合してある。

出典: 国土交通省 Project PLATEAU「3D 都市モデル（Project PLATEAU）東京都 23 区（2023 年度）」を加工して作成。
[PLATEAU サイトポリシー](https://www.mlit.go.jp/plateau/site-policy/)（CC BY 4.0 互換）に従う。
取得元は PLATEAU SDK for Unreal の GIS サンプル（Synesthesias/PLATEAU-SDK-for-Unreal-GISSample）が Git LFS で同梱している同データ。
