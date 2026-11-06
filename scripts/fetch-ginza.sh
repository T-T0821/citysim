#!/bin/sh
# 銀座・有楽町・新橋を含む 3 次メッシュ 53394601 の公式 CityGML（東京都 23 区 2023 年度、LOD2 建物 + LOD3 道路）を取得する。
# G 空間情報センターの配布データを PLATEAU SDK for Unreal の GIS サンプルが Git LFS で同梱しているものをそのまま使う。
# 出典: 国土交通省 Project PLATEAU 3D 都市モデル（東京都 23 区）CC BY 4.0
set -eu
dst=${1:-data/ginza}
mkdir -p "$dst"
base="https://media.githubusercontent.com/media/Synesthesias/PLATEAU-SDK-for-Unreal-GISSample/main/Content/PLATEAU/Datasets/13101_chiyoda-ku_city_2023_citygml_1_op/udx"
for f in bldg/53394601_bldg_6697_op.gml tran/53394601_tran_6697_op.gml; do
  echo "$f ..."
  curl -L --fail --progress-bar -o "$dst/$(basename "$f")" "$base/$f"
done
ls -la "$dst"
