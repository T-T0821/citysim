#!/usr/bin/env python3
"""PLATEAU の CityGML から範囲を切り出し、シミュレーションに要る要素だけ残した小さな GML を作る。

使い方: extract_gml.py 入力.gml 出力.gml 中心緯度 中心経度 半径m [--keep-lod2]

- 範囲に一部でもかかる cityObjectMember だけ残す
- 建物は LOD0 / LOD1 と属性だけ残し、LOD2 の面・テクスチャ（appearance）は落とす（--keep-lod2 で残す）
- 道路は TrafficArea（LOD2/LOD3）を残す。2D の計算には平面ポリゴンの LOD2 で足りるので、LOD2 がある区分面の LOD3（三角形メッシュ）と LOD1 路面は落とす
- LOD3 しかない区分面は三角形メッシュなので、shapely があれば 1 つの平面ポリゴンに結合する（無ければそのまま）
- 既定で詳細属性 (uro:*)・ポリゴンの gml:id を落とし、座標を 1mm 単位に丸め、インデントを詰める（--full で元のまま）
"""
import math
import re
import sys

try:
    from shapely.geometry import Polygon as _SPoly
    from shapely.ops import unary_union as _union
except ImportError:  # 無くても動く（LOD3 の三角形はそのまま残る）
    _SPoly = None


def merge_lod3(block):
    """lod3MultiSurface の三角形メッシュを結合して、同じ範囲の平面ポリゴン群 (MultiSurface) に置き換える。"""
    if _SPoly is None:
        return block
    tris = []
    hs = []
    lat0 = lon0 = None
    for pl in re.findall(r"<gml:posList>([^<]*)</gml:posList>", block):
        v = [float(x) for x in pl.split()]
        pts = [(v[i], v[i + 1]) for i in range(0, len(v) - 2, 3)]
        hs.extend(v[2::3])
        if lat0 is None:
            lat0, lon0 = pts[0]
        k = 111_320.0
        xy = [((lon - lon0) * k * math.cos(math.radians(lat0)), (lat - lat0) * k) for lat, lon in pts]
        if len(xy) >= 3:
            tris.append(_SPoly(xy).buffer(0))
    if not tris:
        return block
    u = _union(tris).buffer(0.01).buffer(-0.01).simplify(0.02)
    polys = [u] if u.geom_type == "Polygon" else [g for g in getattr(u, "geoms", []) if g.geom_type == "Polygon"]
    h = sum(hs) / len(hs) if hs else 0.0
    k = 111_320.0

    def ring(coords):
        out = []
        for x, y in coords:
            out.append(f"{lat0 + y / k:.8f} {lon0 + x / (k * math.cos(math.radians(lat0))):.8f} {h:.2f}")
        return "<gml:LinearRing><gml:posList>" + " ".join(out) + "</gml:posList></gml:LinearRing>"

    members = []
    for g in polys:
        if g.area < 0.05:
            continue
        inner = "".join(f"<gml:interior>{ring(i.coords)}</gml:interior>" for i in g.interiors if _SPoly(i.coords).area >= 0.05)
        members.append(f"<gml:surfaceMember><gml:Polygon><gml:exterior>{ring(g.exterior.coords)}</gml:exterior>{inner}</gml:Polygon></gml:surfaceMember>")
    if not members:
        return block
    return "<tran:lod3MultiSurface><gml:MultiSurface>\n" + "\n".join(members) + "\n</gml:MultiSurface></tran:lod3MultiSurface>"


def round_pos(m):
    v = m.group(1).split()
    out = []
    for i in range(0, len(v) - 2, 3):
        out.append(f"{float(v[i]):.8f} {float(v[i + 1]):.8f} {float(v[i + 2]):.2f}")
    return "<gml:posList>" + " ".join(out) + "</gml:posList>"


def compact(m):
    # LOD0 のフットプリントがあれば LOD1 立体は要らない
    if "<bldg:lod0RoofEdge>" in m or "<bldg:lod0FootPrint>" in m:
        m = re.sub(r"\s*<bldg:lod1Solid>.*?</bldg:lod1Solid>", "", m, flags=re.S)
    # LOD2 がある区分面の LOD3 面（三角形メッシュで大きい）と、区分面がある道路の LOD1 路面
    def strip_lod3(t):
        a = t.group(0)
        if "<tran:lod2MultiSurface>" in a:
            a = re.sub(r"\s*<tran:lod3MultiSurface>.*?</tran:lod3MultiSurface>", "", a, flags=re.S)
        else:
            a = re.sub(r"<tran:lod3MultiSurface>.*?</tran:lod3MultiSurface>", lambda t: merge_lod3(t.group(0)), a, flags=re.S)
        return a
    m = re.sub(r"<tran:(?:Auxiliary)?TrafficArea .*?</tran:(?:Auxiliary)?TrafficArea>", strip_lod3, m, flags=re.S)
    if "<tran:TrafficArea" in m:
        m = re.sub(r"\s*<tran:lod1MultiSurface>.*?</tran:lod1MultiSurface>", "", m, flags=re.S)
    m = re.sub(r"\s*<bldg:address>.*?</bldg:address>", "", m, flags=re.S)
    m = re.sub(r"\s*<uro:(\w+)(?:\s[^>]*)?>.*?</uro:\1>", "", m, flags=re.S)
    m = re.sub(r"<(gml:Polygon|gml:LinearRing|gml:MultiSurface|gml:CompositeSurface|gml:Solid) gml:id=\"[^\"]*\"", r"<\1", m)
    m = re.sub(r"<gml:posList>([^<]*)</gml:posList>", round_pos, m)
    m = re.sub(r"\n\t+", "\n", m)
    return m


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    keep_lod2 = "--keep-lod2" in sys.argv
    full = "--full" in sys.argv
    src, dst, lat, lon, r = args[0], args[1], float(args[2]), float(args[3]), float(args[4])
    dlat = r / 111_320.0
    dlon = r / (111_320.0 * __import__("math").cos(__import__("math").radians(lat)))
    b = (lat - dlat, lon - dlon, lat + dlat, lon + dlon)

    data = open(src, "rb").read().decode("utf-8")
    head_end = data.index("<core:cityObjectMember>")
    head = data[:head_end]
    tail_start = data.rindex("</core:cityObjectMember>") + len("</core:cityObjectMember>")
    tail = data[tail_start:]
    # テクスチャ (appearance) は使わない
    tail = re.sub(r"\s*<app:appearanceMember>.*?</app:appearanceMember>", "", tail, flags=re.S)
    head = re.sub(r"\s*<app:appearanceMember>.*?</app:appearanceMember>", "", head, flags=re.S)
    members = re.findall(r"<core:cityObjectMember>.*?</core:cityObjectMember>", data[head_end:tail_start], re.S)

    out = []
    kept = 0
    for m in members:
        nums = []
        for pl in re.findall(r"<gml:posList>([^<]*)</gml:posList>", m):
            v = pl.split()
            nums.extend((float(v[i]), float(v[i + 1])) for i in range(0, len(v) - 2, 3))
        if not nums:
            continue
        lat0 = min(p[0] for p in nums)
        lat1 = max(p[0] for p in nums)
        lon0 = min(p[1] for p in nums)
        lon1 = max(p[1] for p in nums)
        if lat1 < b[0] or lat0 > b[2] or lon1 < b[1] or lon0 > b[3]:
            continue
        if not keep_lod2:
            m = re.sub(r"\s*<bldg:lod2Solid>.*?</bldg:lod2Solid>", "", m, flags=re.S)
            m = re.sub(r"\s*<bldg:boundedBy>.*?</bldg:boundedBy>", "", m, flags=re.S)
            m = re.sub(r"\s*<bldg:outerBuildingInstallation>.*?</bldg:outerBuildingInstallation>", "", m, flags=re.S)
            m = re.sub(r"\s*<bldg:lod2Geometry>.*?</bldg:lod2Geometry>", "", m, flags=re.S)
        if not full:
            m = compact(m)
        out.append(m)
        kept += 1

    # 範囲の boundedBy を書き換える
    head = re.sub(
        r"<gml:lowerCorner>[^<]*</gml:lowerCorner>",
        f"<gml:lowerCorner>{b[0]:.7f} {b[1]:.7f} 0</gml:lowerCorner>",
        head,
    )
    head = re.sub(
        r"<gml:upperCorner>[^<]*</gml:upperCorner>",
        f"<gml:upperCorner>{b[2]:.7f} {b[3]:.7f} 0</gml:upperCorner>",
        head,
    )
    with open(dst, "w", encoding="utf-8") as f:
        f.write(head)
        f.write("\n\t".join(out))
        f.write(tail)
    print(f"{dst}: {kept}/{len(members)} 地物", file=sys.stderr)


if __name__ == "__main__":
    main()
