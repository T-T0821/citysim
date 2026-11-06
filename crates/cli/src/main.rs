//! plateau-crowd: PLATEAU CityGML を読み込み、Social Force Model で人流を計算するヘッドレス CLI。
//!
//! 例:
//!   plateau-crowd --gml data/53394611_bldg_6697.gml --gml data/53394611_tran_6697.gml \
//!       --agents 2000 --duration 600 --csv out.csv --png heatmap.png

mod png;

use anyhow::{bail, Context, Result};
use crowd::Simulation;
use std::io::Write;
use std::path::PathBuf;
use walkmap::WalkGrid;

struct Args {
    gml: Vec<PathBuf>,
    obj: Vec<PathBuf>,
    obj_origin: Option<(f64, f64)>,
    crop: Option<(f64, f64, f64)>,
    agents: usize,
    vehicles: usize,
    cell: f32,
    duration: f32,
    dt: f32,
    seed: u64,
    csv: Option<PathBuf>,
    csv_every: f32,
    png: Option<PathBuf>,
    map_png: Option<PathBuf>,
}

fn parse_args() -> Result<Args> {
    let mut a = Args {
        gml: vec![],
        obj: vec![],
        obj_origin: None,
        crop: None,
        agents: 300,
        vehicles: 40,
        cell: 0.5,
        duration: 300.0,
        dt: 0.05,
        seed: 1,
        csv: None,
        csv_every: 1.0,
        png: None,
        map_png: None,
    };
    let mut it = std::env::args().skip(1);
    while let Some(k) = it.next() {
        let mut v = || it.next().with_context(|| format!("{k} に値がありません"));
        match k.as_str() {
            "--gml" => {
                let p = PathBuf::from(v()?);
                if p.is_dir() {
                    for e in std::fs::read_dir(&p)? {
                        let e = e?.path();
                        if e.extension().is_some_and(|x| x == "gml") {
                            a.gml.push(e);
                        }
                    }
                } else {
                    a.gml.push(p);
                }
            }
            // PLATEAU SDK for Unity などが書き出した OBJ（bldg_* / tran_* グループ付き）
            "--obj" => {
                let p = PathBuf::from(v()?);
                if p.is_dir() {
                    for e in std::fs::read_dir(&p)? {
                        let e = e?.path();
                        if e.extension().is_some_and(|x| x == "obj") {
                            a.obj.push(e);
                        }
                    }
                } else {
                    a.obj.push(p);
                }
            }
            // OBJ の原点の緯度,経度
            "--obj-origin" => a.obj_origin = Some(parse_pair(&v()?)?),
            // 切り出し範囲: 中心の緯度,経度,半径[m]
            "--crop" => {
                let s = v()?;
                let x: Vec<f64> = s
                    .split(',')
                    .map(|t| t.trim().parse())
                    .collect::<std::result::Result<_, _>>()?;
                if x.len() != 3 {
                    bail!("--crop は 緯度,経度,半径m");
                }
                a.crop = Some((x[0], x[1], x[2]));
            }
            "--agents" => a.agents = v()?.parse()?,
            "--vehicles" => a.vehicles = v()?.parse()?,
            "--cell" => a.cell = v()?.parse()?,
            "--duration" => a.duration = v()?.parse()?,
            "--dt" => a.dt = v()?.parse()?,
            "--seed" => a.seed = v()?.parse()?,
            "--csv" => a.csv = Some(v()?.into()),
            "--csv-every" => a.csv_every = v()?.parse()?,
            "--png" => a.png = Some(v()?.into()),
            // 歩行空間・車道・道路網を描いた確認用の地図（北が上）
            "--map-png" => a.map_png = Some(v()?.into()),
            "-h" | "--help" => {
                println!(
                    "usage: plateau-crowd (--gml <file|dir> | --obj <file|dir> --obj-origin 緯度,経度)... [--crop 緯度,経度,半径m] [--agents N] [--vehicles N] [--cell M] [--duration S] [--dt S]\n\
                     \x20                    [--seed N] [--csv out.csv] [--csv-every S] [--png heatmap.png]"
                );
                std::process::exit(0);
            }
            _ => bail!("不明な引数: {k}"),
        }
    }
    if a.gml.is_empty() && a.obj.is_empty() {
        bail!("--gml か --obj を 1 つ以上指定してください (--help)");
    }
    if !a.obj.is_empty() && a.obj_origin.is_none() {
        bail!("--obj には --obj-origin 緯度,経度 が必要です");
    }
    Ok(a)
}

fn parse_pair(s: &str) -> Result<(f64, f64)> {
    let x: Vec<f64> = s
        .split(',')
        .map(|t| t.trim().parse())
        .collect::<std::result::Result<_, _>>()?;
    if x.len() != 2 {
        bail!("緯度,経度 の形式で指定してください: {s}");
    }
    Ok((x[0], x[1]))
}

/// 入力ファイルを読み、必要なら範囲を切り出す。戻り値はモデルとグリッド化する範囲。
fn load_model(args: &Args) -> Result<(plateau_gml::CityModel, Option<(f64, f64, f64, f64)>)> {
    let mut model = plateau_gml::CityModel::default();
    for p in &args.gml {
        model.merge(plateau_gml::parse_file(p)?);
    }
    if let Some((lat0, lon0)) = args.obj_origin {
        let frame = plateau_gml::obj::ObjFrame::unity_export(lat0, lon0);
        for p in &args.obj {
            model.merge(plateau_gml::obj::parse_file(p, frame)?);
        }
    }
    let bounds = args.crop.map(|(lat, lon, r)| {
        let dlat = r / 111_320.0;
        let dlon = r / (111_320.0 * lat.to_radians().cos());
        (lat - dlat, lon - dlon, lat + dlat, lon + dlon)
    });
    if let Some(b) = bounds {
        model.crop(b);
    }
    Ok((model, bounds))
}

fn main() -> Result<()> {
    let args = parse_args()?;
    let t0 = std::time::Instant::now();

    let (model, bounds) = load_model(&args)?;
    if let Some(b) = model.bounds() {
        eprintln!(
            "範囲: 緯度 {:.5}〜{:.5}, 経度 {:.5}〜{:.5}",
            b.0, b.2, b.1, b.3
        );
    }
    eprintln!(
        "読込: 建物 {} 棟, 道路 {} 件 ({:.2?})",
        model.buildings().count(),
        model.roads().count(),
        t0.elapsed()
    );

    let grid = WalkGrid::from_city_bounds(&model, args.cell, bounds)
        .context("座標を含む地物がありません")?;
    let n_walk = grid.walkable.iter().filter(|&&b| b).count();
    eprintln!(
        "グリッド: {}x{} ({:.0}m x {:.0}m), 歩行可能 {:.1}%",
        grid.w,
        grid.h,
        grid.w as f32 * grid.cell,
        grid.h as f32 * grid.cell,
        100.0 * n_walk as f32 / (grid.w * grid.h) as f32
    );
    if grid.has_roads {
        eprintln!(
            "歩道/車道: {}",
            if grid.carriageway_from_data {
                "LOD2/LOD3 の TrafficArea（車道部・歩道部）から"
            } else {
                "LOD1 路面から近似（道路端 3m を歩道）"
            }
        );
    }

    let mut traffic = if args.vehicles > 0 && grid.has_roads {
        let net = traffic::RoadGraph::from_grid(&grid, crowd::scenario::SIDEWALK_W);
        if let Some(p) = &args.map_png {
            write_map(&grid, Some(&net), p)?;
            eprintln!("地図: {}", p.display());
        }
        eprintln!(
            "道路網: 交差点 {} か所, 出入口 {} か所, 区間 {} 本",
            net.nodes.iter().filter(|n| n.edges.len() >= 3).count(),
            net.nodes.iter().filter(|n| n.border).count(),
            net.edges.len()
        );
        Some(traffic::Traffic::new(net, args.vehicles, args.seed))
    } else {
        if let Some(p) = &args.map_png {
            write_map(&grid, None, p)?;
            eprintln!("地図: {}", p.display());
        }
        None
    };

    let (mut sim, goal_names) = crowd::scenario::through_traffic(grid, args.agents, args.seed)
        .context("範囲の縁に歩行可能領域がありません")?;
    eprintln!("目的地: {}", goal_names.join(", "));
    eprintln!(
        "歩行者: {} 人, 車両: {} 台",
        sim.agents.len(),
        traffic.as_ref().map_or(0, |t| t.vehicles.len())
    );

    let mut csv = match &args.csv {
        Some(p) => {
            let mut f = std::io::BufWriter::new(std::fs::File::create(p)?);
            writeln!(f, "t,kind,id,lat,lon,x,y,vx,vy")?;
            Some(f)
        }
        None => None,
    };
    let mut heat = vec![0u32; sim.grid.w * sim.grid.h];
    let steps = (args.duration / args.dt).round() as usize;
    let csv_stride = ((args.csv_every / args.dt).round() as usize).max(1);
    let t_sim = std::time::Instant::now();
    for s in 0..=steps {
        if s % csv_stride == 0 {
            if let Some(f) = csv.as_mut() {
                for (i, a) in sim.agents.iter().enumerate().filter(|(_, a)| !a.arrived) {
                    let (lat, lon) = sim.grid.projection.to_geo(a.pos);
                    writeln!(
                        f,
                        "{:.2},ped,{i},{lat:.8},{lon:.8},{:.3},{:.3},{:.3},{:.3}",
                        sim.time, a.pos.x, a.pos.y, a.vel.x, a.vel.y
                    )?;
                }
                for (i, v) in traffic.iter().flat_map(|t| t.vehicles.iter().enumerate()) {
                    if !v.active {
                        continue;
                    }
                    let (lat, lon) = sim.grid.projection.to_geo(v.pos);
                    let vel = v.heading * v.v;
                    writeln!(
                        f,
                        "{:.2},car,{i},{lat:.8},{lon:.8},{:.3},{:.3},{:.3},{:.3}",
                        sim.time, v.pos.x, v.pos.y, vel.x, vel.y
                    )?;
                }
            }
        }
        for a in sim.agents.iter().filter(|a| !a.arrived) {
            if let Some((x, y)) = sim.grid.cell_of(a.pos) {
                heat[sim.grid.idx(x, y)] += 1;
            }
        }
        if s == steps || sim.active() == 0 {
            break;
        }
        if let Some(t) = traffic.as_mut() {
            let peds: Vec<_> = sim
                .agents
                .iter()
                .filter(|a| !a.arrived)
                .map(|a| a.pos)
                .collect();
            t.step(args.dt, &peds);
            sim.obstacles = t.obstacles();
        }
        sim.step(args.dt);
    }
    let arrived = sim.agents.len() - sim.active();
    eprintln!(
        "完了: t={:.1}s, 到着 {}/{} 人, 計算 {:.2?} ({:.1}x 実時間)",
        sim.time,
        arrived,
        sim.agents.len(),
        t_sim.elapsed(),
        sim.time / t_sim.elapsed().as_secs_f32()
    );

    if let Some(p) = &args.png {
        write_heatmap(&sim, &heat, p)?;
        eprintln!("ヒートマップ: {}", p.display());
    }
    Ok(())
}

/// 確認用の地図（北が上）。緑=地面, 灰=歩行可能, 黒=建物, 濃灰=車道, 白=道路中心線, 赤=交差点, 青=出入口。
fn write_map(
    grid: &WalkGrid,
    net: Option<&traffic::RoadGraph>,
    path: &std::path::Path,
) -> Result<()> {
    let (w, h) = (grid.w, grid.h);
    let mut img = vec![0u8; w * h * 3];
    let mut put = |x: i64, y: i64, c: [u8; 3]| {
        if x >= 0 && y >= 0 && (x as usize) < w && (y as usize) < h {
            let o = ((h - 1 - y as usize) * w + x as usize) * 3;
            img[o..o + 3].copy_from_slice(&c);
        }
    };
    let road_dist = |i: usize| {
        if grid.carriageway.is_empty() {
            grid.obstacle_dist[i] >= crowd::scenario::SIDEWALK_W
        } else {
            grid.carriageway[i]
        }
    };
    for y in 0..h {
        for x in 0..w {
            let i = grid.idx(x, y);
            let c = if !grid.walkable[i] {
                [60, 90, 60]
            } else if road_dist(i) {
                [70, 70, 75]
            } else {
                [200, 200, 195]
            };
            put(x as i64, y as i64, c);
        }
    }
    for b in &grid.buildings {
        for p in &b.footprint {
            walkmap::fill_polygon(p, grid.origin, grid.cell, w, h, |i| {
                put((i % w) as i64, (i / w) as i64, [25, 25, 30]);
            });
        }
    }
    if let Some(net) = net {
        let cell = |p: glam::Vec2| ((p - grid.origin) / grid.cell).floor().as_i64vec2();
        for e in &net.edges {
            for seg in e.pts.windows(2) {
                let (a, b) = (cell(seg[0]), cell(seg[1]));
                let n = (a - b).abs().max_element().max(1);
                for k in 0..=n {
                    let t = k as f32 / n as f32;
                    let x = a.x as f32 + (b.x - a.x) as f32 * t;
                    let y = a.y as f32 + (b.y - a.y) as f32 * t;
                    put(x.round() as i64, y.round() as i64, [255, 255, 255]);
                }
            }
        }
        for nd in &net.nodes {
            let c = cell(nd.pos);
            let col = if nd.border {
                [40, 120, 255]
            } else {
                [255, 40, 40]
            };
            for dy in -2..=2 {
                for dx in -2..=2 {
                    put(c.x + dx, c.y + dy, col);
                }
            }
        }
    }
    std::fs::write(path, png::encode_rgb(w as u32, h as u32, &img))?;
    Ok(())
}

/// 通行量ヒートマップ（北が上）。灰=歩行可能, 黒=建物/範囲外, 黄〜赤=通行量。
fn write_heatmap(sim: &Simulation, heat: &[u32], path: &std::path::Path) -> Result<()> {
    let g = &sim.grid;
    let max = heat.iter().copied().max().unwrap_or(1).max(1) as f32;
    let mut img = vec![0u8; g.w * g.h * 3];
    for y in 0..g.h {
        for x in 0..g.w {
            let i = g.idx(x, y);
            let o = ((g.h - 1 - y) * g.w + x) * 3;
            let c = if !g.walkable[i] {
                [30, 30, 36]
            } else if heat[i] == 0 {
                [200, 200, 200]
            } else {
                let t = (heat[i] as f32).ln_1p() / max.ln_1p();
                [255, (230.0 * (1.0 - t)) as u8, (60.0 * (1.0 - t)) as u8]
            };
            img[o..o + 3].copy_from_slice(&c);
        }
    }
    std::fs::write(path, png::encode_rgb(g.w as u32, g.h as u32, &img))?;
    Ok(())
}
