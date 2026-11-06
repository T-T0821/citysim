//! plateau-crowd: PLATEAU CityGML を読み込み、Social Force Model で人流を計算するヘッドレス CLI。
//!
//! 例:
//!   plateau-crowd --gml data/53394611_bldg_6697.gml --gml data/53394611_tran_6697.gml \
//!       --agents 2000 --duration 600 --csv out.csv --png heatmap.png

mod png;

use anyhow::{bail, Context, Result};
use crowd::event::{Scenario, ScenarioFile};
use crowd::report::ZoneStats;
use crowd::Simulation;
use std::io::Write;
use std::path::PathBuf;
use walkmap::WalkGrid;

struct Args {
    gml: Vec<PathBuf>,
    obj: Vec<PathBuf>,
    obj_origin: Option<(f64, f64)>,
    crop: Option<(f64, f64, f64)>,
    agents: Option<usize>,
    vehicles: Option<usize>,
    cell: f32,
    duration: Option<f32>,
    dt: f32,
    seed: u64,
    csv: Option<PathBuf>,
    csv_every: f32,
    png: Option<PathBuf>,
    map_png: Option<PathBuf>,
    scenario: Option<PathBuf>,
    export: Option<PathBuf>,
    zone_size: Option<f32>,
    compare: bool,
    /// 信号機（None = 信号なし）
    signals: Option<traffic::SignalConfig>,
    place: Option<String>,
    radius: f64,
    data_dir: PathBuf,
    shelters: Vec<(String, f64, f64)>,
    evacuate_at: Option<f32>,
}

fn parse_args() -> Result<Args> {
    let mut a = Args {
        gml: vec![],
        obj: vec![],
        obj_origin: None,
        crop: None,
        agents: None,
        vehicles: None,
        cell: 0.5,
        duration: None,
        dt: 0.05,
        seed: 1,
        csv: None,
        csv_every: 1.0,
        png: None,
        map_png: None,
        scenario: None,
        export: None,
        zone_size: None,
        compare: false,
        signals: Some(traffic::SignalConfig::default()),
        place: None,
        radius: 220.0,
        data_dir: PathBuf::from("data/plateau"),
        shelters: vec![],
        evacuate_at: None,
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
            "--agents" => a.agents = Some(v()?.parse()?),
            "--vehicles" => a.vehicles = Some(v()?.parse()?),
            "--cell" => a.cell = v()?.parse()?,
            "--duration" => a.duration = Some(v()?.parse()?),
            "--dt" => a.dt = v()?.parse()?,
            "--seed" => a.seed = v()?.parse()?,
            "--csv" => a.csv = Some(v()?.into()),
            "--csv-every" => a.csv_every = v()?.parse()?,
            "--png" => a.png = Some(v()?.into()),
            // 歩行空間・車道・道路網を描いた確認用の地図（北が上）
            "--map-png" => a.map_png = Some(v()?.into()),
            // イベント・混雑シナリオ／通行止めの設定ファイル（TOML, scenarios/ を参照）
            "--scenario" => a.scenario = Some(v()?.into()),
            // 区画ごとの集計 CSV・画像を書き出すディレクトリ
            "--export" => a.export = Some(v()?.into()),
            // 集計区画の一辺 [m]（既定 20）
            "--zone-size" => a.zone_size = Some(v()?.parse()?),
            // 通行止めなし（Before）とあり（After）を続けて計算し、差分を出力する
            "--compare" => a.compare = true,
            // 信号機: auto（既定, 大交差点はスクランブル）| concurrent | exclusive | scramble | off
            "--signals" => {
                let m = v()?;
                a.signals = if m == "off" {
                    None
                } else {
                    let mode = traffic::PedMode::parse(&m).with_context(|| {
                        format!("--signals は auto|concurrent|exclusive|scramble|off: {m}")
                    })?;
                    Some(traffic::SignalConfig {
                        mode,
                        ..a.signals.unwrap_or_default()
                    })
                };
            }
            "--cycle" => {
                let x = v()?.parse()?;
                if let Some(c) = a.signals.as_mut() {
                    c.cycle = x;
                }
            }
            "--main-split" => {
                let x = v()?.parse()?;
                if let Some(c) = a.signals.as_mut() {
                    c.main_split = x;
                }
            }
            "--ped-green" => {
                let x = v()?.parse()?;
                if let Some(c) = a.signals.as_mut() {
                    c.ped_green = x;
                }
            }
            // 場所（住所 or 緯度,経度）。--gml が無ければ PLATEAU の CityGML を自動で取得する
            "--place" => a.place = Some(v()?),
            "--radius" => a.radius = v()?.parse()?,
            "--data-dir" => a.data_dir = v()?.into(),
            // 避難所: 緯度,経度[,名前]（複数可）
            "--shelter" => a.shelters.push(parse_shelter(&v()?)?),
            // 避難開始時刻 [s]（--shelter があれば既定 0）
            "--evacuate-at" => a.evacuate_at = Some(v()?.parse()?),
            "-h" | "--help" => {
                println!(
                    "usage: plateau-crowd (--gml <file|dir> | --obj <file|dir> --obj-origin 緯度,経度 | --place <住所|緯度,経度>)... [--crop 緯度,経度,半径m] [--agents N] [--vehicles N] [--cell M] [--duration S] [--dt S]\n\
                     \x20                    [--seed N] [--csv out.csv] [--csv-every S] [--png heatmap.png] [--map-png map.png]\n\
                     \x20                    [--scenario scenario.toml] [--export <dir>] [--zone-size M] [--compare]\n\
                     \x20                    [--signals auto|concurrent|exclusive|scramble|off] [--cycle S] [--main-split 0.6] [--ped-green S]\n\
                     \x20                    [--radius m] [--data-dir data/plateau] [--shelter 緯度,経度[,名前]]... [--evacuate-at 秒]"
                );
                std::process::exit(0);
            }
            _ => bail!("不明な引数: {k}"),
        }
    }
    if let Some(place) = a.place.clone() {
        let p = if a.gml.is_empty() && a.obj.is_empty() {
            // 入力が無ければ PLATEAU から取得（手元の --data-dir にあればそれを使う）
            let r = plateau_fetch::prepare(&place, a.radius, &a.data_dir, &mut |m| eprintln!("{m}"))?;
            a.gml.extend(r.files);
            r.place
        } else {
            plateau_fetch::resolve_place(&place)?
        };
        a.crop = Some((p.lat, p.lon, a.radius));
    }
    if a.gml.is_empty() && a.obj.is_empty() {
        bail!("--gml か --obj か --place を指定してください (--help)");
    }
    if !a.obj.is_empty() && a.obj_origin.is_none() {
        bail!("--obj には --obj-origin 緯度,経度 が必要です");
    }
    Ok(a)
}

fn parse_shelter(s: &str) -> Result<(String, f64, f64)> {
    let t: Vec<&str> = s.splitn(3, ',').collect();
    if t.len() < 2 {
        bail!("--shelter は 緯度,経度[,名前]: {s}");
    }
    let (lat, lon) = (t[0].trim().parse()?, t[1].trim().parse()?);
    let name = t.get(2).map_or_else(
        || format!("避難所({lat:.5},{lon:.5})"),
        |n| n.trim().to_string(),
    );
    Ok((name, lat, lon))
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

    let scenario = match &args.scenario {
        Some(p) => {
            let f = ScenarioFile::load(p).map_err(anyhow::Error::msg)?;
            let sc = Scenario::from_file(&f, &grid.projection).map_err(anyhow::Error::msg)?;
            eprintln!(
                "シナリオ: {} (発生源 {} か所, 行き先 {} か所, 通行止め {} か所)",
                if sc.name.is_empty() {
                    "(無名)"
                } else {
                    &sc.name
                },
                sc.sources.len(),
                sc.destinations.len(),
                sc.closures.len()
            );
            Some(sc)
        }
        None => None,
    };
    let opts = RunOpts {
        agents: args
            .agents
            .or(scenario.as_ref().and_then(|s| s.background_agents))
            .unwrap_or(300),
        vehicles: args
            .vehicles
            .or(scenario.as_ref().and_then(|s| s.vehicles))
            .unwrap_or(40),
        duration: args
            .duration
            .or(scenario.as_ref().and_then(|s| s.duration))
            .unwrap_or(300.0),
        zone_size: args
            .zone_size
            .or(scenario.as_ref().and_then(|s| s.zone_size))
            .unwrap_or(20.0),
    };

    let net = ((opts.vehicles > 0 || args.signals.is_some()) && grid.has_roads)
        .then(|| traffic::RoadGraph::from_grid(&grid, crowd::scenario::SIDEWALK_W));
    if let Some(net) = &net {
        eprintln!(
            "道路網: 交差点 {} か所, 出入口 {} か所, 区間 {} 本",
            net.nodes.iter().filter(|n| n.edges.len() >= 3).count(),
            net.nodes.iter().filter(|n| n.border).count(),
            net.edges.len()
        );
    }

    // --compare: まず通行止めなしで計算して Before にする
    let before = match &scenario {
        Some(sc) if args.compare => {
            eprintln!("--- Before（通行止めなし） ---");
            let mut open = sc.clone();
            open.closures_on = false;
            Some(simulate(
                &args,
                &opts,
                &grid,
                net.as_ref(),
                Some(&open),
                None,
                None,
            )?)
        }
        _ => {
            if args.compare {
                bail!("--compare には --scenario（通行止めを含むもの）が必要です");
            }
            None
        }
    };
    if before.is_some() {
        eprintln!("--- After（通行止めあり） ---");
    }

    let mut csv = match &args.csv {
        Some(p) => {
            let mut f = std::io::BufWriter::new(std::fs::File::create(p)?);
            writeln!(f, "t,kind,id,lat,lon,x,y,vx,vy")?;
            Some(f)
        }
        None => None,
    };
    let after = simulate(
        &args,
        &opts,
        &grid,
        net.as_ref(),
        scenario.as_ref(),
        csv.as_mut().map(|f| f as &mut dyn Write),
        args.map_png.as_deref(),
    )?;

    if let Some(p) = &args.png {
        write_heatmap(&after.sim, &after.heat, p)?;
        eprintln!("ヒートマップ: {}", p.display());
    }
    if let Some(dir) = &args.export {
        let clock = |t: f32| match &scenario {
            Some(sc) => sc.clock_label(t),
            None => format!("{t:.0} s"),
        };
        let files = after.stats.export_dir(
            &after.sim.grid,
            dir,
            before.as_ref().map(|b| &b.stats),
            &after.closed,
            clock,
        )?;
        if let Some(b) = &before {
            b.stats
                .export_dir(&b.sim.grid, &dir.join("before"), None, &[], clock)?;
        }
        eprintln!(
            "エクスポート: {} ({}{})",
            dir.display(),
            files.join(", "),
            if before.is_some() { ", before/" } else { "" }
        );
    }
    if let Some(b) = &before {
        print_compare(&b.stats, &after.stats);
    }
    Ok(())
}

struct RunOpts {
    agents: usize,
    vehicles: usize,
    duration: f32,
    zone_size: f32,
}

struct RunOut {
    sim: Simulation,
    heat: Vec<u32>,
    stats: ZoneStats,
    /// 通行止めで通れなくなったセル
    closed: Vec<bool>,
}

/// 1 回分のシミュレーション。シナリオがあれば通行止めを反映し、発生源から人を出す。
fn simulate(
    args: &Args,
    opts: &RunOpts,
    base: &WalkGrid,
    net: Option<&traffic::RoadGraph>,
    sc: Option<&Scenario>,
    mut csv: Option<&mut dyn Write>,
    map_png: Option<&std::path::Path>,
) -> Result<RunOut> {
    let mut grid = base.clone();
    if let Some(sc) = sc {
        let n = sc.apply_closures(&mut grid);
        if n > 0 {
            eprintln!(
                "通行止め: {} か所, 通れなくした面積 {:.0} m²",
                sc.active_closures().count(),
                n as f32 * grid.cell * grid.cell
            );
        }
    }
    let closed: Vec<bool> = base
        .walkable
        .iter()
        .zip(&grid.walkable)
        .map(|(&a, &b)| a && !b)
        .collect();
    let mut traffic = net.map(|n| {
        let n = match sc {
            Some(sc) => {
                let g = n.retain_edges(|e| !sc.blocks_road(&e.pts));
                if g.edges.len() < n.edges.len() {
                    eprintln!("車両通行止め: 区間 {} 本", n.edges.len() - g.edges.len());
                }
                g
            }
            None => n.clone(),
        };
        traffic::Traffic::new(n, opts.vehicles, args.seed)
    });

    let (mut sim, goal_names) = crowd::scenario::through_traffic(grid, opts.agents, args.seed)
        .context("範囲の縁に歩行可能領域がありません")?;
    if let (Some(t), Some(cfg)) = (traffic.as_mut(), &args.signals) {
        let s = traffic::Signals::build(&t.graph, &sim.grid, cfg.clone());
        let n_scr = s
            .intersections
            .iter()
            .filter(|i| i.plan == traffic::signal::Plan::Scramble)
            .count();
        eprintln!(
            "信号: {} か所（うちスクランブル {} か所）, 横断歩道 {} か所",
            s.intersections.len(),
            n_scr,
            s.intersections
                .iter()
                .map(|i| i.crosswalks.len())
                .sum::<usize>()
        );
        s.apply_walk_costs(&mut sim.grid);
        sim.recompute_flows();
        sim.closed = vec![false; sim.grid.w * sim.grid.h];
        t.set_signals(Some(s));
    }
    if let Some(p) = map_png {
        write_map(&sim.grid, traffic.as_ref(), sc, p)?;
        eprintln!("地図: {}", p.display());
    }
    let mut goal_names: Vec<String> = goal_names.iter().map(|s| s.to_string()).collect();
    let mut spawner = match sc {
        Some(sc) => Some(
            sc.install(&mut sim, &mut goal_names, args.seed)
                .map_err(anyhow::Error::msg)?,
        ),
        None => None,
    };
    eprintln!("目的地: {}", goal_names.join(", "));
    eprintln!(
        "歩行者: {} 人{}, 車両: {} 台",
        sim.agents.len(),
        if spawner.is_some() {
            "（＋発生源から）"
        } else {
            ""
        },
        traffic.as_ref().map_or(0, |t| t.vehicles.len())
    );
    if let Some(t) = &traffic {
        let mut n = std::collections::BTreeMap::new();
        for v in &t.vehicles {
            *n.entry(v.kind.label_ja()).or_insert(0) += 1;
        }
        eprintln!(
            "車種: {}",
            n.iter()
                .map(|(k, c)| format!("{k} {c}"))
                .collect::<Vec<_>>()
                .join(", ")
        );
    }
    let shelters: Vec<(String, glam::Vec2)> = args
        .shelters
        .iter()
        .map(|(n, lat, lon)| {
            let g = plateau_gml::GeoPoint {
                lat: *lat,
                lon: *lon,
                h: 0.0,
            };
            (n.clone(), sim.grid.projection.to_local(&g))
        })
        .collect();
    let evacuate_at = (!shelters.is_empty()).then(|| args.evacuate_at.unwrap_or(0.0));
    let mut evac: Option<crowd::evacuation::Evacuation> = None;
    let n_initial = sim.agents.len();
    // 発生源の折れ線が終わるまでは、全員到着しても止めない
    let spawn_until = sc.map_or(0.0, |s| s.last_time());

    let mut stats = ZoneStats::new(&sim.grid, opts.zone_size);
    let mut heat = vec![0u32; sim.grid.w * sim.grid.h];
    let steps = (opts.duration / args.dt).round() as usize;
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
                        "{:.2},{},{i},{lat:.8},{lon:.8},{:.3},{:.3},{:.3},{:.3}",
                        sim.time,
                        v.kind.name(),
                        v.pos.x,
                        v.pos.y,
                        vel.x,
                        vel.y
                    )?;
                }
            }
        }
        for a in sim.agents.iter().filter(|a| !a.arrived) {
            if let Some((x, y)) = sim.grid.cell_of(a.pos) {
                heat[sim.grid.idx(x, y)] += 1;
            }
        }
        if let (Some(at), None) = (evacuate_at, &evac) {
            if sim.time + 1e-4 >= at {
                let ev = crowd::evacuation::Evacuation::start(
                    &mut sim,
                    &shelters,
                    &Default::default(),
                )
                .context("避難所の近くに歩行空間がありません（範囲外か建物の中）")?;
                eprintln!(
                    "避難開始: t={:.1}s, 対象 {} 人, 避難所 {} か所{}",
                    sim.time,
                    ev.total,
                    ev.shelters.len(),
                    if ev.unreachable > 0 {
                        format!("（到達できない人 {}）", ev.unreachable)
                    } else {
                        String::new()
                    }
                );
                if let Some(t) = traffic.as_mut() {
                    t.halted = true;
                }
                evac = Some(ev);
            }
        }
        if let Some(ev) = evac.as_mut() {
            // 車が止まりきったら、止まった車を避ける経路に引き直す
            if let (false, Some(t)) = (ev.rerouted, &traffic) {
                if t.vehicles.iter().all(|v| !v.active || v.v == 0.0) {
                    ev.reroute_around(&mut sim, &t.obstacles());
                }
            }
            ev.update(&sim);
            if ev.done() {
                break;
            }
        }
        if s == steps || (sim.active() == 0 && sim.time >= spawn_until && evac.is_none()) {
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
            // 避難中（車が止まっている間）は信号に関係なく渡る
            if let (Some(s), false) = (&t.signals, t.halted) {
                s.write_closed(t.time, &mut sim.closed);
            }
        }
        // 避難中は発生源から新しい人を出さない
        if let (Some(sp), Some(sc), None) = (spawner.as_mut(), sc, &evac) {
            sp.step(sc, &mut sim, args.dt, 1.0);
        }
        sim.step(args.dt);
        stats.record(
            &sim,
            traffic
                .iter()
                .flat_map(|t| t.vehicles.iter().map(|v| (v.pos, v.active))),
            args.dt,
        );
    }
    // 発生源の人は到着後の枠を使い回すので、延べ人数で数える
    let total = n_initial + spawner.as_ref().map_or(0, |sp| sp.total_spawned() as usize);
    let arrived = total - sim.active();
    eprintln!(
        "完了: t={:.1}s, 到着 {}/{} 人{}, 計算 {:.2?} ({:.1}x 実時間, {:.0} ステップ/秒)",
        sim.time,
        arrived,
        total,
        spawner.as_ref().map_or(String::new(), |sp| format!(
            "（うち発生源から {} 人、出口で待ち {} 人）",
            sp.total_spawned(),
            sp.backlog()
        )),
        t_sim.elapsed(),
        sim.time / t_sim.elapsed().as_secs_f32(),
        sim.time / args.dt / t_sim.elapsed().as_secs_f32()
    );
    if let Some(ev) = &evac {
        let fmt = |t: Option<f32>| t.map_or("未完了".to_string(), |t| format!("{t:.0} 秒"));
        eprintln!(
            "避難: 完了 {}/{} 人, 50% {} / 90% {} / 全員 {}",
            ev.evacuated(),
            ev.total - ev.unreachable,
            fmt(ev.time_to_fraction(0.5)),
            fmt(ev.time_to_fraction(0.9)),
            fmt(ev.time_to_fraction(1.0)),
        );
        for s in &ev.shelters {
            eprintln!("  {}: {} 人", s.name, s.arrived);
        }
    }
    Ok(RunOut {
        sim,
        heat,
        stats,
        closed,
    })
}

/// Before/After の要約を表示する。
fn print_compare(b: &ZoneStats, a: &ZoneStats) {
    let sum = |s: &ZoneStats, f: &dyn Fn(&ZoneStats, usize) -> f32| -> f32 {
        (0..s.nx * s.ny).map(|z| f(s, z)).sum()
    };
    let passes = |s: &ZoneStats| s.ped_passes.iter().sum::<u32>().max(1) as f32;
    let mean_wait = |s: &ZoneStats| s.wait.iter().sum::<f64>() as f32 / passes(s);
    let mean_dwell = |s: &ZoneStats| s.dwell.iter().sum::<f64>() as f32 / passes(s);
    let peak = |s: &ZoneStats| {
        (0..s.nx * s.ny)
            .map(|z| s.peak_density(z))
            .fold(0.0, f32::max)
    };
    eprintln!("比較 (Before → After):");
    eprintln!(
        "  区画通過（延べ, /h）: {:.0} → {:.0}",
        sum(b, &|s, z| s.passes_per_h(z)),
        sum(a, &|s, z| s.passes_per_h(z))
    );
    eprintln!(
        "  平均待ち時間 / 区画: {:.2} s → {:.2} s",
        mean_wait(b),
        mean_wait(a)
    );
    eprintln!(
        "  平均滞留時間 / 区画: {:.2} s → {:.2} s",
        mean_dwell(b),
        mean_dwell(a)
    );
    eprintln!("  最大密度: {:.2} → {:.2} 人/m²", peak(b), peak(a));
    // 増えた区画の上位
    let mut d: Vec<(usize, f32)> = (0..a.nx * a.ny)
        .map(|z| (z, a.passes_per_h(z) - b.passes_per_h(z)))
        .collect();
    d.sort_by(|x, y| y.1.total_cmp(&x.1));
    let top: Vec<String> = d
        .iter()
        .take(5)
        .filter(|x| x.1 > 0.0)
        .map(|(z, v)| format!("({},{}) +{:.0}/h", z % a.nx, z / a.nx, v))
        .collect();
    if !top.is_empty() {
        eprintln!("  通過が増えた区画 (ix,iy): {}", top.join(", "));
    }
}

/// 確認用の地図（北が上）。緑=地面, 灰=歩行可能, 黒=建物, 濃灰=車道, 白=道路中心線, 赤=交差点, 青=出入口。
/// 信号があれば 縞=横断歩道, 黄=停止線, 紫=スクランブルの交差点内。
/// シナリオがあれば 黄黒の斜線=通行止め, 緑の円=発生源, 紫の円=行き先 も描く。
fn write_map(
    grid: &WalkGrid,
    traffic: Option<&traffic::Traffic>,
    sc: Option<&Scenario>,
    path: &std::path::Path,
) -> Result<()> {
    let net = traffic.map(|t| &t.graph);
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
    if let Some(s) = traffic.and_then(|t| t.signals.as_ref()) {
        for inter in &s.intersections {
            if inter.plan == traffic::signal::Plan::Scramble {
                for &i in &inter.interior {
                    let i = i as usize;
                    put((i % w) as i64, (i / w) as i64, [110, 80, 130]);
                }
            }
            for cw in &inter.crosswalks {
                for &i in &cw.cells {
                    let i = i as usize;
                    let q = grid.center(i % w, i / w) - cw.center;
                    // 横断方向に 0.5m おきの縞
                    if (q.dot(cw.across) / 0.5).floor() as i64 % 2 == 0 {
                        put((i % w) as i64, (i / w) as i64, [235, 235, 235]);
                    }
                }
            }
            for ap in &inter.approaches {
                let n = ap.dir.perp();
                let mut d = -ap.width.1;
                while d <= ap.width.0 {
                    let p = (ap.stop_pos + n * d - grid.origin) / grid.cell;
                    put(p.x as i64, p.y as i64, [250, 200, 30]);
                    d += grid.cell * 0.5;
                }
            }
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
    if let Some(sc) = sc {
        for c in sc.closures.iter().filter(|c| c.enabled) {
            for p in &c.polys {
                walkmap::fill_polygon(p, grid.origin, grid.cell, w, h, |i| {
                    let (x, y) = (i % w, i / w);
                    let col = if (x + y) % 6 < 3 {
                        [250, 210, 0]
                    } else {
                        [20, 20, 20]
                    };
                    put(x as i64, y as i64, col);
                });
            }
        }
        let mut disc = |c: glam::Vec2, r: f32, col: [u8; 3]| {
            let q = (c - grid.origin) / grid.cell;
            let rr = (r / grid.cell).max(3.0);
            for dy in -(rr as i64)..=(rr as i64) {
                for dx in -(rr as i64)..=(rr as i64) {
                    let d = ((dx * dx + dy * dy) as f32).sqrt();
                    if d <= rr && d >= rr - 2.5 {
                        put(q.x as i64 + dx, q.y as i64 + dy, col);
                    }
                }
            }
        };
        for s in &sc.sources {
            disc(s.pos, s.radius, [30, 200, 60]);
        }
        for d in &sc.destinations {
            disc(d.pos, d.radius, [200, 40, 220]);
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
