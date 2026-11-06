//! plateau-crowd: PLATEAU CityGML を読み込み、Social Force Model で人流を計算するヘッドレス CLI。
//!
//! 例:
//!   plateau-crowd --gml data/53394611_bldg_6697.gml --gml data/53394611_tran_6697.gml \
//!       --agents 2000 --duration 600 --csv out.csv --png heatmap.png

mod png;

use anyhow::{bail, Context, Result};
use crowd::Simulation;
use plateau_gml::CityModel;
use std::io::Write;
use std::path::PathBuf;
use walkmap::WalkGrid;

struct Args {
    gml: Vec<PathBuf>,
    agents: usize,
    vehicles: usize,
    cell: f32,
    duration: f32,
    dt: f32,
    seed: u64,
    csv: Option<PathBuf>,
    csv_every: f32,
    png: Option<PathBuf>,
}

fn parse_args() -> Result<Args> {
    let mut a = Args {
        gml: vec![],
        agents: 300,
        vehicles: 40,
        cell: 0.5,
        duration: 300.0,
        dt: 0.05,
        seed: 1,
        csv: None,
        csv_every: 1.0,
        png: None,
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
            "--agents" => a.agents = v()?.parse()?,
            "--vehicles" => a.vehicles = v()?.parse()?,
            "--cell" => a.cell = v()?.parse()?,
            "--duration" => a.duration = v()?.parse()?,
            "--dt" => a.dt = v()?.parse()?,
            "--seed" => a.seed = v()?.parse()?,
            "--csv" => a.csv = Some(v()?.into()),
            "--csv-every" => a.csv_every = v()?.parse()?,
            "--png" => a.png = Some(v()?.into()),
            "-h" | "--help" => {
                println!(
                    "usage: plateau-crowd --gml <file|dir>... [--agents N] [--vehicles N] [--cell M] [--duration S] [--dt S]\n\
                     \x20                    [--seed N] [--csv out.csv] [--csv-every S] [--png heatmap.png]"
                );
                std::process::exit(0);
            }
            _ => bail!("不明な引数: {k}"),
        }
    }
    if a.gml.is_empty() {
        bail!("--gml を 1 つ以上指定してください (--help)");
    }
    Ok(a)
}

fn main() -> Result<()> {
    let args = parse_args()?;
    let t0 = std::time::Instant::now();

    let mut model = CityModel::default();
    for p in &args.gml {
        model.merge(plateau_gml::parse_file(p)?);
    }
    eprintln!(
        "読込: 建物 {} 棟, 道路 {} 件 ({:.2?})",
        model.buildings().count(),
        model.roads().count(),
        t0.elapsed()
    );

    let grid = WalkGrid::from_city(&model, args.cell).context("座標を含む地物がありません")?;
    let n_walk = grid.walkable.iter().filter(|&&b| b).count();
    eprintln!(
        "グリッド: {}x{} ({:.0}m x {:.0}m), 歩行可能 {:.1}%",
        grid.w,
        grid.h,
        grid.w as f32 * grid.cell,
        grid.h as f32 * grid.cell,
        100.0 * n_walk as f32 / (grid.w * grid.h) as f32
    );

    let mut traffic = if args.vehicles > 0 && grid.has_roads {
        let net = traffic::RoadGraph::from_grid(&grid, crowd::scenario::SIDEWALK_W);
        eprintln!(
            "道路網: 交差点 {} か所, 出入口 {} か所, 区間 {} 本",
            net.nodes.iter().filter(|n| n.edges.len() >= 3).count(),
            net.nodes.iter().filter(|n| n.border).count(),
            net.edges.len()
        );
        Some(traffic::Traffic::new(net, args.vehicles, args.seed))
    } else {
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
