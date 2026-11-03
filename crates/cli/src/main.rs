//! plateau-crowd: PLATEAU CityGML を読み込み、Social Force Model で人流を計算するヘッドレス CLI。
//!
//! 例:
//!   plateau-crowd --gml data/53394611_bldg_6697.gml --gml data/53394611_tran_6697.gml \
//!       --agents 2000 --duration 600 --csv out.csv --png heatmap.png

mod png;

use anyhow::{bail, Context, Result};
use crowd::{Agent, Params, Rng, Simulation};
use glam::Vec2;
use plateau_gml::CityModel;
use std::io::Write;
use std::path::PathBuf;
use walkmap::{FlowField, WalkGrid};

struct Args {
    gml: Vec<PathBuf>,
    agents: usize,
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
            "--cell" => a.cell = v()?.parse()?,
            "--duration" => a.duration = v()?.parse()?,
            "--dt" => a.dt = v()?.parse()?,
            "--seed" => a.seed = v()?.parse()?,
            "--csv" => a.csv = Some(v()?.into()),
            "--csv-every" => a.csv_every = v()?.parse()?,
            "--png" => a.png = Some(v()?.into()),
            "-h" | "--help" => {
                println!(
                    "usage: plateau-crowd --gml <file|dir>... [--agents N] [--cell M] [--duration S] [--dt S]\n\
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

    // 目的地: 対象範囲の東西南北の縁にある歩行可能セル（通過交通を想定）
    let (w, h) = (grid.w, grid.h);
    let edges: [(&str, Box<dyn Fn(usize, usize) -> bool>); 4] = [
        ("西", Box::new(|x, _| x < 2)),
        ("東", Box::new(move |x, _| x + 3 > w)),
        ("南", Box::new(|_, y| y < 2)),
        ("北", Box::new(move |_, y| y + 3 > h)),
    ];
    let mut flows = vec![];
    for (name, pred) in edges.iter() {
        let cells: Vec<_> = grid.walkable_cells().filter(|&(x, y)| pred(x, y)).collect();
        if cells.is_empty() {
            continue;
        }
        eprintln!("目的地 {name}: {} セル", cells.len());
        flows.push(FlowField::compute(&grid, &cells));
    }
    if flows.is_empty() {
        bail!("範囲の縁に歩行可能領域がありません");
    }

    let mut sim = Simulation::new(grid, flows, Params::default());
    let mut rng = Rng::new(args.seed);
    let cells: Vec<_> = sim.grid.walkable_cells().collect();
    let mut tries = 0;
    while sim.agents.len() < args.agents && tries < args.agents * 100 {
        tries += 1;
        let (cx, cy) = cells[(rng.next_u64() % cells.len() as u64) as usize];
        let pos = sim.grid.center(cx, cy)
            + Vec2::new(rng.range(-0.2, 0.2), rng.range(-0.2, 0.2)) * sim.grid.cell;
        let goal = (rng.next_u64() % sim.goals.len() as u64) as usize;
        if sim.grid.obstacle_sample(pos).0 < 0.35
            || !sim.goals[goal].distance(&sim.grid, pos).is_finite()
        {
            continue;
        }
        if sim
            .agents
            .iter()
            .any(|a| a.pos.distance_squared(pos) < 0.36)
        {
            continue;
        }
        sim.agents.push(Agent {
            pos,
            vel: Vec2::ZERO,
            radius: rng.range(0.22, 0.28),
            desired_speed: rng.normal(1.34, 0.26).clamp(0.6, 2.0),
            goal,
            arrived: false,
        });
    }
    eprintln!("エージェント: {} 人", sim.agents.len());

    let mut csv = match &args.csv {
        Some(p) => {
            let mut f = std::io::BufWriter::new(std::fs::File::create(p)?);
            writeln!(f, "t,id,lat,lon,x,y,vx,vy")?;
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
                        "{:.2},{i},{lat:.8},{lon:.8},{:.3},{:.3},{:.3},{:.3}",
                        sim.time, a.pos.x, a.pos.y, a.vel.x, a.vel.y
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
