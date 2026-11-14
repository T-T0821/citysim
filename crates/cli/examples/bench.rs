//! Web デモ相当（銀座 4 丁目, 半径 220m, 信号あり, 車 40 台）で 1 ステップの内訳を計る。
//!   RAYON_NUM_THREADS=1 cargo run --release -p plateau-crowd --example bench -- <人数> [ステップ数]
//! 環境変数 LITE を付けると軽量モデル（密度場）で計る。Web 版は 1 スレッドなので RAYON_NUM_THREADS=1 で近い値になる。
use glam::Vec2;
use std::time::Instant;
use walkmap::WalkGrid;

fn main() {
    let n: usize = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(10000);
    let steps: usize = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(200);
    let mut model = plateau_gml::CityModel::default();
    for f in [
        "testdata/ginza/53394601_bldg_6697_op.gml",
        "testdata/ginza/53394601_tran_6697_op.gml",
    ] {
        model.merge(plateau_gml::parse_file(std::path::Path::new(f)).unwrap());
    }
    let (lat, lon, r) = (35.67165f64, 139.76500f64, 220.0f64);
    let dlat = r / 111_320.0;
    let dlon = r / (111_320.0 * lat.to_radians().cos());
    let b = (lat - dlat, lon - dlon, lat + dlat, lon + dlon);
    model.crop(b);
    let t0 = Instant::now();
    let grid = WalkGrid::from_city_bounds(&model, 0.5, Some(b)).unwrap();
    let walk = grid.walkable.iter().filter(|&&b| b).count() as f32 * 0.25;
    let net = traffic::RoadGraph::from_grid(&grid, crowd::scenario::SIDEWALK_W);
    let mut t = traffic::Traffic::new(net, 40, 1);
    let (mut sim, _) = crowd::scenario::through_traffic(grid, n, 1).unwrap();
    if std::env::var("LITE").is_ok() {
        sim.model = crowd::Model::Lite;
    }
    let s = traffic::Signals::build(&t.graph, &sim.grid, traffic::SignalConfig::default());
    s.apply_walk_costs(&mut sim.grid);
    sim.recompute_flows();
    sim.closed = vec![false; sim.grid.w * sim.grid.h];
    sim.curb = s.curb_cells(&sim.grid);
    t.set_signals(Some(s));
    eprintln!(
        "setup {:.2?}; walkable {:.0} m2; placed {} agents",
        t0.elapsed(),
        walk,
        sim.agents.len()
    );
    let mut heat = vec![0u32; sim.grid.w * sim.grid.h];
    let (mut tt, mut ts, mut th) = (0.0, 0.0, 0.0);
    for k in 0..steps {
        let a = Instant::now();
        let peds: Vec<Vec2> = sim
            .agents
            .iter()
            .filter(|a| !a.arrived)
            .map(|a| a.pos)
            .collect();
        t.step(0.05, &peds);
        sim.obstacles = t.obstacles();
        if let Some(s) = &t.signals {
            s.write_closed(t.time, &mut sim.closed);
        }
        let b = Instant::now();
        sim.step(0.05);
        let c = Instant::now();
        for a in sim.agents.iter() {
            if !a.arrived {
                if let Some((x, y)) = sim.grid.cell_of(a.pos) {
                    heat[sim.grid.idx(x, y)] += 1;
                }
            }
        }
        let d = Instant::now();
        if k >= 20 {
            tt += (b - a).as_secs_f64();
            ts += (c - b).as_secs_f64();
            th += (d - c).as_secs_f64();
        }
    }
    let m = (steps - 20) as f64 / 1000.0;
    eprintln!(
        "per step us: traffic {}  crowd {}  heat {}  active {}",
        (tt / m * 1000.0) as u64,
        (ts / m * 1000.0) as u64,
        (th / m * 1000.0) as u64,
        sim.active()
    );
}
