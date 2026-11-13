//! 信号のある街で、歩行者が赤信号の横断歩道・交差点内へ踏み込まず、
//! 横断歩道以外の車道へはみ出さないこと（サンプルの街区で確かめる）。

use crowd::{scenario::through_traffic, Simulation};
use glam::Vec2;
use traffic::{RoadGraph, SignalConfig, Signals, Traffic};
use walkmap::WalkGrid;

struct Town {
    sim: Simulation,
    traffic: Traffic,
    /// 横断歩道・交差点内のセル
    crossing: Vec<bool>,
}

fn town(agents: usize) -> Town {
    let m = plateau_gml::parse_str(include_str!("../../../testdata/sample.gml")).unwrap();
    let grid = WalkGrid::from_city(&m, 0.5).unwrap();
    let (mut sim, _) = through_traffic(grid, agents, 5).unwrap();
    let net = RoadGraph::from_grid(&sim.grid, crowd::scenario::SIDEWALK_W);
    let mut traffic = Traffic::new(net, 20, 5);
    let s = Signals::build(
        &traffic.graph,
        &sim.grid,
        SignalConfig {
            cycle: 60.0,
            ..Default::default()
        },
    );
    assert!(!s.intersections.is_empty());
    s.apply_walk_costs(&mut sim.grid);
    sim.recompute_flows();
    sim.closed = vec![false; sim.grid.w * sim.grid.h];
    s.write_closed(0.0, &mut sim.closed);
    sim.curb = s.curb_cells(&sim.grid);
    let mut crossing = vec![false; sim.grid.w * sim.grid.h];
    for inter in &s.intersections {
        let cells = inter.crosswalks.iter().flat_map(|c| &c.cells);
        for &i in cells.chain(&inter.interior) {
            crossing[i as usize] = true;
        }
    }
    // 横断歩道以外の車道 = curb
    for (i, &c) in sim.curb.iter().enumerate() {
        assert_eq!(c, sim.grid.carriageway[i] && !crossing[i]);
    }
    traffic.set_signals(Some(s));
    Town {
        sim,
        traffic,
        crossing,
    }
}

#[test]
fn pedestrians_wait_for_green_and_stay_on_sidewalks() {
    let Town {
        mut sim,
        mut traffic,
        crossing,
    } = town(250);
    let dt = 0.05;
    let cell = |g: &WalkGrid, p: Vec2| g.cell_of(p).map(|(x, y)| g.idx(x, y));
    let (mut red_entries, mut curb_entries, mut on_crossing) = (0, 0, 0);
    let start_active = sim.active();
    for _ in 0..(150.0 / dt) as usize {
        let peds: Vec<Vec2> = sim
            .agents
            .iter()
            .filter(|a| !a.arrived)
            .map(|a| a.pos)
            .collect();
        traffic.step(dt, &peds);
        sim.obstacles = traffic.obstacles();
        traffic
            .signals
            .as_ref()
            .unwrap()
            .write_closed(traffic.time, &mut sim.closed);
        let before: Vec<_> = sim.agents.iter().map(|a| cell(&sim.grid, a.pos)).collect();
        sim.step(dt);
        for (a, b) in sim.agents.iter().zip(before) {
            let (Some(k0), Some(k)) = (b, cell(&sim.grid, a.pos)) else {
                continue;
            };
            if a.arrived || k0 == k {
                continue;
            }
            if sim.closed[k] && !sim.closed[k0] {
                red_entries += 1;
            }
            if sim.curb[k] && !sim.curb[k0] {
                curb_entries += 1;
            }
            if crossing[k] && !crossing[k0] {
                on_crossing += 1;
            }
        }
    }
    eprintln!("赤への踏み込み {red_entries}, 車道へのはみ出し {curb_entries}, 横断 {on_crossing}");
    assert_eq!(red_entries, 0, "赤の横断歩道・交差点内へ踏み込まない");
    assert!(on_crossing > 50, "青になれば横断歩道を渡る: {on_crossing}");
    // 経路が車道を通るしかない人を除き、横断歩道以外の車道へは出ない
    assert!(
        curb_entries * 20 <= on_crossing,
        "車道へのはみ出し {curb_entries} 回（横断 {on_crossing} 回）"
    );
    let arrived = start_active - sim.active();
    assert!(
        arrived > start_active / 3,
        "信号待ちがあっても進む: {arrived}"
    );
}

#[test]
fn evacuation_lifts_the_curb() {
    let Town { mut sim, .. } = town(20);
    assert!(!sim.curb.is_empty());
    let shelter = vec![("避難所".to_string(), Vec2::new(70.0, 70.0))];
    crowd::evacuation::Evacuation::start(&mut sim, &shelter, &Default::default()).unwrap();
    assert!(sim.closed.is_empty() && sim.curb.is_empty());
}
