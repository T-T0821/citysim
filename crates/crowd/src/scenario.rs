//! よく使うシナリオの組み立て。

use crate::{Agent, Params, Rng, Simulation};
use glam::Vec2;
use walkmap::{FlowField, WalkGrid};

/// 目的地の名前（ログ・凡例用）。`Simulation::goals` と同じ順。
pub type GoalNames = Vec<&'static str>;

/// 通過交通シナリオ: 範囲の東西南北の縁を目的地とし、
/// 歩行可能セルにランダムに配置した `n_agents` 人がランダムな縁へ向かう。
pub fn through_traffic(
    grid: WalkGrid,
    n_agents: usize,
    seed: u64,
) -> Option<(Simulation, GoalNames)> {
    let (w, h) = (grid.w, grid.h);
    let edges: [(&'static str, Box<dyn Fn(usize, usize) -> bool>); 4] = [
        ("西", Box::new(|x, _| x < 2)),
        ("東", Box::new(move |x, _| x + 3 > w)),
        ("南", Box::new(|_, y| y < 2)),
        ("北", Box::new(move |_, y| y + 3 > h)),
    ];
    let mut flows = vec![];
    let mut names = vec![];
    for (name, pred) in edges.iter() {
        let cells: Vec<_> = grid.walkable_cells().filter(|&(x, y)| pred(x, y)).collect();
        if cells.is_empty() {
            continue;
        }
        flows.push(FlowField::compute(&grid, &cells));
        names.push(*name);
    }
    if flows.is_empty() {
        return None;
    }

    let mut sim = Simulation::new(grid, flows, Params::default());
    let mut rng = Rng::new(seed);
    let cells: Vec<_> = sim.grid.walkable_cells().collect();
    let mut tries = 0;
    while sim.agents.len() < n_agents && tries < n_agents * 100 {
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
    Some((sim, names))
}
