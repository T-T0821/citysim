//! よく使うシナリオの組み立て。

use crate::{Agent, Params, Rng, Simulation};
use glam::Vec2;
use walkmap::{FlowField, WalkGrid};

/// 目的地の名前（ログ・凡例用）。`Simulation::goals` と同じ順。
/// 歩道幅 [m]（道路端からこの距離までを歩道とみなす）
pub const SIDEWALK_W: f32 = 3.0;

pub type GoalNames = Vec<&'static str>;

/// 通過交通シナリオ: 範囲の東西南北の縁を目的地とし、
/// 歩行可能セルにランダムに配置した `n_agents` 人がランダムな縁へ向かう。
pub fn through_traffic(
    mut grid: WalkGrid,
    n_agents: usize,
    seed: u64,
) -> Option<(Simulation, GoalNames)> {
    // 道路データがあれば車道を区別し、歩行者は歩道を優先する（車道横断はコスト 6 倍）
    if grid.has_roads {
        grid.mark_carriageway(SIDEWALK_W, 6.0);
    }
    let on_sidewalk =
        |g: &WalkGrid, x: usize, y: usize| g.carriageway.is_empty() || !g.carriageway[g.idx(x, y)];
    let (w, h) = (grid.w, grid.h);
    let edges: [(&'static str, Box<dyn Fn(usize, usize) -> bool>); 4] = [
        ("西", Box::new(|x, _| x < 2)),
        ("東", Box::new(move |x, _| x + 3 > w)),
        ("南", Box::new(|_, y| y < 2)),
        ("北", Box::new(move |_, y| y + 3 > h)),
    ];
    let mut targets = vec![];
    let mut names = vec![];
    for (name, pred) in edges.iter() {
        let mut cells: Vec<_> = grid.walkable_cells().filter(|&(x, y)| pred(x, y)).collect();
        if cells.iter().any(|&(x, y)| on_sidewalk(&grid, x, y)) {
            cells.retain(|&(x, y)| on_sidewalk(&grid, x, y));
        }
        if cells.is_empty() {
            continue;
        }
        targets.push(cells);
        names.push(*name);
    }
    // 4 方向のフローフィールドは独立なので並列に計算する
    let flows: Vec<FlowField> =
        crate::par::map(targets.len(), |k| FlowField::compute(&grid, &targets[k]));
    if flows.is_empty() {
        return None;
    }

    let mut sim = Simulation::new(grid, flows, Params::default());
    let mut rng = Rng::new(seed);
    let mut cells: Vec<_> = sim.grid.walkable_cells().collect();
    let cell_area = sim.grid.cell * sim.grid.cell;
    let sidewalk: Vec<_> = cells
        .iter()
        .copied()
        .filter(|&(x, y)| on_sidewalk(&sim.grid, x, y))
        .collect();
    // 歩道だけで 1.5 人/m^2 を超える人数なら車道にも置く（10 万人規模）
    if !sidewalk.is_empty() && (n_agents as f32) < sidewalk.len() as f32 * cell_area * 1.5 {
        cells = sidewalk;
    }
    // 既に置いた人の近く (MIN_GAP 以内) を避けるための格子（全員と比べると大人数で O(N²) になる）。
    // 通常は 0.6m あけ、大人数で入りきらないときは詰める（最小 0.35m）
    let area = cells.len() as f32 * cell_area;
    let min_gap = (area / n_agents.max(1) as f32 * 0.5)
        .sqrt()
        .clamp(0.35, 0.6);
    let gap2 = min_gap * min_gap;
    let mut placed: std::collections::HashMap<(i32, i32), Vec<Vec2>> = Default::default();
    let key = |p: Vec2| {
        (
            (p.x / min_gap).floor() as i32,
            (p.y / min_gap).floor() as i32,
        )
    };
    let mut tries = 0;
    while sim.agents.len() < n_agents && tries < n_agents * 30 {
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
        let (kx, ky) = key(pos);
        let crowded = (-1..=1).any(|dy| {
            (-1..=1).any(|dx| {
                placed
                    .get(&(kx + dx, ky + dy))
                    .is_some_and(|v| v.iter().any(|q| q.distance_squared(pos) < gap2))
            })
        });
        if crowded {
            continue;
        }
        placed.entry((kx, ky)).or_default().push(pos);
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
