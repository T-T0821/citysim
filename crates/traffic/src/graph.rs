//! 車道マスクから道路ネットワーク（交差点ノード + 道路中心線エッジ）を抽出する。
//!
//! 1. 車道セルを Zhang–Suen 法で細線化して中心線を得る
//! 2. 分岐・端点の画素をノード、その間の画素列をエッジとして追跡
//! 3. 短いヒゲ（細線化の副産物）を刈り、次数 2 のノードを連結
//! 4. 範囲の縁で途切れる道路は縁まで延長し、車両の出入口にする

use glam::Vec2;
use std::collections::HashMap;
use walkmap::WalkGrid;

#[derive(Debug, Clone)]
pub struct Node {
    pub pos: Vec2,
    pub edges: Vec<usize>,
    /// 交差点の大きさの目安 [m]。停止線はノードからこの距離手前。
    pub radius: f32,
    /// 範囲の縁にある出入口
    pub border: bool,
}

#[derive(Debug, Clone)]
pub struct Edge {
    pub a: usize,
    pub b: usize,
    pub pts: Vec<Vec2>,
    cum: Vec<f32>,
}

impl Edge {
    fn new(a: usize, b: usize, pts: Vec<Vec2>) -> Self {
        let mut cum = Vec::with_capacity(pts.len());
        let mut acc = 0.0;
        for (i, p) in pts.iter().enumerate() {
            if i > 0 {
                acc += p.distance(pts[i - 1]);
            }
            cum.push(acc);
        }
        Self { a, b, pts, cum }
    }
    pub fn len(&self) -> f32 {
        *self.cum.last().unwrap_or(&0.0)
    }
    pub fn is_empty(&self) -> bool {
        self.len() <= 0.0
    }
    /// 始点 a からの距離 s における位置と進行方向 (a→b)。
    pub fn at(&self, s: f32) -> (Vec2, Vec2) {
        let s = s.clamp(0.0, self.len());
        let i = match self.cum.binary_search_by(|c| c.total_cmp(&s)) {
            Ok(i) => i.min(self.pts.len() - 2),
            Err(i) => i.saturating_sub(1).min(self.pts.len() - 2),
        };
        let (p, q) = (self.pts[i], self.pts[i + 1]);
        let seg = (self.cum[i + 1] - self.cum[i]).max(1e-6);
        let t = ((s - self.cum[i]) / seg).clamp(0.0, 1.0);
        (p.lerp(q, t), (q - p).normalize_or_zero())
    }
    pub fn other(&self, n: usize) -> usize {
        if self.a == n {
            self.b
        } else {
            self.a
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct RoadGraph {
    pub nodes: Vec<Node>,
    pub edges: Vec<Edge>,
}

impl RoadGraph {
    /// `grid.carriageway` が設定済みならそれを、無ければ道路端から `sidewalk_w` 以上のセルを車道とする。
    pub fn from_grid(grid: &WalkGrid, sidewalk_w: f32) -> Self {
        let (w, h) = (grid.w, grid.h);
        let mask: Vec<bool> = if grid.carriageway.is_empty() {
            (0..w * h)
                .map(|i| grid.walkable[i] && grid.obstacle_dist[i] >= sidewalk_w)
                .collect()
        } else {
            grid.carriageway.clone()
        };
        let skel = zhang_suen(mask, w, h);
        let mut g = trace(&skel, grid);
        let prune_len = 2.5 * sidewalk_w.max(3.0) + 6.0;
        g.cleanup(prune_len);
        g.extend_border_edges(grid);
        g
    }

    /// 端点の判定と縁への延長。
    fn extend_border_edges(&mut self, grid: &WalkGrid) {
        let size = Vec2::new(grid.w as f32, grid.h as f32) * grid.cell;
        let edge_dist = |p: Vec2| p.x.min(p.y).min(size.x - p.x).min(size.y - p.y);
        for n in 0..self.nodes.len() {
            if self.nodes[n].edges.len() != 1 {
                continue;
            }
            let p = self.nodes[n].pos;
            let od = grid.sample_field(&grid.obstacle_dist, p, 0.0);
            // 最寄りの障害物が範囲の縁そのものなら出入口
            if edge_dist(p) > od + 2.0 * grid.cell + 1.0 {
                continue;
            }
            let ei = self.nodes[n].edges[0];
            let e = &self.edges[ei];
            let (inner, end) = if e.a == n {
                (e.pts[1.min(e.pts.len() - 1)], e.pts[0])
            } else {
                (e.pts[e.pts.len().saturating_sub(2)], e.pts[e.pts.len() - 1])
            };
            let dir = (end - inner).normalize_or_zero();
            // 縁までの距離だけ延長（軸に沿った道路を想定し、最も近い縁へ）
            let target = end + dir * (edge_dist(end) / dir.abs().max_element().max(0.3));
            let target = target.clamp(Vec2::ZERO, size);
            let mut pts = e.pts.clone();
            if e.a == n {
                pts.insert(0, target);
            } else {
                pts.push(target);
            }
            let (a, b) = (e.a, e.b);
            self.edges[ei] = Edge::new(a, b, pts);
            self.nodes[n].pos = target;
            self.nodes[n].border = true;
            self.nodes[n].radius = 0.0;
        }
    }

    /// ヒゲの除去と次数 2 ノードの連結を、変化がなくなるまで繰り返す。
    fn cleanup(&mut self, prune_len: f32) {
        let mut alive_e = vec![true; self.edges.len()];
        loop {
            let mut changed = false;
            // ヒゲ: 片端が次数 1 で短いエッジ（ただし両端とも次数 1 の孤立線は残す）
            for ei in 0..self.edges.len() {
                if !alive_e[ei] {
                    continue;
                }
                let e = &self.edges[ei];
                let (da, db) = (self.nodes[e.a].edges.len(), self.nodes[e.b].edges.len());
                let spur = (da == 1) != (db == 1) && e.len() < prune_len;
                if spur || e.a == e.b {
                    alive_e[ei] = false;
                    let (a, b) = (e.a, e.b);
                    self.nodes[a].edges.retain(|&x| x != ei);
                    self.nodes[b].edges.retain(|&x| x != ei);
                    changed = true;
                }
            }
            // 次数 2 のノードを消して 2 本のエッジを 1 本に
            for n in 0..self.nodes.len() {
                if self.nodes[n].edges.len() != 2 {
                    continue;
                }
                let (e1, e2) = (self.nodes[n].edges[0], self.nodes[n].edges[1]);
                if e1 == e2 {
                    continue;
                }
                let (p1, p2) = (&self.edges[e1], &self.edges[e2]);
                // p1 を ...→n、p2 を n→... の向きにそろえて連結
                let mut a_pts = p1.pts.clone();
                let start = if p1.b == n {
                    p1.a
                } else {
                    a_pts.reverse();
                    p1.b
                };
                let mut b_pts = p2.pts.clone();
                let end = if p2.a == n {
                    p2.b
                } else {
                    b_pts.reverse();
                    p2.a
                };
                if start == n || end == n {
                    continue;
                }
                a_pts.pop();
                a_pts.extend(b_pts);
                self.edges[e1] = Edge::new(start, end, a_pts);
                alive_e[e2] = false;
                self.nodes[n].edges.clear();
                let ne = &mut self.nodes[end].edges;
                for x in ne.iter_mut() {
                    if *x == e2 {
                        *x = e1;
                    }
                }
                changed = true;
            }
            if !changed {
                break;
            }
        }
        // 詰め直し
        let mut emap = vec![usize::MAX; self.edges.len()];
        let mut edges = vec![];
        for (i, e) in self.edges.drain(..).enumerate() {
            if alive_e[i] {
                emap[i] = edges.len();
                edges.push(e);
            }
        }
        let mut nmap = vec![usize::MAX; self.nodes.len()];
        let mut nodes = vec![];
        for (i, mut n) in self.nodes.drain(..).enumerate() {
            if !n.edges.is_empty() {
                nmap[i] = nodes.len();
                n.edges = n.edges.iter().map(|&e| emap[e]).collect();
                nodes.push(n);
            }
        }
        for e in edges.iter_mut() {
            e.a = nmap[e.a];
            e.b = nmap[e.b];
            let pts = simplify(&smooth(&e.pts, 3), 0.25);
            *e = Edge::new(e.a, e.b, pts);
        }
        self.nodes = nodes;
        self.edges = edges;
    }
}

const N8: [(i32, i32); 8] = [
    (1, 0),
    (1, 1),
    (0, 1),
    (-1, 1),
    (-1, 0),
    (-1, -1),
    (0, -1),
    (1, -1),
];

fn neighbors(skel: &[bool], w: usize, h: usize, i: usize) -> impl Iterator<Item = usize> + '_ {
    let (x, y) = ((i % w) as i32, (i / w) as i32);
    N8.iter().filter_map(move |&(dx, dy)| {
        let (nx, ny) = (x + dx, y + dy);
        if nx < 0 || ny < 0 || nx as usize >= w || ny as usize >= h {
            return None;
        }
        let j = ny as usize * w + nx as usize;
        skel[j].then_some(j)
    })
}

/// 細線から、分岐・端点画素のかたまりをノード、その間をエッジとして追跡する。
fn trace(skel: &[bool], grid: &WalkGrid) -> RoadGraph {
    let (w, h) = (grid.w, grid.h);
    let deg = |i: usize| neighbors(skel, w, h, i).count();
    // ノード画素をかたまり（8連結）ごとにまとめる
    let mut cluster: HashMap<usize, usize> = HashMap::new();
    let mut nodes: Vec<Node> = vec![];
    let mut members: Vec<Vec<usize>> = vec![];
    for i in 0..w * h {
        if !skel[i] || deg(i) == 2 || cluster.contains_key(&i) {
            continue;
        }
        let id = nodes.len();
        let mut stack = vec![i];
        let mut mem = vec![];
        cluster.insert(i, id);
        while let Some(c) = stack.pop() {
            mem.push(c);
            for j in neighbors(skel, w, h, c) {
                if deg(j) != 2 && !cluster.contains_key(&j) {
                    cluster.insert(j, id);
                    stack.push(j);
                }
            }
        }
        let pos = mem.iter().map(|&c| grid.center(c % w, c / w)).sum::<Vec2>() / mem.len() as f32;
        let od = grid.sample_field(&grid.obstacle_dist, pos, 0.0);
        nodes.push(Node {
            pos,
            edges: vec![],
            radius: (od * 0.75).clamp(3.0, 15.0),
            border: false,
        });
        members.push(mem);
    }

    let mut used = vec![false; w * h];
    let mut edges: Vec<Edge> = vec![];
    let mut seen_direct: std::collections::HashSet<(usize, usize)> = Default::default();
    for (nid, mem) in members.iter().enumerate() {
        for &m in mem {
            for start in neighbors(skel, w, h, m).collect::<Vec<_>>() {
                if let Some(&other) = cluster.get(&start) {
                    // ノード同士が直接隣接
                    if other != nid && seen_direct.insert((nid.min(other), nid.max(other))) {
                        edges.push(Edge::new(
                            nid,
                            other,
                            vec![nodes[nid].pos, nodes[other].pos],
                        ));
                    }
                    continue;
                }
                if used[start] {
                    continue;
                }
                let mut pts = vec![nodes[nid].pos];
                let (mut prev, mut cur) = (m, start);
                let end = loop {
                    used[cur] = true;
                    pts.push(grid.center(cur % w, cur / w));
                    let next = neighbors(skel, w, h, cur).find(|&j| {
                        j != prev
                            && (!used[j] || cluster.contains_key(&j))
                            && !mem_contains(&members[nid], j, prev)
                    });
                    match next {
                        Some(j) => {
                            if let Some(&c) = cluster.get(&j) {
                                break Some(c);
                            }
                            prev = cur;
                            cur = j;
                        }
                        None => break None,
                    }
                };
                if let Some(end) = end {
                    pts.push(nodes[end].pos);
                    edges.push(Edge::new(nid, end, pts));
                }
            }
        }
    }
    for (i, e) in edges.iter().enumerate() {
        nodes[e.a].edges.push(i);
        if e.b != e.a {
            nodes[e.b].edges.push(i);
        }
    }
    RoadGraph { nodes, edges }
}

/// 追跡の 2 歩目で出発ノードに戻らないようにする
fn mem_contains(mem: &[usize], j: usize, prev: usize) -> bool {
    mem.contains(&j) && mem.contains(&prev)
}

/// Zhang–Suen 細線化。
fn zhang_suen(mut img: Vec<bool>, w: usize, h: usize) -> Vec<bool> {
    if w < 3 || h < 3 {
        return img;
    }
    let mut to_clear = vec![];
    loop {
        let mut changed = false;
        for pass in 0..2 {
            to_clear.clear();
            for y in 1..h - 1 {
                for x in 1..w - 1 {
                    let i = y * w + x;
                    if !img[i] {
                        continue;
                    }
                    // P2..P9 (北から時計回り)
                    let p = [
                        img[i + w],
                        img[i + w + 1],
                        img[i + 1],
                        img[i - w + 1],
                        img[i - w],
                        img[i - w - 1],
                        img[i - 1],
                        img[i + w - 1],
                    ];
                    let b = p.iter().filter(|&&v| v).count();
                    if !(2..=6).contains(&b) {
                        continue;
                    }
                    let a = (0..8).filter(|&k| !p[k] && p[(k + 1) % 8]).count();
                    if a != 1 {
                        continue;
                    }
                    // y を北向きに取っているので P2=北, P4=東, P6=南, P8=西
                    let (p2, p4, p6, p8) = (p[0], p[2], p[4], p[6]);
                    let ok = if pass == 0 {
                        !(p2 && p4 && p6) && !(p4 && p6 && p8)
                    } else {
                        !(p2 && p4 && p8) && !(p2 && p6 && p8)
                    };
                    if ok {
                        to_clear.push(i);
                    }
                }
            }
            for &i in &to_clear {
                img[i] = false;
            }
            changed |= !to_clear.is_empty();
        }
        if !changed {
            break;
        }
    }
    img
}

/// 端点を固定した移動平均。
fn smooth(pts: &[Vec2], k: usize) -> Vec<Vec2> {
    if pts.len() <= 2 {
        return pts.to_vec();
    }
    let n = pts.len();
    (0..n)
        .map(|i| {
            if i == 0 || i == n - 1 {
                return pts[i];
            }
            let r = k.min(i).min(n - 1 - i);
            pts[i - r..=i + r].iter().sum::<Vec2>() / (2 * r + 1) as f32
        })
        .collect()
}

/// Douglas–Peucker 法による折れ線の簡略化。
fn simplify(pts: &[Vec2], eps: f32) -> Vec<Vec2> {
    if pts.len() <= 2 {
        return pts.to_vec();
    }
    let (a, b) = (pts[0], pts[pts.len() - 1]);
    let ab = b - a;
    let (mut imax, mut dmax) = (0, 0.0);
    for (i, &p) in pts.iter().enumerate().take(pts.len() - 1).skip(1) {
        let d = if ab.length_squared() < 1e-9 {
            p.distance(a)
        } else {
            let t = ((p - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0);
            p.distance(a + ab * t)
        };
        if d > dmax {
            imax = i;
            dmax = d;
        }
    }
    if dmax > eps {
        let mut l = simplify(&pts[..=imax], eps);
        let r = simplify(&pts[imax..], eps);
        l.pop();
        l.extend(r);
        l
    } else {
        vec![a, b]
    }
}
