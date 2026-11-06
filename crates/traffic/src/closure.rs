//! 通行止めによる道路網の区間の除去。

use crate::graph::{Edge, RoadGraph};

impl RoadGraph {
    /// `keep` が false を返す区間を除いた道路網を作る（ノードはそのまま残し、区間の番号を詰める）。
    /// 区間を失った交差点は行き止まりになり、車は U ターンするか別の道を選ぶ。
    pub fn retain_edges(&self, mut keep: impl FnMut(&Edge) -> bool) -> RoadGraph {
        let mut map = vec![usize::MAX; self.edges.len()];
        let mut edges = vec![];
        for (i, e) in self.edges.iter().enumerate() {
            if keep(e) {
                map[i] = edges.len();
                edges.push(e.clone());
            }
        }
        let mut nodes = self.nodes.clone();
        for n in nodes.iter_mut() {
            n.edges = n
                .edges
                .iter()
                .filter_map(|&e| (map[e] != usize::MAX).then_some(map[e]))
                .collect();
        }
        RoadGraph { nodes, edges }
    }
}
