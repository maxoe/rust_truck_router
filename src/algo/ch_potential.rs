use crate::{
    algo::{astar::Potential, ch::ContractionHierarchy},
    timestamped_vector::TimestampedVector,
    types::{Graph, NodeId, OutgoingEdgeIterable, OwnedGraph, Weight, WeightOps},
};

use super::dijkstra::OwnedDijkstra;

#[derive(Clone)]
pub struct CHPotential {
    potentials: TimestampedVector<Option<Weight>>,
    fw_graph: OwnedGraph,
    init_dijkstra: OwnedDijkstra,
    rank: Vec<u32>,
    node_mapping: Option<Vec<NodeId>>,
    t: NodeId,
}

impl CHPotential {
    pub fn new(fw_graph: OwnedGraph, bw_graph: OwnedGraph, rank: Vec<u32>) -> Self {
        let n = fw_graph.num_nodes();
        Self {
            potentials: TimestampedVector::with_size(n),
            fw_graph,
            rank,
            init_dijkstra: OwnedDijkstra::new(bw_graph),
            t: n as NodeId,
            node_mapping: None,
        }
    }

    pub fn new_with_node_mapping(fw_graph: OwnedGraph, bw_graph: OwnedGraph, rank: Vec<u32>, node_mapping: Vec<NodeId>) -> Self {
        let n = fw_graph.num_nodes();

        Self {
            potentials: TimestampedVector::with_size(n),
            fw_graph,
            rank,
            init_dijkstra: OwnedDijkstra::new(bw_graph),
            t: n as NodeId,
            node_mapping: Some(node_mapping),
        }
    }

    pub fn from_ch(ch: ContractionHierarchy) -> Self {
        Self::new(ch.fw_search.graph, ch.bw_search.graph, ch.rank)
    }

    pub fn from_ch_with_node_mapping(ch: ContractionHierarchy, node_mapping: Vec<NodeId>) -> Self {
        Self::new_with_node_mapping(ch.fw_search.graph, ch.bw_search.graph, ch.rank, node_mapping)
    }

    pub fn from_ch_with_node_mapping_backwards(ch: ContractionHierarchy, node_mapping: Vec<NodeId>) -> Self {
        Self::new_with_node_mapping(ch.bw_search.graph, ch.fw_search.graph, ch.rank, node_mapping)
    }

    pub fn reset(&mut self) {
        self.potentials.reset();
        self.init_dijkstra.init_new_s(self.t);
        self.init_dijkstra.to_all();
    }

    pub fn set_node_mapping(&mut self, node_mapping: Vec<NodeId>) {
        self.node_mapping = Some(node_mapping);
        self.reset();
    }

    pub fn clear_node_mapping(&mut self) {
        self.node_mapping = None;
        self.reset();
    }

    pub fn map_node(&self, n: NodeId) -> NodeId {
        match &self.node_mapping {
            Some(mapping) => mapping[n as usize],
            None => n,
        }
    }
}

impl Potential<Weight> for CHPotential {
    fn init_new_t(&mut self, ext_t: NodeId) {
        self.t = self.rank[self.map_node(ext_t) as usize];

        self.reset();
    }

    fn potential(&mut self, ext_n: NodeId) -> Weight {
        let n = self.rank[self.map_node(ext_n) as usize];

        if let Some(p) = self.potentials.get(n as usize) {
            *p
        } else {
            let mut node_stack = vec![n];

            while let Some(&current_node) = node_stack.last() {
                if self.potentials.get(current_node as usize).is_some() {
                    node_stack.pop();
                    continue;
                }

                let mut current_to_t_up_dist = self.init_dijkstra.tentative_distance_at(current_node);
                let mut neighbors_complete = true;
                for (&edge_weight, &neighbor_node) in self.fw_graph.outgoing_edge_iter(current_node) {
                    if let Some(p) = self.potentials.get(neighbor_node as usize) {
                        current_to_t_up_dist = current_to_t_up_dist.min(p.link(edge_weight));
                    } else {
                        node_stack.push(neighbor_node);
                        neighbors_complete = false;
                    }
                }

                if neighbors_complete {
                    node_stack.pop();
                    self.potentials.set(current_node as usize, Some(current_to_t_up_dist));
                }
            }
            self.potentials.get(n as usize).unwrap()
        }
    }
}
