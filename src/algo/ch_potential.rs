use crate::{
    algo::astar::Potential,
    timestamped_vector::TimestampedVector,
    types::{Graph, NodeId, OutgoingEdgeIterable, Weight, WeightOps},
};

use super::{
    ch::BorrowedContractionHierarchy,
    dijkstra::{Dijkstra, DijkstraData},
};

pub struct CHPotential<'a> {
    potentials: TimestampedVector<Option<Weight>>,
    ch: BorrowedContractionHierarchy<'a>,
    init_dijkstra_state: DijkstraData,
    node_mapping: Option<Vec<NodeId>>,
    t: NodeId,
}

impl<'a> CHPotential<'a> {
    pub fn from_ch(ch: BorrowedContractionHierarchy<'a>) -> Self {
        let n = ch.forward().num_nodes();
        Self {
            potentials: TimestampedVector::with_size(n),
            ch,
            init_dijkstra_state: DijkstraData::new(n),
            t: n as NodeId,
            node_mapping: None,
        }
    }

    pub fn from_ch_with_node_mapping(ch: BorrowedContractionHierarchy<'a>, node_mapping: Vec<NodeId>) -> Self {
        let n = ch.forward().num_nodes();
        Self {
            potentials: TimestampedVector::with_size(n),
            ch,
            init_dijkstra_state: DijkstraData::new(n),
            t: n as NodeId,
            node_mapping: Some(node_mapping),
        }
    }

    pub fn from_ch_with_node_mapping_backwards(ch: BorrowedContractionHierarchy<'a>, node_mapping: Vec<NodeId>) -> Self {
        Self::from_ch_with_node_mapping(ch.inverted(), node_mapping)
    }

    pub fn reset(&mut self) {
        self.potentials.reset();
        self.init_dijkstra_state.init_new_s(self.t);
        Dijkstra::new(self.ch.backward()).to_all(&mut self.init_dijkstra_state);
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

impl<'a> Potential for CHPotential<'a> {
    fn init_new_t(&mut self, ext_t: NodeId) {
        self.t = self.ch.rank()[self.map_node(ext_t) as usize];

        self.reset();
    }

    fn potential(&mut self, ext_n: NodeId) -> Weight {
        let n = self.ch.rank()[self.map_node(ext_n) as usize];

        if let Some(p) = self.potentials.get(n as usize) {
            *p
        } else {
            let mut node_stack = vec![n];

            while let Some(&current_node) = node_stack.last() {
                if self.potentials.get(current_node as usize).is_some() {
                    node_stack.pop();
                    continue;
                }

                let mut current_to_t_up_dist = self.init_dijkstra_state.tentative_distance_at(current_node);
                let mut neighbors_complete = true;
                for (&edge_weight, &neighbor_node) in self.ch.forward().outgoing_edge_iter(current_node) {
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
