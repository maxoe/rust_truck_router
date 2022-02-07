use crate::{
    algo::{astar::Potential, ch::ContractionHierarchy, dijkstra::Dijkstra},
    types::{Graph, NodeId, OutgoingEdgeIterable, OwnedGraph, Weight, WeightOps},
};

#[derive(Clone)]
pub struct CHPotential {
    potentials: Vec<Option<Weight>>,
    fw_graph: OwnedGraph,
    bw_graph: OwnedGraph,
    rank: Vec<u32>,
    tent_distances: Vec<Weight>,
    t: NodeId,
}

impl CHPotential {
    pub fn new(fw_graph: OwnedGraph, bw_graph: OwnedGraph, rank: Vec<u32>) -> Self {
        let n = fw_graph.num_nodes();
        Self {
            potentials: vec![None; n],
            fw_graph,
            bw_graph,
            rank,
            tent_distances: vec![Weight::infinity(); n],
            t: n as NodeId,
        }
    }

    pub fn from_ch(ch: ContractionHierarchy) -> Self {
        Self::new(ch.fw_search.graph, ch.bw_search.graph, ch.rank)
    }

    pub fn reset(&mut self) {
        self.potentials = vec![None; self.fw_graph.num_nodes()];
    }
}

impl Potential<Weight> for CHPotential {
    fn init_potentials(&mut self, potentials: &[Weight]) {
        self.potentials = potentials.iter().map(|&p| if p == Weight::infinity() { None } else { Some(p) }).collect();
    }

    fn init_new_t(&mut self, ext_t: NodeId) {
        self.reset();

        let mut d = Dijkstra::new(self.bw_graph.borrow());
        self.t = self.rank[ext_t as usize];
        d.init_new_s(self.t);
        self.tent_distances = d.to_all().to_owned();
    }

    fn potential(&mut self, ext_n: NodeId) -> Weight {
        let n = self.rank[ext_n as usize];

        if let Some(p) = self.potentials[n as usize] {
            p
        } else {
            let mut node_stack = vec![n];

            while let Some(&current_node) = node_stack.last() {
                if self.potentials[current_node as usize].is_some() {
                    node_stack.pop();
                    continue;
                }

                let mut current_to_t_up_dist = self.tent_distances[current_node as usize];
                let mut neighbors_complete = true;
                for (&edge_weight, &neighbor_node) in self.fw_graph.outgoing_edge_iter(current_node) {
                    if let Some(p) = self.potentials[neighbor_node as usize] {
                        current_to_t_up_dist = current_to_t_up_dist.min(p.link(edge_weight));
                    } else {
                        node_stack.push(neighbor_node);
                        neighbors_complete = false;
                    }
                }

                if neighbors_complete {
                    node_stack.pop();
                    self.potentials[current_node as usize] = Some(current_to_t_up_dist);
                }
            }

            self.potentials[n as usize].unwrap()
        }
    }
}
