use super::{
    astar::Potential,
    ch::BorrowedContractionHierarchy,
    csp::{OneRestrictionDijkstra, OneRestrictionDijkstraData},
};
use crate::{algo::ch_potential::CHPotential, types::*};
use bit_vec::BitVec;

pub struct CSPBidirAstarCHPotQuery<'a> {
    fw_graph: BorrowedGraph<'a>,
    bw_graph: BorrowedGraph<'a>,
    is_reset_node: &'a BitVec,
    fw_state: OneRestrictionDijkstraData<CHPotential<'a>>,
    bw_state: OneRestrictionDijkstraData<CHPotential<'a>>,
    restriction: DrivingTimeRestriction,
    fw_finished: bool,
    bw_finished: bool,
    s: NodeId,
    t: NodeId,
    pub last_dist: Option<Weight>,
}

impl<'a> CSPBidirAstarCHPotQuery<'a> {
    pub fn new(fw_graph: BorrowedGraph<'a>, bw_graph: BorrowedGraph<'a>, is_reset_node: &'a BitVec, ch: BorrowedContractionHierarchy<'a>) -> Self {
        let node_count = fw_graph.num_nodes();
        CSPBidirAstarCHPotQuery {
            fw_graph,
            bw_graph,
            is_reset_node,
            fw_state: OneRestrictionDijkstraData::new_with_potential(node_count, CHPotential::from_ch(ch)),
            bw_state: OneRestrictionDijkstraData::new_with_potential(node_count, CHPotential::from_ch_backwards(ch)),
            restriction: DrivingTimeRestriction {
                pause_time: 0,
                max_driving_time: Weight::infinity(),
            },
            fw_finished: false,
            bw_finished: false,
            s: node_count as NodeId,
            t: node_count as NodeId,
            last_dist: None,
        }
    }

    pub fn init_new_s(&mut self, s: NodeId) {
        self.s = s;
    }

    pub fn init_new_t(&mut self, t: NodeId) {
        self.t = t;
    }

    pub fn set_restriction(&mut self, max_driving_time: Weight, pause_time: Weight) {
        self.restriction = DrivingTimeRestriction { pause_time, max_driving_time };

        self.fw_state.set_restriction(max_driving_time, pause_time);
        self.bw_state.set_restriction(max_driving_time, pause_time);
    }

    pub fn clear_restriction(&mut self) {
        self.restriction = DrivingTimeRestriction {
            pause_time: 0,
            max_driving_time: Weight::infinity(),
        };

        self.fw_state.clear_restriction();
        self.bw_state.clear_restriction();
    }

    pub fn reset(&mut self) {
        if self.s != self.fw_graph.num_nodes() as NodeId {
            self.fw_state.init_new_s(self.s);
            self.bw_state.potential.init_new_t(self.s);
        }

        if self.t != self.fw_graph.num_nodes() as NodeId {
            self.bw_state.init_new_s(self.t);
            self.fw_state.potential.init_new_t(self.t);
        }

        self.fw_finished = false;
        self.bw_finished = false;
    }

    pub fn run_query(&mut self) -> Option<Weight> {
        if self.s == self.fw_graph.num_nodes() as NodeId || self.t == self.fw_graph.num_nodes() as NodeId {
            return None;
        }

        self.reset();

        let mut tentative_distance = Weight::infinity();

        let mut settled_fw = BitVec::from_elem(self.fw_graph.num_nodes(), false);
        let mut settled_bw = BitVec::from_elem(self.fw_graph.num_nodes(), false);
        let mut _middle_node = self.fw_graph.num_nodes() as NodeId;
        let mut fw_next = true;

        let fw_search = OneRestrictionDijkstra::new(self.fw_graph, &self.is_reset_node);
        let bw_search = OneRestrictionDijkstra::new(self.bw_graph, &self.is_reset_node);

        while !self.fw_finished || !self.bw_finished {
            if !self.fw_finished && (self.bw_finished || fw_next) {
                if let Some(State {
                    distance: _dist_from_queue_at_v,
                    node,
                }) = if tentative_distance < Weight::infinity() && self.bw_state.min_key().is_some() {
                    fw_search.settle_next_label_prune_bw_lower_bound(
                        &mut self.fw_state,
                        tentative_distance,
                        self.bw_state.peek_queue().map(|s| s.distance).unwrap(),
                        &mut self.bw_state.potential,
                        self.t,
                    )
                } else {
                    fw_search.settle_next_label(&mut self.fw_state, self.t)
                } {
                    settled_fw.set(node as usize, true);

                    // fw search found t -> done here
                    if node == self.t {
                        tentative_distance = self.fw_state.get_settled_labels_at(node).last().unwrap().0.distance[0]; // dist_from_queue_at_v[0];
                        self.fw_finished = true;
                        self.bw_finished = true;

                        break;
                    }

                    if settled_bw.get(node as usize).unwrap() {
                        let tent_dist_at_v = self.fw_state.get_settled_labels_at(node).last().unwrap().0.distance[0]
                            + self.bw_state.get_settled_labels_at(node).last().unwrap().0.distance[0];

                        if tentative_distance > tent_dist_at_v {
                            tentative_distance = tent_dist_at_v;
                            _middle_node = node;
                        }
                    }

                    if self.fw_state.min_key().is_none() || self.fw_state.min_key().unwrap() >= tentative_distance {
                        self.fw_finished = true;
                    }

                    fw_next = false;
                }
            } else if let Some(State {
                distance: _dist_from_queue_at_v,
                node,
            }) = if tentative_distance < Weight::infinity() && self.fw_state.min_key().is_some() {
                bw_search.settle_next_label_prune_bw_lower_bound(
                    &mut self.bw_state,
                    tentative_distance,
                    self.fw_state.peek_queue().map(|s| s.distance).unwrap(),
                    &mut self.fw_state.potential,
                    self.s,
                )
            } else {
                bw_search.settle_next_label(&mut self.bw_state, self.s)
            } {
                settled_bw.set(node as usize, true);

                // bw search found s -> done here
                if node == self.s {
                    tentative_distance = self.bw_state.get_settled_labels_at(node).last().unwrap().0.distance[0]; // dist_from_queue_at_v[0];

                    self.fw_finished = true;
                    self.bw_finished = true;

                    break;
                }

                if settled_fw.get(node as usize).unwrap() {
                    let tent_dist_at_v = self.fw_state.get_settled_labels_at(node).last().unwrap().0.distance[0]
                        + self.bw_state.get_settled_labels_at(node).last().unwrap().0.distance[0];

                    if tentative_distance > tent_dist_at_v {
                        tentative_distance = tent_dist_at_v;
                        _middle_node = node;
                    }
                }

                if self.bw_state.min_key().is_none() || self.bw_state.min_key().unwrap() >= tentative_distance {
                    self.bw_finished = true;
                }

                fw_next = true;
            }
        }

        if tentative_distance == Weight::infinity() {
            self.last_dist = None;
            return None;
        }

        self.last_dist = Some(tentative_distance);
        self.last_dist
    }
}
