use std::rc::Rc;

use crate::types::*;
use bit_vec::BitVec;

use super::{
    core_ch::BorrowedCoreContractionHierarchy,
    csp::{OneRestrictionDijkstra, OneRestrictionDijkstraData},
};

pub struct CSPCoreCHQuery<'a> {
    core_ch: BorrowedCoreContractionHierarchy<'a>,
    pub fw_state: OneRestrictionDijkstraData,
    pub bw_state: OneRestrictionDijkstraData,
    fw_finished: bool,
    bw_finished: bool,
    s: NodeId,
    t: NodeId,
    pub restriction: DrivingTimeRestriction,
    is_reset_node: Rc<BitVec>,
    pub last_dist: Option<Weight>,
}

impl<'a> CSPCoreCHQuery<'a> {
    pub fn new(core_ch: BorrowedCoreContractionHierarchy<'a>) -> Self {
        let node_count = core_ch.rank().len();
        let is_reset_node = core_ch.is_core().clone();

        CSPCoreCHQuery {
            core_ch,
            fw_state: OneRestrictionDijkstraData::new(node_count),
            bw_state: OneRestrictionDijkstraData::new(node_count),
            fw_finished: false,
            bw_finished: false,
            s: node_count as NodeId,
            t: node_count as NodeId,
            restriction: DrivingTimeRestriction {
                pause_time: 0,
                max_driving_time: Weight::infinity(),
            },

            is_reset_node,
            last_dist: None,
        }
    }

    pub fn set_custom_reset_nodes(&mut self, ext_is_reset_node: Rc<BitVec>) {
        let mut new_is_reset_node = BitVec::from_elem(self.is_reset_node.len(), false);

        for (ext_i, b) in ext_is_reset_node.iter().enumerate().filter(|(_, b)| *b) {
            let i = self.core_ch.rank()[ext_i];
            new_is_reset_node.set(i as usize, b);
        }

        self.is_reset_node = Rc::new(new_is_reset_node);
    }

    pub fn clear_custom_reset_nodes(&mut self) {
        self.is_reset_node = self.core_ch.is_core().clone();
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

    /// Equivalent to routingkit's check_contraction_hierarchy_for_errors except the check for only up edges
    pub fn check(&self) {
        let node_count = self.core_ch.forward().num_nodes();
        assert_eq!(self.core_ch.forward().first_out().len(), node_count + 1);
        assert_eq!(self.core_ch.backward().first_out().len(), node_count + 1);

        let forward_arc_count = *self.core_ch.forward().first_out().last().unwrap();

        assert_eq!(*self.core_ch.forward().first_out().first().unwrap(), 0);
        assert!(self.core_ch.forward().first_out().is_sorted_by(|l, r| Some(l.cmp(r))));
        assert_eq!(self.core_ch.forward().head().len(), forward_arc_count as usize);
        assert_eq!(self.core_ch.forward().weights().len(), forward_arc_count as usize);
        assert!(!self.core_ch.forward().head().is_empty() && *self.core_ch.forward().head().iter().max().unwrap() < node_count as NodeId);

        let backward_arc_count = *self.core_ch.backward().first_out().last().unwrap();
        assert_eq!(*self.core_ch.backward().first_out().first().unwrap(), 0);
        assert!(self.core_ch.backward().first_out().is_sorted_by(|l, r| Some(l.cmp(r))));
        assert_eq!(self.core_ch.backward().head().len(), backward_arc_count as usize);
        assert_eq!(self.core_ch.backward().weights().len(), backward_arc_count as usize);
        assert!(!self.core_ch.backward().head().is_empty() && *self.core_ch.backward().head().iter().max().unwrap() < node_count as NodeId);
    }

    pub fn init_new_s(&mut self, ext_s: NodeId) {
        self.s = self.core_ch.rank()[ext_s as usize] as NodeId;
    }

    pub fn init_new_t(&mut self, ext_t: NodeId) {
        self.t = self.core_ch.rank()[ext_t as usize] as NodeId;
    }

    pub fn reset(&mut self) {
        if self.s != self.core_ch.rank().len() as NodeId {
            self.fw_state.init_new_s(self.s);
        }

        if self.t != self.core_ch.rank().len() as NodeId {
            self.bw_state.init_new_s(self.t);
        }

        self.fw_finished = false;
        self.bw_finished = false;
    }

    pub fn clean(&mut self) {
        self.fw_state.clean();
        self.bw_state.clean();
        self.reset();
    }

    fn calculate_distance_with_break_at(
        node: NodeId,
        restriction: &DrivingTimeRestriction,
        fw_label_dist: &Weight2,
        bw_state: &mut OneRestrictionDijkstraData,
    ) -> Weight {
        let v_to_t = bw_state.get_settled_labels_at(node);

        let mut current_bw = v_to_t.rev().map(|r| r.0);

        let mut best_distance = Weight::infinity();
        while let Some(bw_label) = current_bw.next() {
            let total_dist = fw_label_dist.add(bw_label.distance);

            // check if restrictions allows combination of those labels/subpaths
            if total_dist[1] < restriction.max_driving_time {
                // subpaths can be connected without additional break
                best_distance = best_distance.min(total_dist[0]);
            }
        }

        best_distance
    }

    pub fn run_query(&mut self) -> Option<Weight> {
        if self.s == self.core_ch.rank().len() as NodeId || self.t == self.core_ch.rank().len() as NodeId {
            return None;
        }

        self.reset();

        let mut tentative_distance = Weight::infinity();

        let mut settled_fw = BitVec::from_elem(self.core_ch.forward().num_nodes(), false);
        let mut settled_bw = BitVec::from_elem(self.core_ch.backward().num_nodes(), false);
        let mut _middle_node = self.core_ch.forward().num_nodes() as NodeId;
        let mut fw_next = true;

        let fw_search = OneRestrictionDijkstra::new(self.core_ch.forward(), self.is_reset_node.as_ref());
        let bw_search = OneRestrictionDijkstra::new(self.core_ch.backward(), self.is_reset_node.as_ref());

        while !self.fw_finished || !self.bw_finished {
            if self.bw_finished || !self.fw_finished && fw_next {
                if let Some(State {
                    distance: _dist_from_queue_at_v,
                    node,
                }) = fw_search.settle_next_label(&mut self.fw_state, self.t)
                {
                    settled_fw.set(node as usize, true);

                    // fw search found t -> done here
                    if node == self.t {
                        // println!("fw settled t");
                        tentative_distance = self.fw_state.get_settled_labels_at(node).last().unwrap().0.distance[0];
                        // dist_from_queue_at_v[0];
                        self.fw_finished = true;
                        // self.bw_finished = true;

                        // break;
                    }

                    if settled_bw.get(node as usize).unwrap() {
                        let tent_dist_at_v = Self::calculate_distance_with_break_at(
                            node,
                            &self.restriction,
                            &self.fw_state.get_best_label_at(node).unwrap().distance,
                            &mut self.bw_state,
                        );
                        if tentative_distance > tent_dist_at_v {
                            tentative_distance = tent_dist_at_v;
                            _middle_node = node;
                        }
                    }

                    // if self.fw_state.min_key().is_none() {
                    //     println!("fw queue ran out");
                    // }

                    // if self.fw_state.min_key().is_some() && self.fw_state.min_key().unwrap() >= tentative_distance {
                    //     println!("fw min key > µ");
                    // }

                    if self.fw_state.min_key().is_none() || self.fw_state.min_key().unwrap() >= tentative_distance {
                        self.fw_finished = true;
                    }

                    fw_next = false;
                }
            } else if let Some(State {
                distance: _dist_from_queue_at_v,
                node,
            }) = bw_search.settle_next_label(&mut self.bw_state, self.s)
            {
                settled_bw.set(node as usize, true);

                // bw search found s -> done here
                if node == self.s {
                    // println!("bw settled s");
                    tentative_distance = self.bw_state.get_settled_labels_at(node).last().unwrap().0.distance[0];
                    // dist_from_queue_at_v[0];

                    self.bw_finished = true;

                    // break;
                }

                if settled_fw.get(node as usize).unwrap() {
                    let tent_dist_at_v = Self::calculate_distance_with_break_at(
                        node,
                        &self.restriction,
                        &self.bw_state.get_best_label_at(node).unwrap().distance,
                        &mut self.fw_state,
                    );

                    if tentative_distance > tent_dist_at_v {
                        tentative_distance = tent_dist_at_v;
                        _middle_node = node;
                    }
                }

                // if self.bw_state.min_key().is_none() {
                //     println!("bw queue ran out");
                // }

                // if self.bw_state.min_key().is_some() && self.bw_state.min_key().unwrap() >= tentative_distance {
                //     println!("bw min key > µ");
                // }

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
