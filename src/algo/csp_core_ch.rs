use crate::types::*;
use bit_vec::BitVec;

use super::{
    core_ch::BorrowedCoreContractionHierarchy,
    csp::{OneRestrictionDijkstra, OneRestrictionDijkstraData},
};

pub struct CSPCoreCHQuery<'a> {
    core_ch: BorrowedCoreContractionHierarchy<'a>,
    pub is_reachable_from_core_in_fw: BitVec,
    pub is_reachable_from_core_in_bw: BitVec,
    pub fw_state: OneRestrictionDijkstraData,
    pub bw_state: OneRestrictionDijkstraData,
    fw_finished: bool,
    bw_finished: bool,
    s: NodeId,
    t: NodeId,
    pub restriction: DrivingTimeRestriction,
    pub last_dist: Option<Weight>,
}

impl<'a> CSPCoreCHQuery<'a> {
    pub fn new(core_ch: BorrowedCoreContractionHierarchy<'a>) -> Self {
        let node_count = core_ch.forward().num_nodes();

        let mut is_reachable_from_core_in_bw = BitVec::from_elem(node_count, false);
        let mut is_reachable_from_core_in_fw = BitVec::from_elem(node_count, false);

        let mut core_node_count = 0;
        for n in core_ch.is_core().iter().enumerate().filter(|(_, b)| *b).map(|(i, _)| i) {
            core_node_count += 1;

            for i in core_ch.forward().first_out()[n as usize]..core_ch.forward().first_out()[n as usize + 1] {
                if !core_ch.is_core().get(core_ch.forward().head()[i as usize] as usize).unwrap() {
                    is_reachable_from_core_in_fw.set(core_ch.forward().head()[i as usize] as usize, true);
                }
            }

            for i in core_ch.backward().first_out()[n as usize]..core_ch.backward().first_out()[n as usize + 1] {
                if !core_ch.is_core().get(core_ch.backward().head()[i as usize] as usize).unwrap() {
                    is_reachable_from_core_in_bw.set(core_ch.backward().head()[i as usize] as usize, true);
                }
            }
        }

        println!(
            "Core node count: {} ({:.2}%)",
            core_node_count,
            core_node_count as f32 * 100.0 / node_count as f32
        );

        CSPCoreCHQuery {
            core_ch,
            is_reachable_from_core_in_fw,
            is_reachable_from_core_in_bw,
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
            last_dist: None,
        }
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

    fn calculate_distance_with_break_at(
        node: NodeId,
        restriction: &DrivingTimeRestriction,
        fw_state: &mut OneRestrictionDijkstraData,
        bw_state: &mut OneRestrictionDijkstraData,
    ) -> Weight {
        let s_to_v = fw_state.get_settled_labels_at(node);
        let v_to_t = bw_state.get_settled_labels_at(node);

        let mut current_fw = s_to_v.rev().map(|r| r.0).peekable();
        let mut current_bw = v_to_t.rev().map(|r| r.0).peekable();

        if current_fw.peek().is_none() || current_bw.peek().is_none() {
            return Weight::infinity();
        }

        let mut best_distance = Weight::infinity();
        while let (Some(fw_label), Some(bw_label)) = (current_fw.peek(), current_bw.peek()) {
            let total_dist = fw_label.distance.add(bw_label.distance);

            // check if restrictions allows combination of those labels/subpaths
            if total_dist[1] < restriction.max_driving_time {
                // subpaths can be connected without additional break
                best_distance = best_distance.min(total_dist[0]);
            }

            if fw_label.distance[0] < bw_label.distance[0] {
                current_fw.next();
            } else {
                current_bw.next();
            }
        }

        best_distance
    }

    pub fn run_query(&mut self) -> Option<Weight> {
        if self.s == self.core_ch.rank().len() as NodeId || self.t == self.core_ch.rank().len() as NodeId {
            return None;
        }
        let mut tentative_distance = Weight::infinity();

        let mut fw_min_key = 0;
        let mut bw_min_key = 0;

        self.reset();
        if !self.fw_finished {
            // safe after init
            fw_min_key = self.fw_state.min_key().unwrap();
        }

        if !self.bw_finished {
            // safe after init
            bw_min_key = self.bw_state.min_key().unwrap();
        }

        let mut settled_fw = BitVec::from_elem(self.core_ch.forward().num_nodes(), false);
        let mut settled_bw = BitVec::from_elem(self.core_ch.backward().num_nodes(), false);
        let mut _middle_node = self.core_ch.forward().num_nodes() as NodeId;
        let mut fw_next = true;

        // to cancel no path found queries early
        let mut fw_in_core = false;
        let mut bw_in_core = false;
        let mut fw_state_reachable_from_core = false;
        let mut bw_state_reachable_from_core = false;

        let reset_flags = self.core_ch.is_core();
        let fw_search = OneRestrictionDijkstra::new(self.core_ch.forward(), reset_flags.as_ref());
        let bw_search = OneRestrictionDijkstra::new(self.core_ch.backward(), reset_flags.as_ref());

        while (!self.fw_finished || !self.bw_finished)
            && !(self.fw_finished && !fw_state_reachable_from_core && bw_in_core)
            && !(self.bw_finished && !bw_state_reachable_from_core && fw_in_core)
        {
            if self.bw_finished || !self.fw_finished && fw_next {
                if let Some(State {
                    distance: dist_from_queue_at_v,
                    node,
                }) = fw_search.settle_next_label(&mut self.fw_state, self.t)
                {
                    settled_fw.set(node as usize, true);

                    // fw search found t -> done here
                    if node == self.t {
                        tentative_distance = dist_from_queue_at_v[0];
                        self.fw_finished = true;
                        self.bw_finished = true;

                        break;
                    }

                    if settled_bw.get(node as usize).unwrap() {
                        let tent_dist_at_v = Self::calculate_distance_with_break_at(node, &self.restriction, &mut self.fw_state, &mut self.bw_state);

                        if tentative_distance > tent_dist_at_v {
                            tentative_distance = tent_dist_at_v;
                            _middle_node = node;
                        }
                    }

                    fw_min_key = self.fw_state.min_key().unwrap_or_else(|| {
                        self.fw_finished = true;
                        fw_min_key
                    });

                    if fw_min_key >= tentative_distance {
                        self.fw_finished = true;
                    }

                    if self.is_reachable_from_core_in_bw.get(node as usize).unwrap() {
                        fw_state_reachable_from_core = true;
                    }

                    if self.core_ch.is_core().get(node as usize).unwrap() {
                        fw_in_core = true;
                        fw_state_reachable_from_core = true;
                    }

                    fw_next = false;
                }
            } else if let Some(State {
                distance: dist_from_queue_at_v,
                node,
            }) = bw_search.settle_next_label(&mut self.bw_state, self.s)
            {
                settled_bw.set(node as usize, true);

                // bw search found s -> done here
                if node == self.s {
                    tentative_distance = dist_from_queue_at_v[0];

                    self.fw_finished = true;
                    self.bw_finished = true;

                    break;
                }

                if settled_fw.get(node as usize).unwrap() {
                    let tent_dist_at_v = Self::calculate_distance_with_break_at(node, &self.restriction, &mut self.fw_state, &mut self.bw_state);

                    if tentative_distance > tent_dist_at_v {
                        tentative_distance = tent_dist_at_v;
                        _middle_node = node;
                    }
                }

                bw_min_key = self.bw_state.min_key().unwrap_or_else(|| {
                    self.bw_finished = true;
                    bw_min_key
                });

                if bw_min_key >= tentative_distance {
                    self.bw_finished = true;
                }

                if self.is_reachable_from_core_in_fw.get(node as usize).unwrap() {
                    bw_state_reachable_from_core = true;
                }

                if self.core_ch.is_core().get(node as usize).unwrap() {
                    bw_in_core = true;
                    bw_state_reachable_from_core = true;
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
