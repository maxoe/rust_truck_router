use crate::{
    algo::{ch::ContractionHierarchy, ch_potential::CHPotential},
    io::Load,
    types::*,
};
use bit_vec::BitVec;
use std::{path::Path, time::Instant};

use super::{astar::Potential, csp::OneRestrictionDijkstra};

pub struct CSPAstarCoreContractionHierarchy<'a> {
    pub rank: Vec<u32>,
    pub is_core: BitVec,
    pub fw_search: OneRestrictionDijkstra<'a, CHPotential>,
    pub bw_search: OneRestrictionDijkstra<'a, CHPotential>,
    fw_finished: bool,
    bw_finished: bool,
    s: NodeId,
    t: NodeId,
    pub restriction: DrivingTimeRestriction,
    pub last_dist: Option<Weight>,
}

impl<'a> CSPAstarCoreContractionHierarchy<'a> {
    pub fn load_from_routingkit_dir<P: AsRef<Path>>(path: P) -> Result<Self, std::io::Error> {
        Ok(CSPAstarCoreContractionHierarchy::build(
            Vec::<u32>::load_from(path.as_ref().join("rank"))?,
            Vec::<u32>::load_from(path.as_ref().join("order"))?,
            Vec::<u32>::load_from(path.as_ref().join("core"))?,
            OwnedGraph::load_from_routingkit_dir(path.as_ref().join("forward"))?,
            OwnedGraph::load_from_routingkit_dir(path.as_ref().join("backward"))?,
            ContractionHierarchy::load_from_routingkit_dir(path.as_ref().join("../ch"))?,
        ))
    }

    pub fn build(rank: Vec<u32>, order: Vec<u32>, core: Vec<NodeId>, forward: OwnedGraph, backward: OwnedGraph, ch: ContractionHierarchy) -> Self {
        let node_count = forward.num_nodes();

        let mut is_core = BitVec::from_elem(node_count, false);
        for &n in core.iter() {
            is_core.set(rank[n as usize] as usize, true);
        }

        let core_node_count = core.len();
        println!(
            "Core node count: {} ({:.2}%)",
            core_node_count,
            core_node_count as f32 * 100.0 / rank.len() as f32
        );

        ch.check();

        CSPAstarCoreContractionHierarchy {
            rank,
            is_core,
            fw_search: OneRestrictionDijkstra::new_with_potential_from_owned(forward, CHPotential::from_ch_with_node_mapping(ch.clone(), order.clone())),
            bw_search: OneRestrictionDijkstra::new_with_potential_from_owned(backward, CHPotential::from_ch_with_node_mapping_backwards(ch, order)),
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

        self.fw_search.set_reset_flags(&self.is_core.to_bytes());
        self.fw_search.set_restriction(max_driving_time, pause_time);
        self.bw_search.set_reset_flags(&self.is_core.to_bytes());
        self.bw_search.set_restriction(max_driving_time, pause_time);
    }

    pub fn clear_restriction(&mut self) {
        self.restriction = DrivingTimeRestriction {
            pause_time: 0,
            max_driving_time: Weight::infinity(),
        };

        self.fw_search.clear_reset_flags();
        self.fw_search.clear_restriction();
        self.bw_search.clear_reset_flags();
        self.bw_search.clear_restriction();
    }

    /// Equivalent to routingkit's check_contraction_hierarchy_for_errors except the check for only up edges
    pub fn check(&self) {
        let node_count = self.fw_search.graph.num_nodes();
        assert_eq!(self.fw_search.graph.first_out().len(), node_count + 1);
        assert_eq!(self.bw_search.graph.first_out().len(), node_count + 1);

        let forward_arc_count = *self.fw_search.graph.first_out().last().unwrap();

        assert_eq!(*self.fw_search.graph.first_out().first().unwrap(), 0);
        assert!(self.fw_search.graph.first_out().is_sorted_by(|l, r| Some(l.cmp(r))));
        assert_eq!(self.fw_search.graph.head().len(), forward_arc_count as usize);
        assert_eq!(self.fw_search.graph.weights().len(), forward_arc_count as usize);
        assert!(!self.fw_search.graph.head().is_empty() && *self.fw_search.graph.head().iter().max().unwrap() < node_count as NodeId);

        let backward_arc_count = *self.bw_search.graph.first_out().last().unwrap();
        assert_eq!(*self.bw_search.graph.first_out().first().unwrap(), 0);
        assert!(self.bw_search.graph.first_out().is_sorted_by(|l, r| Some(l.cmp(r))));
        assert_eq!(self.bw_search.graph.head().len(), backward_arc_count as usize);
        assert_eq!(self.bw_search.graph.weights().len(), backward_arc_count as usize);
        assert!(!self.bw_search.graph.head().is_empty() && *self.bw_search.graph.head().iter().max().unwrap() < node_count as NodeId);
    }

    pub fn init_new_s(&mut self, ext_s: NodeId) {
        self.s = self.rank[ext_s as usize] as NodeId;
        self.reset();
    }

    pub fn init_new_t(&mut self, ext_t: NodeId) {
        self.t = self.rank[ext_t as usize] as NodeId;
        self.reset();
    }

    pub fn reset(&mut self) {
        if self.s != self.rank.len() as NodeId {
            self.fw_search.init_new_s(self.s);
            self.bw_search.potential.init_new_t(self.s);
        }

        if self.t != self.rank.len() as NodeId {
            self.bw_search.init_new_s(self.t);
            self.fw_search.potential.init_new_t(self.t);
        }

        self.fw_finished = false;
        self.bw_finished = false;
    }

    fn calculate_distance_with_break_at(&mut self, node: NodeId) -> Weight {
        let s_to_v = self.fw_search.get_settled_labels_at(node);
        let v_to_t = self.bw_search.get_settled_labels_at(node);

        assert!(s_to_v.is_sorted());
        assert!(v_to_t.is_sorted());

        if s_to_v.is_empty() || v_to_t.is_empty() {
            return Weight::infinity();
        }

        let mut current_fw = s_to_v.into_iter().rev().map(|r| r.0).peekable();
        let mut current_bw = v_to_t.into_iter().rev().map(|r| r.0).peekable();

        while let (Some(fw_label), Some(bw_label)) = (current_fw.peek(), current_bw.peek()) {
            let total_dist = fw_label.distance.add(bw_label.distance);

            // check if restrictions allows combination of those labels/subpaths
            if total_dist[1] < self.restriction.max_driving_time {
                // subpaths can be connected without additional break
                return total_dist[0];
            }

            // need parking at node
            if self.is_core.get(node as usize).unwrap() {
                // can park at node and connect subpaths
                if fw_label.distance[1].max(bw_label.distance[1]) < self.restriction.max_driving_time {
                    return total_dist[0] + self.restriction.pause_time;
                }
            }

            if fw_label.distance[0] < bw_label.distance[0] {
                current_fw.next();
            } else {
                current_bw.next();
            }
        }

        Weight::infinity()
    }

    pub fn run_query(&mut self) -> Option<Weight> {
        if self.s == self.rank.len() as NodeId || self.t == self.rank.len() as NodeId {
            return None;
        }

        self.reset();

        let mut tentative_distance = Weight::infinity();

        let mut fw_min_key = 0;
        let mut bw_min_key = 0;

        if !self.fw_finished {
            // safe after init
            fw_min_key = self.fw_search.min_key().unwrap();
        }

        if !self.bw_finished {
            // safe after init
            bw_min_key = self.bw_search.min_key().unwrap();
        }

        let mut settled_fw = BitVec::from_elem(self.fw_search.graph.num_nodes(), false);
        let mut settled_bw = BitVec::from_elem(self.bw_search.graph.num_nodes(), false);
        let mut _middle_node = self.fw_search.graph.num_nodes() as NodeId;
        let mut fw_next = true;

        let mut _needs_core = false;

        let time = Instant::now();
        while !self.fw_finished || !self.bw_finished {
            if self.bw_finished || !self.fw_finished && fw_next {
                if let Some(State {
                    distance: dist_from_queue_at_v,
                    node,
                }) = self.fw_search.settle_next_label(self.t)
                {
                    settled_fw.set(node as usize, true);

                    // fw search found t -> done here
                    if node == self.t {
                        tentative_distance = self.fw_search.get_settled_labels_at(node).last().unwrap().0.distance[0]; // dist_from_queue_at_v[0];
                        self.fw_finished = true;
                        self.bw_finished = true;
                        _needs_core = false;

                        break;
                    }

                    if settled_bw.get(node as usize).unwrap() {
                        let tent_dist_at_v = self.calculate_distance_with_break_at(node);

                        if tentative_distance > tent_dist_at_v {
                            tentative_distance = tent_dist_at_v;
                            _middle_node = node;
                        }
                    }

                    if self.fw_search.min_key().is_none() {
                        // || self.fw_search.get_settled_labels_at(node).last().unwrap().0.distance[0] >= tentative_distance {
                        self.fw_finished = true;
                    }

                    if self.fw_finished {
                        println!("fw finished in {} ms", time.elapsed().as_secs_f64() * 1000.0);
                    }

                    if self.is_core.get(node as usize).unwrap() && !_needs_core {
                        println!("fw core reached in {} ms", time.elapsed().as_secs_f64() * 1000.0);
                    }

                    if self.is_core.get(node as usize).unwrap() {
                        _needs_core = true;
                    }

                    fw_next = false;
                }
            } else {
                if let Some(State {
                    distance: dist_from_queue_at_v,
                    node,
                }) = self.bw_search.settle_next_label(self.s)
                {
                    settled_bw.set(node as usize, true);

                    // bw search found s -> done here
                    if node == self.s {
                        tentative_distance = self.bw_search.get_settled_labels_at(node).last().unwrap().0.distance[0]; // dist_from_queue_at_v[0];

                        self.fw_finished = true;
                        self.bw_finished = true;
                        _needs_core = false;

                        break;
                    }

                    if settled_fw.get(node as usize).unwrap() {
                        let tent_dist_at_v = self.calculate_distance_with_break_at(node);

                        if tentative_distance > tent_dist_at_v {
                            tentative_distance = tent_dist_at_v;
                            _middle_node = node;
                        }
                    }

                    if self.bw_search.min_key().is_none() || self.bw_search.min_key().unwrap() >= tentative_distance {
                        // self.bw_search.get_settled_labels_at(node).last().unwrap().0.distance[0] >= tentative_distance {
                        self.bw_finished = true;
                    }

                    if self.bw_finished {
                        println!("bw finished in {} ms", time.elapsed().as_secs_f64() * 1000.0);
                    }

                    if self.is_core.get(node as usize).unwrap() && !_needs_core {
                        println!("bw core reached in {} ms", time.elapsed().as_secs_f64() * 1000.0);
                    }

                    if self.is_core.get(node as usize).unwrap() {
                        _needs_core = true;
                    }

                    fw_next = true;
                }
            }
        }

        if tentative_distance == Weight::infinity() {
            self.last_dist = None;
            return None;
        }

        self.last_dist = Some(tentative_distance);
        return self.last_dist;
    }
}
