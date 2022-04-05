use crate::{algo::dijkstra::OwnedDijkstra, io::Load, types::*};
use bit_vec::BitVec;
use std::{path::Path, time::Instant};

#[derive(Clone)]
pub struct OneBreakCoreContractionHierarchy {
    pub rank: Vec<u32>,
    pub is_core: BitVec,
    pub fw_search: OwnedDijkstra,
    pub bw_search: OwnedDijkstra,
    fw_finished: bool,
    bw_finished: bool,
    s: NodeId,
    t: NodeId,
    pub restrictions: Vec<DrivingTimeRestriction>,
    pub last_dist: Option<Weight>,
    pub last_break: Option<DrivingTimeRestriction>,
}

impl OneBreakCoreContractionHierarchy {
    pub fn load_from_routingkit_dir<P: AsRef<Path>>(path: P) -> Result<Self, std::io::Error> {
        Ok(OneBreakCoreContractionHierarchy::build(
            Vec::<u32>::load_from(path.as_ref().join("rank"))?,
            Vec::<u32>::load_from(path.as_ref().join("core"))?,
            OwnedGraph::load_from_routingkit_dir(path.as_ref().join("forward"))?,
            OwnedGraph::load_from_routingkit_dir(path.as_ref().join("backward"))?,
        ))
    }

    pub fn build(rank: Vec<u32>, core: Vec<NodeId>, forward: OwnedGraph, backward: OwnedGraph) -> Self {
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

        OneBreakCoreContractionHierarchy {
            rank,
            is_core,
            fw_search: OwnedDijkstra::new(forward),
            bw_search: OwnedDijkstra::new(backward),
            fw_finished: false,
            bw_finished: false,
            s: node_count as NodeId,
            t: node_count as NodeId,
            restrictions: vec![],
            last_dist: None,
            last_break: None,
        }
    }

    pub fn add_restriction(&mut self, max_driving_time: Weight, pause_time: Weight) {
        self.restrictions.push(DrivingTimeRestriction { pause_time, max_driving_time });
        self.restrictions.sort();
    }

    pub fn clear_restrictions(&mut self) {
        self.restrictions.clear();
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
        if self.s != self.rank.len() as NodeId && !self.is_core.get(self.s as usize).unwrap() {
            self.fw_search.init_new_s(self.s);
            self.fw_finished = false;
        } else {
            self.fw_finished = true;
        }

        if self.t != self.rank.len() as NodeId && !self.is_core.get(self.t as usize).unwrap() {
            self.bw_search.init_new_s(self.t);
            self.bw_finished = false;
        } else {
            self.bw_finished = true;
        }
    }

    fn calculate_distance_with_break_at(&self, node: NodeId) -> (Weight, Option<DrivingTimeRestriction>) {
        let s_to_v = self.fw_search.tentative_distance_at(node);
        let v_to_t = self.bw_search.tentative_distance_at(node);
        let total_time = s_to_v + v_to_t;

        if let Some(&restriction) = self.restrictions.first() {
            if total_time < restriction.max_driving_time {
                return (total_time, None);
            }

            // needs break
            // cannot do break if not at a core node
            if !self.is_core.get(node as usize).unwrap() {
                return (INFINITY, None);
            }

            // if path does exist at all
            let longer_part = s_to_v.max(v_to_t);
            if longer_part < restriction.max_driving_time {
                // choose required pause time
                for r in self.restrictions.iter().rev() {
                    if total_time >= r.max_driving_time {
                        return (total_time + r.pause_time, Some(*r));
                    }
                }
            }

            // no path
            (INFINITY, None)
        } else {
            return (total_time, None);
        }
    }

    pub fn run_query(&mut self) -> Option<Weight> {
        let mut tentative_distance = Weight::infinity();

        let mut fw_min_key = 0;
        let mut bw_min_key = 0;

        self.reset();
        if !self.fw_finished {
            // safe after dijkstra init
            fw_min_key = self.fw_search.min_key().unwrap();
        }

        if !self.bw_finished {
            // safe after dijkstra init
            bw_min_key = self.bw_search.min_key().unwrap();
        }

        let mut settled_fw = BitVec::from_elem(self.fw_search.graph.num_nodes(), false);
        let mut settled_bw = BitVec::from_elem(self.bw_search.graph.num_nodes(), false);
        let mut _middle_node = self.fw_search.graph.num_nodes() as NodeId;
        let mut fw_next = true;

        let driving_time_limit = if self.restrictions.is_empty() {
            INFINITY
        } else {
            self.restrictions.first().unwrap().max_driving_time
        };

        let mut needs_break = None;
        let mut _needs_core = false;

        let time = Instant::now();
        while !self.fw_finished || !self.bw_finished {
            if self.bw_finished || !self.fw_finished && fw_next {
                if let Some(State { distance: _, node }) = self.fw_search.settle_next_node_not_exceeding(driving_time_limit) {
                    settled_fw.set(node as usize, true);

                    // fw search found t -> done here
                    if node == self.t {
                        tentative_distance = self.fw_search.tentative_distance_at(self.t);
                        self.fw_finished = true;
                        self.bw_finished = true;
                        _needs_core = false;

                        break;
                    }
                    if settled_bw.get(node as usize).unwrap() {
                        let (tent_dist_at_v, break_type) = self.calculate_distance_with_break_at(node);

                        if tentative_distance > tent_dist_at_v {
                            tentative_distance = tent_dist_at_v;
                            _middle_node = node;
                            needs_break = break_type;
                        }
                    }
                    fw_min_key = self.fw_search.min_key().unwrap_or_else(|| {
                        self.fw_finished = true;
                        fw_min_key
                    });

                    if fw_min_key >= tentative_distance {
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
                if let Some(State { distance: _, node }) = self.bw_search.settle_next_node_not_exceeding(driving_time_limit) {
                    settled_bw.set(node as usize, true);

                    // bw search found s -> done here
                    if node == self.s {
                        tentative_distance = self.bw_search.tentative_distance_at(self.s);
                        self.fw_finished = true;
                        self.bw_finished = true;
                        _needs_core = false;

                        break;
                    }

                    if settled_fw.get(node as usize).unwrap() {
                        let (tent_dist_at_v, break_type) = self.calculate_distance_with_break_at(node);
                        if tentative_distance > tent_dist_at_v {
                            tentative_distance = tent_dist_at_v;
                            _middle_node = node;
                            needs_break = break_type;
                        }
                    }
                    bw_min_key = self.bw_search.min_key().unwrap_or_else(|| {
                        self.bw_finished = true;
                        bw_min_key
                    });

                    if bw_min_key >= tentative_distance {
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
            self.last_break = None;
            self.last_dist = None;
            return None;
        }

        self.last_break = needs_break;
        self.last_dist = Some(tentative_distance);

        return self.last_dist;
    }
}
