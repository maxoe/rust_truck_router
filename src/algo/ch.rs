use crate::{algo::dijkstra::OwnedDijkstra, io::Load, types::*};
use bit_vec::BitVec;
use std::path::Path;

#[derive(Clone)]
pub struct ContractionHierarchy {
    order: Vec<u32>,
    pub rank: Vec<u32>,
    pub fw_search: OwnedDijkstra,
    pub bw_search: OwnedDijkstra,
}

impl ContractionHierarchy {
    pub fn load_from_routingkit_dir<P: AsRef<Path>>(path: P) -> Result<Self, std::io::Error> {
        let rank = Vec::<NodeId>::load_from(path.as_ref().join("rank"))?;
        let mut order = rank.clone();
        order.reverse();

        Ok(ContractionHierarchy {
            order,
            rank,
            fw_search: OwnedDijkstra::new(OwnedGraph::load_from_routingkit_dir(path.as_ref().join("forward"))?),
            bw_search: OwnedDijkstra::new(OwnedGraph::load_from_routingkit_dir(path.as_ref().join("backward"))?),
        })
    }

    /// Equivalent to routingkit's check_contraction_hierarchy_for_errors
    pub fn check(&self) {
        let node_count = self.order.len();
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

        // only up edges
        for x in 0..node_count {
            for xy in self.fw_search.graph.first_out()[x]..self.fw_search.graph.first_out()[x + 1] {
                let y = self.fw_search.graph.head()[xy as usize] as usize;
                assert!(y > x);
            }

            for xy in self.bw_search.graph.first_out()[x]..self.bw_search.graph.first_out()[x + 1] {
                let y = self.bw_search.graph.head()[xy as usize] as usize;
                assert!(y > x);
            }
        }
    }

    pub fn init_new_s(&mut self, ext_s: NodeId) {
        let s = self.rank[ext_s as usize];
        self.fw_search.init_new_s(s);
    }

    pub fn init_new_t(&mut self, ext_t: NodeId) {
        let t = self.rank[ext_t as usize];
        self.bw_search.init_new_s(t);
    }

    pub fn run_query(&mut self) -> Option<Weight> {
        let mut tentative_distance = Weight::infinity();

        self.fw_search.reset();
        self.bw_search.reset();

        // safe after dijkstra init
        let mut fw_min_key = self.fw_search.min_key().unwrap();
        let mut bw_min_key = self.bw_search.min_key().unwrap();

        let mut fw_finished = false;
        let mut bw_finished = false;
        let mut settled_fw = BitVec::from_elem(self.fw_search.graph.num_nodes(), false);
        let mut settled_bw = BitVec::from_elem(self.bw_search.graph.num_nodes(), false);
        let mut _middle_node = self.fw_search.graph.num_nodes() as u32;
        let mut fw_next = true;

        while !fw_finished || !bw_finished {
            let tent_dist_at_v;

            if bw_finished || !fw_finished && fw_next {
                if let Some(State { distance: _, node }) = self.fw_search.settle_next_node() {
                    settled_fw.set(node as usize, true);

                    if settled_bw.get(node as usize).unwrap() {
                        tent_dist_at_v = self.fw_search.tentative_distance_at(node) + self.bw_search.tentative_distance_at(node);
                        if tentative_distance > tent_dist_at_v {
                            tentative_distance = tent_dist_at_v;
                            _middle_node = node;
                        }
                    }
                    fw_min_key = self.fw_search.min_key().unwrap_or_else(|| {
                        fw_finished = true;
                        fw_min_key
                    });

                    if fw_min_key >= tentative_distance {
                        fw_finished = true;
                    }
                    fw_next = false;
                }
            } else {
                if let Some(State { distance: _, node }) = self.bw_search.settle_next_node() {
                    settled_bw.set(node as usize, true);

                    if settled_fw.get(node as usize).unwrap() {
                        tent_dist_at_v = self.fw_search.tentative_distance_at(node) + self.bw_search.tentative_distance_at(node);

                        if tentative_distance > tent_dist_at_v {
                            tentative_distance = tent_dist_at_v;
                            _middle_node = node;
                        }
                    }
                    bw_min_key = self.bw_search.min_key().unwrap_or_else(|| {
                        bw_finished = true;
                        bw_min_key
                    });

                    if bw_min_key >= tentative_distance {
                        bw_finished = true;
                    }
                }
                fw_next = true;
            }
        }

        if tentative_distance == Weight::infinity() {
            return None;
        }

        Some(tentative_distance)
    }
}
