use crate::{algo::dijkstra::OwnedDijkstra, io::Load, types::*};
use bit_vec::BitVec;
use std::{path::Path, time::Instant};

#[derive(Clone)]
pub struct CoreContractionHierarchy {
    pub rank: Vec<u32>,
    pub is_core: BitVec,
    pub core_search: OwnedDijkstra,
    pub fw_search: OwnedDijkstra,
    pub bw_search: OwnedDijkstra,
    fw_finished: bool,
    bw_finished: bool,
    needs_core: bool,
    s: NodeId,
    t: NodeId,
}

impl CoreContractionHierarchy {
    pub fn load_from_routingkit_dir<P: AsRef<Path>>(path: P) -> Result<Self, std::io::Error> {
        Ok(CoreContractionHierarchy::build(
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
        let mut first_out = Vec::with_capacity(core_node_count);
        first_out.push(0);
        let mut head = Vec::with_capacity(forward.num_arcs());
        let mut weights = Vec::with_capacity(forward.num_arcs());

        // let mut first_out_i = 0;
        // for node_id in 0..node_count {
        //     if is_core.get(node_id).unwrap() {
        //         first_out.push(first_out_i as NodeId);

        //         for edge_id in forward.first_out()[node_id]..forward.first_out()[node_id + 1] {
        //             let current_head = is_core.iter().take((forward.head()[edge_id as usize] + 1) as usize).filter(|b| *b).count() as NodeId;

        //             if is_core.get(current_head as usize).unwrap() {
        //                 head.push(current_head);
        //                 weights.push(forward.weights()[edge_id as usize]);
        //                 first_out_i += 1;
        //             }
        //         }

        //         for edge_id in backward.first_out()[node_id]..backward.first_out()[node_id + 1] {
        //             let current_head = is_core.iter().take((backward.head()[edge_id as usize] + 1) as usize).filter(|b| *b).count() as NodeId;

        //             if is_core.get(current_head as usize).unwrap() {
        //                 head.push(current_head);
        //                 weights.push(backward.weights()[edge_id as usize]);
        //                 first_out_i += 1;
        //             }
        //         }
        //     }
        // }

        // first_out.push(core_node_count as NodeId);

        first_out.shrink_to_fit();
        head.shrink_to_fit();
        weights.shrink_to_fit();

        // assert_eq!(first_out.len(), core_node_count + 1);
        // assert_eq!(head.len(), weights.len());
        // assert_eq!(first_out[0], 0);
        // assert_eq!(*first_out.last().unwrap() as usize, first_out.len() - 1);

        let core = OwnedGraph::new(first_out, head, weights);

        // dbg!(core.num_arcs());
        // dbg!(core.num_nodes());

        CoreContractionHierarchy {
            rank,
            is_core,
            core_search: OwnedDijkstra::new(core),
            fw_search: OwnedDijkstra::new(forward),
            bw_search: OwnedDijkstra::new(backward),
            fw_finished: false,
            bw_finished: false,
            needs_core: false,
            s: node_count as NodeId,
            t: node_count as NodeId,
        }
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

        if !self.is_core.get(self.s as usize).unwrap() {
            self.fw_search.init_new_s(self.s);
            self.fw_finished = false;
            // self.needs_core |= false;
            // self.core_s = self.fw_search.graph.num_nodes() as NodeId;
        } else {
            self.fw_finished = true;
            // self.needs_core = true;
            // self.core_s = ext_s;
        }
    }

    pub fn init_new_t(&mut self, ext_t: NodeId) {
        self.t = self.rank[ext_t as usize] as NodeId;

        if !self.is_core.get(self.t as usize).unwrap() {
            self.bw_search.init_new_s(self.t);
            self.bw_finished = false;
            // self.needs_core |= false;
            // self.core_t = self.bw_search.graph.num_nodes() as NodeId;
        } else {
            self.bw_finished = true;
            // self.needs_core = true;
            // self.core_t = ext_t;
        }
    }

    // fn to_core_index(&self, n: NodeId) -> Option<NodeId> {
    //     if self.is_core.get(n as usize).is_none() {
    //         None
    //     } else {
    //         Some(self.is_core.iter().take((n) as usize).filter(|b| *b).count() as NodeId)
    //     }
    // }

    pub fn run_query(&mut self) -> Option<Weight> {
        let mut tentative_distance = Weight::infinity();

        let mut fw_min_key = 0;
        let mut bw_min_key = 0;

        if !self.fw_finished {
            self.fw_search.reset();
            // safe after dijkstra init
            fw_min_key = self.fw_search.min_key().unwrap();
        }

        if !self.bw_finished {
            self.bw_search.reset();
            // safe after dijkstra init
            bw_min_key = self.bw_search.min_key().unwrap();
        }

        let mut settled_fw = BitVec::from_elem(self.fw_search.graph.num_nodes(), false);
        let mut settled_bw = BitVec::from_elem(self.bw_search.graph.num_nodes(), false);
        let mut _middle_node = self.fw_search.graph.num_nodes() as u32;
        let mut fw_next = true;

        let time = Instant::now();
        while !self.fw_finished || !self.bw_finished {
            let tent_dist_at_v;

            if self.bw_finished || !self.fw_finished && fw_next {
                if let Some(State { distance: _, node }) = self.fw_search.settle_next_node() {
                    //dbg!("fw settled {}", node);
                    settled_fw.set(node as usize, true);

                    // fw search found t -> done here
                    if node == self.t {
                        tentative_distance = self.fw_search.tentative_distance_at(self.t);
                        self.fw_finished = true;
                        self.bw_finished = true;
                        self.needs_core = false;

                        break;
                    }
                    if settled_bw.get(node as usize).unwrap() {
                        tent_dist_at_v = self.fw_search.tentative_distance_at(node) + self.bw_search.tentative_distance_at(node);
                        if tentative_distance > tent_dist_at_v {
                            tentative_distance = tent_dist_at_v;
                            _middle_node = node;
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
                    // if !self.is_core.get(node as usize).unwrap() && self.needs_core {
                    //     println!("non-core after core reached");
                    // }
                    if self.is_core.get(node as usize).unwrap() && !self.needs_core {
                        println!("fw core reached in {} ms", time.elapsed().as_secs_f64() * 1000.0);
                    }
                    if self.is_core.get(node as usize).unwrap() {
                        // dbg!("forward reached core");
                        //     self.fw_finished = true;
                        self.needs_core = true;
                        //     self.core_s.push(node);
                    }

                    fw_next = false;
                }
            } else {
                if let Some(State { distance: _, node }) = self.bw_search.settle_next_node() {
                    //dbg!("bw settled {}", node);
                    settled_bw.set(node as usize, true);

                    // bw search found s -> done here
                    if node == self.s {
                        tentative_distance = self.bw_search.tentative_distance_at(self.s);
                        self.fw_finished = true;
                        self.bw_finished = true;
                        self.needs_core = false;

                        break;
                    }

                    if settled_fw.get(node as usize).unwrap() {
                        tent_dist_at_v = self.fw_search.tentative_distance_at(node) + self.bw_search.tentative_distance_at(node);

                        if tentative_distance > tent_dist_at_v {
                            tentative_distance = tent_dist_at_v;
                            _middle_node = node;
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

                    if self.is_core.get(node as usize).unwrap() && !self.needs_core {
                        println!("bw core reached in {} ms", time.elapsed().as_secs_f64() * 1000.0);
                    }

                    if self.is_core.get(node as usize).unwrap() {
                        // dbg!("backwards reached core");
                        //     // self.bw_finished = true;
                        self.needs_core = true;
                        //     self.core_t.push(node);
                    }

                    fw_next = true;
                }
            }
        }

        if
        /* !self.needs_core &&*/
        tentative_distance == Weight::infinity() {
            return None;
        }

        //if !self.needs_core {
        return Some(tentative_distance);
        //}

        // dbg!("Searching in core");
        // dbg!(self.core_s);
        // dbg!(self.core_t);

        // for current_core_s in self.core_s
        // self.core_search.init_new_s(self.to_core_index(self.core_s).unwrap());

        // if let Some(core_dist) = self.core_search.dist_query(self.to_core_index(self.core_t).unwrap()) {
        //     dbg!(core_dist);
        //     Some(
        //         self.fw_search.tentative_distance_at(self.to_core_index(self.core_s).unwrap())
        //             + self.bw_search.tentative_distance_at(self.to_core_index(self.core_t).unwrap())
        //             + core_dist,
        //     )
        // } else {
        //     dbg!("core search failed");
        //     None
        // }
        // None
    }
}
