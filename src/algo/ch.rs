use std::{fmt::Debug, mem, path::Path, slice};

use bit_vec::BitVec;

use crate::{
    io::{DataBytesMut, Load},
    types::*,
};

use super::dijkstra::OwnedDijkstra;

#[derive(Debug, Clone)]
pub struct CHGraph {
    pub graph: OwnedGraph,
    // is_sc_orig_arc: BitVec,
    // sc_first_arc: Vec<Weight>,
    // sc_second_arc: Vec<Weight>,
}

#[repr(C)]
// #[repr(C, packed)]
#[derive(Debug, Copy, Clone)]
struct CHHeader {
    magic: u64,
    node_count: u32,
    fwd_arc_count: u32,
    bw_arc_count: u32,
}

impl DataBytesMut for CHHeader {
    fn data_bytes_mut(&mut self) -> &mut [u8] {
        let num_bytes = mem::size_of::<Self>();
        unsafe { slice::from_raw_parts_mut(self as *mut _ as *mut u8, num_bytes) }
    }
}

pub struct ContractionHierarchy {
    order: Vec<u32>,
    rank: Vec<u32>,
    // pub forward: CHGraph,
    // pub backward: CHGraph,
    fw_search: OwnedDijkstra,
    bw_search: OwnedDijkstra,
}

impl ContractionHierarchy {
    // pub fn load_from_routingkit_file<P: AsRef<Path>>(path: P) -> Result<Self, std::io::Error> {
    //     let mut file = File::open(path)?;
    //     // let mut header: CHHeader = unsafe { mem::zeroed() };
    //     // file.read_exact(header.data_bytes_mut())?;
    //     // println!("{:?}", header);

    //     let mut order = vec![0u32; header.node_count as usize];
    //     file.read_exact(order.data_bytes_mut())?;
    //     // order.reverse();

    //     let mut fwd_first_out = vec![0u32; header.node_count as usize + 1];
    //     file.read_exact(fwd_first_out.data_bytes_mut())?;
    //     let mut fwd_head = vec![0u32; header.fwd_arc_count as usize];
    //     file.read_exact(fwd_head.data_bytes_mut())?;
    //     let mut fwd_weights = vec![0u32; header.fwd_arc_count as usize];
    //     file.read_exact(fwd_weights.data_bytes_mut())?;
    //     let mut fwd_is_sc_orig_arc_buffer = vec![0u8; ((header.fwd_arc_count as usize + 511) / 512) * 64];
    //     file.read_exact(fwd_is_sc_orig_arc_buffer.data_bytes_mut())?;
    //     let fwd_is_sc_orig_arc = BitVec::from_bytes(fwd_is_sc_orig_arc_buffer.data_bytes());
    //     let mut fwd_sc_first_arc = vec![0u32; header.fwd_arc_count as usize];
    //     file.read_exact(fwd_sc_first_arc.data_bytes_mut())?;
    //     let mut fwd_sc_second_arc = vec![0u32; header.fwd_arc_count as usize];
    //     file.read_exact(fwd_sc_second_arc.data_bytes_mut())?;

    //     let fwd_graph = CHGraph {
    //         graph: OwnedGraph::new(fwd_first_out, fwd_head, fwd_weights),
    //         is_sc_orig_arc: fwd_is_sc_orig_arc,
    //         sc_first_arc: fwd_sc_first_arc,
    //         sc_second_arc: fwd_sc_second_arc,
    //     };

    //     let mut bw_first_out = vec![0u32; header.node_count as usize + 1];
    //     file.read_exact(bw_first_out.data_bytes_mut())?;
    //     let mut bw_head = vec![0u32; header.bw_arc_count as usize];
    //     file.read_exact(bw_head.data_bytes_mut())?;
    //     let mut bw_weights = vec![0u32; header.bw_arc_count as usize];
    //     file.read_exact(bw_weights.data_bytes_mut())?;
    //     let mut bw_is_sc_orig_arc_buffer = vec![0u8; ((header.bw_arc_count as usize + 511) / 512) * 64];
    //     file.read_exact(bw_is_sc_orig_arc_buffer.data_bytes_mut())?;
    //     let bw_is_sc_orig_arc = BitVec::from_bytes(bw_is_sc_orig_arc_buffer.data_bytes());
    //     let mut bw_sc_first_arc = vec![0u32; header.bw_arc_count as usize];
    //     file.read_exact(bw_sc_first_arc.data_bytes_mut())?;
    //     let mut bw_sc_second_arc = vec![0u32; header.bw_arc_count as usize];
    //     file.read_exact(bw_sc_second_arc.data_bytes_mut())?;

    //     let mut buf = Vec::new();
    //     let bytes_left = file.read_to_end(&mut buf)?;
    //     if bytes_left > 0 {
    //         return Err(std::io::Error::new(
    //             ErrorKind::InvalidData,
    //             format!("CH file exceeded expected file length by {} bytes", bytes_left),
    //         ));
    //     }

    //     let bw_graph = CHGraph {
    //         graph: OwnedGraph::new(bw_first_out, bw_head, bw_weights),
    //         is_sc_orig_arc: bw_is_sc_orig_arc,
    //         sc_first_arc: bw_sc_first_arc,
    //         sc_second_arc: bw_sc_second_arc,
    //     };

    //     Ok(ContractionHierarchy {
    //         order,
    //         forward: fwd_graph,
    //         backward: bw_graph,
    //     })
    // }

    pub fn load_from_routingkit_dir<P: AsRef<Path>>(path: P) -> Result<Self, std::io::Error> {
        let rank = Vec::<NodeId>::load_from(path.as_ref().join("rank"))?;
        let mut order = rank.clone();
        order.reverse();

        // let fwd_graph = CHGraph {
        //     graph: OwnedGraph::load_from_routingkit_dir(path.as_ref().join("forward"))?,
        // };

        // let bw_graph = CHGraph {
        //     graph: OwnedGraph::load_from_routingkit_dir(path.as_ref().join("backward"))?,
        // };

        Ok(ContractionHierarchy {
            order,
            rank,
            // forward: fwd_graph,
            // backward: bw_graph,
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
