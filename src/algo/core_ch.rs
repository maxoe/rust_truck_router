use crate::{io::Load, types::*};
use bit_vec::BitVec;
use std::{path::Path, rc::Rc};

use super::dijkstra::{Dijkstra, DijkstraData};

#[derive(Clone)]
pub struct CoreContractionHierarchy<RankOrderContainer, FirstOutContainer, HeadContainer, WeightsContainer> {
    rank: RankOrderContainer,
    order: RankOrderContainer,
    is_core: Rc<BitVec>,
    pub forward: FirstOutGraph<FirstOutContainer, HeadContainer, WeightsContainer>,
    pub backward: FirstOutGraph<FirstOutContainer, HeadContainer, WeightsContainer>,
}

impl<RankOrderContainer, FirstOutContainer, HeadContainer, WeightsContainer>
    CoreContractionHierarchy<RankOrderContainer, FirstOutContainer, HeadContainer, WeightsContainer>
where
    RankOrderContainer: AsRef<[NodeId]>,
    FirstOutContainer: AsRef<[EdgeId]>,
    HeadContainer: AsRef<[NodeId]>,
    WeightsContainer: AsRef<[Weight]>,
{
    pub fn new(
        rank: RankOrderContainer,
        order: RankOrderContainer,
        core: RankOrderContainer,
        forward: FirstOutGraph<FirstOutContainer, HeadContainer, WeightsContainer>,
        backward: FirstOutGraph<FirstOutContainer, HeadContainer, WeightsContainer>,
    ) -> Self {
        let node_count = forward.num_nodes();

        let mut is_core = BitVec::from_elem(node_count, false);
        for &n in core.as_ref().iter() {
            is_core.set(rank.as_ref()[n as usize] as usize, true);
        }

        let core_node_count = core.as_ref().len();
        println!(
            "Core node count: {} ({:.2}%)",
            core_node_count,
            core_node_count as f32 * 100.0 / rank.as_ref().len() as f32
        );

        CoreContractionHierarchy {
            rank,
            order,
            is_core: Rc::new(is_core),
            forward,
            backward,
        }
    }

    pub fn rank(&self) -> &[NodeId] {
        self.rank.as_ref()
    }

    pub fn order(&self) -> &[NodeId] {
        self.order.as_ref()
    }

    pub fn is_core(&self) -> Rc<BitVec> {
        self.is_core.clone()
    }

    pub fn forward(&self) -> BorrowedGraph {
        self.forward.borrow()
    }

    pub fn backward(&self) -> BorrowedGraph {
        self.backward.borrow()
    }

    pub fn borrow(&self) -> BorrowedCoreContractionHierarchy {
        CoreContractionHierarchy {
            rank: self.rank(),
            order: self.order(),
            is_core: self.is_core(),
            forward: self.forward(),
            backward: self.backward(),
        }
    }

    /// Equivalent to routingkit's check_contraction_hierarchy_for_errors except the check for only up edges
    pub fn check(&self) {
        let node_count = self.forward.num_nodes();
        assert_eq!(self.forward.first_out().len(), node_count + 1);
        assert_eq!(self.backward.first_out().len(), node_count + 1);

        let forward_arc_count = *self.forward.first_out().last().unwrap();

        assert_eq!(*self.forward.first_out().first().unwrap(), 0);
        assert!(self.forward.first_out().is_sorted_by(|l, r| Some(l.cmp(r))));
        assert_eq!(self.forward.head().len(), forward_arc_count as usize);
        assert_eq!(self.forward.weights().len(), forward_arc_count as usize);
        assert!(!self.forward.head().is_empty() && *self.forward.head().iter().max().unwrap() < node_count as NodeId);

        let backward_arc_count = *self.backward.first_out().last().unwrap();
        assert_eq!(*self.backward.first_out().first().unwrap(), 0);
        assert!(self.backward.first_out().is_sorted_by(|l, r| Some(l.cmp(r))));
        assert_eq!(self.backward.head().len(), backward_arc_count as usize);
        assert_eq!(self.backward.weights().len(), backward_arc_count as usize);
        assert!(!self.backward.head().is_empty() && *self.backward.head().iter().max().unwrap() < node_count as NodeId);
    }
}

impl OwnedCoreContractionHierarchy {
    pub fn load_from_routingkit_dir<P: AsRef<Path>>(path: P) -> Result<Self, std::io::Error> {
        Ok(CoreContractionHierarchy::new(
            Vec::<NodeId>::load_from(path.as_ref().join("rank"))?,
            Vec::<NodeId>::load_from(path.as_ref().join("order"))?,
            Vec::<NodeId>::load_from(path.as_ref().join("core"))?,
            OwnedGraph::load_from_routingkit_dir(path.as_ref().join("forward"))?,
            OwnedGraph::load_from_routingkit_dir(path.as_ref().join("backward"))?,
        ))
    }
}

pub type OwnedCoreContractionHierarchy = CoreContractionHierarchy<Vec<NodeId>, Vec<EdgeId>, Vec<NodeId>, Vec<Weight>>;
pub type BorrowedCoreContractionHierarchy<'a> = CoreContractionHierarchy<&'a [NodeId], &'a [EdgeId], &'a [NodeId], &'a [Weight]>;

pub struct CoreContractionHierarchyQuery<'a> {
    core_ch: BorrowedCoreContractionHierarchy<'a>,
    fw_state: DijkstraData,
    bw_state: DijkstraData,
    fw_finished: bool,
    bw_finished: bool,
    s: NodeId,
    t: NodeId,
}
impl<'a> CoreContractionHierarchyQuery<'a> {
    pub fn new(core_ch: BorrowedCoreContractionHierarchy<'a>) -> Self {
        let n = core_ch.forward().num_nodes();
        CoreContractionHierarchyQuery {
            core_ch,
            fw_state: DijkstraData::new(n),
            bw_state: DijkstraData::new(n),
            fw_finished: false,
            bw_finished: false,
            s: n as NodeId,
            t: n as NodeId,
        }
    }

    pub fn init_new_s(&mut self, ext_s: NodeId) {
        self.s = self.core_ch.rank[ext_s as usize] as NodeId;

        if !self.core_ch.is_core.get(self.s as usize).unwrap() {
            self.fw_state.init_new_s(self.s);
            self.fw_finished = false;
        } else {
            self.fw_finished = true;
        }
    }

    pub fn init_new_t(&mut self, ext_t: NodeId) {
        self.t = self.core_ch.rank[ext_t as usize] as NodeId;

        if !self.core_ch.is_core.get(self.t as usize).unwrap() {
            self.bw_state.init_new_s(self.t);
            self.bw_finished = false;
        } else {
            self.bw_finished = true;
        }
    }

    pub fn run_query(&mut self) -> Option<Weight> {
        let mut tentative_distance = Weight::infinity();

        let mut fw_min_key = 0;
        let mut bw_min_key = 0;

        if !self.fw_finished {
            self.fw_state.reset();
            // safe after dijkstra init
            fw_min_key = self.fw_state.min_key().unwrap();
        }

        if !self.bw_finished {
            self.bw_state.reset();
            // safe after dijkstra init
            bw_min_key = self.bw_state.min_key().unwrap();
        }

        let mut settled_fw = BitVec::from_elem(self.core_ch.forward.num_nodes(), false);
        let mut settled_bw = BitVec::from_elem(self.core_ch.backward.num_nodes(), false);
        let mut _middle_node = self.core_ch.forward.num_nodes() as u32;
        let mut fw_next = true;

        let fw_search = Dijkstra::new(self.core_ch.forward());
        let bw_search = Dijkstra::new(self.core_ch.backward());

        while !self.fw_finished || !self.bw_finished {
            let tent_dist_at_v;

            if self.bw_finished || !self.fw_finished && fw_next {
                if let Some(State { distance: _, node }) = fw_search.settle_next_node(&mut self.fw_state) {
                    settled_fw.set(node as usize, true);

                    // fw search found t -> done here
                    if node == self.t {
                        tentative_distance = self.fw_state.tentative_distance_at(self.t);
                        self.fw_finished = true;
                        // self.bw_finished = true;

                        // break;
                    }
                    if settled_bw.get(node as usize).unwrap() {
                        tent_dist_at_v = self.fw_state.tentative_distance_at(node) + self.bw_state.tentative_distance_at(node);
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

                    fw_next = false;
                }
            } else if let Some(State { distance: _, node }) = bw_search.settle_next_node(&mut self.bw_state) {
                settled_bw.set(node as usize, true);

                // bw search found s -> done here
                if node == self.s {
                    tentative_distance = self.bw_state.tentative_distance_at(self.s);
                    // self.fw_finished = true;
                    self.bw_finished = true;
                    // self.needs_core = false;

                    // break;
                }

                if settled_fw.get(node as usize).unwrap() {
                    tent_dist_at_v = self.fw_state.tentative_distance_at(node) + self.bw_state.tentative_distance_at(node);

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

                fw_next = true;
            }
        }

        if tentative_distance == Weight::infinity() {
            return None;
        }

        Some(tentative_distance)
    }
}
