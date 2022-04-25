use crate::{io::Load, types::*};
use bit_vec::BitVec;
use std::path::Path;

use super::dijkstra::{Dijkstra, DijkstraData};

#[derive(Clone)]
pub struct ContractionHierarchy<RankOrderContainer, FirstOutContainer, HeadContainer, WeightsContainer> {
    rank: RankOrderContainer,
    order: RankOrderContainer,
    forward: FirstOutGraph<FirstOutContainer, HeadContainer, WeightsContainer>,
    backward: FirstOutGraph<FirstOutContainer, HeadContainer, WeightsContainer>,
}

impl<RankOrderContainer, FirstOutContainer, HeadContainer, WeightsContainer>
    ContractionHierarchy<RankOrderContainer, FirstOutContainer, HeadContainer, WeightsContainer>
where
    RankOrderContainer: AsRef<[NodeId]>,
    FirstOutContainer: AsRef<[EdgeId]>,
    HeadContainer: AsRef<[NodeId]>,
    WeightsContainer: AsRef<[Weight]>,
{
    pub fn new(
        rank: RankOrderContainer,
        order: RankOrderContainer,
        forward: FirstOutGraph<FirstOutContainer, HeadContainer, WeightsContainer>,
        backward: FirstOutGraph<FirstOutContainer, HeadContainer, WeightsContainer>,
    ) -> Self {
        Self {
            rank,
            order,
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

    pub fn forward(&self) -> BorrowedGraph {
        self.forward.borrow()
    }

    pub fn backward(&self) -> BorrowedGraph {
        self.backward.borrow()
    }

    pub fn borrow(&self) -> BorrowedContractionHierarchy {
        ContractionHierarchy {
            rank: self.rank(),
            order: self.order(),
            forward: self.forward(),
            backward: self.backward(),
        }
    }

    pub fn inverted(self) -> Self {
        ContractionHierarchy {
            rank: self.rank,
            order: self.order,
            forward: self.backward,
            backward: self.forward,
        }
    }

    /// From routingkit's check_contraction_hierarchy_for_errors
    pub fn check(&self) {
        let node_count = self.rank().len();
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

        // only up edges
        for x in 0..node_count {
            for xy in self.forward.first_out()[x]..self.forward.first_out()[x + 1] {
                let y = self.forward.head()[xy as usize] as usize;
                assert!(y > x);
            }

            for xy in self.backward.first_out()[x]..self.backward.first_out()[x + 1] {
                let y = self.backward.head()[xy as usize] as usize;
                assert!(y > x);
            }
        }
    }
}

impl OwnedContractionHierarchy {
    pub fn load_from_routingkit_dir<P: AsRef<Path>>(path: P) -> Result<Self, std::io::Error> {
        Ok(ContractionHierarchy {
            rank: Vec::<NodeId>::load_from(path.as_ref().join("rank"))?,
            order: Vec::<NodeId>::load_from(path.as_ref().join("order"))?,
            forward: OwnedGraph::load_from_routingkit_dir(path.as_ref().join("forward"))?,
            backward: OwnedGraph::load_from_routingkit_dir(path.as_ref().join("backward"))?,
        })
    }
}

pub type OwnedContractionHierarchy = ContractionHierarchy<Vec<NodeId>, Vec<EdgeId>, Vec<NodeId>, Vec<Weight>>;
pub type BorrowedContractionHierarchy<'a> = ContractionHierarchy<&'a [NodeId], &'a [EdgeId], &'a [NodeId], &'a [Weight]>;

impl<'a> Copy for BorrowedContractionHierarchy<'a> {}

pub struct ContractionHierarchyQuery<'a> {
    ch: BorrowedContractionHierarchy<'a>,
    fw_state: DijkstraData,
    bw_state: DijkstraData,
}

impl<'a> ContractionHierarchyQuery<'a> {
    pub fn new(ch: BorrowedContractionHierarchy<'a>) -> Self {
        let n = ch.forward().num_nodes();
        ContractionHierarchyQuery {
            ch,
            fw_state: DijkstraData::new(n),
            bw_state: DijkstraData::new(n),
        }
    }

    pub fn init_new_s(&mut self, ext_s: NodeId) {
        let s = self.ch.rank()[ext_s as usize];
        self.fw_state.init_new_s(s);
    }

    pub fn init_new_t(&mut self, ext_t: NodeId) {
        let t = self.ch.rank()[ext_t as usize];
        self.bw_state.init_new_s(t);
    }

    pub fn run_query(&mut self) -> Option<Weight> {
        let mut tentative_distance = Weight::infinity();

        self.fw_state.reset();
        self.bw_state.reset();

        // safe after dijkstra init
        let mut fw_min_key = self.fw_state.min_key().unwrap();
        let mut bw_min_key = self.bw_state.min_key().unwrap();

        let mut fw_finished = false;
        let mut bw_finished = false;
        let mut settled_fw = BitVec::from_elem(self.ch.forward().num_nodes(), false);
        let mut settled_bw = BitVec::from_elem(self.ch.forward().num_nodes(), false);
        let mut _middle_node = self.ch.forward().num_nodes() as u32;
        let mut fw_next = true;

        let fw_search = Dijkstra::new(self.ch.forward());
        let bw_search = Dijkstra::new(self.ch.backward());

        while !fw_finished || !bw_finished {
            let tent_dist_at_v;

            if bw_finished || !fw_finished && fw_next {
                if let Some(State { distance: _, node }) = fw_search.settle_next_node(&mut self.fw_state) {
                    settled_fw.set(node as usize, true);

                    if settled_bw.get(node as usize).unwrap() {
                        tent_dist_at_v = self.fw_state.tentative_distance_at(node) + self.bw_state.tentative_distance_at(node);
                        if tentative_distance > tent_dist_at_v {
                            tentative_distance = tent_dist_at_v;
                            _middle_node = node;
                        }
                    }
                    fw_min_key = self.fw_state.min_key().unwrap_or_else(|| {
                        fw_finished = true;
                        fw_min_key
                    });

                    if fw_min_key >= tentative_distance {
                        fw_finished = true;
                    }
                    fw_next = false;
                }
            } else {
                if let Some(State { distance: _, node }) = bw_search.settle_next_node(&mut self.bw_state) {
                    settled_bw.set(node as usize, true);

                    if settled_fw.get(node as usize).unwrap() {
                        tent_dist_at_v = self.fw_state.tentative_distance_at(node) + self.bw_state.tentative_distance_at(node);

                        if tentative_distance > tent_dist_at_v {
                            tentative_distance = tent_dist_at_v;
                            _middle_node = node;
                        }
                    }
                    bw_min_key = self.bw_state.min_key().unwrap_or_else(|| {
                        bw_finished = true;
                        bw_min_key
                    });

                    if bw_min_key >= tentative_distance {
                        bw_finished = true;
                    }
                    fw_next = true;
                }
            }
        }

        if tentative_distance == Weight::infinity() {
            return None;
        }

        Some(tentative_distance)
    }
}
