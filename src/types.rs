//! This module contains a few basic type and constant definitions
use crate::{index_heap::*, io::Load, rrr_indexed_heap::AutoIndexedHeap};
use std::{cmp::Reverse, path::Path};

/// Node ids are unsigned 32 bit integers
pub type NodeId = u32;
/// Edge ids are unsigned 32 bit integers
pub type EdgeId = u32;
/// Edge weights are unsigned 32 bit integers
pub type Weight = u32;
/// The `INFINITY` weight is defined so that the check `INFINITY + INFINITY < INFINITY` does not cause any overflows
pub const INFINITY: Weight = std::u32::MAX / 2;

pub type Weight2 = [Weight; 2];
pub type Weight3 = [Weight; 3];

pub const EU_SHORT_PAUSE_TIME: Weight = 2_700_000; // 45 minutes
pub const EU_LONG_PAUSE_TIME: Weight = 32_400_000; // 9 hours
pub const EU_SHORT_DRIVING_TIME: Weight = 16_200_000; // 4.5 hours
pub const EU_LONG_DRIVING_TIME: Weight = 32_400_000; // 9 hours

pub trait DefaultReset: Clone {
    const DEFAULT: Self;
    fn reset(&mut self) {
        *self = Self::DEFAULT
    }
}

impl DefaultReset for Weight {
    const DEFAULT: Weight = INFINITY;
}

impl<T: Clone> DefaultReset for Option<T> {
    const DEFAULT: Option<T> = None;
}

impl DefaultReset for Weight2 {
    const DEFAULT: Weight2 = [INFINITY; 2];
}

impl DefaultReset for Weight3 {
    const DEFAULT: Weight3 = [INFINITY; 3];
}

pub trait WeightOps: Ord + Clone + Copy + std::fmt::Debug {
    fn dominates(&self, other: &Self) -> bool;
    #[must_use]
    fn link(&self, other: Weight) -> Self;
    fn zero() -> Self;
    fn infinity() -> Self;
    fn reset_distance(&mut self, i: usize, pause_time: Weight);
    #[must_use]
    fn add(&self, other: Self) -> Self;
}

impl WeightOps for Weight {
    #[inline(always)]
    fn dominates(&self, other: &Self) -> bool {
        self <= other
    }

    #[inline(always)]
    fn link(&self, other: Weight) -> Self {
        self + other
    }

    #[inline(always)]
    fn zero() -> Self {
        0
    }

    #[inline(always)]
    fn infinity() -> Self {
        INFINITY
    }

    #[inline(always)]
    fn reset_distance(&mut self, _i: usize, _pause_time: Weight) {}

    #[inline(always)]
    fn add(&self, other: Self) -> Self {
        self + other
    }
}

impl WeightOps for Weight2 {
    #[inline(always)]
    fn dominates(&self, other: &Self) -> bool {
        self[0] <= other[0] && self[1] <= other[1]
    }

    #[inline(always)]
    fn link(&self, other: Weight) -> Self {
        [self[0] + other, self[1] + other]
    }

    #[inline(always)]
    fn zero() -> Self {
        [0, 0]
    }

    #[inline(always)]
    fn infinity() -> Self {
        [INFINITY, INFINITY]
    }

    #[inline(always)]
    fn reset_distance(&mut self, i: usize, pause_time: Weight) {
        if i == 1 {
            self[0] += pause_time;
            self[1] = 0;
        }
    }

    #[inline(always)]
    fn add(&self, other: Self) -> Self {
        [self[0] + other[0], self[1] + other[1]]
    }
}

impl WeightOps for Weight3 {
    #[inline(always)]
    fn dominates(&self, other: &Self) -> bool {
        self[0] <= other[0] && self[1] <= other[1] && self[2] <= other[2]
    }

    #[inline(always)]
    fn link(&self, other: Weight) -> Self {
        [self[0] + other, self[1] + other, self[2] + other]
    }

    #[inline(always)]
    fn zero() -> Self {
        [0, 0, 0]
    }

    #[inline(always)]
    fn infinity() -> Self {
        [INFINITY, INFINITY, INFINITY]
    }

    #[inline(always)]
    fn reset_distance(&mut self, i: usize, pause_time: Weight) {
        if i == 1 {
            self[0] += pause_time;
            self[1] = 0;
        } else if i == 2 {
            self[0] += pause_time;
            self[1] = 0;
            self[2] = 0;
        }
    }

    #[inline(always)]
    fn add(&self, other: Self) -> Self {
        [self[0] + other[0], self[1] + other[1], self[2] + other[2]]
    }
}

#[derive(Debug, Clone, Copy, Ord, PartialOrd, PartialEq, Eq)]
pub struct DrivingTimeRestriction {
    pub pause_time: Weight,
    pub max_driving_time: Weight,
}

#[derive(Copy, Clone, Eq, PartialEq, Debug, Ord, PartialOrd)]
pub struct State<T> {
    pub distance: T,
    pub node: NodeId,
}

impl<T> Indexing for State<T> {
    fn as_index(&self) -> usize {
        self.node as usize
    }
}

#[derive(Debug, Clone)]
pub struct FirstOutGraph<FirstOutContainer, HeadContainer, WeightsContainer> {
    first_out: FirstOutContainer,
    head: HeadContainer,
    weights: WeightsContainer,
}

pub trait Graph {
    fn num_nodes(&self) -> usize;
    fn num_arcs(&self) -> usize;
    fn degree(&self, node: NodeId) -> usize;
}

pub trait OutgoingEdgeIterable: Graph {
    type Iter<'a>: Iterator<Item = (&'a Weight, &'a NodeId)>
    where
        Self: 'a;

    fn outgoing_edge_iter(&self, node: NodeId) -> Self::Iter<'_>;
}

pub type OwnedGraph = FirstOutGraph<Vec<EdgeId>, Vec<NodeId>, Vec<Weight>>;
pub type BorrowedGraph<'a> = FirstOutGraph<&'a [EdgeId], &'a [NodeId], &'a [Weight]>;

impl<FirstOutContainer, HeadContainer, WeightsContainer> FirstOutGraph<FirstOutContainer, HeadContainer, WeightsContainer>
where
    FirstOutContainer: AsRef<[EdgeId]>,
    HeadContainer: AsRef<[NodeId]>,
    WeightsContainer: AsRef<[Weight]>,
{
    pub fn new(first_out: FirstOutContainer, head: HeadContainer, weights: WeightsContainer) -> Self {
        Self { first_out, head, weights }
    }

    pub fn first_out(&self) -> &[EdgeId] {
        self.first_out.as_ref()
    }
    pub fn head(&self) -> &[NodeId] {
        self.head.as_ref()
    }
    pub fn weights(&self) -> &[Weight] {
        self.weights.as_ref()
    }

    pub fn borrow(&self) -> BorrowedGraph {
        FirstOutGraph {
            first_out: self.first_out(),
            head: self.head(),
            weights: self.weights(),
        }
    }
}

impl<FirstOutContainer, HeadContainer, WeightsContainer> Graph for FirstOutGraph<FirstOutContainer, HeadContainer, WeightsContainer>
where
    FirstOutContainer: AsRef<[EdgeId]>,
    HeadContainer: AsRef<[NodeId]>,
    WeightsContainer: AsRef<[Weight]>,
{
    fn num_nodes(&self) -> usize {
        self.first_out().len() - 1
    }

    fn num_arcs(&self) -> usize {
        self.head().len()
    }

    fn degree(&self, node: NodeId) -> usize {
        let node = node as usize;
        (self.first_out()[node + 1] - self.first_out()[node]) as usize
    }
}

impl OwnedGraph {
    pub fn load_from_routingkit_dir<P: AsRef<Path>>(path: P) -> Result<Self, std::io::Error> {
        Ok(Self {
            first_out: Vec::<EdgeId>::load_from(path.as_ref().join("first_out"))?,
            head: Vec::<NodeId>::load_from(path.as_ref().join("head"))?,
            weights: Vec::<Weight>::load_from(path.as_ref().join("travel_time"))?,
        })
    }

    pub fn reverse<G: Graph + OutgoingEdgeIterable>(graph: G) -> Self {
        let mut reversed_first_out = vec![0u32; graph.num_nodes() + 1];
        for node in 0..(graph.num_nodes() as NodeId) {
            for (_, &neighbor) in graph.outgoing_edge_iter(node) {
                reversed_first_out[neighbor as usize + 1] += 1;
            }
        }

        let mut prefix_sum = 0;
        for deg in &mut reversed_first_out {
            prefix_sum += *deg;
            *deg = prefix_sum;
        }

        let mut head = vec![0; graph.num_arcs()];
        let mut weights = vec![0; graph.num_arcs()];

        // iterate over all edges and insert them in the reversed structure
        for node in 0..(graph.num_nodes() as NodeId) {
            for (&weight, &neighbor) in graph.outgoing_edge_iter(node) {
                let idx = reversed_first_out[neighbor as usize] as usize;
                reversed_first_out[neighbor as usize] += 1;
                head[idx] = node;
                weights[idx] = weight;
            }
        }

        for node in (1..graph.num_nodes()).rev() {
            reversed_first_out[node] = reversed_first_out[node - 1];
        }
        reversed_first_out[0] = 0;

        OwnedGraph::new(reversed_first_out, head, weights)
    }
}

impl<'a> Copy for BorrowedGraph<'a> {}

impl<FirstOutContainer, HeadContainer, WeightsContainer> OutgoingEdgeIterable for FirstOutGraph<FirstOutContainer, HeadContainer, WeightsContainer>
where
    FirstOutContainer: AsRef<[EdgeId]>,
    HeadContainer: AsRef<[NodeId]>,
    WeightsContainer: AsRef<[Weight]>,
{
    type Iter<'a>
    
    = impl Iterator<Item = (&'a Weight, &'a NodeId)> + 'a where Self: 'a;

    #[inline]
    fn outgoing_edge_iter(&self, node: NodeId) -> Self::Iter<'_> {
        let range = self.first_out()[node as usize] as usize..self.first_out()[node as usize + 1] as usize;
        self.weights()[range.clone()].iter().zip(self.head()[range.clone()].iter())
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Debug, Ord, PartialOrd)]
pub struct Label<T> {
    pub distance_with_potential: T,
    pub distance: T,
    pub prev_node: NodeId,
    pub prev_label: Option<usize>,
}

pub type MCDHeap<L> = AutoIndexedHeap<Reverse<Label<L>>>;

impl<L: Ord + Clone + Copy> DefaultReset for MCDHeap<L> {
    const DEFAULT: MCDHeap<L> = MCDHeap::<L>::new();

    fn reset(&mut self) {
        self.reset();
    }
}

pub struct STQuery {
    pub s: NodeId,
    pub t: NodeId,
}
