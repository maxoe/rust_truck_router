//! This module contains a few basic type and constant definitions

use std;

/// Node ids are unsigned 32 bit integers
pub type NodeId = u32;
/// Edge ids are unsigned 32 bit integers
pub type EdgeId = u32;
/// Edge weights are unsigned 32 bit integers
pub type Weight = u32;
/// The `INFINITY` weight is defined so that the check `INFINITY + INFINITY < INFINITY` does not cause any overflows
pub const INFINITY: Weight = std::u32::MAX / 2;

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

    pub fn borrow(&self) -> FirstOutGraph<&[EdgeId], &[NodeId], &[EdgeId]> {
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

impl<FirstOutContainer, HeadContainer, WeightsContainer> OutgoingEdgeIterable for FirstOutGraph<FirstOutContainer, HeadContainer, WeightsContainer>
where
    FirstOutContainer: AsRef<[EdgeId]>,
    HeadContainer: AsRef<[NodeId]>,
    WeightsContainer: AsRef<[Weight]>,
{
    type Iter<'a>
    where
        Self: 'a,
    = impl Iterator<Item = (&'a Weight, &'a NodeId)> + 'a;

    #[inline]
    fn outgoing_edge_iter(&self, node: NodeId) -> Self::Iter<'_> {
        let range = self.first_out()[node as usize] as usize..self.first_out()[node as usize + 1] as usize;
        self.weights()[range.clone()].iter().zip(self.head()[range].iter())
    }
}
