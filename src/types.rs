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

// pub trait LinkIterable {
// 	fn link_iter(&self, node: NodeId) -> Self::Iter;
// }

pub type OwnedGraph = FirstOutGraph<Vec<EdgeId>, Vec<NodeId>, Vec<Weight>>;
pub type BorrowedGraph<'a> = FirstOutGraph<&'a [EdgeId], &'a [NodeId], &'a [Weight]>;

impl<FirstOutContainer, HeadContainer, WeightContainer> FirstOutGraph<FirstOutContainer, HeadContainer, WeightContainer>
where
	FirstOutContainer: AsRef<[EdgeId]>,
	HeadContainer: AsRef<[NodeId]>,
	WeightContainer: AsRef<[Weight]>,
{
	pub fn new(first_out: FirstOutContainer, head: HeadContainer, weights: WeightContainer) -> Self {
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

impl<FirstOutContainer, HeadContainer, WeightContainer> Graph for FirstOutGraph<FirstOutContainer, HeadContainer, WeightContainer>
where
	FirstOutContainer: AsRef<[EdgeId]>,
	HeadContainer: AsRef<[NodeId]>,
	WeightContainer: AsRef<[Weight]>,
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
