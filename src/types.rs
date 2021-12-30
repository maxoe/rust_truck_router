//! This module contains a few basic type and constant definitions
use crate::index_heap::*;
use num::traits::Zero;
use std;
use std::ops::Add;

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

// pub trait Label: Ord + Zero + Add {
//     fn link(&self, other: &Self) -> Self;
// }

pub trait WeightOps: Ord + Clone + Copy {
    fn dominates(&self, other: &Self) -> bool;
    // fn dominates_any<'a>(&self, others: impl Iterator<Item = Self>) -> bool {
    //     return others.all(|e| self > &e);
    // }
    fn link(&self, other: &Self) -> Self;
    fn zero() -> Self;
    fn infinity() -> Self;
}

// impl<T> WeightOps for T
// where
//     T: Add + Ord + std::ops::Add<Output = T> + Zero,
// {
//     fn link(self, other: T) -> T {
//         self + other
//     }

//     fn zero() -> Self {
//         self.zero()
//     }
// }

impl WeightOps for Weight {
    #[inline(always)]
    fn dominates(&self, other: &Self) -> bool {
        self < other
    }

    #[inline(always)]
    fn link(&self, other: &Self) -> Self {
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
}

impl WeightOps for Weight2 {
    #[inline(always)]
    fn dominates(&self, other: &Self) -> bool {
        self[0] <= other[0] && self[1] <= other[1]
    }

    #[inline(always)]
    fn link(&self, other: &Self) -> Self {
        [self[0] + other[0], self[1] + other[1]]
    }

    #[inline(always)]
    fn zero() -> Self {
        [0, 0]
    }

    #[inline(always)]
    fn infinity() -> Self {
        [INFINITY, INFINITY]
    }
}

// pub struct Label {
//     pub weights: [Weight],
// }

// impl Label {
//     fn new(label_weights: [&Weight]) -> Self {
//         Self { weights: label_weights }
//     }

//     fn new_from_2_tuple(label_weights: (Weight, Weight)) -> Self {
//         Self {
//             weights: [label_weights.0, label_weights.1],
//         }
//     }
// }

// impl Ord for Label {
//     fn cmp(&self, other: &Self) -> std::cmp::Ordering {
//         self.weights.cmp(&other.weights)
//     }
// }

// impl WeightOps for Label {
//     fn dominates(&self, other: &Self) -> bool {
//         self.gt(other)
//     }

//     fn link(&self, rhs: Self) -> Self {
//         self.add(rhs)
//     }
// }

// impl Zero for Weight2 {
//     fn zero() -> Self {
//         (0, 0)
//     }
// }

// impl Label for Label2 {
//     fn link(&self, other: &Self) -> Self {
//         todo!()
//     }
// }

// pub struct Label<T: WeightOps> {
//     pub weights: &[T],
// }

// impl Label {
//     pub fn link(&self, other: &Self) -> Self {}
// }

// #[derive(Ord, Clone, Zero, Add)]
// pub struct Label {}

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
    type WeightType: WeightOps;
    fn num_nodes(&self) -> usize;
    fn num_arcs(&self) -> usize;
    fn degree(&self, node: NodeId) -> usize;
}

pub trait OutgoingEdgeIterable: Graph {
    type Iter<'a>: Iterator<Item = (&'a Self::WeightType, &'a NodeId)>
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
    pub fn weights(&self) -> &[<FirstOutGraph<FirstOutContainer, HeadContainer, WeightsContainer> as Graph>::WeightType] {
        self.weights.as_ref()
    }

    pub fn borrow(&self) -> FirstOutGraph<&[EdgeId], &[NodeId], &[<FirstOutGraph<FirstOutContainer, HeadContainer, WeightsContainer> as Graph>::WeightType]> {
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
    type WeightType = Weight;

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
    = impl Iterator<Item = (&'a Self::WeightType, &'a NodeId)> + 'a;

    #[inline]
    fn outgoing_edge_iter(&self, node: NodeId) -> Self::Iter<'_> {
        let range = self.first_out()[node as usize] as usize..self.first_out()[node as usize + 1] as usize;
        self.weights()[range.clone()].iter().zip(self.head()[range].iter())
    }
}
