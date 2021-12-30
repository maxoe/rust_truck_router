use crate::{index_heap::*, types::*};
// use rust_road_router::datastr::heap::Heap;
use std::collections::BinaryHeap;

pub type OwnedMultiCritGraph = MultiCritFirstOutGraph<Vec<EdgeId>, Vec<NodeId>, Vec<Weight2>>;
pub type BorrowedMultiCritGraph<'a> = MultiCritFirstOutGraph<&'a [EdgeId], &'a [NodeId], &'a [Weight2]>;

#[derive(Debug, Clone)]
pub struct MultiCritFirstOutGraph<FirstOutContainer, HeadContainer, WeightsContainer> {
    first_out: FirstOutContainer,
    head: HeadContainer,
    weights: WeightsContainer,
}

impl<FirstOutContainer, HeadContainer, WeightsContainer> MultiCritFirstOutGraph<FirstOutContainer, HeadContainer, WeightsContainer>
where
    FirstOutContainer: AsRef<[EdgeId]>,
    HeadContainer: AsRef<[NodeId]>,
    WeightsContainer: AsRef<[Weight2]>,
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
    pub fn weights(&self) -> &[Weight2] {
        self.weights.as_ref()
    }

    pub fn borrow(&self) -> MultiCritFirstOutGraph<&[EdgeId], &[NodeId], &[Weight2]> {
        MultiCritFirstOutGraph {
            first_out: self.first_out(),
            head: self.head(),
            weights: self.weights(),
        }
    }
}
pub struct MultiCriteriaDijkstraData<L: WeightOps> {
    pub queue: IndexdMinHeap<State<L>>,
    pub per_node_labels: Vec<BinaryHeap<State<L>>>,
    // pub per_node_labels: Vec<Vec<(NodeId, L)>>,
}

impl<L: WeightOps> MultiCriteriaDijkstraData<L> {
    pub fn new(n: usize) -> Self {
        let mut per_node_labels = Vec::with_capacity(n);
        per_node_labels.resize_with(n, || BinaryHeap::new());

        Self {
            queue: IndexdMinHeap::new(n),
            per_node_labels: per_node_labels, // per_node_labels: vec![vec![]; n],
        }
    }

    // pub fn path(&self, s: NodeId, t: NodeId) -> Option<Vec<NodeId>> {
    //     let mut path = vec![t];
    //     let mut current_id = t;
    //     while current_id != s && current_id as usize != self.pred.len() {
    //         current_id = self.pred[current_id as usize].0;
    //         path.push(current_id);
    //     }

    //     if *path.last().unwrap() != s {
    //         return None;
    //     }

    //     path.reverse();
    //     Some(path)
    // }
}

pub struct MultiCriteriaDijkstra<G>
where
    G: Graph + OutgoingEdgeIterable,
{
    data: MultiCriteriaDijkstraData<G::WeightType>,
    s: NodeId,
    graph: G,
}

// impl<G: Graph> MultiCriteriaDijkstra<G> {
// pub fn current_node_path_to(&self, t: NodeId) -> Option<Vec<NodeId>> {
//     return self.data.path(self.s, t);
// }
// }

impl<G> MultiCriteriaDijkstra<G>
where
    G: Graph + OutgoingEdgeIterable,
{
    pub fn new(graph: G, s: NodeId) -> Self {
        Self {
            data: MultiCriteriaDijkstraData::new(graph.num_nodes()),
            s: s,
            graph: graph,
        }
    }
}

impl<FirstOutContainer, HeadContainer, WeightsContainer> Graph for MultiCritFirstOutGraph<FirstOutContainer, HeadContainer, WeightsContainer>
where
    FirstOutContainer: AsRef<[EdgeId]>,
    HeadContainer: AsRef<[NodeId]>,
    WeightsContainer: AsRef<[Weight2]>,
{
    type WeightType = Weight2;

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

impl<FirstOutContainer, HeadContainer, WeightsContainer> OutgoingEdgeIterable for MultiCritFirstOutGraph<FirstOutContainer, HeadContainer, WeightsContainer>
where
    FirstOutContainer: AsRef<[EdgeId]>,
    HeadContainer: AsRef<[NodeId]>,
    WeightsContainer: AsRef<[Weight2]>,
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

impl<G> MultiCriteriaDijkstra<G>
where
    G: Graph + OutgoingEdgeIterable,
{
    pub fn dist_query(&mut self, t: NodeId) -> Vec<G::WeightType> {
        let per_node_labels = &mut self.data.per_node_labels;
        let queue = &mut self.data.queue;
        let n = self.graph.num_nodes();

        per_node_labels[self.s as usize].push(State {
            node: n as NodeId,
            distance: G::WeightType::zero(),
        });
        queue.push(State {
            node: self.s,
            distance: G::WeightType::zero(),
        });

        while let Some(State {
            distance: pareto_dist,
            node: node_id,
        }) = queue.pop()
        {
            // if node_id == t {
            //     return Some(pareto_dist);
            // }

            // with hopping reduction
            for (&edge_weight, &neighbor_node) in self.graph.outgoing_edge_iter(node_id).filter(|s| *s.1 != node_id) {
                {
                    // TODO turns out per_node_labels is a bad idea in rust, .clone() until refactoring
                    let old_label_set = per_node_labels[node_id as usize].clone();

                    for State {
                        distance: node_distance,
                        node: _,
                    } in old_label_set.iter()
                    {
                        let new_dist = node_distance.link(&edge_weight);

                        // target pruning
                        if per_node_labels[t as usize].iter().any(|&s| s.distance.dominates(&new_dist)) {
                            continue;
                        }
                        let neighbor_label_set = &mut per_node_labels[neighbor_node as usize];
                        if !neighbor_label_set.iter().any(|&s| s.distance.dominates(&new_dist)) {
                            neighbor_label_set.retain(|&s| !new_dist.dominates(&s.distance));

                            neighbor_label_set.push(State {
                                distance: new_dist,
                                node: node_id,
                            });

                            if queue.contains_index(neighbor_node as usize) {
                                queue.decrease_key(State {
                                    distance: new_dist,
                                    node: neighbor_node,
                                });
                            } else {
                                queue.push(State {
                                    distance: new_dist,
                                    node: neighbor_node,
                                });
                            }
                        }
                    }
                };
            }
        }

        per_node_labels[t as usize].drain().map(|s| s.distance).collect()
    }
}
