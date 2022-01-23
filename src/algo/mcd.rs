use crate::{index_heap::*, types::*};
use bit_vec::BitVec;
use std::cmp::Reverse;
use std::collections::BinaryHeap;
pub type OwnedOneRestrictionGraph = OneRestrictionFirstOutGraph<Vec<EdgeId>, Vec<NodeId>, Vec<Weight>>;
pub type BorrowedOneRestrictionGraph<'a> = OneRestrictionFirstOutGraph<&'a [EdgeId], &'a [NodeId], &'a [Weight]>;

#[derive(Debug, Clone)]
pub struct OneRestrictionFirstOutGraph<FirstOutContainer, HeadContainer, WeightsContainer> {
    first_out: FirstOutContainer,
    head: HeadContainer,
    weights: WeightsContainer,
}

impl<FirstOutContainer, HeadContainer, WeightsContainer> OneRestrictionFirstOutGraph<FirstOutContainer, HeadContainer, WeightsContainer>
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

    pub fn borrow(&self) -> OneRestrictionFirstOutGraph<&[EdgeId], &[NodeId], &[Weight]> {
        OneRestrictionFirstOutGraph {
            first_out: self.first_out(),
            head: self.head(),
            weights: self.weights(),
        }
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Debug, Ord, PartialOrd)]
pub struct Label<T> {
    pub distance: T,
    pub prev_node: NodeId,
    pub incoming_edge_weight: Weight,
}

pub struct MultiCriteriaDijkstraData<L: WeightOps> {
    pub queue: IndexdMinHeap<State<L>>,
    pub per_node_labels: Vec<BinaryHeap<Reverse<(bool, Label<L>)>>>,
    invalid_node_id: NodeId,
}

impl<L: WeightOps> MultiCriteriaDijkstraData<L> {
    pub fn new(n: usize) -> Self {
        let mut per_node_labels = Vec::with_capacity(n);
        per_node_labels.resize_with(n, || BinaryHeap::new());

        Self {
            queue: IndexdMinHeap::new(n),
            per_node_labels,
            invalid_node_id: n as NodeId,
        }
    }
}

pub struct OneRestrictionDijkstra<'a> {
    data: MultiCriteriaDijkstraData<Weight2>,
    s: NodeId,
    graph: BorrowedOneRestrictionGraph<'a>,
    restriction: DrivingTimeRestriction,
    reset_flags: BitVec,
    pub num_queue_pushes: u32,
    pub num_settled: u32,
    pub num_labels_propagated: u32,
    pub num_labels_reset: u32,
}

impl<'a> OneRestrictionDijkstra<'a> {
    pub fn new(graph: BorrowedOneRestrictionGraph<'a>, s: NodeId) -> Self {
        let n = graph.num_nodes();
        Self {
            data: MultiCriteriaDijkstraData::new(graph.num_nodes()),
            s,
            graph,
            restriction: DrivingTimeRestriction {
                pause_time: 0,
                max_driving_time: Weight::infinity(),
            },
            reset_flags: BitVec::from_elem(n, false),
            num_queue_pushes: 0,
            num_settled: 0,
            num_labels_propagated: 0,
            num_labels_reset: 0,
        }
    }

    pub fn current_best_path_to(&self, t: NodeId, with_distances: bool) -> Option<(Vec<NodeId>, Vec<Weight2>)> {
        let mut path = vec![];
        let mut distances = vec![];
        let mut current_node = t;

        //get best settled or unsettled label
        //max because of Reverse()
        let best_settled_at_t = self.data.per_node_labels[t as usize].iter().filter(|Reverse((settled, _))| *settled).max();
        let best_unsettled_at_t = self.data.per_node_labels[t as usize].iter().filter(|Reverse((settled, _))| !*settled).max();

        let best_label_at_t = match (best_settled_at_t, best_unsettled_at_t) {
            (None, Some(best)) => Some(best),
            (Some(best), None) => Some(best),
            (Some(best_left), Some(best_right)) => {
                if best_left.0 .1.distance < best_right.0 .1.distance {
                    Some(best_left)
                } else {
                    Some(best_right)
                }
            }
            _ => None,
        };

        if let Some(Reverse((_, mut current_label))) = best_label_at_t {
            let mut next_node = current_label.prev_node;
            path.push(current_node);
            if with_distances {
                distances.push(current_label.distance);
            }

            while next_node != self.data.invalid_node_id {
                // search label in current_node's label set which matches distance
                'outer: for Reverse((_, next_label)) in self.data.per_node_labels[next_node as usize].iter().filter(|Reverse((settled, _))| *settled) {
                    let mut dist_candidates = [next_label.distance.link(current_label.incoming_edge_weight); 2];

                    if self.reset_flags.get(current_node as usize).unwrap() {
                        dist_candidates[1].reset(1, self.restriction.pause_time);
                    };

                    for candidate in dist_candidates {
                        if candidate == current_label.distance {
                            current_label = *next_label;
                            current_node = next_node;
                            next_node = current_label.prev_node;

                            path.push(current_node);
                            if with_distances {
                                distances.push(current_label.distance);
                            }
                            break 'outer;
                        }
                    }
                }
            }

            path.reverse();
            distances.reverse();

            if path[0] != self.s {
                None
            } else {
                Some((path, distances))
            }
        } else {
            None
        }
    }

    pub fn current_best_node_path_to(&self, t: NodeId) -> Option<Vec<NodeId>> {
        self.current_best_path_to(t, false).map_or(None, |p| Some(p.0))
    }

    pub fn flagged_nodes_on_path(&self, path: &(Vec<NodeId>, Vec<Weight2>)) -> Vec<NodeId> {
        self.flagged_nodes_on_node_path(&path.0)
    }

    pub fn flagged_nodes_on_node_path(&self, path: &Vec<NodeId>) -> Vec<NodeId> {
        let mut flagged_nodes = Vec::new();
        for &node in path {
            if self.reset_flags.get(node as usize).unwrap() {
                flagged_nodes.push(node)
            }
        }

        flagged_nodes
    }

    pub fn flagged_nodes_on_best_path_to(&self, t: NodeId) -> Option<Vec<NodeId>> {
        if let Some(path) = self.current_best_path_to(t, true) {
            Some(self.flagged_nodes_on_path(&path))
        } else {
            None
        }
    }

    pub fn reset_nodes_on_path(&self, path: &(Vec<NodeId>, Vec<Weight2>)) -> Vec<NodeId> {
        let mut reset_nodes = Vec::new();
        for (node, dist) in path.0.iter().zip(&path.1) {
            if self.reset_flags.get(*node as usize).unwrap() && dist[1] == 0 {
                reset_nodes.push(*node)
            }
        }

        reset_nodes
    }

    pub fn set_restriction(&mut self, max_driving_time: Weight, pause_time: Weight) -> &mut Self {
        self.restriction = DrivingTimeRestriction { pause_time, max_driving_time };
        self
    }

    pub fn set_reset_flags(&mut self, flags: BitVec) -> &mut Self {
        self.reset_flags = flags;
        self
    }
}

impl<FirstOutContainer, HeadContainer, WeightsContainer> Graph for OneRestrictionFirstOutGraph<FirstOutContainer, HeadContainer, WeightsContainer>
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

impl<FirstOutContainer, HeadContainer, WeightsContainer> OutgoingEdgeIterable
    for OneRestrictionFirstOutGraph<FirstOutContainer, HeadContainer, WeightsContainer>
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

impl<'a> OneRestrictionDijkstra<'a> {
    pub fn dist_query(&mut self, t: NodeId) -> Option<Weight> {
        // check if query already finished for t
        if let Some(Reverse(best_tuple_at_t)) = self.data.per_node_labels[t as usize].peek() {
            if best_tuple_at_t.0 {
                return Some(best_tuple_at_t.1.distance[0]);
            }
        }

        let n = self.graph.num_nodes();

        self.data.per_node_labels[self.s as usize].push(Reverse((
            false,
            Label {
                prev_node: n as NodeId,
                distance: [0, 0],
                incoming_edge_weight: Weight::infinity(),
            },
        )));

        self.num_queue_pushes += 1;
        self.data.queue.push(State {
            node: self.s,
            distance: [0, 0],
        });

        while let Some(State {
            distance: tentative_distance,
            node: node_id,
        }) = self.data.queue.pop()
        {
            self.num_settled += 1;

            if node_id == t {
                // push again for eventual next query
                self.data.queue.push(State {
                    distance: tentative_distance,
                    node: node_id,
                });
                return Some(tentative_distance[0]);
            }

            // remove and push again as settled
            let label = self.data.per_node_labels[node_id as usize].pop().unwrap().0 .1;
            self.data.per_node_labels[node_id as usize].push(Reverse((true, label)));

            // check if next unsettled lable exists for node and push to queue
            if let Some(Reverse((settled, next_best_label))) = self.data.per_node_labels[node_id as usize].peek() {
                if !settled {
                    self.data.queue.push(State {
                        distance: next_best_label.distance,
                        node: node_id,
                    });
                }
            }

            // with hopping reduction
            for (&edge_weight, &neighbor_node) in self.graph.outgoing_edge_iter(node_id).filter(|&s| *(s.1) != node_id) {
                {
                    // [new_dist without, new_dist with parking]
                    let mut new_dist_arr = [tentative_distance.link(edge_weight); 2];

                    // constraint and target pruning
                    if new_dist_arr[0][1] > self.restriction.max_driving_time
                        || self.data.per_node_labels[t as usize]
                            .iter()
                            .any(|&s| s.0 .1.distance.dominates(&new_dist_arr[0]))
                    {
                        continue;
                    }

                    if self.reset_flags.get(neighbor_node as usize).unwrap() {
                        new_dist_arr[1].reset(1, self.restriction.pause_time);
                        self.num_labels_reset += 1;
                    }

                    for new_dist in new_dist_arr {
                        let neighbor_label_set = &mut self.data.per_node_labels[neighbor_node as usize];
                        if !neighbor_label_set.iter().any(|&s| s.0 .1.distance.dominates(&new_dist)) {
                            neighbor_label_set.retain(|&s| !new_dist.dominates(&s.0 .1.distance));

                            self.num_labels_propagated += 1;
                            neighbor_label_set.push(Reverse((
                                false,
                                Label {
                                    distance: new_dist,
                                    prev_node: node_id,
                                    incoming_edge_weight: edge_weight,
                                },
                            )));

                            if self.data.queue.contains_index(neighbor_node as usize) {
                                // decrease key seems to increase key if given a larger key than existing
                                if self.data.queue.get_key_by_index(neighbor_node as usize).unwrap().distance > new_dist {
                                    self.data.queue.decrease_key(State {
                                        distance: new_dist,
                                        node: neighbor_node,
                                    });
                                }
                            } else {
                                self.num_queue_pushes += 1;
                                self.data.queue.push(State {
                                    distance: new_dist,
                                    node: neighbor_node,
                                });
                            }
                        }
                    }
                };
            }
        }

        None
    }
}
