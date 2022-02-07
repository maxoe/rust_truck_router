use std::cmp::Reverse;

use crate::{
    algo::astar::{NoPotential, Potential},
    index_heap::*,
    rrr_heap::Heap,
    types::*,
};
use bit_vec::BitVec;

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
    pub per_node_labels: Vec<Heap<Reverse<Label<L>>>>,
    invalid_node_id: NodeId,
}

impl<L: WeightOps> MultiCriteriaDijkstraData<L> {
    pub fn new(n: usize) -> Self {
        let mut per_node_labels = Vec::with_capacity(n);
        per_node_labels.resize_with(n, Heap::new);

        Self {
            queue: IndexdMinHeap::new(n),
            per_node_labels,
            invalid_node_id: n as NodeId,
        }
    }
}

pub struct OneRestrictionDijkstra<'a, P = NoPotential>
where
    P: Potential<Weight>,
{
    data: MultiCriteriaDijkstraData<Weight2>,
    s: NodeId,
    graph: BorrowedOneRestrictionGraph<'a>,
    restriction: DrivingTimeRestriction,
    reset_flags: BitVec,
    potential: P,
    pub num_queue_pushes: u32,
    pub num_settled: u32,
    pub num_labels_propagated: u32,
    pub num_labels_reset: u32,
}
impl<'a> OneRestrictionDijkstra<'a, NoPotential> {
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
            potential: NoPotential {},
            num_queue_pushes: 0,
            num_settled: 0,
            num_labels_propagated: 0,
            num_labels_reset: 0,
        }
    }
}

impl<'a, P> OneRestrictionDijkstra<'a, P>
where
    P: Potential<Weight>,
{
    pub fn reset(&mut self) {
        self.num_settled = 0;
        self.num_labels_propagated = 0;
        self.num_queue_pushes = 0;
        self.num_labels_reset = 0;

        self.data.per_node_labels.fill_with(Heap::new);

        self.data.queue.clear();
        self.num_queue_pushes += 1;
        let pot = self.potential.potential(self.s);
        self.data.queue.push(State {
            node: self.s,
            distance: self.estimated_dist_with_restriction([0, 0], pot),
        });
        self.data.per_node_labels[self.s as usize].push(Reverse(Label {
            prev_node: self.graph.num_nodes() as NodeId,
            distance: [0, 0],
            incoming_edge_weight: Weight::infinity(),
        }));
    }

    pub fn new_with_potential(graph: BorrowedOneRestrictionGraph<'a>, potential: P) -> Self {
        let n = graph.num_nodes();

        Self {
            data: MultiCriteriaDijkstraData::new(graph.num_nodes()),
            s: graph.num_nodes() as NodeId,
            graph,
            restriction: DrivingTimeRestriction {
                pause_time: 0,
                max_driving_time: Weight::infinity(),
            },
            reset_flags: BitVec::from_elem(n, false),
            potential,
            num_queue_pushes: 0,
            num_settled: 0,
            num_labels_propagated: 0,
            num_labels_reset: 0,
        }
    }

    pub fn init_new_s(&mut self, s: NodeId) {
        self.s = s;
        self.reset();
    }

    pub fn current_best_path_to(&self, t: NodeId, with_distances: bool) -> Option<(Vec<NodeId>, Vec<Weight2>)> {
        let mut path = vec![];
        let mut distances = vec![];
        let mut current_node = t;

        //get best settled or unsettled label
        //max because the type is Reverse<Label<..>>
        let best_settled_at_t = self.data.per_node_labels[t as usize].popped().iter().max();
        let best_unsettled_at_t = self.data.per_node_labels[t as usize].iter().max();

        let best_label_at_t = match (best_settled_at_t, best_unsettled_at_t) {
            (None, Some(best)) => Some(best),
            (Some(best), None) => Some(best),
            (Some(best_left), Some(best_right)) => {
                if best_left.0.distance < best_right.0.distance {
                    Some(best_left)
                } else {
                    Some(best_right)
                }
            }
            _ => None,
        };

        if let Some(mut current_label) = best_label_at_t {
            let mut next_node = current_label.0.prev_node;
            path.push(current_node);
            if with_distances {
                distances.push(current_label.0.distance);
            }

            while next_node != self.data.invalid_node_id {
                // search label in current_node's label set which matches distance
                'outer: for next_label in self.data.per_node_labels[next_node as usize].popped() {
                    let mut dist_candidates = [next_label.0.distance.link(current_label.0.incoming_edge_weight); 2];

                    if self.reset_flags.get(current_node as usize).unwrap() {
                        dist_candidates[1].reset(1, self.restriction.pause_time);
                    };

                    for candidate in dist_candidates {
                        if candidate == current_label.0.distance {
                            current_label = next_label;
                            current_node = next_node;
                            next_node = current_label.0.prev_node;

                            path.push(current_node);
                            if with_distances {
                                distances.push(current_label.0.distance);
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
        self.current_best_path_to(t, false).map(|p| p.0)
    }

    pub fn flagged_nodes_on_path(&self, path: &(Vec<NodeId>, Vec<Weight2>)) -> Vec<NodeId> {
        self.flagged_nodes_on_node_path(&path.0)
    }

    pub fn flagged_nodes_on_node_path(&self, path: &[NodeId]) -> Vec<NodeId> {
        let mut flagged_nodes = Vec::new();
        for &node in path {
            if self.reset_flags.get(node as usize).unwrap() {
                flagged_nodes.push(node)
            }
        }

        flagged_nodes
    }

    pub fn flagged_nodes_on_best_path_to(&self, t: NodeId) -> Option<Vec<NodeId>> {
        self.current_best_path_to(t, true).map(|path| self.flagged_nodes_on_path(&path))
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

    pub fn set_reset_flags<B: AsRef<[u8]>>(&mut self, flags: B) -> &mut Self {
        self.reset_flags = BitVec::from_bytes(flags.as_ref());
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

impl<'a, P> OneRestrictionDijkstra<'a, P>
where
    P: Potential<Weight>,
{
    fn estimated_dist_with_restriction(&self, distance_at_node: [Weight; 2], potential_to_target: Weight) -> [Weight; 2] {
        let estimated = distance_at_node.link(potential_to_target);
        [
            estimated[0].link((estimated[1] / self.restriction.max_driving_time) * self.restriction.pause_time),
            estimated[1],
        ]
    }

    pub fn dist_query(&mut self, t: NodeId) -> Option<Weight> {
        self.reset();
        self.potential.init_new_t(t);

        while let Some(State {
            distance: tentative_distance_from_queue,
            node: node_id,
        }) = self.data.queue.pop()
        {
            if node_id == t {
                // push again for eventual next query
                self.data.queue.push(State {
                    distance: tentative_distance_from_queue,
                    node: node_id,
                });
                return self.data.per_node_labels[node_id as usize].peek().map(|label| label.0.distance[0]);
            }

            self.num_settled += 1;

            let label = self.data.per_node_labels[node_id as usize].pop().unwrap();
            let tentative_dist_without_pot = label.0.distance;

            // check if next unsettled lable exists for node and push to queue
            if let Some(next_best_label) = self.data.per_node_labels[node_id as usize].peek() {
                let pot = self.potential.potential(node_id);
                self.data.queue.push(State {
                    distance: self.estimated_dist_with_restriction(next_best_label.0.distance, pot),
                    node: node_id,
                });
            }

            // with hopping reduction
            for (&edge_weight, &neighbor_node) in self.graph.outgoing_edge_iter(node_id).filter(|&s| *(s.1) != node_id) {
                {
                    // [new_dist without, new_dist with parking]
                    let mut new_dist = Vec::with_capacity(2);
                    new_dist.push(tentative_dist_without_pot.link(edge_weight));

                    // constraint and target pruning
                    if new_dist[0][1] >= self.restriction.max_driving_time
                        || self.data.per_node_labels[t as usize].iter().any(|&s| s.0.distance.dominates(&new_dist[0]))
                    {
                        continue;
                    }

                    if self.reset_flags.get(neighbor_node as usize).unwrap() {
                        new_dist.push(new_dist[0]);
                        new_dist[1].reset(1, self.restriction.pause_time);
                        self.num_labels_reset += 1;
                    }

                    for current_new_dist in new_dist {
                        let neighbor_label_set = &mut self.data.per_node_labels[neighbor_node as usize];
                        let mut dominated = false;
                        neighbor_label_set.retain(|&neighbor_label| {
                            dominated |= neighbor_label.0.distance.dominates(&current_new_dist);
                            dominated || !current_new_dist.dominates(&neighbor_label.0.distance)
                        });

                        if !dominated {
                            self.num_labels_propagated += 1;
                            neighbor_label_set.push(Reverse(Label {
                                distance: current_new_dist,
                                prev_node: node_id,
                                incoming_edge_weight: edge_weight,
                            }));

                            let pot = self.potential.potential(neighbor_node);
                            let dist_with_potential = self.estimated_dist_with_restriction(current_new_dist, pot);
                            if self.data.queue.contains_index(neighbor_node as usize) {
                                // decrease key seems to increase key if given a larger key than existing
                                if self.data.queue.get_key_by_index(neighbor_node as usize).unwrap().distance > dist_with_potential {
                                    self.data.queue.decrease_key(State {
                                        distance: dist_with_potential,
                                        node: neighbor_node,
                                    });
                                }
                            } else {
                                self.num_queue_pushes += 1;
                                self.data.queue.push(State {
                                    distance: dist_with_potential,
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

    pub fn dist_query_propagate_all_labels(&mut self, t: NodeId) -> Option<Weight> {
        self.reset();
        self.potential.init_new_t(t);

        while let Some(State {
            distance: tentative_distance_from_queue,
            node: node_id,
        }) = self.data.queue.pop()
        {
            if node_id == t {
                // push again for eventual next query
                self.data.queue.push(State {
                    distance: tentative_distance_from_queue,
                    node: node_id,
                });
                return self.data.per_node_labels[node_id as usize].peek().map(|label| label.0.distance[0]);
            }

            self.num_settled += 1;

            while let Some(label) = self.data.per_node_labels[node_id as usize].pop() {
                let tentative_dist_without_pot = label.0.distance;

                // check if next unsettled lable exists for node and push to queue
                // if let Some(next_best_label) = self.data.per_node_labels[node_id as usize].peek() {
                //     let pot = self.potential.potential(node_id);
                //     self.data.queue.push(State {
                //         distance: self.estimated_dist_with_restriction(next_best_label.distance, pot),
                //         node: node_id,
                //     });
                // }

                // with hopping reduction
                for (&edge_weight, &neighbor_node) in self.graph.outgoing_edge_iter(node_id).filter(|&s| *(s.1) != node_id) {
                    {
                        // [new_dist without, new_dist with parking]
                        let mut new_dist = Vec::with_capacity(2);
                        new_dist.push(tentative_dist_without_pot.link(edge_weight));

                        // constraint and target pruning
                        if new_dist[0][1] >= self.restriction.max_driving_time
                            || self.data.per_node_labels[t as usize].iter().any(|&s| s.0.distance.dominates(&new_dist[0]))
                        {
                            continue;
                        }

                        if self.reset_flags.get(neighbor_node as usize).unwrap() {
                            new_dist.push(new_dist[0]);
                            new_dist[1].reset(1, self.restriction.pause_time);
                            self.num_labels_reset += 1;
                        }

                        for current_new_dist in new_dist {
                            let neighbor_label_set = &mut self.data.per_node_labels[neighbor_node as usize];
                            let mut dominated = false;
                            neighbor_label_set.retain(|&neighbor_label| {
                                dominated |= neighbor_label.0.distance.dominates(&current_new_dist);
                                dominated || !current_new_dist.dominates(&neighbor_label.0.distance)
                            });

                            if !dominated {
                                self.num_labels_propagated += 1;
                                neighbor_label_set.push(Reverse(Label {
                                    distance: current_new_dist,
                                    prev_node: node_id,
                                    incoming_edge_weight: edge_weight,
                                }));

                                let pot = self.potential.potential(neighbor_node);
                                let dist_with_potential = self.estimated_dist_with_restriction(current_new_dist, pot);
                                if self.data.queue.contains_index(neighbor_node as usize) {
                                    // decrease key seems to increase key if given a larger key than existing
                                    if self.data.queue.get_key_by_index(neighbor_node as usize).unwrap().distance > dist_with_potential {
                                        self.data.queue.decrease_key(State {
                                            distance: dist_with_potential,
                                            node: neighbor_node,
                                        });
                                    }
                                } else {
                                    self.num_queue_pushes += 1;
                                    self.data.queue.push(State {
                                        distance: dist_with_potential,
                                        node: neighbor_node,
                                    });
                                }
                            }
                        }
                    };
                }
            }
        }

        None
    }
}
