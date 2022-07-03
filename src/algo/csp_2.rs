use crate::{
    algo::astar::{NoPotential, Potential},
    index_heap::*,
    timestamped_vector::TimestampedVector,
    types::*,
};
use bit_vec::BitVec;
use num::Integer;
use std::{
    cmp::Reverse,
    fmt::Write,
    time::{Duration, Instant},
};

pub struct TwoRestrictionDijkstraData<P = NoPotential>
where
    P: Potential,
{
    pub queue: IndexdMinHeap<State<Weight3>>,
    pub per_node_labels: TimestampedVector<MCDHeap<Weight3>, u16>,
    invalid_node_id: NodeId,
    s: NodeId,
    restriction_short: DrivingTimeRestriction,
    restriction_long: DrivingTimeRestriction,
    pub potential: P,
    pub num_queue_pushes: u32,
    pub num_settled: u32,
    pub num_labels_propagated: u32,
    pub num_labels_reset: u32,
    pub time_elapsed: Duration,
    pub last_t: NodeId,
    pub last_distance: Option<Weight>,
}
impl TwoRestrictionDijkstraData<NoPotential> {
    pub fn new(num_nodes: usize) -> Self {
        Self {
            queue: IndexdMinHeap::new(num_nodes),
            per_node_labels: TimestampedVector::with_size(num_nodes),
            invalid_node_id: num_nodes as NodeId,
            s: num_nodes as NodeId,
            restriction_short: DrivingTimeRestriction {
                pause_time: 0,
                max_driving_time: Weight::infinity(),
            },
            restriction_long: DrivingTimeRestriction {
                pause_time: 0,
                max_driving_time: Weight::infinity(),
            },
            potential: NoPotential {},
            num_queue_pushes: 0,
            num_settled: 0,
            num_labels_propagated: 0,
            num_labels_reset: 0,
            time_elapsed: Duration::ZERO,
            last_t: num_nodes as NodeId,
            last_distance: None,
        }
    }
}

impl<P> TwoRestrictionDijkstraData<P>
where
    P: Potential,
{
    pub fn reset(&mut self) {
        if self.s != self.invalid_node_id {
            self.num_settled = 0;
            self.num_labels_propagated = 0;
            self.num_queue_pushes = 0;
            self.num_labels_reset = 0;

            self.per_node_labels.reset();

            self.queue.clear();
            self.num_queue_pushes += 1;
            let pot = self.potential.potential(self.s);
            self.queue.push(State {
                node: self.s,
                distance: self.estimated_dist_with_restriction([0, 0, 0], pot),
            });
            let pot = self.potential.potential(self.s);
            let distance_with_potential = self.estimated_dist_with_restriction([0, 0, 0], pot);
            self.per_node_labels.get_mut(self.s as usize).push(Reverse(Label {
                distance_with_potential,
                distance: [0, 0, 0],
                prev_node: self.invalid_node_id,
                prev_label: None,
            }));
        }
    }

    pub fn clean(&mut self) {
        self.per_node_labels.clean();
        self.reset();
    }

    pub fn new_with_potential(num_nodes: usize, potential: P) -> Self {
        Self {
            queue: IndexdMinHeap::new(num_nodes),
            per_node_labels: TimestampedVector::with_size(num_nodes),
            invalid_node_id: num_nodes as NodeId,
            s: num_nodes as NodeId,
            restriction_short: DrivingTimeRestriction {
                pause_time: 0,
                max_driving_time: Weight::infinity(),
            },
            restriction_long: DrivingTimeRestriction {
                pause_time: 0,
                max_driving_time: Weight::infinity(),
            },
            potential,
            num_queue_pushes: 0,
            num_settled: 0,
            num_labels_propagated: 0,
            num_labels_reset: 0,
            time_elapsed: Duration::ZERO,
            last_t: num_nodes as NodeId,
            last_distance: None,
        }
    }

    pub fn init_new_s(&mut self, s: NodeId) {
        self.s = s;
        self.reset();
    }

    pub fn min_key(&self) -> Option<Weight> {
        self.queue.peek().map(|s| s.distance[0])
    }

    pub fn current_best_path_to(&self, t: NodeId, with_distances: bool) -> Option<(Vec<NodeId>, Vec<Weight3>)> {
        let mut path = vec![];
        let mut distances = vec![];
        let mut current_node = t;

        //get best settled or unsettled label
        //max because the type is Reverse<Label<..>>
        let best_settled_at_t = self.per_node_labels.get(t as usize).popped().max();
        let best_unsettled_at_t = self.per_node_labels.get(t as usize).iter().max();

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

            while next_node != self.invalid_node_id {
                current_label = self
                    .per_node_labels
                    .get(next_node as usize)
                    .get_key_by_index(current_label.0.prev_label.unwrap())
                    .unwrap(); //next_label;
                current_node = next_node;
                next_node = current_label.0.prev_node;

                path.push(current_node);
                if with_distances {
                    distances.push(current_label.0.distance);
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

    pub fn set_restriction(
        &mut self,
        max_driving_time_long: Weight,
        pause_time_long: Weight,
        max_driving_time_short: Weight,
        pause_time_short: Weight,
    ) -> &mut Self {
        assert!(max_driving_time_long >= max_driving_time_short);
        assert!(pause_time_long >= pause_time_short);
        self.restriction_short = DrivingTimeRestriction {
            pause_time: pause_time_short,
            max_driving_time: max_driving_time_short,
        };
        self.restriction_long = DrivingTimeRestriction {
            pause_time: pause_time_long,
            max_driving_time: max_driving_time_long,
        };
        self
    }

    pub fn clear_restriction(&mut self) -> &mut Self {
        self.restriction_short = DrivingTimeRestriction {
            pause_time: 0,
            max_driving_time: Weight::infinity(),
        };
        self.restriction_long = DrivingTimeRestriction {
            pause_time: 0,
            max_driving_time: Weight::infinity(),
        };

        self.reset();
        self
    }

    pub fn get_settled_labels_at(&mut self, node: NodeId) -> impl DoubleEndedIterator<Item = &Reverse<Label<Weight3>>> + '_ {
        self.per_node_labels.get_mut(node as usize).popped_sorted()
    }

    pub fn get_best_label_at(&self, node: NodeId) -> Option<Label<Weight3>> {
        let best_settled = self.per_node_labels.get(node as usize).popped().max();
        let best_unsettled = self.per_node_labels.get(node as usize).iter().max();

        match (best_settled, best_unsettled) {
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
        }
        .map(|r| r.0)
    }

    pub fn get_tentative_dist_at(&self, node: NodeId) -> Weight3 {
        self.get_best_label_at(node).map_or(Weight3::infinity(), |l| l.distance)
    }

    pub fn peek_queue(&self) -> Option<&State<Weight3>> {
        self.queue.peek()
    }

    pub fn get_per_node_number_of_labels(&self) -> Vec<usize> {
        (0..self.per_node_labels.len())
            .map(|i| {
                let h = self.per_node_labels.get(i);
                h.iter().count() + h.popped().count()
            })
            .collect()
    }

    pub fn get_number_of_visited_nodes(&self) -> usize {
        (0..self.per_node_labels.len())
            .filter(|i| self.per_node_labels.get(*i).popped().count() != 0)
            .count()
    }

    fn estimated_dist_with_restriction(&self, distance_at_node: [Weight; 3], potential_to_target: Weight) -> [Weight; 3] {
        if potential_to_target == Weight::infinity() {
            Weight3::infinity()
        } else {
            let estimated = distance_at_node.link(potential_to_target);

            if self.restriction_short.max_driving_time == Weight::infinity() {
                estimated
            } else {
                let amount_long_breaks = estimated[2] / self.restriction_long.max_driving_time;
                let amount_short_breaks = (estimated[1] / self.restriction_short.max_driving_time)
                    .checked_sub(amount_long_breaks)
                    .unwrap_or(0);

                [
                    estimated[0]
                        .link(amount_short_breaks * self.restriction_short.pause_time)
                        .link(amount_long_breaks * self.restriction_long.pause_time),
                    estimated[1],
                    estimated[2],
                ]
            }
        }
    }
}

pub struct TwoRestrictionDijkstra<'a> {
    graph: BorrowedGraph<'a>,
    reset_flags: &'a BitVec,
}

impl<'a> TwoRestrictionDijkstra<'a> {
    pub fn new(graph: BorrowedGraph<'a>, reset_flags: &'a BitVec) -> Self {
        Self { graph, reset_flags }
    }

    pub fn settle_next_label<P: Potential>(&self, state: &mut TwoRestrictionDijkstraData<P>, t: NodeId) -> Option<State<Weight3>> {
        let next = state.queue.pop();

        if let Some(State {
            distance: _tentative_distance_from_queue,
            node: node_id,
        }) = next
        {
            state.num_settled += 1;

            if node_id == t {
                state.last_distance = state.per_node_labels.get(node_id as usize).peek().map(|label| label.0.distance[0]);
            }

            let label_index = state.per_node_labels.get_mut(node_id as usize).peek_index().unwrap();
            let tentative_dist_without_pot = state.per_node_labels.get_mut(node_id as usize).pop().unwrap().0.distance;
            // let mut dist_list = vec![tentative_dist_without_pot];

            // add all labels with equal dist[0] to list since those may be in invalid order
            // while let Some(next_best_label) = state.per_node_labels.get_mut(node_id as usize).pop() {
            //     if next_best_label.0.distance[0] == tentative_dist_without_pot[0] {
            //         dist_list.push(next_best_label.0.distance);
            //     } else {
            //         break;
            //     }
            // }

            // push next best to queue for later query
            if let Some(next_best_label) = state.per_node_labels.get(node_id as usize).peek() {
                let pot = state.potential.potential(node_id);
                state.queue.push(State {
                    distance: state.estimated_dist_with_restriction(next_best_label.0.distance, pot),
                    node: node_id,
                });
            }

            // with hopping reduction
            // for current_tent_dist in dist_list {
            for (&edge_weight, &neighbor_node) in self.graph.outgoing_edge_iter(node_id).filter(|&s| *(s.1) != node_id) {
                // [new_dist without, new_dist with parking]
                let mut new_dist = Vec::with_capacity(3); // for current_tent_dist in dist_list {
                                                          // new_dist.push(current_tent_dist.link(edge_weight));
                new_dist.push(tentative_dist_without_pot.link(edge_weight));

                // constraint and target pruning
                if new_dist[0][1] >= state.restriction_short.max_driving_time
                    || new_dist[0][2] >= state.restriction_long.max_driving_time
                    || state.per_node_labels.get(t as usize).iter().any(|&s| s.0.distance.dominates(&new_dist[0]))
                {
                    continue;
                }

                if self.reset_flags.get(neighbor_node as usize).unwrap() {
                    new_dist.push(new_dist[0]);
                    new_dist[1].reset_distance(1, state.restriction_short.pause_time);

                    new_dist.push(new_dist[0]);
                    new_dist[2].reset_distance(2, state.restriction_long.pause_time);
                    state.num_labels_reset += 1;
                }

                for current_new_dist in new_dist {
                    let pot = state.potential.potential(neighbor_node);
                    let distance_with_potential = state.estimated_dist_with_restriction(current_new_dist, pot);

                    if distance_with_potential[0] == Weight::infinity() {
                        continue;
                    }

                    let neighbor_label_set = state.per_node_labels.get_mut(neighbor_node as usize);
                    let mut dominated = false;
                    neighbor_label_set.retain(|&neighbor_label| {
                        dominated |= neighbor_label.0.distance.dominates(&current_new_dist);
                        dominated || !current_new_dist.dominates(&neighbor_label.0.distance)
                    });

                    if !dominated {
                        state.num_labels_propagated += 1;
                        neighbor_label_set.push(Reverse(Label {
                            distance_with_potential,
                            distance: current_new_dist,
                            prev_label: Some(label_index),
                            prev_node: node_id,
                        }));

                        let pot = state.potential.potential(neighbor_node);
                        let dist_with_potential = state.estimated_dist_with_restriction(current_new_dist, pot);

                        if state.queue.contains_index(neighbor_node as usize) {
                            // decrease key seems to increase key if given a larger key than existing
                            if state.queue.get_key_by_index(neighbor_node as usize).unwrap().distance > dist_with_potential {
                                state.queue.decrease_key(State {
                                    distance: dist_with_potential,
                                    node: neighbor_node,
                                });
                            }
                        } else {
                            state.num_queue_pushes += 1;
                            state.queue.push(State {
                                distance: dist_with_potential,
                                node: neighbor_node,
                            });
                        }
                    }
                }
            }
            // }
        } else {
            state.last_distance = None;
        }

        next
    }

    pub fn settle_next_label_report_pushed_non_core_nodes<P: Potential>(
        &self,
        state: &mut TwoRestrictionDijkstraData<P>,
        is_core: &BitVec,
        non_core_count: &mut u32,
        t: NodeId,
    ) -> Option<State<Weight3>> {
        let next = state.queue.pop();

        if let Some(State {
            distance: _tentative_distance_from_queue,
            node: node_id,
        }) = next
        {
            state.num_settled += 1;

            if !is_core.get(node_id as usize).unwrap() {
                *non_core_count -= 1;
            }

            if node_id == t {
                state.last_distance = state.per_node_labels.get(node_id as usize).peek().map(|label| label.0.distance[0]);
            }

            let label_index = state.per_node_labels.get_mut(node_id as usize).peek_index().unwrap();
            let tentative_dist_without_pot = state.per_node_labels.get_mut(node_id as usize).pop().unwrap().0.distance;
            // let mut dist_list = vec![tentative_dist_without_pot];

            // add all labels with equal dist[0] to list since those may be in invalid order
            // while let Some(next_best_label) = state.per_node_labels.get_mut(node_id as usize).pop() {
            //     if next_best_label.0.distance[0] == tentative_dist_without_pot[0] {
            //         dist_list.push(next_best_label.0.distance);
            //     } else {
            //         break;
            //     }
            // }

            // push next best to queue for later query
            if let Some(next_best_label) = state.per_node_labels.get(node_id as usize).peek() {
                let pot = state.potential.potential(node_id);
                state.queue.push(State {
                    distance: state.estimated_dist_with_restriction(next_best_label.0.distance, pot),
                    node: node_id,
                });

                if !is_core.get(node_id as usize).unwrap() {
                    *non_core_count += 1;
                }
            }

            // with hopping reduction
            // for current_tent_dist in dist_list {
            for (&edge_weight, &neighbor_node) in self.graph.outgoing_edge_iter(node_id).filter(|&s| *(s.1) != node_id) {
                // [new_dist without, new_dist with parking]
                let mut new_dist = Vec::with_capacity(3); // for current_tent_dist in dist_list {
                                                          // new_dist.push(current_tent_dist.link(edge_weight));
                new_dist.push(tentative_dist_without_pot.link(edge_weight));

                // constraint and target pruning
                if new_dist[0][1] >= state.restriction_short.max_driving_time
                    || new_dist[0][2] >= state.restriction_long.max_driving_time
                    || state.per_node_labels.get(t as usize).iter().any(|&s| s.0.distance.dominates(&new_dist[0]))
                {
                    continue;
                }

                if self.reset_flags.get(neighbor_node as usize).unwrap() {
                    new_dist.push(new_dist[0]);
                    new_dist[1].reset_distance(1, state.restriction_short.pause_time);

                    new_dist.push(new_dist[0]);
                    new_dist[2].reset_distance(2, state.restriction_long.pause_time);
                    state.num_labels_reset += 1;
                }

                for current_new_dist in new_dist {
                    let pot = state.potential.potential(neighbor_node);
                    let distance_with_potential = state.estimated_dist_with_restriction(current_new_dist, pot);

                    if distance_with_potential[0] == Weight::infinity() {
                        continue;
                    }
                    let neighbor_label_set = state.per_node_labels.get_mut(neighbor_node as usize);
                    let mut dominated = false;
                    neighbor_label_set.retain(|&neighbor_label| {
                        dominated |= neighbor_label.0.distance.dominates(&current_new_dist);
                        dominated || !current_new_dist.dominates(&neighbor_label.0.distance)
                    });

                    if !dominated {
                        state.num_labels_propagated += 1;
                        neighbor_label_set.push(Reverse(Label {
                            distance_with_potential,
                            distance: current_new_dist,
                            prev_label: Some(label_index),
                            prev_node: node_id,
                        }));

                        let pot = state.potential.potential(neighbor_node);
                        let dist_with_potential = state.estimated_dist_with_restriction(current_new_dist, pot);

                        if state.queue.contains_index(neighbor_node as usize) {
                            // decrease key seems to increase key if given a larger key than existing
                            if state.queue.get_key_by_index(neighbor_node as usize).unwrap().distance > dist_with_potential {
                                state.queue.decrease_key(State {
                                    distance: dist_with_potential,
                                    node: neighbor_node,
                                });
                            }
                        } else {
                            state.num_queue_pushes += 1;
                            state.queue.push(State {
                                distance: dist_with_potential,
                                node: neighbor_node,
                            });

                            if !is_core.get(neighbor_node as usize).unwrap() {
                                *non_core_count += 1;
                            }
                        }
                    }
                }
            }
            // }
        } else {
            state.last_distance = None;
        }

        next
    }

    pub fn dist_query<P: Potential>(&self, state: &mut TwoRestrictionDijkstraData<P>, t: NodeId) -> Option<Weight> {
        let start = Instant::now();
        state.reset();
        state.potential.init_new_t(t);
        state.last_t = t;

        while let Some(State { distance: _, node: node_id }) = self.settle_next_label(state, t) {
            if node_id == t {
                state.time_elapsed = start.elapsed();
                return state.last_distance;
            }
        }
        state.time_elapsed = start.elapsed();
        None
    }

    pub fn timeout_dist_query<P: Potential>(
        &self,
        state: &mut TwoRestrictionDijkstraData<P>,
        t: NodeId,
        timeout: Duration,
    ) -> Result<Option<Weight>, QueryTimeoutError> {
        let start = Instant::now();
        state.reset();
        state.potential.init_new_t(t);
        state.last_t = t;

        while let Some(State { distance: _, node: node_id }) = self.settle_next_label(state, t) {
            if node_id == t {
                state.time_elapsed = start.elapsed();
                return Ok(state.last_distance);
            }

            if start.elapsed() > timeout {
                return Err(QueryTimeoutError);
            };
        }
        state.time_elapsed = start.elapsed();
        Ok(None)
    }

    pub fn dist_query_propagate_all_labels<P: Potential>(&self, state: &mut TwoRestrictionDijkstraData<P>, t: NodeId) -> Option<Weight> {
        let start = Instant::now();
        state.reset();
        state.potential.init_new_t(t);
        state.last_t = t;

        while let Some(State { distance: _, node: node_id }) = self.settle_next_label_propagate_all(state, t) {
            if node_id == t {
                state.time_elapsed = start.elapsed();
                return state.last_distance;
            }
        }
        state.time_elapsed = start.elapsed();
        None
    }

    pub fn settle_next_label_propagate_all<P: Potential>(&self, state: &mut TwoRestrictionDijkstraData<P>, t: NodeId) -> Option<State<Weight3>> {
        let next = state.queue.pop();

        if let Some(State {
            distance: _tentative_distance_from_queue,
            node: node_id,
        }) = next
        {
            state.num_settled += 1;

            if node_id == t {
                state.last_distance = state.per_node_labels.get(node_id as usize).peek().map(|label| label.0.distance[0]);
            }

            while let Some(label_index) = state.per_node_labels.get_mut(node_id as usize).peek_index() {
                let label = state.per_node_labels.get_mut(node_id as usize).pop().unwrap();
                let tentative_dist_without_pot = label.0.distance;

                // with hopping reduction
                for (&edge_weight, &neighbor_node) in self.graph.outgoing_edge_iter(node_id).filter(|&s| *(s.1) != node_id) {
                    {
                        // [new_dist without, new_dist with parking]
                        let mut new_dist = Vec::with_capacity(2);
                        new_dist.push(tentative_dist_without_pot.link(edge_weight));

                        // constraint and target pruning
                        if new_dist[0][1] >= state.restriction_short.max_driving_time
                            || new_dist[0][2] >= state.restriction_long.max_driving_time
                            || state.per_node_labels.get(t as usize).iter().any(|&s| s.0.distance.dominates(&new_dist[0]))
                        {
                            continue;
                        }

                        if self.reset_flags.get(neighbor_node as usize).unwrap() {
                            new_dist.push(new_dist[0]);
                            new_dist[1].reset_distance(1, state.restriction_short.pause_time);

                            new_dist.push(new_dist[0]);
                            new_dist[2].reset_distance(2, state.restriction_long.pause_time);
                            state.num_labels_reset += 1;
                        }

                        for current_new_dist in new_dist {
                            let pot = state.potential.potential(neighbor_node);
                            let distance_with_potential = state.estimated_dist_with_restriction(current_new_dist, pot);

                            if distance_with_potential[0] == Weight::infinity() {
                                continue;
                            }
                            let neighbor_label_set = state.per_node_labels.get_mut(neighbor_node as usize);
                            let mut dominated = false;
                            neighbor_label_set.retain(|&neighbor_label| {
                                dominated |= neighbor_label.0.distance.dominates(&current_new_dist);
                                dominated || !current_new_dist.dominates(&neighbor_label.0.distance)
                            });

                            if !dominated {
                                state.num_labels_propagated += 1;
                                neighbor_label_set.push(Reverse(Label {
                                    distance_with_potential,
                                    distance: current_new_dist,
                                    prev_label: Some(label_index),
                                    prev_node: node_id,
                                }));

                                let pot = state.potential.potential(neighbor_node);
                                let dist_with_potential = state.estimated_dist_with_restriction(current_new_dist, pot);
                                if state.queue.contains_index(neighbor_node as usize) {
                                    // decrease key seems to increase key if given a larger key than existing
                                    if state.queue.get_key_by_index(neighbor_node as usize).unwrap().distance > dist_with_potential {
                                        state.queue.decrease_key(State {
                                            distance: dist_with_potential,
                                            node: neighbor_node,
                                        });
                                    }
                                } else {
                                    state.num_queue_pushes += 1;
                                    state.queue.push(State {
                                        distance: dist_with_potential,
                                        node: neighbor_node,
                                    });
                                }
                            }
                        }
                    };
                }
            }
        } else {
            state.last_distance = None;
        }

        next
    }

    pub fn settle_next_label_prune_bw_lower_bound<P: Potential>(
        &self,
        state: &mut TwoRestrictionDijkstraData<P>,
        bw_state: &mut TwoRestrictionDijkstraData<P>,
        tentative_distance: Weight,
        t: NodeId,
    ) -> Option<State<Weight3>> {
        let next = state.queue.pop();

        if let Some(State {
            distance: _tentative_distance_from_queue,
            node: node_id,
        }) = next
        {
            state.num_settled += 1;

            if node_id == t {
                state.last_distance = state.per_node_labels.get(node_id as usize).peek().map(|label| label.0.distance[0]);
            }

            let label_index = state.per_node_labels.get_mut(node_id as usize).peek_index().unwrap();
            // let label = state.per_node_labels.get_mut(node_id as usize).pop().unwrap();
            let tentative_dist_without_pot = state.per_node_labels.get_mut(node_id as usize).pop().unwrap().0.distance;
            // let mut dist_list = vec![tentative_dist_without_pot];

            // add all labels with equal dist[0] to list since those may be in invalid order
            // while let Some(next_best_label) = state.per_node_labels.get_mut(node_id as usize).pop() {
            //     if next_best_label.0.distance[0] == tentative_dist_without_pot[0] {
            //         dist_list.push(next_best_label.0.distance);
            //     } else {
            //         break;
            //     }
            // }

            // check if next unsettled lable exists for node and push to queue
            if let Some(next_best_label) = state.per_node_labels.get(node_id as usize).peek() {
                let pot = state.potential.potential(node_id);
                state.queue.push(State {
                    distance: state.estimated_dist_with_restriction(next_best_label.0.distance, pot),
                    node: node_id,
                });
            }
            // for current_tent_dist in dist_list {
            // with hopping reduction
            for (&edge_weight, &neighbor_node) in self.graph.outgoing_edge_iter(node_id).filter(|&s| *(s.1) != node_id) {
                // [new_dist without, new_dist with parking]
                let mut new_dist = Vec::with_capacity(3);
                new_dist.push(tentative_dist_without_pot.link(edge_weight));
                // new_dist.push(current_tent_dist.link(edge_weight));

                // constraint and target pruning
                if new_dist[0][1] >= state.restriction_short.max_driving_time
                    || new_dist[0][2] >= state.restriction_long.max_driving_time
                    || state.per_node_labels.get(t as usize).iter().any(|&s| s.0.distance.dominates(&new_dist[0]))
                {
                    continue;
                }

                if self.reset_flags.get(neighbor_node as usize).unwrap() {
                    new_dist.push(new_dist[0]);
                    new_dist[1].reset_distance(1, state.restriction_short.pause_time);

                    new_dist.push(new_dist[0]);
                    new_dist[2].reset_distance(2, state.restriction_long.pause_time);
                    state.num_labels_reset += 1;
                }

                for current_new_dist in new_dist {
                    let pot = state.potential.potential(neighbor_node);
                    let distance_with_potential = state.estimated_dist_with_restriction(current_new_dist, pot);

                    if distance_with_potential[0] == Weight::infinity() {
                        continue;
                    }
                    // pruning with bw lower bound
                    let best_label = bw_state.get_settled_labels_at(neighbor_node).max();
                    if let Some(Reverse(Label {
                        distance: best_label_distance, ..
                    })) = best_label
                    {
                        // best settled label as lower bound for D(neighbor_node,t)
                        if current_new_dist[0] + best_label_distance[0] >= tentative_distance {
                            continue;
                        }
                    } else if bw_state.queue.contains_index(neighbor_node as usize) {
                        // bw_min_key - bw_pot(neighbor_node) as lower bound for D(neighbor_node,t)
                        let bw_pot_at_neighbor =
                            bw_state.get_best_label_at(neighbor_node).unwrap().distance_with_potential[0] - bw_state.get_tentative_dist_at(neighbor_node)[0];
                        let bw_min_key = bw_state.peek_queue().map(|s| s.distance).unwrap();
                        if current_new_dist[0] + bw_min_key[0] - bw_pot_at_neighbor >= tentative_distance {
                            continue;
                        }
                    }

                    let neighbor_label_set = state.per_node_labels.get_mut(neighbor_node as usize);
                    let mut dominated = false;
                    neighbor_label_set.retain(|&neighbor_label| {
                        dominated |= neighbor_label.0.distance.dominates(&current_new_dist);
                        dominated || !current_new_dist.dominates(&neighbor_label.0.distance)
                    });

                    if !dominated {
                        state.num_labels_propagated += 1;
                        neighbor_label_set.push(Reverse(Label {
                            distance_with_potential,
                            distance: current_new_dist,
                            prev_label: Some(label_index),
                            prev_node: node_id,
                        }));

                        let pot = state.potential.potential(neighbor_node);
                        let dist_with_potential = state.estimated_dist_with_restriction(current_new_dist, pot);

                        if state.queue.contains_index(neighbor_node as usize) {
                            // decrease key seems to increase key if given a larger key than existing
                            if state.queue.get_key_by_index(neighbor_node as usize).unwrap().distance > dist_with_potential {
                                state.queue.decrease_key(State {
                                    distance: dist_with_potential,
                                    node: neighbor_node,
                                });
                            }
                        } else {
                            state.num_queue_pushes += 1;
                            state.queue.push(State {
                                distance: dist_with_potential,
                                node: neighbor_node,
                            });
                        }
                    }
                }
                // }
            }
        } else {
            state.last_distance = None;
        }

        next
    }

    pub fn settle_next_label_prune_bw_lower_bound_report_pushed_non_core_nodes<P: Potential>(
        &self,
        state: &mut TwoRestrictionDijkstraData<P>,
        bw_state: &mut TwoRestrictionDijkstraData<P>,
        tentative_distance: Weight,
        is_core: &BitVec,
        non_core_count: &mut u32,
        t: NodeId,
    ) -> Option<State<Weight3>> {
        let next = state.queue.pop();

        if let Some(State {
            distance: _tentative_distance_from_queue,
            node: node_id,
        }) = next
        {
            state.num_settled += 1;

            if !is_core.get(node_id as usize).unwrap() {
                *non_core_count -= 1;
            }

            if node_id == t {
                state.last_distance = state.per_node_labels.get(node_id as usize).peek().map(|label| label.0.distance[0]);
            }

            let label_index = state.per_node_labels.get_mut(node_id as usize).peek_index().unwrap();
            // let label = state.per_node_labels.get_mut(node_id as usize).pop().unwrap();
            let tentative_dist_without_pot = state.per_node_labels.get_mut(node_id as usize).pop().unwrap().0.distance;
            // let mut dist_list = vec![tentative_dist_without_pot];

            // add all labels with equal dist[0] to list since those may be in invalid order
            // while let Some(next_best_label) = state.per_node_labels.get_mut(node_id as usize).pop() {
            //     if next_best_label.0.distance[0] == tentative_dist_without_pot[0] {
            //         dist_list.push(next_best_label.0.distance);
            //     } else {
            //         break;
            //     }
            // }

            // check if next unsettled lable exists for node and push to queue
            if let Some(next_best_label) = state.per_node_labels.get(node_id as usize).peek() {
                let pot = state.potential.potential(node_id);
                state.queue.push(State {
                    distance: state.estimated_dist_with_restriction(next_best_label.0.distance, pot),
                    node: node_id,
                });

                if !is_core.get(node_id as usize).unwrap() {
                    *non_core_count += 1;
                }
            }
            // for current_tent_dist in dist_list {
            // with hopping reduction
            for (&edge_weight, &neighbor_node) in self.graph.outgoing_edge_iter(node_id).filter(|&s| *(s.1) != node_id) {
                // [new_dist without, new_dist with parking]
                let mut new_dist = Vec::with_capacity(3);
                new_dist.push(tentative_dist_without_pot.link(edge_weight));
                // new_dist.push(current_tent_dist.link(edge_weight));

                // constraint and target pruning
                if new_dist[0][1] >= state.restriction_short.max_driving_time
                    || new_dist[0][2] >= state.restriction_long.max_driving_time
                    || state.per_node_labels.get(t as usize).iter().any(|&s| s.0.distance.dominates(&new_dist[0]))
                {
                    continue;
                }

                if self.reset_flags.get(neighbor_node as usize).unwrap() {
                    new_dist.push(new_dist[0]);
                    new_dist[1].reset_distance(1, state.restriction_short.pause_time);

                    new_dist.push(new_dist[0]);
                    new_dist[2].reset_distance(2, state.restriction_long.pause_time);
                    state.num_labels_reset += 1;
                }

                for current_new_dist in new_dist {
                    let pot = state.potential.potential(neighbor_node);
                    let distance_with_potential = state.estimated_dist_with_restriction(current_new_dist, pot);

                    if distance_with_potential[0] == Weight::infinity() {
                        continue;
                    }
                    // pruning with bw lower bound
                    if is_core.get(neighbor_node as usize).unwrap() {
                        let best_label = bw_state.get_settled_labels_at(neighbor_node).max();
                        if let Some(Reverse(Label {
                            distance: best_label_distance, ..
                        })) = best_label
                        {
                            // best settled label as lower bound for D(neighbor_node,t)
                            if current_new_dist[0] + best_label_distance[0] >= tentative_distance {
                                continue;
                            }
                        } else if bw_state.queue.contains_index(neighbor_node as usize) {
                            // bw_min_key - bw_pot(neighbor_node) as lower bound for D(neighbor_node,t)
                            let bw_pot_at_neighbor = bw_state.get_best_label_at(neighbor_node).unwrap().distance_with_potential[0]
                                - bw_state.get_tentative_dist_at(neighbor_node)[0];
                            let bw_min_key = bw_state.peek_queue().map(|s| s.distance).unwrap();
                            if current_new_dist[0] + bw_min_key[0] - bw_pot_at_neighbor >= tentative_distance {
                                continue;
                            }
                        }
                    }

                    let neighbor_label_set = state.per_node_labels.get_mut(neighbor_node as usize);
                    let mut dominated = false;
                    neighbor_label_set.retain(|&neighbor_label| {
                        dominated |= neighbor_label.0.distance.dominates(&current_new_dist);
                        dominated || !current_new_dist.dominates(&neighbor_label.0.distance)
                    });

                    if !dominated {
                        state.num_labels_propagated += 1;
                        neighbor_label_set.push(Reverse(Label {
                            distance_with_potential,
                            distance: current_new_dist,
                            prev_label: Some(label_index),
                            prev_node: node_id,
                        }));

                        let pot = state.potential.potential(neighbor_node);
                        let dist_with_potential = state.estimated_dist_with_restriction(current_new_dist, pot);

                        if state.queue.contains_index(neighbor_node as usize) {
                            // decrease key seems to increase key if given a larger key than existing
                            if state.queue.get_key_by_index(neighbor_node as usize).unwrap().distance > dist_with_potential {
                                state.queue.decrease_key(State {
                                    distance: dist_with_potential,
                                    node: neighbor_node,
                                });
                            }
                        } else {
                            state.num_queue_pushes += 1;
                            state.queue.push(State {
                                distance: dist_with_potential,
                                node: neighbor_node,
                            });

                            if !is_core.get(node_id as usize).unwrap() {
                                *non_core_count += 1;
                            }
                        }
                    }
                }
                // }
            }
        } else {
            state.last_distance = None;
        }

        next
    }

    pub fn flagged_nodes_on_path(&self, path: &(Vec<NodeId>, Vec<Weight3>)) -> Vec<NodeId> {
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

    pub fn reset_nodes_on_path(&self, path: &(Vec<NodeId>, Vec<Weight3>)) -> (Vec<NodeId>, Vec<NodeId>) {
        let mut reset_nodes_short = Vec::new();
        let mut reset_nodes_long = Vec::new();

        for (node, dist) in path.0.iter().zip(&path.1) {
            if self.reset_flags.get(*node as usize).unwrap() && dist[1] == 0 && dist[0] != 0 {
                reset_nodes_short.push(*node)
            }

            if self.reset_flags.get(*node as usize).unwrap() && dist[2] == 0 && dist[0] != 0 {
                reset_nodes_long.push(*node)
            }
        }

        (reset_nodes_short, reset_nodes_long)
    }

    pub fn summary<P: Potential>(&self, state: &TwoRestrictionDijkstraData<P>) -> String {
        let mut s = "\n\nSummary for CSP2\n\n".to_string();

        if state.num_settled == 0 {
            writeln!(s, "No query").unwrap();
            return s;
        }

        writeln!(s, "Graph: ").unwrap();
        writeln!(s, "\tnumber of nodes: {}", state.invalid_node_id as usize).unwrap();
        writeln!(s, "\tnumber of arcs: {}", self.graph.num_arcs()).unwrap();

        writeln!(s).unwrap();
        writeln!(s, "Restriction: ").unwrap();

        if state.restriction_short.max_driving_time < Weight::infinity() && state.restriction_long.max_driving_time < Weight::infinity() {
            let num_reset_nodes = self.reset_flags.iter().filter(|b| *b).count();
            writeln!(
                s,
                "\tShort break with\n\t\tmax. driving time: {}\n\t\tpause time: {}\n\t\tnumber of flagged reset nodes: {} ({:.2}%)",
                state.restriction_short.max_driving_time,
                state.restriction_short.pause_time,
                num_reset_nodes,
                100.0 * num_reset_nodes as f32 / state.invalid_node_id as f32
            )
            .unwrap();
            writeln!(
                s,
                "\tLong break with\n\t\tmax. driving time: {}\n\t\tpause time: {}\n\t\tnumber of flagged reset nodes: {} ({:.2}%)",
                state.restriction_long.max_driving_time,
                state.restriction_long.pause_time,
                num_reset_nodes,
                100.0 * num_reset_nodes as f32 / state.invalid_node_id as f32
            )
            .unwrap();
        } else {
            writeln!(s, "no driving time restriction").unwrap();
        }

        writeln!(s).unwrap();
        writeln!(s, "Potential:").unwrap();
        writeln!(s, "\t{}", std::any::type_name::<P>()).unwrap();
        writeln!(s).unwrap();
        writeln!(s, "Query:").unwrap();
        writeln!(s, "\ts: {} t: {}", state.s, state.last_t).unwrap();
        writeln!(s, "\ttime elapsed: {:.3} ms", state.time_elapsed.as_secs_f64() * 1000.0).unwrap();
        writeln!(s, "\tnumber of queue pushes: {}", state.num_queue_pushes).unwrap();
        writeln!(s, "\tnumber of settled nodes: {}", state.num_settled).unwrap();
        writeln!(s, "\tnumber of propagated labels: {}", state.num_labels_propagated).unwrap();
        writeln!(s, "\tnumber of labels which were reset: {}", state.num_labels_reset).unwrap();

        writeln!(s).unwrap();
        writeln!(s, "Path:").unwrap();

        if state.last_distance.is_some() {
            writeln!(s, "\tdistance: {}", state.last_distance.unwrap()).unwrap();

            let (node_path, weights) = state.current_best_path_to(state.last_t, true).unwrap();
            writeln!(s, "\tnumber of nodes: {}", node_path.len()).unwrap();
            let flagged_p = self.flagged_nodes_on_node_path(&node_path);
            writeln!(s, "\t  -thereof number of flagged nodes: {}", flagged_p.len()).unwrap();

            let (reset_p_short, reset_p_long) = self.reset_nodes_on_path(&(node_path, weights));
            writeln!(s, "\t  -thereof number of flagged nodes actually used: {}", reset_p_short.len()).unwrap();
            writeln!(s, "\t  -thereof long breaks: {}", reset_p_long.len()).unwrap();
        } else {
            writeln!(s, "\tno path found").unwrap();
        }

        let mut label_sizes_settled = Vec::with_capacity(state.per_node_labels.len());
        let mut label_sizes_unsettled = Vec::with_capacity(state.per_node_labels.len());
        let mut label_sizes = Vec::with_capacity(state.per_node_labels.len());

        for i in 0..state.per_node_labels.len() {
            let label = state.per_node_labels.get(i);
            let number_of_unsettled = label.iter().count();
            let number_of_settled = label.popped().count();

            if number_of_settled != 0 {
                label_sizes.push(number_of_unsettled + number_of_settled);
                label_sizes_settled.push(number_of_settled);
                label_sizes_unsettled.push(number_of_unsettled);
            }
        }

        writeln!(s).unwrap();
        writeln!(s, "Label Statistics:").unwrap();

        if label_sizes.is_empty() {
            writeln!(s, "\t  -no labels were propagated").unwrap();
        } else {
            writeln!(
                s,
                "\tnumber of nodes with settled labels: {} ({:.2}%)",
                label_sizes.len(),
                100.0 * label_sizes.len() as f32 / state.invalid_node_id as f32
            )
            .unwrap();
            writeln!(s, "\t  -thereof maximum number of labels: {}", label_sizes.iter().max().unwrap()).unwrap();
            writeln!(
                s,
                "\t  -thereof avg number of labels: {:.2}",
                label_sizes.iter().sum::<usize>() as f32 / label_sizes.len() as f32
            )
            .unwrap();
            label_sizes.sort_unstable();
            let mean = if label_sizes.len().is_odd() {
                label_sizes[label_sizes.len() / 2] as f32
            } else {
                (label_sizes[(label_sizes.len() / 2) - 1] as f32 + label_sizes[label_sizes.len() / 2] as f32) / 2.0
            };
            writeln!(s, "\t  -thereof mean number of labels: {:.2}", mean).unwrap();
        }

        s
    }
}
