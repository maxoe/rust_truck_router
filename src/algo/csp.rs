use std::{
    borrow::Cow,
    cmp::Reverse,
    fmt::Write,
    time::{Duration, Instant},
};

use crate::{
    algo::astar::{NoPotential, Potential},
    index_heap::*,
    rrr_heap::Heap,
    timestamped_vector::TimestampedVector,
    types::*,
};
use bit_vec::BitVec;
use num::Integer;

#[derive(Copy, Clone, Eq, PartialEq, Debug, Ord, PartialOrd)]
pub struct Label<T> {
    pub distance: T,
    pub prev_node: NodeId,
    pub incoming_edge_weight: Weight,
}

type MCDHeap<L> = Heap<Reverse<Label<L>>>;

impl<L: Clone> DefaultReset for MCDHeap<L> {
    const DEFAULT: MCDHeap<L> = MCDHeap::<L>::new();
}
pub struct MultiCriteriaDijkstraData<L: WeightOps> {
    pub queue: IndexdMinHeap<State<L>>,
    pub per_node_labels: TimestampedVector<MCDHeap<L>>,
    invalid_node_id: NodeId,
}

impl<L: WeightOps> MultiCriteriaDijkstraData<L> {
    pub fn new(n: usize) -> Self {
        Self {
            queue: IndexdMinHeap::new(n),
            per_node_labels: TimestampedVector::with_size(n),
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
    pub graph: Cow<'a, OwnedGraph>,
    restriction: DrivingTimeRestriction,
    reset_flags: BitVec,
    potential: P,
    pub num_queue_pushes: u32,
    pub num_settled: u32,
    pub num_labels_propagated: u32,
    pub num_labels_reset: u32,
    pub time_elapsed: Duration,
    pub last_t: NodeId,
    pub last_distance: Option<Weight>,
}

impl<'a> OneRestrictionDijkstra<'a, NoPotential>
where
    Self: 'a,
{
    pub fn new_from_owned(graph: OwnedGraph) -> Self {
        let n = graph.num_nodes();
        Self {
            data: MultiCriteriaDijkstraData::new(graph.num_nodes()),
            s: n as NodeId,
            graph: Cow::Owned(graph),
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
            time_elapsed: Duration::ZERO,
            last_t: n as NodeId,
            last_distance: None,
        }
    }

    pub fn new(graph: BorrowedGraph<'a>) -> Self {
        let n = graph.num_nodes();
        Self {
            data: MultiCriteriaDijkstraData::new(graph.num_nodes()),
            s: n as NodeId,
            graph: Cow::Borrowed(graph),
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
            time_elapsed: Duration::ZERO,
            last_t: n as NodeId,
            last_distance: None,
        }
    }
}

impl<'a, P> OneRestrictionDijkstra<'a, P>
where
    P: Potential<Weight>,
{
    pub fn reset(&mut self) {
        if self.s != self.graph.num_nodes() as NodeId {
            self.num_settled = 0;
            self.num_labels_propagated = 0;
            self.num_queue_pushes = 0;
            self.num_labels_reset = 0;

            self.data.per_node_labels.reset();

            self.data.queue.clear();
            self.num_queue_pushes += 1;
            let pot = self.potential.potential(self.s);
            self.data.queue.push(State {
                node: self.s,
                distance: self.estimated_dist_with_restriction([0, 0], pot),
            });
            self.data.per_node_labels.get_mut(self.s as usize).push(Reverse(Label {
                prev_node: self.graph.num_nodes() as NodeId,
                distance: [0, 0],
                incoming_edge_weight: Weight::infinity(),
            }));
        }
    }

    pub fn new_with_potential(graph: BorrowedGraph<'a>, potential: P) -> Self {
        let n = graph.num_nodes();

        Self {
            data: MultiCriteriaDijkstraData::new(graph.num_nodes()),
            s: graph.num_nodes() as NodeId,
            graph: Cow::Borrowed(graph),
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
            time_elapsed: Duration::ZERO,
            last_t: n as NodeId,
            last_distance: None,
        }
    }

    pub fn new_with_potential_from_owned(graph: OwnedGraph, potential: P) -> Self {
        let n = graph.num_nodes();

        Self {
            data: MultiCriteriaDijkstraData::new(graph.num_nodes()),
            s: graph.num_nodes() as NodeId,
            graph: Cow::Owned(graph),
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
            time_elapsed: Duration::ZERO,
            last_t: n as NodeId,
            last_distance: None,
        }
    }

    pub fn init_new_s(&mut self, s: NodeId) {
        self.s = s;
        self.reset();
    }

    pub fn min_key(&self) -> Option<Weight> {
        self.data.queue.peek().map(|s| s.distance[0])
    }

    pub fn current_best_path_to(&self, t: NodeId, with_distances: bool) -> Option<(Vec<NodeId>, Vec<Weight2>)> {
        let mut path = vec![];
        let mut distances = vec![];
        let mut current_node = t;

        //get best settled or unsettled label
        //max because the type is Reverse<Label<..>>
        let best_settled_at_t = self.data.per_node_labels.get(t as usize).popped().iter().max();
        let best_unsettled_at_t = self.data.per_node_labels.get(t as usize).iter().max();

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
                'outer: for next_label in self.data.per_node_labels.get(next_node as usize).popped() {
                    let mut dist_candidates = [next_label.0.distance.link(current_label.0.incoming_edge_weight); 2];

                    if self.reset_flags.get(current_node as usize).unwrap() {
                        dist_candidates[1].reset_distance(1, self.restriction.pause_time);
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
            if self.reset_flags.get(*node as usize).unwrap() && dist[1] == 0 && dist[0] != 0 {
                reset_nodes.push(*node)
            }
        }

        reset_nodes
    }

    pub fn set_restriction(&mut self, max_driving_time: Weight, pause_time: Weight) -> &mut Self {
        assert!(max_driving_time > 0);
        self.restriction = DrivingTimeRestriction { pause_time, max_driving_time };
        self.reset();
        self
    }

    pub fn clear_restriction(&mut self) -> &mut Self {
        self.restriction = DrivingTimeRestriction {
            pause_time: 0,
            max_driving_time: Weight::infinity(),
        };
        self.reset();
        self
    }

    pub fn set_reset_flags<B: AsRef<[u8]>>(&mut self, flags: B) -> &mut Self {
        self.reset_flags = BitVec::from_bytes(flags.as_ref());
        self.reset();
        self
    }

    pub fn clear_reset_flags(&mut self) {
        self.reset_flags = BitVec::from_elem(self.graph.num_nodes(), false);
        self.reset();
    }

    pub fn get_settled_labels_at(&mut self, node: NodeId) -> &[Reverse<Label<Weight2>>] {
        self.data.per_node_labels.get_mut(node as usize).popped_sorted()
    }

    pub fn peek_queue(&self) -> Option<&State<Weight2>> {
        self.data.queue.peek()
    }
}

impl<'a, P> OneRestrictionDijkstra<'a, P>
where
    P: Potential<Weight>,
    Self: 'a,
{
    fn estimated_dist_with_restriction(&self, distance_at_node: [Weight; 2], potential_to_target: Weight) -> [Weight; 2] {
        let estimated = distance_at_node.link(potential_to_target);

        if self.restriction.max_driving_time == Weight::infinity() {
            estimated
        } else {
            [
                estimated[0].link((estimated[1] / self.restriction.max_driving_time) * self.restriction.pause_time),
                estimated[1],
            ]
        }
    }

    pub fn settle_next_label(&mut self, t: NodeId) -> Option<State<Weight2>> {
        let next = self.data.queue.pop();

        if let Some(State {
            distance: tentative_distance_from_queue,
            node: node_id,
        }) = next
        {
            self.num_settled += 1;

            if node_id == t {
                // push again for eventual next query
                self.data.queue.push(State {
                    distance: tentative_distance_from_queue,
                    node: node_id,
                });

                self.last_distance = self.data.per_node_labels.get(node_id as usize).peek().map(|label| label.0.distance[0]);
            } else {
                let label = self.data.per_node_labels.get_mut(node_id as usize).pop().unwrap();
                let tentative_dist_without_pot = label.0.distance;

                // check if next unsettled lable exists for node and push to queue
                if let Some(next_best_label) = self.data.per_node_labels.get(node_id as usize).peek() {
                    let pot = self.potential.potential(node_id);
                    self.data.queue.push(State {
                        distance: self.estimated_dist_with_restriction(next_best_label.0.distance, pot),
                        node: node_id,
                    });
                }

                // with hopping reduction
                for (&edge_weight, &neighbor_node) in self.graph.outgoing_edge_iter(node_id).filter(|&s| *(s.1) != node_id) {
                    // [new_dist without, new_dist with parking]
                    let mut new_dist = Vec::with_capacity(2);
                    new_dist.push(tentative_dist_without_pot.link(edge_weight));

                    // constraint and target pruning
                    if new_dist[0][1] >= self.restriction.max_driving_time
                        || self.data.per_node_labels.get(t as usize).iter().any(|&s| s.0.distance.dominates(&new_dist[0]))
                    {
                        continue;
                    }

                    if self.reset_flags.get(neighbor_node as usize).unwrap() {
                        new_dist.push(new_dist[0]);
                        new_dist[1].reset_distance(1, self.restriction.pause_time);
                        self.num_labels_reset += 1;
                    }

                    for current_new_dist in new_dist {
                        let is_set = self.data.per_node_labels.is_set(neighbor_node as usize);
                        self.data.per_node_labels.get_mut(neighbor_node as usize);
                        assert!(
                            is_set
                                || (self.data.per_node_labels.get(neighbor_node as usize).iter().count()
                                    + self.data.per_node_labels.get(neighbor_node as usize).popped().len())
                                    == 0
                        );
                        let neighbor_label_set = self.data.per_node_labels.get_mut(neighbor_node as usize);
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
                }
            }
        } else {
            self.last_distance = None;
        }

        next
    }

    pub fn dist_query(&mut self, t: NodeId) -> Option<Weight> {
        self.reset();
        self.potential.init_new_t(t);
        self.last_t = t;

        let start = Instant::now();

        while let Some(State { distance: _, node: node_id }) = self.settle_next_label(t) {
            if node_id == t {
                self.time_elapsed = start.elapsed();
                return self.last_distance;
            }
        }
        self.time_elapsed = start.elapsed();
        None
    }

    pub fn dist_query_propagate_all_labels(&mut self, t: NodeId) -> Option<Weight> {
        self.reset();
        self.potential.init_new_t(t);
        self.last_t = t;

        let start = Instant::now();

        while let Some(State { distance: _, node: node_id }) = self.settle_next_label_propagate_all(t) {
            if node_id == t {
                self.time_elapsed = start.elapsed();
                return self.last_distance;
            }
        }
        self.time_elapsed = start.elapsed();
        None
    }

    pub fn settle_next_label_propagate_all(&mut self, t: NodeId) -> Option<State<Weight2>> {
        let next = self.data.queue.pop();

        if let Some(State {
            distance: tentative_distance_from_queue,
            node: node_id,
        }) = next
        {
            self.num_settled += 1;

            if node_id == t {
                // push again for eventual next query
                self.data.queue.push(State {
                    distance: tentative_distance_from_queue,
                    node: node_id,
                });

                self.last_distance = self.data.per_node_labels.get(node_id as usize).peek().map(|label| label.0.distance[0]);
            } else {
                while let Some(label) = self.data.per_node_labels.get_mut(node_id as usize).pop() {
                    let tentative_dist_without_pot = label.0.distance;

                    // check if next unsettled lable exists for node and push to queue
                    // if let Some(next_best_label) = self.data.per_node_labels.get(node_id as usize).peek() {
                    //     let pot = self.potential.potential(node_id);
                    //     self.data.queue.push(State {
                    //         distance: self.estimated_dist_with_restriction(next_best_label.0.distance, pot),
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
                                || self.data.per_node_labels.get(t as usize).iter().any(|&s| s.0.distance.dominates(&new_dist[0]))
                            {
                                continue;
                            }

                            if self.reset_flags.get(neighbor_node as usize).unwrap() {
                                new_dist.push(new_dist[0]);
                                new_dist[1].reset_distance(1, self.restriction.pause_time);
                                self.num_labels_reset += 1;
                            }

                            for current_new_dist in new_dist {
                                let neighbor_label_set = self.data.per_node_labels.get_mut(neighbor_node as usize);
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

                                    if dist_with_potential[0] == INFINITY {
                                        continue;
                                    }

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
        } else {
            self.last_distance = None;
        }

        next
    }

    pub fn info(&self) -> String {
        let mut s = "\n\nSummary for CSP\n\n".to_string();

        if self.num_settled == 0 {
            writeln!(s, "No query").unwrap();
            return s;
        }

        writeln!(s, "Graph: ").unwrap();
        writeln!(s, "\tnumber of nodes: {}", self.graph.num_nodes()).unwrap();
        writeln!(s, "\tnumber of arcs: {}", self.graph.num_arcs()).unwrap();

        writeln!(s, "").unwrap();
        writeln!(s, "Restriction: ").unwrap();

        if self.restriction.max_driving_time < Weight::infinity() {
            let num_reset_nodes = self.reset_flags.iter().filter(|b| *b).count();
            writeln!(
                s,
                "\tmax. driving time: {}\n\tpause time: {}\n\tnumber of flagged reset nodes: {} ({:.2}%)",
                self.restriction.max_driving_time,
                self.restriction.pause_time,
                num_reset_nodes,
                100.0 * num_reset_nodes as f32 / self.graph.num_nodes() as f32
            )
            .unwrap();
        } else {
            writeln!(s, "no driving time restriction").unwrap();
        }

        writeln!(s, "").unwrap();
        writeln!(s, "Potential:").unwrap();
        writeln!(s, "\t{}", std::any::type_name::<P>()).unwrap();
        writeln!(s, "").unwrap();
        writeln!(s, "Query:").unwrap();
        writeln!(s, "\ts: {} t: {}", self.s, self.last_t).unwrap();
        writeln!(s, "\ttime elapsed: {:.3} ms", self.time_elapsed.as_secs_f64() * 1000.0).unwrap();
        writeln!(s, "\tnumber of queue pushes: {}", self.num_queue_pushes).unwrap();
        writeln!(s, "\tnumber of settled nodes: {}", self.num_settled).unwrap();
        writeln!(s, "\tnumber of propagated labels: {}", self.num_labels_propagated).unwrap();
        writeln!(s, "\tnumber of labels which were reset: {}", self.num_labels_reset).unwrap();

        writeln!(s, "").unwrap();
        writeln!(s, "Path:").unwrap();

        if self.last_distance.is_some() {
            writeln!(s, "\tdistance: {}", self.last_distance.unwrap()).unwrap();

            let (node_path, weights) = self.current_best_path_to(self.last_t, true).unwrap();
            writeln!(s, "\tnumber of nodes: {}", node_path.len()).unwrap();
            let flagged_p = self.flagged_nodes_on_node_path(&node_path);
            writeln!(s, "\t  -thereof number of flagged nodes: {}", flagged_p.len()).unwrap();

            let reset_p = self.reset_nodes_on_path(&(node_path, weights));
            writeln!(s, "\t  -thereof number of flagged nodes actually used: {}", reset_p.len()).unwrap();
        } else {
            writeln!(s, "\tno path found").unwrap();
        }

        let mut label_sizes_settled = Vec::with_capacity(self.data.per_node_labels.len());
        let mut label_sizes_unsettled = Vec::with_capacity(self.data.per_node_labels.len());
        let mut label_sizes = Vec::with_capacity(self.data.per_node_labels.len());

        for i in 0..self.data.per_node_labels.len() {
            let label = self.data.per_node_labels.get(i);
            let number_of_unsettled = label.iter().count();
            let number_of_settled = label.popped().len();

            if number_of_settled != 0 {
                label_sizes.push(number_of_unsettled + number_of_settled);
                label_sizes_settled.push(number_of_settled);
                label_sizes_unsettled.push(number_of_unsettled);
            }
        }

        writeln!(s, "").unwrap();
        writeln!(s, "Label Statistics:").unwrap();

        if label_sizes.len() == 0 {
            writeln!(s, "\t  -no labels were propagated").unwrap();
        } else {
            writeln!(
                s,
                "\tnumber of nodes with settled labels: {} ({:.2}%)",
                label_sizes.len(),
                100.0 * label_sizes.len() as f32 / self.graph.num_nodes() as f32
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

    pub fn get_per_node_number_of_labels(&self) -> Vec<usize> {
        (0..self.data.per_node_labels.len())
            .map(|i| {
                let h = self.data.per_node_labels.get(i);
                h.iter().count() + h.popped().len()
            })
            .collect()
    }

    pub fn get_number_of_visited_nodes(&self) -> usize {
        (0..self.data.per_node_labels.len())
            .filter(|i| self.data.per_node_labels.get(*i).popped().len() != 0)
            .count()
    }
}
