use crate::{algo::astar::*, index_heap::*, timestamped_vector::TimestampedVector, types::*};

pub trait DijkstraQuery<'a> {
    fn dijkstra_query(q: STQuery, graph: BorrowedGraph<'a>) -> Option<Weight>;
}

#[derive(Debug, Clone)]
pub struct DijkstraData<P = NoPotential> {
    pub queue: IndexdMinHeap<State<Weight>>,
    pub pred: TimestampedVector<NodeId>,
    pub dist: TimestampedVector<Weight>,
    potential: P,
    pub num_queue_pushes: u32,
    pub num_settled: u32,
    pub num_labels_propagated: u32,
    pub settled_nodes_vec: Vec<(NodeId, Weight)>,
    num_nodes: usize,
    s: NodeId,
}

impl DijkstraData<NoPotential> {
    pub fn new(num_nodes: usize) -> Self {
        Self {
            queue: IndexdMinHeap::new(num_nodes),
            pred: TimestampedVector::with_size(num_nodes),
            dist: TimestampedVector::with_size(num_nodes),
            potential: NoPotential {},
            num_queue_pushes: 0,
            num_settled: 0,
            num_labels_propagated: 0,
            settled_nodes_vec: Vec::new(),
            num_nodes,
            s: num_nodes as NodeId,
        }
    }
}

impl<P> DijkstraData<P>
where
    P: Potential,
{
    pub fn new_with_potential(num_nodes: usize, potential: P) -> Self {
        Self {
            queue: IndexdMinHeap::new(num_nodes),
            pred: TimestampedVector::with_size(num_nodes),
            dist: TimestampedVector::with_size(num_nodes),
            potential,
            num_queue_pushes: 0,
            num_settled: 0,
            num_labels_propagated: 0,
            settled_nodes_vec: Vec::new(),
            num_nodes,
            s: num_nodes as NodeId,
        }
    }

    pub fn path(&self, s: NodeId, t: NodeId) -> Option<Vec<NodeId>> {
        let mut path = vec![t];
        let mut current_id = t;
        while self.pred.is_set(current_id as usize) {
            current_id = *self.pred.get(current_id as usize);
            path.push(current_id);
        }

        if *path.last().unwrap() != s {
            return None;
        }

        path.reverse();
        Some(path)
    }

    pub fn current_node_path_to(&self, t: NodeId) -> Option<Vec<NodeId>> {
        self.path(self.s, t)
    }

    pub fn reset(&mut self) {
        self.num_settled = 0;
        self.num_labels_propagated = 0;
        self.num_queue_pushes = 0;
        self.settled_nodes_vec.clear();
        self.dist.reset();
        self.pred.reset();
        self.dist.set(self.s as usize, 0);
        self.queue.clear();
        self.queue.push(State {
            node: self.s,
            distance: 0.link(self.potential.potential(self.s)),
        });
    }

    pub fn init_new_s(&mut self, s: NodeId) {
        self.s = s;
        self.reset();
    }

    #[inline(always)]
    pub fn tentative_distance_at(&self, t: NodeId) -> Weight {
        *self.dist.get(t as usize)
    }

    #[inline(always)]
    pub fn min_key(&self) -> Option<Weight> {
        self.queue.peek().map(|s| s.distance)
    }
}

pub struct Dijkstra<'a> {
    graph: BorrowedGraph<'a>,
}

impl<'a> Dijkstra<'a> {
    pub fn new(graph: BorrowedGraph<'a>) -> Self {
        Self { graph }
    }

    pub fn ranks_only_exponentials<P: Potential>(&self, state: &mut DijkstraData<P>) -> Vec<NodeId> {
        state.reset();

        let log_num_nodes = (state.num_nodes as f32).log2() as usize;
        let mut rank_order = Vec::with_capacity(log_num_nodes);

        let mut exp_counter = 1;
        let mut counter = 0;
        while let Some(State { distance: _, node }) = self.settle_next_node(state) {
            counter += 1;

            if counter == exp_counter {
                exp_counter *= 2;
                rank_order.push(node);

                if rank_order.len() == log_num_nodes {
                    break;
                }
            }
        }

        rank_order
    }

    pub fn settle_next_node_not_exceeding<P: Potential>(&self, state: &mut DijkstraData<P>, distance_limit: Weight) -> Option<State<Weight>> {
        let dist = &mut state.dist;
        let pred = &mut state.pred;
        let queue = &mut state.queue;
        let pot = &mut state.potential;

        if let Some(next) = queue.pop() {
            state.settled_nodes_vec.push((next.node, next.distance));
            state.num_settled += 1;
            let node_id = next.node;

            for (&edge_weight, &neighbor_node) in self.graph.outgoing_edge_iter(node_id) {
                let new_dist = dist.get(node_id as usize).link(edge_weight);

                if new_dist >= distance_limit {
                    continue;
                }

                if !dist.is_set(neighbor_node as usize) {
                    state.num_queue_pushes += 1;
                    queue.push(State {
                        distance: new_dist.link(pot.potential(neighbor_node)),
                        node: neighbor_node,
                    });
                    state.num_labels_propagated += 1;
                    dist.set(neighbor_node as usize, new_dist);
                    pred.set(neighbor_node as usize, node_id);
                } else if new_dist < *dist.get(neighbor_node as usize) {
                    if !queue.contains_index(neighbor_node as usize) {
                        state.num_queue_pushes += 1;
                        queue.push(State {
                            distance: new_dist.link(pot.potential(neighbor_node)),
                            node: neighbor_node,
                        });
                    } else {
                        queue.decrease_key(State {
                            distance: new_dist.link(pot.potential(neighbor_node)),
                            node: neighbor_node,
                        });
                    }
                    state.num_labels_propagated += 1;
                    dist.set(neighbor_node as usize, new_dist);
                    pred.set(neighbor_node as usize, node_id);
                }
            }
            Some(next)
        } else {
            None
        }
    }

    pub fn settle_next_node<P: Potential>(&self, state: &mut DijkstraData<P>) -> Option<State<Weight>> {
        let dist = &mut state.dist;
        let pred = &mut state.pred;
        let queue = &mut state.queue;
        let pot = &mut state.potential;

        if let Some(next) = queue.pop() {
            state.settled_nodes_vec.push((next.node, next.distance));
            state.num_settled += 1;
            let node_id = next.node;

            for (&edge_weight, &neighbor_node) in self.graph.outgoing_edge_iter(node_id) {
                let new_dist = dist.get(node_id as usize).link(edge_weight);

                if !dist.is_set(neighbor_node as usize) {
                    state.num_queue_pushes += 1;
                    queue.push(State {
                        distance: new_dist.link(pot.potential(neighbor_node)),
                        node: neighbor_node,
                    });
                    state.num_labels_propagated += 1;
                    dist.set(neighbor_node as usize, new_dist);
                    pred.set(neighbor_node as usize, node_id);
                } else if new_dist < *dist.get(neighbor_node as usize) {
                    if !queue.contains_index(neighbor_node as usize) {
                        state.num_queue_pushes += 1;
                        queue.push(State {
                            distance: new_dist.link(pot.potential(neighbor_node)),
                            node: neighbor_node,
                        });
                    } else {
                        queue.decrease_key(State {
                            distance: new_dist.link(pot.potential(neighbor_node)),
                            node: neighbor_node,
                        });
                    }
                    state.num_labels_propagated += 1;
                    dist.set(neighbor_node as usize, new_dist);
                    pred.set(neighbor_node as usize, node_id);
                }
            }
            Some(next)
        } else {
            None
        }
    }

    pub fn dist_query<P: Potential>(&self, state: &mut DijkstraData<P>, t: NodeId) -> Option<Weight> {
        state.reset();
        state.potential.init_new_t(t);

        while let Some(State { distance: _, node }) = self.settle_next_node(state) {
            if node == t {
                return Some(*state.dist.get(node as usize));
            }
        }

        None
    }

    pub fn to_all<P: Potential>(&self, state: &mut DijkstraData<P>) {
        state.reset();

        while self.settle_next_node(state).is_some() {}
    }
}

impl<'a> DijkstraQuery<'a> for Dijkstra<'a> {
    fn dijkstra_query(q: STQuery, graph: BorrowedGraph<'a>) -> Option<NodeId> {
        let mut state = DijkstraData::new(graph.num_nodes());
        state.init_new_s(q.s);
        let dijkstra = Self::new(graph);
        dijkstra.dist_query(&mut state, q.t)
    }
}
