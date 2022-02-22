use crate::{algo::astar::*, index_heap::*, timestamped_vector::TimestampedVector, types::*};

#[derive(Debug, Clone)]
pub struct DijkstraData<W: WeightOps + DefaultReset> {
    pub queue: IndexdMinHeap<State<W>>,
    pub pred: TimestampedVector<NodeId>,
    pub dist: TimestampedVector<W>,
}

impl<W: WeightOps + DefaultReset> DijkstraData<W> {
    pub fn new(n: usize) -> Self {
        Self {
            queue: IndexdMinHeap::new(n),
            pred: TimestampedVector::with_size(n),
            dist: TimestampedVector::with_size(n),
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
}

pub struct Dijkstra<'a, P = NoPotential>
where
    P: Potential<Weight>,
{
    data: DijkstraData<Weight>,
    s: NodeId,
    graph: BorrowedGraph<'a>,
    potential: P,
    pub num_queue_pushes: u32,
    pub num_settled: u32,
    pub num_labels_propagated: u32,
    pub settled_nodes_vec: Vec<(NodeId, Weight)>,
}

impl<'a, P> Dijkstra<'a, P>
where
    P: Potential<Weight>,
{
    pub fn new_with_potential(graph: BorrowedGraph<'a>, potential: P) -> Self {
        Self {
            data: DijkstraData::new(graph.num_nodes()),
            s: graph.num_nodes() as NodeId,
            graph,
            potential,
            num_queue_pushes: 0,
            num_settled: 0,
            num_labels_propagated: 0,
            settled_nodes_vec: Vec::new(),
        }
    }
    pub fn current_node_path_to(&self, t: NodeId) -> Option<Vec<NodeId>> {
        self.data.path(self.s, t)
    }
}

impl<'a> Dijkstra<'a, NoPotential> {
    pub fn new(graph: BorrowedGraph<'a>) -> Self {
        Self {
            data: DijkstraData::new(graph.num_nodes()),
            s: graph.num_nodes() as NodeId,
            graph,
            potential: NoPotential {},
            num_queue_pushes: 0,
            num_settled: 0,
            num_labels_propagated: 0,
            settled_nodes_vec: Vec::new(),
        }
    }

    pub fn ranks_only_exponentials(&mut self) -> Vec<NodeId> {
        self.reset();

        let log_num_nodes = (self.graph.num_nodes() as f32).log2() as usize;
        let mut rank_order = Vec::with_capacity(log_num_nodes);

        let mut exp_counter = 1;
        let mut counter = 0;
        while let Some(State { distance: _, node }) = self.settle_next_node() {
            counter += 1;

            if counter == exp_counter {
                exp_counter = 2 * exp_counter;
                rank_order.push(node);
            }
        }

        rank_order
    }
}

impl<'a, P> Dijkstra<'a, P>
where
    P: Potential<Weight>,
{
    pub fn reset(&mut self) {
        self.num_settled = 0;
        self.num_labels_propagated = 0;
        self.num_queue_pushes = 0;
        self.data.dist.reset();
        self.data.pred.reset();
        self.data.dist.set(self.s as usize, 0);
        self.data.queue.clear();
        self.data.queue.push(State {
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
        *self.data.dist.get(t as usize)
    }

    #[inline(always)]
    pub fn min_key(&self) -> Option<Weight> {
        self.data.queue.peek().map(|s| s.distance)
    }

    pub fn settle_next_node(&mut self) -> Option<State<Weight>> {
        let dist = &mut self.data.dist;
        let pred = &mut self.data.pred;
        let queue = &mut self.data.queue;
        let pot = &mut self.potential;

        if let Some(next) = queue.pop() {
            self.settled_nodes_vec.push((next.node, next.distance));
            self.num_settled += 1;
            let node_id = next.node;

            for (&edge_weight, &neighbor_node) in self.graph.outgoing_edge_iter(node_id) {
                let new_dist = dist.get(node_id as usize).link(edge_weight);

                if !dist.is_set(neighbor_node as usize) {
                    self.num_queue_pushes += 1;
                    queue.push(State {
                        distance: new_dist.link(pot.potential(neighbor_node)),
                        node: neighbor_node,
                    });
                    self.num_labels_propagated += 1;
                    dist.set(neighbor_node as usize, new_dist);
                    pred.set(neighbor_node as usize, node_id);
                } else if new_dist < *dist.get(neighbor_node as usize) {
                    if !queue.contains_index(neighbor_node as usize) {
                        self.num_queue_pushes += 1;
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
                    self.num_labels_propagated += 1;
                    dist.set(neighbor_node as usize, new_dist);
                    pred.set(neighbor_node as usize, node_id);
                }
            }
            Some(next)
        } else {
            None
        }
    }

    pub fn dist_query(&mut self, t: NodeId) -> Option<Weight> {
        self.reset();
        self.potential.init_new_t(t);

        while let Some(State { distance: _, node }) = self.settle_next_node() {
            if node == t {
                return Some(*self.data.dist.get(node as usize));
            }
        }

        None
    }

    pub fn to_all(&mut self) {
        self.reset();

        while self.settle_next_node().is_some() {}
    }
}

#[derive(Clone)]
pub struct OwnedDijkstra<P = NoPotential>
where
    P: Potential<Weight>,
{
    data: DijkstraData<Weight>,
    s: NodeId,
    pub graph: OwnedGraph,
    potential: P,
    pub num_queue_pushes: u32,
    pub num_settled: u32,
    pub num_labels_propagated: u32,
    pub settled_nodes_vec: Vec<(NodeId, Weight)>,
}

impl OwnedDijkstra<NoPotential> {
    pub fn new(graph: OwnedGraph) -> Self {
        Self {
            data: DijkstraData::new(graph.num_nodes()),
            s: graph.num_nodes() as NodeId,
            graph,
            potential: NoPotential {},
            num_queue_pushes: 0,
            num_settled: 0,
            num_labels_propagated: 0,
            settled_nodes_vec: Vec::new(),
        }
    }

    pub fn ranks_only_exponentials(&mut self) -> Vec<NodeId> {
        self.reset();

        let log_num_nodes = (self.graph.num_nodes() as f32).log2() as usize;
        let mut rank_order = Vec::with_capacity(log_num_nodes);

        let mut exp_counter = 1;
        let mut counter = 0;
        while let Some(State { distance: _, node }) = self.settle_next_node() {
            counter += 1;

            if counter == exp_counter {
                exp_counter = 2 * exp_counter;
                rank_order.push(node);

                if rank_order.len() == log_num_nodes {
                    break;
                }
            }
        }

        rank_order
    }
}

impl<P> OwnedDijkstra<P>
where
    P: Potential<Weight>,
{
    pub fn new_with_potential(graph: OwnedGraph, potential: P) -> Self {
        Self {
            data: DijkstraData::new(graph.num_nodes()),
            s: graph.num_nodes() as NodeId,
            graph,
            potential,
            num_queue_pushes: 0,
            num_settled: 0,
            num_labels_propagated: 0,
            settled_nodes_vec: Vec::new(),
        }
    }
    pub fn current_node_path_to(&self, t: NodeId) -> Option<Vec<NodeId>> {
        self.data.path(self.s, t)
    }

    pub fn reset(&mut self) {
        self.num_settled = 0;
        self.num_labels_propagated = 0;
        self.num_queue_pushes = 0;
        self.data.dist.reset();
        self.data.pred.reset();
        self.data.dist.set(self.s as usize, 0);
        self.data.queue.clear();
        self.data.queue.push(State {
            node: self.s,
            distance: 0.link(self.potential.potential(self.s)),
        });
    }

    pub fn init_new_s(&mut self, s: NodeId) {
        self.s = s;
        self.reset();
    }

    // #[inline(always)]
    pub fn tentative_distance_at(&self, t: NodeId) -> Weight {
        *self.data.dist.get(t as usize)
    }

    #[inline(always)]
    pub fn min_key(&self) -> Option<Weight> {
        self.data.queue.peek().map(|s| s.distance)
    }

    pub fn settle_next_node(&mut self) -> Option<State<Weight>> {
        let dist = &mut self.data.dist;
        let pred = &mut self.data.pred;
        let queue = &mut self.data.queue;
        let pot = &mut self.potential;

        if let Some(next) = queue.pop() {
            self.settled_nodes_vec.push((next.node, next.distance));
            self.num_settled += 1;
            let node_id = next.node;

            for (&edge_weight, &neighbor_node) in self.graph.outgoing_edge_iter(node_id) {
                let new_dist = dist.get(node_id as usize).link(edge_weight);

                if !dist.is_set(neighbor_node as usize) {
                    self.num_queue_pushes += 1;
                    queue.push(State {
                        distance: new_dist.link(pot.potential(neighbor_node)),
                        node: neighbor_node,
                    });
                    self.num_labels_propagated += 1;
                    dist.set(neighbor_node as usize, new_dist);
                    pred.set(neighbor_node as usize, node_id);
                } else if new_dist < *dist.get(neighbor_node as usize) {
                    if !queue.contains_index(neighbor_node as usize) {
                        self.num_queue_pushes += 1;
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
                    self.num_labels_propagated += 1;
                    dist.set(neighbor_node as usize, new_dist);
                    pred.set(neighbor_node as usize, node_id);
                }
            }
            Some(next)
        } else {
            None
        }
    }

    pub fn dist_query(&mut self, t: NodeId) -> Option<Weight> {
        self.reset();
        self.potential.init_new_t(t);

        while let Some(State { distance: _, node }) = self.settle_next_node() {
            if node == t {
                return Some(*self.data.dist.get(node as usize));
            }
        }

        None
    }

    pub fn to_all(&mut self) {
        self.reset();

        while self.settle_next_node().is_some() {}
    }
}
