use crate::{algo::astar::*, index_heap::*, types::*};

#[derive(Debug)]
pub struct DijkstraData<W: WeightOps> {
    pub queue: IndexdMinHeap<State<W>>,
    pub pred: Vec<(NodeId, EdgeId)>,
    pub dist: Vec<W>,
    pub run: Vec<usize>,
    pub run_count: usize,
}

impl<W: WeightOps> DijkstraData<W> {
    pub fn new(n: usize) -> Self {
        Self {
            queue: IndexdMinHeap::new(n),
            pred: vec![(n as NodeId, EdgeId::MAX); n],
            dist: vec![W::infinity(); n],
            run: vec![0; n],
            run_count: 0,
        }
    }

    pub fn path(&self, s: NodeId, t: NodeId) -> Option<Vec<NodeId>> {
        let mut path = vec![t];
        let mut current_id = t;
        while current_id != s && current_id as usize != self.pred.len() && self.run[current_id as usize] == self.run_count {
            current_id = self.pred[current_id as usize].0;
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
    pub fn new_with_potential(graph: BorrowedGraph<'a>, s: NodeId, potential: P) -> Self {
        Self {
            data: DijkstraData::new(graph.num_nodes()),
            s: s,
            graph: graph,
            potential: potential,
            num_queue_pushes: 0,
            num_settled: 0,
            num_labels_propagated: 0,
            settled_nodes_vec: Vec::new(),
        }
    }
    pub fn current_node_path_to(&self, t: NodeId) -> Option<Vec<NodeId>> {
        return self.data.path(self.s, t);
    }
}

impl<'a> Dijkstra<'a, NoPotential> {
    pub fn new(graph: BorrowedGraph<'a>) -> Self {
        Self {
            data: DijkstraData::new(graph.num_nodes()),
            s: graph.num_nodes() as NodeId,
            graph: graph,
            potential: NoPotential {},
            num_queue_pushes: 0,
            num_settled: 0,
            num_labels_propagated: 0,
            settled_nodes_vec: Vec::new(),
        }
    }
}

impl<'a> Dijkstra<'a> {
    pub fn reset(&mut self) {
        self.data.run_count += 1;
        self.num_settled = 0;
        self.num_labels_propagated = 0;
        self.num_queue_pushes = 0;
        self.data.dist[self.s as usize] = 0;
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
        if self.data.run[t as usize] == self.data.run_count {
            self.data.dist[t as usize]
        } else {
            Weight::infinity()
        }
    }

    #[inline(always)]
    pub fn min_key(&self) -> Option<Weight> {
        match self.data.queue.peek() {
            Some(s) => Some(s.distance),
            None => None,
        }
    }

    pub fn settle_next_node(&mut self) -> Option<State<Weight>> {
        let dist = &mut self.data.dist;
        let pred = &mut self.data.pred;
        let queue = &mut self.data.queue;
        let run = &mut self.data.run;
        let pot = &self.potential;

        if let Some(next) = queue.pop() {
            self.settled_nodes_vec.push((next.node, next.distance));
            self.num_settled += 1;
            let node_id = next.node;

            for (&edge_weight, &neighbor_node) in self.graph.outgoing_edge_iter(node_id) {
                let new_dist = dist[node_id as usize].link(edge_weight);

                if run[neighbor_node as usize] != self.data.run_count {
                    self.num_queue_pushes += 1;
                    queue.push(State {
                        distance: new_dist.link(pot.potential(neighbor_node)),
                        node: neighbor_node,
                    });
                    self.num_labels_propagated += 1;
                    dist[neighbor_node as usize] = new_dist;
                    pred[neighbor_node as usize] = (node_id, EdgeId::MAX);
                    run[neighbor_node as usize] = self.data.run_count;
                } else if new_dist < dist[neighbor_node as usize] {
                    queue.decrease_key(State {
                        distance: new_dist.link(pot.potential(neighbor_node)),
                        node: neighbor_node,
                    });
                    self.num_labels_propagated += 1;
                    dist[neighbor_node as usize] = new_dist;
                    pred[neighbor_node as usize] = (node_id, EdgeId::MAX);
                }
            }
            return Some(next);
        } else {
            None
        }
    }

    pub fn dist_query(&mut self, t: NodeId) -> Option<Weight> {
        self.reset();

        while let Some(State { distance, node }) = self.settle_next_node() {
            if node == t {
                return Some(distance);
            }
        }

        None
    }
}

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

impl<P> OwnedDijkstra<P>
where
    P: Potential<Weight>,
{
    pub fn new_with_potential(graph: OwnedGraph, s: NodeId, potential: P) -> Self {
        Self {
            data: DijkstraData::new(graph.num_nodes()),
            s: s,
            graph: graph,
            potential: potential,
            num_queue_pushes: 0,
            num_settled: 0,
            num_labels_propagated: 0,
            settled_nodes_vec: Vec::new(),
        }
    }
    pub fn current_node_path_to(&self, t: NodeId) -> Option<Vec<NodeId>> {
        return self.data.path(self.s, t);
    }
}

impl OwnedDijkstra<NoPotential> {
    pub fn new(graph: OwnedGraph) -> Self {
        Self {
            data: DijkstraData::new(graph.num_nodes()),
            s: graph.num_nodes() as NodeId,
            graph: graph,
            potential: NoPotential {},
            num_queue_pushes: 0,
            num_settled: 0,
            num_labels_propagated: 0,
            settled_nodes_vec: Vec::new(),
        }
    }
}

impl OwnedDijkstra {
    pub fn reset(&mut self) {
        self.data.run_count += 1;
        self.num_settled = 0;
        self.num_labels_propagated = 0;
        self.num_queue_pushes = 0;
        self.data.dist[self.s as usize] = 0;
        self.data.queue.clear();
        self.data.queue.push(State {
            node: self.s,
            distance: 0.link(self.potential.potential(self.s)),
        });
        self.data.run[self.s as usize] = self.data.run_count;
    }

    pub fn init_new_s(&mut self, s: NodeId) {
        self.s = s;
        self.reset();
    }

    // #[inline(always)]
    pub fn tentative_distance_at(&self, t: NodeId) -> Weight {
        if self.data.run[t as usize] == self.data.run_count {
            self.data.dist[t as usize]
        } else {
            Weight::infinity()
        }
    }

    #[inline(always)]
    pub fn min_key(&self) -> Option<Weight> {
        match self.data.queue.peek() {
            Some(s) => Some(s.distance),
            None => None,
        }
    }

    pub fn settle_next_node(&mut self) -> Option<State<Weight>> {
        let dist = &mut self.data.dist;
        let pred = &mut self.data.pred;
        let queue = &mut self.data.queue;
        let run = &mut self.data.run;
        let pot = &self.potential;

        if let Some(next) = queue.pop() {
            self.settled_nodes_vec.push((next.node, next.distance));
            self.num_settled += 1;
            let node_id = next.node;

            for (&edge_weight, &neighbor_node) in self.graph.outgoing_edge_iter(node_id) {
                let new_dist = dist[node_id as usize].link(edge_weight);

                if run[neighbor_node as usize] != self.data.run_count {
                    self.num_queue_pushes += 1;
                    queue.push(State {
                        distance: new_dist.link(pot.potential(neighbor_node)),
                        node: neighbor_node,
                    });
                    self.num_labels_propagated += 1;
                    dist[neighbor_node as usize] = new_dist;
                    pred[neighbor_node as usize] = (node_id, EdgeId::MAX);
                    run[neighbor_node as usize] = self.data.run_count;
                } else if new_dist < dist[neighbor_node as usize] {
                    queue.decrease_key(State {
                        distance: new_dist.link(pot.potential(neighbor_node)),
                        node: neighbor_node,
                    });
                    self.num_labels_propagated += 1;
                    dist[neighbor_node as usize] = new_dist;
                    pred[neighbor_node as usize] = (node_id, EdgeId::MAX);
                }
            }
            return Some(next);
        } else {
            None
        }
    }

    pub fn dist_query(&mut self, t: NodeId) -> Option<Weight> {
        self.reset();

        while let Some(State { distance, node }) = self.settle_next_node() {
            if node == t {
                return Some(distance);
            }
        }

        None
    }
}
