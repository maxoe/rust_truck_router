use crate::{algo::astar::*, index_heap::*, types::*};

pub struct DijkstraData<W: WeightOps> {
    pub queue: IndexdMinHeap<State<W>>,
    pub pred: Vec<(NodeId, EdgeId)>,
    pub dist: Vec<W>,
}

impl<W: WeightOps> DijkstraData<W> {
    pub fn new(n: usize) -> Self {
        Self {
            queue: IndexdMinHeap::new(n),
            pred: vec![(n as NodeId, EdgeId::MAX); n],
            dist: vec![W::infinity(); n],
        }
    }

    pub fn path(&self, s: NodeId, t: NodeId) -> Option<Vec<NodeId>> {
        let mut path = vec![t];
        let mut current_id = t;
        while current_id != s && current_id as usize != self.pred.len() {
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
    pub fn new(graph: BorrowedGraph<'a>, s: NodeId) -> Self {
        Self {
            data: DijkstraData::new(graph.num_nodes()),
            s: s,
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
        self.data = DijkstraData::new(self.graph.num_nodes());
        self.num_settled = 0;
        self.num_labels_propagated = 0;
        self.num_queue_pushes = 0;
    }

    pub fn init(&mut self) {
        self.data.dist[self.s as usize] = 0;
        // pred[self.s as usize] = (s, edge_weight);
        self.data.queue.push(State {
            node: self.s,
            distance: 0.link(self.potential.potential(self.s)),
        });
    }

    #[inline(always)]
    pub fn tentative_distance_at(&self, t: NodeId) -> Weight {
        self.data.dist[t as usize]
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
        let pot = &self.potential;

        if let Some(next) = queue.pop() {
            self.settled_nodes_vec.push((next.node, next.distance));
            self.num_settled += 1;
            let node_id = next.node;

            for (&edge_weight, &neighbor_node) in self.graph.outgoing_edge_iter(node_id) {
                let new_dist = dist[node_id as usize].link(edge_weight);

                if new_dist < dist[neighbor_node as usize] {
                    if queue.contains_index(neighbor_node as usize) {
                        queue.decrease_key(State {
                            distance: new_dist.link(pot.potential(neighbor_node)),
                            node: neighbor_node,
                        });
                    } else {
                        self.num_queue_pushes += 1;
                        queue.push(State {
                            distance: new_dist.link(pot.potential(neighbor_node)),
                            node: neighbor_node,
                        });
                    }
                    self.num_labels_propagated += 1;
                    dist[neighbor_node as usize] = new_dist;
                    // TODO actual edge id ,is it even necessary?
                    pred[neighbor_node as usize] = (node_id, EdgeId::MAX);
                }
            }
            return Some(next);
        } else {
            None
        }
    }

    pub fn dist_query(&mut self, t: NodeId) -> Option<Weight> {
        self.init();

        while let Some(State { distance, node }) = self.settle_next_node() {
            if node == t {
                return Some(distance);
            }
        }

        None
    }
}
