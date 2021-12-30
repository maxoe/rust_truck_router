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

pub struct Dijkstra<G, P = NoPotential>
where
    G: Graph + OutgoingEdgeIterable,
    P: Potential<G::WeightType>,
{
    data: DijkstraData<G::WeightType>,
    s: NodeId,
    graph: G,
    potential: P,
}

impl<G, P> Dijkstra<G, P>
where
    G: Graph + OutgoingEdgeIterable,
    P: Potential<G::WeightType>,
{
    pub fn new_with_potential(graph: G, s: NodeId, potential: P) -> Self {
        Self {
            data: DijkstraData::new(graph.num_nodes()),
            s: s,
            graph: graph,
            potential: potential,
        }
    }
    pub fn current_node_path_to(&self, t: NodeId) -> Option<Vec<NodeId>> {
        return self.data.path(self.s, t);
    }
}

impl<G: Graph + OutgoingEdgeIterable> Dijkstra<G, NoPotential> {
    pub fn new(graph: G, s: NodeId) -> Self {
        Self {
            data: DijkstraData::new(graph.num_nodes()),
            s: s,
            graph: graph,
            potential: NoPotential {},
        }
    }
}

impl<G: OutgoingEdgeIterable> Dijkstra<G> {
    pub fn dist_query(&mut self, t: NodeId) -> Option<G::WeightType> {
        let dist = &mut self.data.dist;
        let pred = &mut self.data.pred;
        let queue = &mut self.data.queue;
        let pot = &self.potential;

        dist[self.s as usize] = G::WeightType::zero();
        // pred[self.s as usize] = (s, edge_weight);
        queue.push(State {
            node: self.s,
            distance: G::WeightType::zero().link(&pot.potential(self.s)),
        });

        while let Some(State { distance: _, node: node_id }) = queue.pop() {
            if node_id == t {
                return Some(dist[node_id as usize]);
            }

            for (&edge_weight, &neighbor_node) in self.graph.outgoing_edge_iter(node_id) {
                let new_dist = dist[node_id as usize].link(&edge_weight);

                if new_dist < dist[neighbor_node as usize] {
                    if queue.contains_index(neighbor_node as usize) {
                        queue.decrease_key(State {
                            distance: new_dist.link(&pot.potential(neighbor_node)),
                            node: neighbor_node,
                        });
                    } else {
                        queue.push(State {
                            distance: new_dist.link(&pot.potential(neighbor_node)),
                            node: neighbor_node,
                        });
                    }

                    dist[neighbor_node as usize] = new_dist;
                    // TODO actual edge id ,is it even necessary?
                    pred[neighbor_node as usize] = (node_id, EdgeId::MAX);
                }
            }
        }

        None
    }
}
