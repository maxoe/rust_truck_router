use crate::types::*;

use super::dijkstra::{Dijkstra, DijkstraData};

pub trait Potential {
    fn potential(&mut self, node: NodeId) -> Weight;
    fn init_new_t(&mut self, t: NodeId);
}

#[derive(Clone)]
pub struct NoPotential {}

impl Potential for NoPotential {
    fn potential(&mut self, _node: NodeId) -> Weight {
        Weight::zero()
    }
    fn init_new_t(&mut self, _t: NodeId) {}
}

pub struct Astar {}

impl<'a, P: Potential> AstarQuery<'a, P> for Astar {
    fn astar_query(q: STQuery, graph: BorrowedGraph<'a>, p: P) -> Option<NodeId> {
        let mut state = DijkstraData::new_with_potential(graph.num_nodes(), p);
        state.init_new_s(q.s);
        let dijkstra = Dijkstra::new(graph);
        dijkstra.dist_query(&mut state, q.t)
    }
}

pub trait AstarQuery<'a, P: Potential> {
    fn astar_query(q: STQuery, graph: BorrowedGraph<'a>, p: P) -> Option<Weight>;
}
