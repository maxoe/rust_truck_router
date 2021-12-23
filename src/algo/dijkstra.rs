use crate::{index_heap::*, types::*};

#[derive(Copy, Clone, Eq, PartialEq, Debug, Ord, PartialOrd)]
pub struct State {
	pub distance: Weight,
	pub node: NodeId,
}

impl Indexing for State {
	fn as_index(&self) -> usize {
		self.node as usize
	}
}

pub struct DijkstraData {
	pub queue: IndexdMinHeap<State>,
	pub pred: Vec<(NodeId, EdgeId)>,
	pub dist: Vec<Weight>,
}

impl DijkstraData {
	pub fn new(n: usize) -> Self {
		Self {
			queue: IndexdMinHeap::new(n),
			pred: vec![(n as NodeId, EdgeId::MAX); n],
			dist: vec![INFINITY; n],
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

pub struct Dijkstra<G: Graph> {
	data: DijkstraData,
	s: NodeId,
	graph: G,
}

impl Dijkstra<OwnedGraph> {
	pub fn new(graph: OwnedGraph, s: NodeId) -> Self {
		Self {
			data: DijkstraData::new(graph.num_nodes()),
			s: s,
			graph: graph,
		}
	}
	pub fn dist_query(&mut self, t: NodeId) -> Option<Weight> {
		let dist = &mut self.data.dist;
		let pred = &mut self.data.pred;
		let queue = &mut self.data.queue;
		dist[self.s as usize] = 0;
		// pred[self.s as usize] = (s, edge_weight);
		queue.push(State { node: self.s, distance: 0 });

		while let Some(State { distance: d, node: node_id }) = queue.pop() {
			if node_id == t {
				return Some(d);
			}
			for edge_id in self.graph.first_out()[node_id as usize]..self.graph.first_out()[node_id as usize + 1] {
				let edge_weight = self.graph.weights()[edge_id as usize];
				let new_dist = d + edge_weight;
				let neighbor_node: NodeId = self.graph.head()[edge_id as usize];

				if new_dist < dist[neighbor_node as usize] {
					if queue.contains_index(neighbor_node as usize) {
						queue.decrease_key(State {
							distance: new_dist,
							node: neighbor_node,
						});
					} else {
						queue.push(State {
							distance: new_dist,
							node: neighbor_node,
						});
					}

					dist[neighbor_node as usize] = new_dist;
					pred[neighbor_node as usize] = (node_id, edge_weight);
				}
			}
		}

		None
	}

	pub fn current_node_path_to(&self, t: NodeId) -> Option<Vec<NodeId>> {
		return self.data.path(self.s, t);
	}
}
