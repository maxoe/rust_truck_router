use stud_rust_base::{algo::dijkstra::*, io::*, time::report_time, types::*};

use rand::Rng;
use std::{env, error::Error, path::Path};

fn main() -> Result<(), Box<dyn Error>> {
	let arg = &env::args().skip(1).next().expect("No directory arg given");
	let path = Path::new(arg);

	let first_out = Vec::<EdgeId>::load_from(path.join("first_out"))?;
	let head = Vec::<NodeId>::load_from(path.join("head"))?;
	let travel_time = Vec::<Weight>::load_from(path.join("travel_time"))?;

	// let first_out = vec![0, 1, 2, 3, 5];
	// let head = vec![1, 2, 3, 0, 1];
	// let travel_time = vec![2, 3, 3, 1, 5];

	let graph = OwnedGraph::new(first_out, head, travel_time);

	println!("Graph with {} nodes and {} edges", graph.num_nodes(), graph.num_arcs());

	let s = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);
	let t = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);

	let mut instance = Dijkstra::new(graph.borrow(), s);

	report_time("random dijkstra one-to-one distance query", || {
		println!("From {} to {}: {:?}", s, t, instance.dist_query(t));
	});

	let path = instance.current_node_path_to(t);

	match path {
		None => println!("No path found from {} to {}", s, t),
		Some(p) => {
			// print!("[");
			// for node_id in p {
			// 	print!("{}, ", node_id);
			// }
			// println!("]")
			println!("Path has length {}", p.len());
		}
	}

	Ok(())
}
