use stud_rust_base::{
    algo::{ch::*, dijkstra::*},
    io::*,
    time::report_time,
    types::*,
};

use rand::Rng;
use std::{env, error::Error, path::Path};

fn main() -> Result<(), Box<dyn Error>> {
    let arg = &env::args().skip(1).next().expect("No directory arg given");
    let path = Path::new(arg);

    let first_out = Vec::<stud_rust_base::types::EdgeId>::load_from(path.join("first_out"))?;
    let head = Vec::<stud_rust_base::types::NodeId>::load_from(path.join("head"))?;
    let travel_time = Vec::<stud_rust_base::types::Weight>::load_from(path.join("travel_time"))?;
    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;

    ch.check();

    let graph = stud_rust_base::types::OwnedGraph::new(first_out, head, travel_time);

    println!("Graph with {} nodes and {} edges", graph.num_nodes(), graph.num_arcs());

    let s = rand::thread_rng().gen_range(0..graph.num_nodes() as stud_rust_base::types::NodeId);
    let t = rand::thread_rng().gen_range(0..graph.num_nodes() as stud_rust_base::types::NodeId);

    let mut instance = Dijkstra::new(graph.borrow(), s);

    report_time("random dijkstra one-to-one distance query", || {
        println!("From {} to {}: {:?}", s, t, instance.dist_query(t));
    });
    report_time("random ch one-to-one distance query", || {
        println!("From {} to {}: {:?}", s, t, ch.query(s, t));
    });

    let path = instance.current_node_path_to(t);
    match path {
        None => println!("No path found from {} to {}", s, t),
        Some(p) => {
            // print!("[");
            println!("Path has length {}", p.len());
        }
    }

    Ok(())
}
