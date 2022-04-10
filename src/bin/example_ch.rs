use rust_truck_router::{
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

    let first_out = Vec::<rust_truck_router::types::EdgeId>::load_from(path.join("first_out"))?;
    let head = Vec::<rust_truck_router::types::NodeId>::load_from(path.join("head"))?;
    let travel_time = Vec::<rust_truck_router::types::Weight>::load_from(path.join("travel_time"))?;

    let mut ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
    ch.check();

    let graph = rust_truck_router::types::OwnedGraph::new(first_out, head, travel_time);
    let mut instance = Dijkstra::new(&graph);

    println!("Graph with {} nodes and {} edges", graph.num_nodes(), graph.num_arcs());

    let s = rand::thread_rng().gen_range(0..graph.num_nodes() as rust_truck_router::types::NodeId);
    let t = rand::thread_rng().gen_range(0..graph.num_nodes() as rust_truck_router::types::NodeId);

    ch.init_new_s(s);
    ch.init_new_t(t);
    instance.init_new_s(s);

    report_time("random dijkstra one-to-one distance query", || {
        println!("From {} to {}: {:?}", s, t, instance.dist_query(t));
    });
    report_time("random ch one-to-one distance query", || {
        println!("From {} to {}: {:?}", s, t, ch.run_query());
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
