use rust_truck_router::{
    algo::{astar::*, dijkstra::*},
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

    // let first_out = vec![0, 1, 2, 3, 5];
    // let head = vec![1, 2, 3, 0, 1];
    // let travel_time = vec![2, 3, 3, 1, 5];

    let graph = rust_truck_router::types::OwnedGraph::new(first_out, head, travel_time);

    println!("Graph with {} nodes and {} edges", graph.num_nodes(), graph.num_arcs());

    let s = rand::thread_rng().gen_range(0..graph.num_nodes() as rust_truck_router::types::NodeId);
    let t = rand::thread_rng().gen_range(0..graph.num_nodes() as rust_truck_router::types::NodeId);

    let mut instance = Dijkstra::new(&graph);
    instance.init_new_s(s);
    let mut instance_astar = Dijkstra::new_with_potential(&graph, NoPotential {});
    instance_astar.init_new_s(s);

    report_time("random dijkstra one-to-one distance query", || {
        println!("From {} to {}: {:?}", s, t, instance.dist_query(t));
    });
    report_time("random astar one-to-one distance query", || {
        println!("From {} to {}: {:?}", s, t, instance_astar.dist_query(t));
    });

    let path = instance.current_node_path_to(t);
    let astar_path = instance_astar.current_node_path_to(t);

    for (dijkstra_i, astar_i) in path.iter().zip(astar_path.iter()) {
        assert_eq!(dijkstra_i, astar_i);
    }

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
