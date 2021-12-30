use stud_rust_base::{
    algo::{astar::*, dijkstra::Dijkstra, mcd::*},
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
    let mut weights_vector = vec![[0u32, 0u32]; travel_time.len()];

    let graph = stud_rust_base::types::OwnedGraph::new(first_out.clone(), head.clone(), travel_time.clone());

    // let first_out = vec![0, 1, 2, 3, 5];
    // let head = vec![1, 2, 3, 0, 1];
    // let travel_time = vec![2, 3, 3, 1, 5];
    // let mut weights_vector = vec![[2, 5], [3, 1], [3, 3], [1, 3], [5, 1]];

    for (w, t) in weights_vector.iter_mut().zip(travel_time) {
        w[0] = t;
    }

    let graph_mcd = stud_rust_base::algo::mcd::OwnedMultiCritGraph::new(first_out, head, weights_vector);

    println!("Graph with {} nodes and {} edges", graph.num_nodes(), graph.num_arcs());

    let s = rand::thread_rng().gen_range(0..graph_mcd.num_nodes() as stud_rust_base::types::NodeId);
    let t = rand::thread_rng().gen_range(0..graph_mcd.num_nodes() as stud_rust_base::types::NodeId);

    let mut instance = Dijkstra::new(graph.borrow(), s);
    let mut instance_mcd = MultiCriteriaDijkstra::new(graph_mcd.borrow(), s);

    report_time("random dijkstra one-to-one distance query", || {
        println!("From {} to {}: {:?}", s, t, instance.dist_query(t));
    });

    report_time("random mcdijkstra one-to-one distance query", || {
        println!("From {} to {}: {:?}", s, t, instance_mcd.dist_query(t));
    });
    //     let path = instance.current_node_path_to(t);

    //     match path {
    //         None => println!("No path found from {} to {}", s, t),
    //         Some(p) => {
    //             // print!("[");
    //             // for node_id in p {
    //             // 	print!("{}, ", node_id);
    //             // }
    //             // println!("]")
    //             println!("Path has length {}", p.len());
    //         }
    //     }

    Ok(())
}
