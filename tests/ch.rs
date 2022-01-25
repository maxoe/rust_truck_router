use std::error::Error;
use std::path::Path;

use stud_rust_base::{
    algo::{ch::ContractionHierarchy, dijkstra::Dijkstra, mcd::*},
    io::*,
    types::{OwnedGraph, *},
};

use rand::{Rng, SeedableRng};

#[test]
fn load_ch() -> Result<(), Box<dyn Error>> {
    let path = std::env::current_dir()?.as_path().join(Path::new("test_data/ch_instances/"));
    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
    ch.check();
    Ok(())
}

#[test]
fn some_ch_queries() -> Result<(), Box<dyn Error>> {
    // 0 -> 1 -> 2p -> 4
    //      | -> 3p -> |
    let first_out = vec![0, 1, 3, 4, 5, 5];
    let head = vec![1, 2, 3, 4, 4];
    let travel_time = vec![1, 4, 3, 2, 4];

    let path = std::env::current_dir()?.as_path().join(Path::new("test_data/ch_instances/"));
    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;

    let graph = OwnedGraph::new(first_out, head, travel_time);

    for s in 0..5 {
        for t in 0..5 {
            println!("Testing {} to {}", s, t);

            let mut dijkstra = Dijkstra::new(graph.borrow(), s);
            assert_eq!(dijkstra.dist_query(t), ch.query(s, t));
        }
    }

    Ok(())
}

#[test]
#[ignore]
fn hundred_ka_queries() -> Result<(), Box<dyn Error>> {
    let path = std::env::current_dir()?.as_path().join(Path::new("test_data/parking_ka_hgv/"));
    let first_out = Vec::<EdgeId>::load_from(path.join("first_out"))?;
    let head = Vec::<NodeId>::load_from(path.join("head"))?;
    let travel_time = Vec::<Weight>::load_from(path.join("travel_time"))?;
    let graph = OwnedGraph::new(first_out.clone(), head.clone(), travel_time.clone());
    println!("Graph with {} nodes and {} edges", graph.num_nodes(), graph.num_arcs());

    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
    ch.check();

    let graph_mcd = OwnedOneRestrictionGraph::new(first_out, head, travel_time);

    let mut gen = rand::rngs::StdRng::seed_from_u64(1269803542210214824);

    for i in 0..100 {
        let s = gen.gen_range(0..graph_mcd.num_nodes() as NodeId);
        let t = gen.gen_range(0..graph_mcd.num_nodes() as NodeId);
        println!("Query #{} from {} to {}", i, s, t);
        let mut instance = Dijkstra::new(graph.borrow(), s);
        assert_eq!(instance.dist_query(t), ch.query(s, t));
        // assert_eq!(instance.current_node_path_to(t), instance_mcd.current_best_node_path_to(t))
    }

    Ok(())
}
