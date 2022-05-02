use rust_truck_router::{
    algo::{
        ch::{ContractionHierarchy, ContractionHierarchyQuery},
        dijkstra::{Dijkstra, DijkstraData},
    },
    types::{OwnedGraph, *},
};
use std::error::Error;
use std::path::Path;

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
    let mut ch_query = ContractionHierarchyQuery::new(ch.borrow());

    let graph = OwnedGraph::new(first_out, head, travel_time);
    let mut dijkstra_state = DijkstraData::new(graph.num_nodes());
    let dijkstra = Dijkstra::new(graph.borrow());

    for s in 0..5 {
        dijkstra_state.init_new_s(s);
        ch_query.init_new_s(s);
        for t in 0..5 {
            println!("Testing {} to {}", s, t);
            ch_query.init_new_t(t);
            assert_eq!(dijkstra.dist_query(&mut dijkstra_state, t), ch_query.run_query());
        }
    }

    Ok(())
}
