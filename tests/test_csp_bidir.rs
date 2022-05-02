use bit_vec::BitVec;
use rust_truck_router::{
    algo::{
        ch::ContractionHierarchy,
        csp_bidir::CSPBidirAstarCHPotQuery,
        dijkstra::{Dijkstra, DijkstraData},
    },
    types::{OwnedGraph, *},
};
use std::error::Error;
use std::path::Path;

#[test]
fn some_astar_bidir_queries() -> Result<(), Box<dyn Error>> {
    let path = std::env::current_dir()?.as_path().join(Path::new("test_data/ch_instances/ch"));
    let first_out = vec![0, 1, 3, 4, 5, 5];
    let head = vec![1, 2, 3, 4, 4];
    let travel_time = vec![1, 4, 3, 2, 4];
    let fw_graph = OwnedGraph::new(first_out, head, travel_time);
    let bw_graph = OwnedGraph::reverse(fw_graph.borrow());
    let ch = ContractionHierarchy::load_from_routingkit_dir(path)?;
    let is_parking_node = BitVec::from_elem(fw_graph.num_nodes(), false);
    let mut query = CSPBidirAstarCHPotQuery::new(fw_graph.borrow(), bw_graph.borrow(), &is_parking_node, ch.borrow());

    let mut dijkstra_state = DijkstraData::new(fw_graph.num_nodes());
    let dijkstra = Dijkstra::new(fw_graph.borrow());

    for s in 0..5 {
        dijkstra_state.init_new_s(s);
        query.init_new_s(s);
        for t in 0..5 {
            println!("Testing {} to {}", s, t);
            query.init_new_t(t);
            assert_eq!(dijkstra.dist_query(&mut dijkstra_state, t), query.run_query());
        }
    }

    Ok(())
}
