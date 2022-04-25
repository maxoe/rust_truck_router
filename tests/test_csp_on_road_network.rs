use rust_truck_router::{
    algo::{
        csp::*,
        dijkstra::{Dijkstra, DijkstraData},
    },
    io::*,
    types::*,
};

use rand::{Rng, SeedableRng};
use std::{error::Error, path::Path};

#[test]
#[ignore]
fn hundred_ka_queries_without_constraints() -> Result<(), Box<dyn Error>> {
    let path = std::env::current_dir()?.as_path().join(Path::new("test_data/large/parking_ka_hgv/"));
    let first_out = Vec::<EdgeId>::load_from(path.join("first_out"))?;
    let head = Vec::<NodeId>::load_from(path.join("head"))?;
    let travel_time = Vec::<Weight>::load_from(path.join("travel_time"))?;
    let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;

    let graph = OwnedGraph::new(first_out, head, travel_time);
    println!("Graph with {} nodes and {} edges", graph.num_nodes(), graph.num_arcs());

    let mut gen = rand::rngs::StdRng::seed_from_u64(1269803542210214824);
    let mut instance_state = DijkstraData::new(graph.num_nodes());
    let dijkstra = Dijkstra::new(graph.borrow());

    for i in 0..100 {
        let s = gen.gen_range(0..graph.num_nodes() as NodeId);
        let t = gen.gen_range(0..graph.num_nodes() as NodeId);
        println!("Query #{} from {} to {} without constraints", i, s, t);
        instance_state.init_new_s(s);
        let mut instance_mcd_state = OneRestrictionDijkstraData::new(graph.num_nodes());
        instance_mcd_state.init_new_s(s);
        instance_mcd_state.set_reset_flags(is_parking_node.to_bytes());
        let instance_mcd = OneRestrictionDijkstra::new(graph.borrow());
        assert_eq!(dijkstra.dist_query(&mut instance_state, t), instance_mcd.dist_query(&mut instance_mcd_state, t));
        assert_eq!(instance_state.current_node_path_to(t), instance_mcd_state.current_best_node_path_to(t));
    }

    Ok(())
}
