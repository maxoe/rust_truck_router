use bit_vec::BitVec;

use stud_rust_base::{
    algo::{dijkstra::Dijkstra, mcd::*},
    io::*,
    types::*,
};

use rand::{Rng, SeedableRng};
use std::{error::Error, path::Path};

#[test]
#[ignore]
fn hundred_ka_queries() -> Result<(), Box<dyn Error>> {
    let path = std::env::current_dir()?.as_path().join(Path::new("test_data/parking_ka_hgv/"));
    let first_out = Vec::<EdgeId>::load_from(path.join("first_out"))?;
    let head = Vec::<NodeId>::load_from(path.join("head"))?;
    let travel_time = Vec::<Weight>::load_from(path.join("travel_time"))?;
    let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;

    let graph = OwnedGraph::new(first_out.clone(), head.clone(), travel_time.clone());
    println!("Graph with {} nodes and {} edges", graph.num_nodes(), graph.num_arcs());

    let graph_mcd = OwnedOneRestrictionGraph::new(first_out, head, travel_time);

    let mut gen = rand::rngs::StdRng::seed_from_u64(1269803542210214824);

    for i in 0..100 {
        let s = gen.gen_range(0..graph_mcd.num_nodes() as NodeId);
        let t = gen.gen_range(0..graph_mcd.num_nodes() as NodeId);
        println!("Query #{} from {} to {} without constraints", i, s, t);
        let mut instance = Dijkstra::new(graph.borrow(), s);
        let mut instance_mcd = OneRestrictionDijkstra::new(graph_mcd.borrow(), s);
        instance_mcd.set_reset_flags(is_parking_node.clone());

        assert_eq!(instance.dist_query(t), instance_mcd.dist_query(t));
        assert_eq!(instance.current_node_path_to(t), instance_mcd.current_best_node_path_to(t))
    }

    Ok(())
}
