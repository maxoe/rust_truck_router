use rand::{Rng, SeedableRng};
use rust_truck_router::{
    algo::{csp_2::TwoRestrictionDijkstra, csp_2_core_ch_chpot::CSP2AstarCoreContractionHierarchy},
    io::{load_routingkit_bitvector, Load},
    types::{EdgeId, Graph, NodeId, OwnedGraph, Weight},
};
use std::{error::Error, path::Path};

#[test]
#[ignore]
fn hundred_ka_queries() -> Result<(), Box<dyn Error>> {
    let path = std::env::current_dir()?.as_path().join(Path::new("test_data/large/parking_ka_hgv/"));
    let first_out = Vec::<EdgeId>::load_from(path.join("first_out"))?;
    let head = Vec::<NodeId>::load_from(path.join("head"))?;
    let travel_time = Vec::<Weight>::load_from(path.join("travel_time"))?;
    let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;

    let max_driving_time_short = 4_000_000;
    let pause_time_short = 2_700_000;
    let max_driving_time_long = 32_400_000;
    let pause_time_long = 32_400_000;
    let graph = OwnedGraph::new(first_out.clone(), head.clone(), travel_time.clone());
    println!("Graph with {} nodes and {} edges", graph.num_nodes(), graph.num_arcs());

    let graph_mcd = OwnedGraph::new(first_out, head, travel_time);

    let mut gen = rand::rngs::StdRng::seed_from_u64(1269803542210214824);
    let mut core_ch = CSP2AstarCoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;
    core_ch.check();
    core_ch.set_restriction(max_driving_time_long, pause_time_long, max_driving_time_short, pause_time_short);

    let mut csp_pot = TwoRestrictionDijkstra::new(&graph);
    csp_pot
        .set_reset_flags(is_parking_node.to_bytes())
        .set_restriction(max_driving_time_long, pause_time_long, max_driving_time_short, pause_time_short);

    for i in 0..100 {
        let s = gen.gen_range(0..graph_mcd.num_nodes() as NodeId);
        let t = gen.gen_range(0..graph_mcd.num_nodes() as NodeId);
        println!("Query #{} from {} to {} with constraint", i, s, t);
        core_ch.init_new_s(s);
        core_ch.init_new_t(t);
        let dist = core_ch.run_query();

        csp_pot.init_new_s(s);
        let csp_pot_dist = csp_pot.dist_query(t);

        assert_eq!(dist, csp_pot_dist);
    }

    Ok(())
}
