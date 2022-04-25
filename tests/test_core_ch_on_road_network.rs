use rust_truck_router::{
    algo::{
        core_ch::{CoreContractionHierarchy, CoreContractionHierarchyQuery},
        csp::*,
    },
    io::*,
    types::*,
};

use rand::{Rng, SeedableRng};
use std::{error::Error, path::Path};

#[test]
#[ignore]
fn hundred_ka_queries() -> Result<(), Box<dyn Error>> {
    let path = std::env::current_dir()?.as_path().join(Path::new("test_data/large/parking_ka_hgv/"));
    let first_out = Vec::<EdgeId>::load_from(path.join("first_out"))?;
    let head = Vec::<NodeId>::load_from(path.join("head"))?;
    let travel_time = Vec::<Weight>::load_from(path.join("travel_time"))?;

    let graph = OwnedGraph::new(first_out, head, travel_time);
    println!("Graph with {} nodes and {} edges", graph.num_nodes(), graph.num_arcs());

    let mut gen = rand::rngs::StdRng::seed_from_u64(1269803542210214824);
    let core_ch = CoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;
    core_ch.check();
    let mut core_ch_query = CoreContractionHierarchyQuery::new(core_ch.borrow());
    let mut csp_pot_state = OneRestrictionDijkstraData::new(graph.num_nodes());
    let csp_pot = OneRestrictionDijkstra::new(graph.borrow());

    for i in 0..100 {
        let s = gen.gen_range(0..graph.num_nodes() as NodeId);
        let t = gen.gen_range(0..graph.num_nodes() as NodeId);
        println!("Query #{} from {} to {} without constraints", i, s, t);
        core_ch_query.init_new_s(s);
        core_ch_query.init_new_t(t);
        let dist = core_ch_query.run_query();

        csp_pot_state.init_new_s(s);
        let csp_pot_dist = csp_pot.dist_query(&mut csp_pot_state, t);

        assert_eq!(dist, csp_pot_dist);
    }

    Ok(())
}
