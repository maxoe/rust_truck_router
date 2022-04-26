use rust_truck_router::{
    algo::{
        ch::ContractionHierarchy,
        ch_potential::CHPotential,
        core_ch::CoreContractionHierarchy,
        csp_2::{TwoRestrictionDijkstra, TwoRestrictionDijkstraData},
        csp_2_core_ch_chpot::CSP2AstarCoreCHQuery,
    },
    io::*,
    types::*,
};

use rand::Rng;
use std::{env, error::Error, path::Path, time::Instant};

fn main() -> Result<(), Box<dyn Error>> {
    let arg = &env::args().skip(1).next().expect("No directory arg given");
    let path = Path::new(arg);
    let first_out = Vec::<EdgeId>::load_from(path.join("first_out"))?;
    let head = Vec::<NodeId>::load_from(path.join("head"))?;
    let travel_time = Vec::<Weight>::load_from(path.join("travel_time"))?;
    let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;

    let graph = OwnedGraph::new(first_out, head, travel_time);

    let s = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);
    let t = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);

    // let is_routing_node = load_routingkit_bitvector(path.join("is_routing_node"))?;
    // path with distance 20517304
    // let s = is_routing_node.to_local(80232745).unwrap(); // osm_id
    // let t = is_routing_node.to_local(824176810).unwrap(); // osm_id

    // let mut core_ch = CSPAstarCoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;
    // core_ch.set_restriction(16_200_000, 2_700_000);
    let core_ch = CoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;
    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;

    let mut core_ch_query = CSP2AstarCoreCHQuery::new(core_ch.borrow(), ch.borrow());

    core_ch_query.set_restriction(32_400_000, 32_400_000, 16_200_000, 2_700_000);
    core_ch_query.check();

    core_ch_query.init_new_s(s);
    core_ch_query.init_new_t(t);

    let time = Instant::now();
    let dist = core_ch_query.run_query();
    println!("Took {} ms", time.elapsed().as_secs_f64() * 1000.0);

    if dist.is_some() {
        println!("From {} to {}: {}", s, t, dist.unwrap());
    } else {
        println!("No path found")
    }

    print!("Validating result using constrained dijkstra");
    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
    ch.check();
    let mut csp_pot_state = TwoRestrictionDijkstraData::new_with_potential(graph.num_nodes(), CHPotential::from_ch(ch.borrow()));
    let csp_pot = TwoRestrictionDijkstra::new(graph.borrow());
    csp_pot_state.init_new_s(s);
    csp_pot_state
        .set_reset_flags(is_parking_node.to_bytes())
        .set_restriction(32_400_000, 32_400_000, 16_200_000, 2_700_000); //.set_restriction(16_200_000, 2_700_000);
    let csp_pot_dist = csp_pot.dist_query(&mut csp_pot_state, t);

    let csp_pot_num_breaks = if let Some(path) = csp_pot_state.current_best_path_to(t, true) {
        Some(csp_pot_state.reset_nodes_on_path(&path).0.len())
    } else {
        None
    };
    assert_eq!(dist, csp_pot_dist);
    assert!(dist == csp_pot_dist || (csp_pot_num_breaks.is_some() && csp_pot_num_breaks.unwrap() > 1));
    assert_eq!(dist, core_ch_query.last_dist);

    println!(" - Done");

    Ok(())
}
