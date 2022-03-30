use rust_truck_router::{
    algo::{
        ch::*, ch_potential::CHPotential, core_ch::CoreContractionHierarchy, dijkstra::OwnedDijkstra, mcd::OneRestrictionDijkstra,
        mcd_2::TwoRestrictionDijkstra,
    },
    io::*,
    types::*,
};

use rand::Rng;
use std::{
    env,
    error::Error,
    fs::File,
    io::{LineWriter, Write},
    path::Path,
    time::Instant,
};

fn main() -> Result<(), Box<dyn Error>> {
    let arg = &env::args().skip(1).next().expect("No directory arg given");
    let path = Path::new(arg);
    let first_out = Vec::<EdgeId>::load_from(path.join("first_out"))?;
    let head = Vec::<NodeId>::load_from(path.join("head"))?;
    let travel_time = Vec::<Weight>::load_from(path.join("travel_time"))?;
    let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;

    // let graph_mcd = OwnedOneRestrictionGraph::new(first_out, head, travel_time);
    let graph = OwnedGraph::new(first_out, head, travel_time);

    let s = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);
    let t = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);

    // let is_routing_node = load_routingkit_bitvector(path.join("is_routing_node"))?;
    // path with distance 20517304
    // let s = is_routing_node.to_local(80232745).unwrap(); // osm_id
    // let t = is_routing_node.to_local(824176810).unwrap(); // osm_id

    let mut core_ch = CoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;
    core_ch.check();

    let mut time = Instant::now();
    core_ch.init_new_s(s);
    core_ch.init_new_t(t);
    println!("Core ch init s and t took {} ms", time.elapsed().as_secs_f64() * 1000.0);

    time = Instant::now();
    let dist = core_ch.run_query();

    if dist.is_some() {
        println!("From {} to {}: {}", s, t, dist.unwrap());
        println!("Took {} ms", time.elapsed().as_secs_f64() * 1000.0);
    }

    println!("Validating result using dijkstra");
    let mut csp_pot = OneRestrictionDijkstra::new(graph.borrow());
    csp_pot.init_new_s(s);
    let csp_pot_dist = csp_pot.dist_query(t);

    assert_eq!(dist, csp_pot_dist);

    Ok(())
}
