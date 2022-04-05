use rust_truck_router::{
    algo::{ch::ContractionHierarchy, ch_potential::CHPotential, csp::OneRestrictionDijkstra, one_break_core_ch::OneBreakCoreContractionHierarchy},
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

    // let graph_mcd = OwnedOneRestrictionGraph::new(first_out, head, travel_time);
    let graph = OwnedGraph::new(first_out, head, travel_time);

    // those use a break on hgv ger and europe
    // let s = 5945495;
    // let t = 11838613;
    let s = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);
    let t = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);

    // let is_routing_node = load_routingkit_bitvector(path.join("is_routing_node"))?;
    // path with distance 20517304
    // let s = is_routing_node.to_local(80232745).unwrap(); // osm_id
    // let t = is_routing_node.to_local(824176810).unwrap(); // osm_id

    let mut core_ch = OneBreakCoreContractionHierarchy::load_from_routingkit_dir(path.join("core_ch"))?;
    core_ch.add_restriction(16_200_000, 270_000);
    core_ch.check();

    let mut time = Instant::now();
    core_ch.init_new_s(s);
    core_ch.init_new_t(t);
    println!("Core ch init s and t took {} ms", time.elapsed().as_secs_f64() * 1000.0);

    time = Instant::now();
    let dist = core_ch.run_query();

    println!("Took {} ms", time.elapsed().as_secs_f64() * 1000.0);

    if dist.is_some() {
        println!("From {} to {}: {}", s, t, dist.unwrap());

        if core_ch.last_break.is_some() {
            println!(
                "Used one break:\n\t- max driving time: {} ms\n\t- pause time: {} ms",
                core_ch.last_break.unwrap().max_driving_time,
                core_ch.last_break.unwrap().pause_time
            );
        } else {
            println!("No break used");
        }
    } else {
        println!("No path found")
    }

    print!("Validating result using constrained dijkstra");
    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
    ch.check();
    let mut csp_pot = OneRestrictionDijkstra::new_with_potential(&graph, CHPotential::from_ch(ch));
    csp_pot.init_new_s(s);
    csp_pot.set_reset_flags(is_parking_node.to_bytes()).set_restriction(16_200_000, 270_000);
    let csp_pot_dist = csp_pot.dist_query(t);

    let csp_pot_num_breaks = if let Some(path) = csp_pot.current_best_path_to(t, true) {
        Some(csp_pot.reset_nodes_on_path(&path).len())
    } else {
        None
    };
    assert_eq!(dist, csp_pot_dist);
    assert!(dist == csp_pot_dist || (csp_pot_num_breaks.is_some() && csp_pot_num_breaks.unwrap() > 1));
    assert!(!dist.is_none() || core_ch.last_break.is_none());
    assert_eq!(dist, core_ch.last_dist);

    if core_ch.last_dist.is_some() {
        if core_ch.last_break.is_some() {
            assert!(csp_pot_num_breaks.unwrap() == 1);
        } else {
            assert!(csp_pot_num_breaks.unwrap() == 0);
        }
    }

    println!(" - Done");

    Ok(())
}
