use std::{env, error::Error, path::Path, time::Instant};

use rand::Rng;
use rust_truck_router::{
    algo::{ch::ContractionHierarchy, csp_bidir_chpot::CSPBidirAstarCHPotQuery},
    io::load_routingkit_bitvector,
    types::{Graph, NodeId, OwnedGraph, EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME},
};

fn main() -> Result<(), Box<dyn Error>> {
    let arg = &env::args().skip(1).next().expect("No directory arg given");
    let path = Path::new(arg);

    let graph = OwnedGraph::load_from_routingkit_dir(path)?;
    let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;
    let bw_graph = OwnedGraph::reverse(graph.borrow());
    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;

    let mut bidir_astar_query = CSPBidirAstarCHPotQuery::new(graph.borrow(), bw_graph.borrow(), &is_parking_node, ch.borrow());
    bidir_astar_query.set_restriction(EU_SHORT_DRIVING_TIME, EU_SHORT_PAUSE_TIME);
    {
        let s = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);
        let t = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);
        let start = Instant::now();

        bidir_astar_query.init_new_s(s);
        bidir_astar_query.init_new_t(t);
        let dist = bidir_astar_query.run_query();
        println!("Time elapsed: {:.2}ms", start.elapsed().as_secs_f64() * 1000.0);
        println!("Result: {:?}", dist);
    }

    Ok(())
}
