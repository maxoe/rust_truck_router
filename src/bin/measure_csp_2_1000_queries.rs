use rand::Rng;
use rust_truck_router::{
    algo::{ch::*, ch_potential::CHPotential, csp_2::TwoRestrictionDijkstra},
    experiments::measurement::{CSPMeasurementResult, MeasurementResult},
    io::*,
    types::*,
};
use std::{
    env,
    error::Error,
    fs::File,
    io::{LineWriter, Write},
    path::Path,
    time::{Duration, Instant},
};

fn main() -> Result<(), Box<dyn Error>> {
    let arg = &env::args().skip(1).next().expect("No directory arg given");
    let path = Path::new(arg);

    let first_out = Vec::<rust_truck_router::types::EdgeId>::load_from(path.join("first_out"))?;
    let head = Vec::<rust_truck_router::types::NodeId>::load_from(path.join("head"))?;
    let travel_time = Vec::<rust_truck_router::types::Weight>::load_from(path.join("travel_time"))?;

    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
    ch.check();
    let pot = CHPotential::from_ch(ch);
    let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;
    let graph_mcd = OwnedGraph::new(first_out, head, travel_time);
    let mut search = TwoRestrictionDijkstra::new_with_potential(&graph_mcd, pot);
    search
        .set_reset_flags(is_parking_node.to_bytes())
        .set_restriction(32_400_000, 32_400_000, 16_200_000, 2_700_000);

    let mut time = Duration::ZERO;
    let mut results = Vec::with_capacity(1000);

    for _i in 0..1000 {
        if _i % 100 == 0 {
            println!("{}", _i);
        }

        let s = rand::thread_rng().gen_range(0..graph_mcd.num_nodes() as NodeId);
        let t = rand::thread_rng().gen_range(0..graph_mcd.num_nodes() as NodeId);

        let start = Instant::now();
        search.init_new_s(s);
        search.dist_query(t);
        time = time.checked_add(Instant::now() - start).unwrap();

        results.push(CSPMeasurementResult {
            graph_num_nodes: graph_mcd.num_nodes(),
            graph_num_edges: graph_mcd.num_arcs(),
            num_queue_pushes: search.num_queue_pushes,
            num_settled: search.num_settled,
            num_labels_propagated: search.num_labels_propagated,
            num_labels_reset: search.num_labels_reset,
            num_nodes_searched: search.get_number_of_visited_nodes(),
            time,
            path_distance: None,
            path_number_nodes: None,
            path_number_flagged_nodes: None,
        });
    }

    let file = File::create("measure_csp_2_1000_queries-".to_owned() + path.file_name().unwrap().to_str().unwrap() + ".txt")?;
    let mut file = LineWriter::new(file);
    writeln!(file, "{}", CSPMeasurementResult::get_header())?;
    for r in results {
        writeln!(file, "{}", r.as_csv())?;
    }

    Ok(())
}
