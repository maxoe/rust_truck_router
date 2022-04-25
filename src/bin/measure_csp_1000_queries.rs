use rand::Rng;
use rust_truck_router::{
    algo::{
        ch::*,
        ch_potential::CHPotential,
        csp::{OneRestrictionDijkstra, OneRestrictionDijkstraData},
    },
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
    let first_out = Vec::<EdgeId>::load_from(path.join("first_out"))?;
    let head = Vec::<NodeId>::load_from(path.join("head"))?;
    let travel_time = Vec::<Weight>::load_from(path.join("travel_time"))?;
    let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;

    let graph = OwnedGraph::new(first_out, head, travel_time);

    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
    ch.check();

    let mut search_state = OneRestrictionDijkstraData::new_with_potential(graph.num_nodes(), CHPotential::from_ch(ch.borrow()));
    let search = OneRestrictionDijkstra::new(graph.borrow());
    search_state.set_reset_flags(is_parking_node.to_bytes()).set_restriction(16_200_000, 2_700_000);

    let mut time = Duration::ZERO;
    let mut results = Vec::with_capacity(1000);

    for _i in 0..1000 {
        let s = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);
        let t = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);

        let start = Instant::now();
        search_state.init_new_s(s);
        search.dist_query(&mut search_state, t);
        time = time.checked_add(Instant::now() - start).unwrap();

        results.push(CSPMeasurementResult {
            graph_num_nodes: graph.num_nodes(),
            graph_num_edges: graph.num_arcs(),
            num_queue_pushes: search_state.num_queue_pushes,
            num_settled: search_state.num_settled,
            num_labels_propagated: search_state.num_labels_propagated,
            num_labels_reset: search_state.num_labels_reset,
            num_nodes_searched: search_state.get_number_of_visited_nodes(),
            time,
            path_distance: None,
            path_number_nodes: None,
            path_number_flagged_nodes: None,
        });

        // if _i % 100 == 0 {
        println!("{}: {}ms", _i, (Instant::now() - start).as_secs_f64() * 1000.0);
        println!("{}", search_state.info())
        // };
    }

    let file = File::create("measure_csp_1000_queries-".to_owned() + path.file_name().unwrap().to_str().unwrap() + ".txt")?;
    let mut file = LineWriter::new(file);
    writeln!(file, "{}", CSPMeasurementResult::get_header())?;

    for r in results {
        writeln!(file, "{}", r.as_csv())?;
    }

    Ok(())
}
