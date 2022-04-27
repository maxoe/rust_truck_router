use rand::Rng;
use rust_truck_router::{
    algo::{ch::ContractionHierarchy, ch_potential::CHPotential, csp::*},
    experiments::measurement::{CSP1MeasurementResult, CSPMeasurementResult, MeasurementResult},
    io::*,
    // osm_id_mapper::OSMIDMapper,
    types::*,
};

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
    let path = Path::new(arg).canonicalize()?;
    let first_out = Vec::<EdgeId>::load_from(path.join("first_out"))?;
    let head = Vec::<NodeId>::load_from(path.join("head"))?;
    let travel_time = Vec::<Weight>::load_from(path.join("travel_time"))?;
    let is_parking_node = load_routingkit_bitvector(path.join("routing_parking_flags"))?;
    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
    ch.check();

    let graph = OwnedGraph::new(first_out, head, travel_time);
    let mut search_state = OneRestrictionDijkstraData::new_with_potential(graph.num_nodes(), CHPotential::from_ch(ch.borrow()));
    let search = OneRestrictionDijkstra::new(graph.borrow(), &is_parking_node);

    // let is_routing_node = load_routingkit_bitvector(path.join("is_routing_node"))?;
    // path with distance 20517304
    // let s = 422258;
    // let s = is_routing_node.to_local(80232745).unwrap(); // osm_id
    let s = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);
    search_state.init_new_s(s);
    // let t = 4548361;
    // let t = is_routing_node.to_local(824176810).unwrap(); // osm_id
    let t = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);

    let n = 1;
    let max_driving_dist_step = 10000;
    let max_driving_dist_start = 1_500_000;
    let max_driving_dist_stop = 25_000_000;
    let pause_duration = 3_600_000;

    #[derive(Debug, Clone, Copy)]
    struct LocalMeasurementResult {
        pub max_driving_time_ms: u32,
        standard: CSP1MeasurementResult,
    }

    impl MeasurementResult for LocalMeasurementResult {
        const OWN_HEADER: &'static str = "max_driving_time_ms";

        fn get_header() -> String {
            format!("{},{}", Self::OWN_HEADER, CSP1MeasurementResult::get_header())
        }
        fn as_csv(&self) -> String {
            format!("{},{}", self.max_driving_time_ms, self.standard.as_csv())
        }
    }

    let it = (max_driving_dist_start..max_driving_dist_stop + 1).step_by(max_driving_dist_step);
    let number_of_steps = it.clone().count() * n;

    let mut results = Vec::with_capacity(number_of_steps);
    for (i, current_max_distance) in it.enumerate() {
        for j in 0..n {
            println!("Query {}/{}", i * n + j + 1, number_of_steps);
            search_state.set_restriction(current_max_distance, pause_duration);
            search_state.reset();

            let start = Instant::now();
            let result = search.dist_query(&mut search_state, t);
            let time = start.elapsed();

            // println!("{}", search.info());

            if let Some(dist) = result {
                let path = search_state.current_best_path_to(t, true).unwrap();
                let number_flagged_nodes = search.flagged_nodes_on_path(&path);
                let number_pauses = search.reset_nodes_on_path(&path);

                results.push(LocalMeasurementResult {
                    max_driving_time_ms: current_max_distance,
                    standard: CSP1MeasurementResult {
                        standard: CSPMeasurementResult {
                            graph_num_nodes: graph.num_nodes(),
                            graph_num_edges: graph.num_arcs(),
                            num_queue_pushes: search_state.num_queue_pushes,
                            num_settled: search_state.num_settled,
                            num_labels_propagated: search_state.num_labels_propagated,
                            num_labels_reset: search_state.num_labels_reset,
                            num_nodes_searched: search_state.get_number_of_visited_nodes(),
                            time,
                            path_distance: Some(dist),
                            path_number_nodes: Some(path.0.len()),
                            path_number_flagged_nodes: Some(number_flagged_nodes.len()),
                        },
                        path_number_pauses: Some(number_pauses.len()),
                    },
                });
            } else {
                results.push(LocalMeasurementResult {
                    max_driving_time_ms: current_max_distance,
                    standard: CSP1MeasurementResult {
                        standard: CSPMeasurementResult {
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
                        },
                        path_number_pauses: None,
                    },
                });
            }
        }
    }

    let file = File::create("measure_csp_variable_max_driving_time-".to_owned() + path.file_name().unwrap().to_str().unwrap() + ".txt")?;
    let mut file = LineWriter::new(file);
    writeln!(file, "{}", LocalMeasurementResult::get_header())?;
    for r in results {
        writeln!(file, "{}", r.as_csv())?;
    }

    Ok(())
}
