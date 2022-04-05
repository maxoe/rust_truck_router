use rand::Rng;
use rust_truck_router::{
    algo::{ch::*, ch_potential::CHPotential, dijkstra::Dijkstra, mcd_2::TwoRestrictionDijkstra},
    experiments::measurement::{CSP2MeasurementResult, CSPMeasurementResult, MeasurementResult},
    io::*,
    types::*,
};
use std::{
    env,
    error::Error,
    fs::File,
    io::{stdout, LineWriter, Write},
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

    let graph = OwnedGraph::new(first_out, head, travel_time);

    let ch = ContractionHierarchy::load_from_routingkit_dir(path.join("ch"))?;
    ch.check();

    let mut search = TwoRestrictionDijkstra::new_with_potential(&graph, CHPotential::from_ch(ch));
    search
        .set_reset_flags(is_parking_node.to_bytes())
        .set_restriction(32_400_000, 32_400_000, 16_200_000, 270_000);

    let log_num_nodes = (graph.num_nodes() as f32).log2() as usize;
    let mut dijkstra = Dijkstra::new(&graph);

    let n = 100;
    let mut result = Vec::with_capacity(n);

    #[derive(Debug, Clone, Copy)]
    struct LocalMeasurementResult {
        pub dijkstra_rank_exponent: usize,
        standard: CSP2MeasurementResult,
    }

    impl MeasurementResult for LocalMeasurementResult {
        const OWN_HEADER: &'static str = "dijkstra_rank_exponent";

        fn get_header() -> String {
            format!("{},{}", Self::OWN_HEADER, CSP2MeasurementResult::get_header())
        }
        fn as_csv(&self) -> String {
            format!("{},{}", self.dijkstra_rank_exponent, self.standard.as_csv())
        }
    }

    let mut stat_logs = Vec::with_capacity(n * log_num_nodes);

    for _i in 0..n {
        print!("Progress {}/{}\r", _i, n);
        stdout().flush()?;

        let s = rand::thread_rng().gen_range(0..graph.num_nodes() as NodeId);

        search.init_new_s(s);
        dijkstra.init_new_s(s);

        let rank_order = dijkstra.ranks_only_exponentials();
        dijkstra.ranks_only_exponentials();

        let mut rank_times = Vec::with_capacity(log_num_nodes);
        for (i, current_t) in rank_order.into_iter().enumerate() {
            let start = Instant::now();
            search.init_new_s(s);
            let dist = search.dist_query(current_t);
            let time = start.elapsed();
            rank_times.push(time);

            if dist.is_some() {
                let path = search.current_best_path_to(current_t, true).unwrap();
                let number_flagged_nodes = search.flagged_nodes_on_path(&path);
                let number_pauses = search.reset_nodes_on_path(&path);

                stat_logs.push(LocalMeasurementResult {
                    dijkstra_rank_exponent: i,
                    standard: CSP2MeasurementResult {
                        standard: CSPMeasurementResult {
                            graph_num_nodes: graph.num_nodes(),
                            graph_num_edges: graph.num_arcs(),
                            num_queue_pushes: search.num_queue_pushes,
                            num_settled: search.num_settled,
                            num_labels_propagated: search.num_labels_propagated,
                            num_labels_reset: search.num_labels_reset,
                            num_nodes_searched: search.get_number_of_visited_nodes(),
                            time,
                            path_distance: dist,
                            path_number_nodes: Some(path.0.len()),
                            path_number_flagged_nodes: Some(number_flagged_nodes.len()),
                        },
                        path_number_short_pauses: Some(number_pauses.0.len()),
                        path_number_long_pauses: Some(number_pauses.1.len()),
                    },
                });
            } else {
                stat_logs.push(LocalMeasurementResult {
                    dijkstra_rank_exponent: i,
                    standard: CSP2MeasurementResult {
                        standard: CSPMeasurementResult {
                            graph_num_nodes: graph.num_nodes(),
                            graph_num_edges: graph.num_arcs(),
                            num_queue_pushes: search.num_queue_pushes,
                            num_settled: search.num_settled,
                            num_labels_propagated: search.num_labels_propagated,
                            num_labels_reset: search.num_labels_reset,
                            num_nodes_searched: search.get_number_of_visited_nodes(),
                            time,
                            path_distance: None,
                            path_number_nodes: None,
                            path_number_flagged_nodes: None,
                        },
                        path_number_short_pauses: None,
                        path_number_long_pauses: None,
                    },
                });
            }
        }
        result.push(rank_times);
    }
    println!("Progress {}/{}", n, n);

    // let file = File::create("measure_csp_2_1000_queries_rank_times-".to_owned() + path.file_name().unwrap().to_str().unwrap() + ".txt")?;
    // let mut file = LineWriter::new(file);

    // writeln!(
    //     file,
    //     "{}",
    //     (0..log_num_nodes).map(|d| u32::pow(2, d as u32).to_string()).collect::<Vec<_>>().join(",")
    // )?;
    // for one_result in result {
    //     writeln!(
    //         file,
    //         "{}",
    //         one_result
    //             .into_iter()
    //             .map(|d| (d.as_secs_f64() * 1000.0).to_string())
    //             .collect::<Vec<_>>()
    //             .join(",")
    //     )?;
    // }

    let file = File::create("measure_csp_2_1000_queries_rank_times-".to_owned() + path.file_name().unwrap().to_str().unwrap() + ".txt")?;
    let mut file = LineWriter::new(file);
    writeln!(file, "{}", LocalMeasurementResult::get_header())?;
    for r in stat_logs {
        writeln!(file, "{}", r.as_csv())?;
    }

    Ok(())
}
